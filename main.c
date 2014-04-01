/**HEADER********************************************************************
* 
* Copyright (c) 2008 Freescale Semiconductor;
* All Rights Reserved
*
*************************************************************************** 
*
* THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR 
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  
* IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
* THE POSSIBILITY OF SUCH DAMAGE.
*
**************************************************************************
*
* $FileName: main.c$
* $Version : 3.0.2.0$
* $Date    : Nov-21-2008$
*
* Comments:
*
*   This file contains the source for your new program.
*
*END************************************************************************/

#include <stdint.h>
#include <mqx.h>
#include <bsp.h>
#include <fio.h>
#include <message.h>
#include <stdlib.h>
#include <string.h>

#include "MCF52259.h"
#include "main.h"
#include "init.h"
#include "my_math.h"

extern void main_task(uint_32);
extern void lowpass_task(uint_32);
extern void highpass_task(uint_32);
extern void bandpass_task(uint_32);
extern void isr_task(uint_32);

TIME_STRUCT time_global1;
TIME_STRUCT time_global2;
uint_32 time_int = 0;
int counter = 0;


uint_32 uartRegisterMask = 0;

VMCF5225_STRUCT_PTR reg_ptr = (VMCF5225_STRUCT_PTR)BSP_IPSBAR;

#define pi 3.1415962
#define CUTOFF 100
#define SAMPLING_FREQUENCY 2075
/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 2000 Hz

fixed point precision: 16 bits

* 0 Hz - 400 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 500 Hz - 1000 Hz
  gain = 0
  desired attenuation = -60 dB
  actual attenuation = n/a

*/

/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 2000 Hz

fixed point precision: 16 bits

* 0 Hz - 400 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 500 Hz - 1000 Hz
  gain = 0
  desired attenuation = -35 dB
  actual attenuation = n/a

*/

#define FILTER_TAP_NUM 19

/*static int lowpass_hw[FILTER_TAP_NUM] = {
  -1617,
  -1182,
  2153,
  5648,
  3908,
  -2697,
  -5168,
  4330,
  20679,
  28879,
  20679,
  4330,
  -5168,
  -2697,
  3908,
  5648,
  2153,
  -1182,
  -1617
};

*/

float lowpass_hw[MAX_FILTER_LEN] = {
0.0003, 0.0009, 0.0013, 0.0014, 0.0007, -0.0009, -0.0029, -0.0046, -0.0047, -0.0022, 0.0028, 0.0089, 0.0133, 0.0130, 0.0060, -0.0072, -0.0227, -0.0340, -0.0336, -0.0159, 0.0202, 0.0700, 0.1240, 0.1698, 0.1960, 0.1960, 0.1698, 0.1240, 0.0700, 0.0202, -0.0159, -0.0336, -0.0340, -0.0227, -0.0072, 0.0060, 0.0130, 0.0133, 0.0089, 0.0028, -0.0022, -0.0047, -0.0046, -0.0029, -0.0009, 0.0007, 0.0014, 0.0013, 0.0009, 0.0003
};

float highpass_hw[MAX_FILTER_LEN] = {-0.0000,0.0037,0.0114,0.0164,-0.0000,-0.0538,-0.1378,-0.2174,0.7507,-0.2174,-0.1378,-0.0538,-0.0000,0.0164,0.0114,0.0037,-0.0000};
float bandpass_hw[MAX_FILTER_LEN] = {0.0000,0.0169,0.0384,0.0149,-0.0849,-0.1636,-0.0797,0.1316,0.2470,0.1316,-0.0797,-0.1636,-0.0849,0.0149,0.0384,0.0169,0.0000};

_task_id main_tid = 0;
_task_id lowpass_tid = 0;
_task_id highpass_tid = 0;
_task_id bandpass_tid = 0;

int samples[MAX_FILTER_LEN] = {0, };
int sample_start = 0;
int sample_end = 0;
int filter_flag = 0;

TASK_TEMPLATE_STRUCT  MQX_template_list[] = 
{ 
    {MAIN_TASK,     main_task,     700, 3, "main",     MQX_TIME_SLICE_TASK | MQX_AUTO_START_TASK, 0, 0},
    {LOWPASS_TASK,  lowpass_task,  700, 8, "lowpass",  MQX_TIME_SLICE_TASK, 0, 0},
    {HIGHPASS_TASK, highpass_task, 700, 8, "highpass", MQX_TIME_SLICE_TASK, 0, 0},
    {BANDPASS_TASK, bandpass_task, 700, 8, "bandpass", MQX_TIME_SLICE_TASK, 0, 0},
    {ISR_TASK,      isr_task,      700, 8, "isr",      MQX_TIME_SLICE_TASK | MQX_AUTO_START_TASK, 0, 0},
    {0,          0,          0,   0, 0,       0,                   0, 0}
};

void adc_init() {
    reg_ptr -> ADC.POWER &= ~(MCF_ADC_POWER_APD | MCF_ADC_POWER_PD0);
    reg_ptr -> ADC.POWER |= MCF_ADC_POWER_PD1;
    while(MCF_ADC_POWER & MCF_ADC_POWER_PSTS0);
    reg_ptr -> ADC.CTRL2 = 0x0008;
    reg_ptr -> ADC.ADSDIS = 0x00FE;
    reg_ptr -> ADC.CTRL1 = 0x2802;
}

void adcISR() {
    _int_disable();

    add_sample((uint_32)((reg_ptr -> ADC.ADRSLT[0]) >> 3));

    _int_enable();
}

void setupISR() {
    _int_disable();

    uartRegisterMask = MCF_UART_UIMR_FFULL_RXRDY;
	MCF_UART0_UMR2 |= MCF_UART_UMR_CM_NORMAL;

    // Initialize ADC interrupt
    MCF_INTC0_IMRH &= ~MCF_INTC_IMRL_MASKALL;
    MCF_INTC0_IMRH &= ~MCF_INTC_IMRH_INT_MASK49;
    MCF_INTC0_IMRL &= ~MCF_INTC_IMRL_MASKALL;
    MCF_INTC0_ICR49 |= MCF_INTC_ICR_IP(0x5) | MCF_INTC_ICR_IL(0x5);
    _int_install_isr(64+49, &adcISR, NULL);

    _int_enable();
}

void add_sample(int sample) {

    sample_start = (sample_start + 1) % MAX_FILTER_LEN;

    samples[sample_end] = sample;
    sample_end = (sample_end + 1) % MAX_FILTER_LEN;
}

void set_lowpass_hw(int_32 cutoff_freq, int_32 sample_freq) {
    int i = 0, midpoint = MAX_FILTER_LEN - 1;
    float trans_freq, hw_sum = 0;

    lowpass_tid = _task_get_id();

    trans_freq = (float)cutoff_freq / (float)sample_freq;

    for(i = 0; i < MAX_FILTER_LEN; i++) {
        float h;
        if(i == (int)(midpoint / 2)) {
            h = 2*trans_freq;
        } else {
            h = (my_sin(2 * MY_PI * trans_freq * ( i+ 1 - midpoint / 2)) / (MY_PI * (i + 1 - midpoint / 2)));
        }

        lowpass_hw[i] = h * (0.5 * (1 - my_cos(2 * MY_PI * i / (MAX_FILTER_LEN - 1))));

        if(lowpass_hw[i] >= 0) {
            hw_sum += lowpass_hw[i];
        } else {
            hw_sum += -lowpass_hw[i];
        }
    }

    for(i = 0; i < MAX_FILTER_LEN; i++) {
        lowpass_hw[i] = lowpass_hw[i] / hw_sum;
    }
}

void set_lowpass_hw_slow(int cutoff_freq, int sample_freq) {
	int i, midpoint = MAX_FILTER_LEN - 1;
	float trans_freq, hw_sum = 0.0;
	float h[MAX_FILTER_LEN] = {0.0, };
	float hw[MAX_FILTER_LEN] = {0.0, };
    float w[MAX_FILTER_LEN] = {0.0, 0.030154, 0.116978, 0.250001, 0.413177,
                               0.586825, 0.750001, 0.883021, 0.969847, 1.0,
                               0.969847, 0.883021, 0.750001, 0.586825, 0.413177,
                               0.250001, 0.116978, 0.030154};

	trans_freq = (float)cutoff_freq / (float)sample_freq;

	for(i = 0; i < MAX_FILTER_LEN; i++) {
		if(i == midpoint / 2) {
			h[i] = 2 * trans_freq;
		} else {
			h[i] = my_sin(2 * MY_PI * trans_freq * (i - midpoint / 2)) / (MY_PI * (i - midpoint / 2));
		}
	}

	/*for(i = 0; i < MAX_FILTER_LEN; i++) {
		w[i] = 0.5 * (1 - my_cos(2 * MY_PI * i / midpoint));
	}*/

	for(i = 0; i < MAX_FILTER_LEN; i++) {
		hw[i] = h[i] * w[i];
	}

	for(i = 0; i < MAX_FILTER_LEN; i++) {
		hw_sum += abs(hw[i]);
	}

	for(i = 0; i < MAX_FILTER_LEN; i++) {
		hw[i] = hw[i] / hw_sum;
        lowpass_hw[i] = hw[i];
	}
}

extern void main_task(uint_32 initial_data) {

    lowpass_tid = _task_create(0, LOWPASS_TASK, 0);
    if (lowpass_tid == MQX_NULL_TASK_ID){
        printf("Unable to create lowpass task!\n");
        _task_block();
    }

    adc_init();
    init_pins();
    init_qspi();

    while(1) {
        _task_block();
    }
}

extern void lowpass_task(uint_32 initial_data) {
    int i, j, sum;

    while(1) {
        sum = 0;        
        j = sample_start;
        for (i = 0; i < MAX_FILTER_LEN; i++){
            sum += samples[j] * lowpass_hw[i];
            j = (j + 1) % MAX_FILTER_LEN;
        }

        sum = (sum > 4095) ? (4095) : ((int)sum);
        MCF_QSPI_QAR = 0x0000;
        MCF_QSPI_QDR = ((int)sum | 0x3000);
        
        _task_block();
    }
}

extern void highpass_task(uint_32 initial_data) {
    while(1) {

    }
}

extern void bandpass_task(uint_32 initial_data) {
    while(1) {

    }
}

extern void isr_task(uint_32 initial_data) {
    TD_STRUCT_PTR lowpass_td_ptr;
    MQX_TICK_STRUCT ticks;
    uint_32 sample_read = 0;
    
    lowpass_td_ptr = _task_get_td(lowpass_tid);

    while(1) {
        _time_get_ticks(&ticks);
        
        // 100Hz sample frequency
        _time_add_usec_to_ticks(&ticks, 10000);
        _time_delay_until(&ticks);
        
        // wait for the ADC to finish sampling
        while(!((reg_ptr -> ADC.ADSTAT) & 0x0001));
        
        // only for debugging purposes. 
        sample_read = (uint_32)((reg_ptr -> ADC.ADRSLT[0]) >> 3);

        add_sample(sample_read);

        if (lowpass_td_ptr != NULL) {
			_task_ready(lowpass_td_ptr);
	    }

        //apply_filter(lp);
        //apply_filter(hp);
        //apply_filter(bp);
    }
}

void apply_filter(filter_type type) {
    int i, j, sum;

    sum = 0;
    
    j = sample_start;
    for (i = 0; i < MAX_FILTER_LEN; i++){
        
        switch(type) {
            case lp:
                sum += samples[j] * lowpass_hw[i];
                break;
            case hp:
                sum += samples[j] * highpass_hw[i];
                break;
            case bp:
                sum += samples[j] * bandpass_hw[i];
                break;
            default:
                break;
        }
        j = (j + 1) % MAX_FILTER_LEN;
    }
    sum = (sum > 4095) ? (4095) : (sum);
    MCF_QSPI_QAR = 0x0000;
    MCF_QSPI_QDR = ((int)sum | 0x3000);

}