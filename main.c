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

sampling frequency: 200 Hz

fixed point precision: 16 bits

* 0 Hz - 50 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 51 Hz - 100 Hz
  gain = 0
  desired attenuation = -20 dB
  actual attenuation = n/a

*/

#define FILTER_TAP_NUM 101

static int lowpass_hw[FILTER_TAP_NUM] = {
  1462,
  5631,
  1378,
  -357,
  -971,
  396,
  624,
  -382,
  -498,
  404,
  429,
  -433,
  -389,
  469,
  362,
  -509,
  -346,
  551,
  339,
  -595,
  -332,
  643,
  319,
  -707,
  -320,
  769,
  314,
  -849,
  -312,
  941,
  310,
  -1052,
  -309,
  1185,
  308,
  -1353,
  -307,
  1571,
  306,
  -1868,
  -306,
  2296,
  306,
  -2962,
  -304,
  4160,
  306,
  -6946,
  -305,
  20859,
  33074,
  20859,
  -305,
  -6946,
  306,
  4160,
  -304,
  -2962,
  306,
  2296,
  -306,
  -1868,
  306,
  1571,
  -307,
  -1353,
  308,
  1185,
  -309,
  -1052,
  310,
  941,
  -312,
  -849,
  314,
  769,
  -320,
  -707,
  319,
  643,
  -332,
  -595,
  339,
  551,
  -346,
  -509,
  362,
  469,
  -389,
  -433,
  429,
  404,
  -498,
  -382,
  624,
  396,
  -971,
  -357,
  1378,
  5631,
  1462
};


//float lowpass_hw[MAX_FILTER_LEN] = {0.0000,-0.0037,-0.0113,-0.0163,0.000,0.0535,0.1371,0.2163,0.2489,0.2163,0.1371,0.0535,0.0000,-0.0163,-0.0113,-0.0037,0.0000};
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
    //reg_ptr -> ADC.CTRL1 = 0x4802;

    _int_disable();

    /*if(counter == 0) {
        _time_get(&time_global1);
        counter = 1;
    } else if(counter == 500) {
        _time_get(&time_global2);
        printf("difference: %d\n", time_global2.MILLISECONDS - time_global1.MILLISECONDS);
        counter++;
    } else {
        counter++;
    }

    switch(time_int) {
        case 0:
            counter = 0;
            _time_get(&time_global1);
            time_int = 1;
            break;
        case 1:
            counter++;
            if(counter >= 10000000000) time_int = 2;
        case 2:
            _time_get(&time_global2);
            time_int = 3;
            printf("difference: %d\n", time_global2.MILLISECONDS - time_global1.MILLISECONDS);
            break;
        default:
            break;
    }*/

    //while(!((reg_ptr -> ADC.ADSTAT) & 0x0001));

    //MCF_QSPI_QAR = 0x0000;
    //MCF_QSPI_QDR = (((reg_ptr -> ADC.ADRSLT[0]) >> 3) | 0x3000);

    add_sample((uint_32)((reg_ptr -> ADC.ADRSLT[0]) >> 3));

    //filter_flag = 1;

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
    //if(sample_end == (sample_start - 1) % MAX_FILTER_LEN) {
    sample_start = (sample_start + 1) % MAX_FILTER_LEN;
    //}
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
    /*lowpass_tid = _task_create(0, LOWPASS_TASK, 0);
    if (lowpass_tid == MQX_NULL_TASK_ID){
        printf("Unable to create lowpass task!\n");
        _task_block();
    }

    highpass_tid = _task_create(0, HIGHPASS_TASK, 0);
    if (highpass_tid == MQX_NULL_TASK_ID){
        printf("Unable to create highpass task!\n");
        _task_block();
    }

    bandpass_tid = _task_create(0, BANDPASS_TASK, 0);
    if (bandpass_tid == MQX_NULL_TASK_ID){
        printf("Unable to create bandstop task!\n");
        _task_block();
    }*/

    adc_init();
    init_pins();
    init_qspi();
    //setupISR();

    //set_lowpass_hw_slow(300, 5000000);

    while(1) {
        _task_block();
    }
}

extern void lowpass_task(uint_32 initial_data) {
    int i, j;
    double sum;
    float dt = 1.0 / SAMPLING_FREQUENCY;
    float rc = 1.0 / (CUTOFF*2*pi);
    float alpha = dt / (rc + dt);
    
    float y[MAX_FILTER_LEN] = {0.0,};

    while(1) {
        //while(filter_flag == 0);

        //reg_ptr -> ADC.CTRL1 = 0x4802;

        //_int_disable();
        //y[0] = samples[0];
        //for (i = MAX_FILTER_LEN+1; i > 1; i--){
        //    y[i] = y[i-1] + alpha * (samples[i] - y[i-1]);
        //}

        sum = 0.0;
        
        i = sample_start;
        j = 0;
        //while(i != (sample_start - 1) % MAX_FILTER_LEN) {
        for (i = MAX_FILTER_LEN; i > 0; i--){
            sum += samples[i-1] * lowpass_hw[i-1];
            //i = (i + 1) % MAX_FILTER_LEN;
            //j++;
            //sum = y[i];// * lowpass_hw[i];
            //MCF_QSPI_QAR = 0x0000;
            //MCF_QSPI_QDR = ((int)sum | 0x3FFF);  
        }

        MCF_QSPI_QAR = 0x0000;
        MCF_QSPI_QDR = ((int)sum | 0x3000);

        //filter_flag = 0;
        
        //_int_enable();

        //reg_ptr -> ADC.CTRL1 = 0x2802;
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
    MQX_TICK_STRUCT ticks;
    uint_32 sample_read = 0;
    
    while(1) {
        _time_get_ticks(&ticks);
        _time_add_usec_to_ticks(&ticks, 5000);  //200Hz sample frequency
        _time_delay_until(&ticks);
        
        // wait for the ADC to finish sampling
        while(!((reg_ptr -> ADC.ADSTAT) & 0x0001));
        
        // only for debugging purposes. 
        sample_read = (uint_32)((reg_ptr -> ADC.ADRSLT[0]) >> 3);
        //sample_read = (sample_read < 3000) ? (0) : (4095);
        add_sample(sample_read);

        //MCF_QSPI_QAR = 0x0000;
        //MCF_QSPI_QDR = (((reg_ptr -> ADC.ADRSLT[0]) >> 3) | 0x3000);
        //MCF_QSPI_QDR = (samples[(sample_end - 1) % MAX_FILTER_LEN] | 0x3000);

        apply_filter(lp);
        //apply_filter(hp);
        //apply_filter(bp);
    }
}

void apply_filter(filter_type type) {
    int i=0, j=0;
    int sum;

    sum = 0;
    
    i = sample_start;
    j = 0;
    //while(i != (sample_start -1) % MAX_FILTER_LEN) {
    for (i = 0; i < MAX_FILTER_LEN; i++){
        j = (i + sample_start) % MAX_FILTER_LEN;
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
    }
    sum = (sum > 4095) ? (4095) : (sum);
    MCF_QSPI_QAR = 0x0000;
    MCF_QSPI_QDR = ((int)sum | 0x3000);
    //MCF_QSPI_QDR = (samples[(sample_end) % MAX_FILTER_LEN] | 0x3000);
}