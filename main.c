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


TIME_STRUCT time_global1;
TIME_STRUCT time_global2;
uint_32 time_int = 0;
int counter = 0;

uint_32 uartRegisterMask = 0;

// Pointer to struct for ADC functions
VMCF5225_STRUCT_PTR reg_ptr = (VMCF5225_STRUCT_PTR)BSP_IPSBAR;

#define pi 3.1415962
#define CUTOFF 100
#define SAMPLING_FREQUENCY 2075


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

char enabled_filter = 10;

TASK_TEMPLATE_STRUCT  MQX_template_list[] = 
{ 
    {MAIN_TASK,     main_task,     700, 3, "main",     MQX_TIME_SLICE_TASK | MQX_AUTO_START_TASK, 0, 0},
    {LOWPASS_TASK,  lowpass_task,  700, 8, "lowpass",  MQX_TIME_SLICE_TASK, 0, 0},
    {HIGHPASS_TASK, highpass_task, 700, 8, "highpass", MQX_TIME_SLICE_TASK, 0, 0},
    {BANDPASS_TASK, bandpass_task, 700, 8, "bandpass", MQX_TIME_SLICE_TASK, 0, 0},
    {ISR_TASK,      isr_task,      700, 8, "isr",      MQX_TIME_SLICE_TASK | MQX_AUTO_START_TASK, 0, 0},
    {0,          0,          0,   0, 0,       0,                   0, 0}
};

// Adds an ADC sample to the circular buffers
void add_sample(int sample) {

    sample_start = (sample_start + 1) % MAX_FILTER_LEN;
    samples[sample_end] = sample;
    sample_end = (sample_end + 1) % MAX_FILTER_LEN;
}

void main_task(uint_32 initial_data) {

    lowpass_tid = _task_create(0, LOWPASS_TASK, 0);
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
        printf("Unable to create bandpass task!\n");
        _task_block();
    }


    init_adc();
    init_pins();
    init_qspi();

    while(1) {
        // hard coded for now, 
        // TODO: take the filter type from command line
        enabled_filter = lowpass;
        _task_block();
    }
}

void lowpass_task(uint_32 initial_data) {
    int sum=0;

    while(1) {

        // run the filter
        sum = apply_filter(lowpass);
        
        // send the sum to be outputted
        output_signal(sum, channel_a, lowpass);

        // block until the next sample is ready
        _task_block();
    }
}

void highpass_task(uint_32 initial_data) {
    int sum=0;

    while(1) {

        // run the filter
        sum = apply_filter(highpass);
        
        // send the sum to be outputted
        output_signal(sum, channel_a, highpass);

        // block until the next sample is ready
        _task_block();
    }
}

void bandpass_task(uint_32 initial_data) {
    int sum=0;

    while(1) {

        // run the filter
        sum = apply_filter(bandpass);
        
        // send the sum to be outputted
        output_signal(sum, channel_a, bandpass);

        // block until the next sample is ready
        _task_block();
    }
}

void isr_task(uint_32 initial_data) {
    uint_32         sample_read = 0;
    TD_STRUCT_PTR   lowpass_td_ptr;
    MQX_TICK_STRUCT ticks;
    
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

        // Add sample to circular buffers
        add_sample(sample_read);

        // wake the lowpass task to process samples
        if (lowpass_td_ptr != NULL) {
			      _task_ready(lowpass_td_ptr);
	      }
    }
}

int apply_filter(filter_type_t filter_type) {
    int i   = 0;
    int j   = sample_start; 
    int sum = 0;
    
    for (i = 0; i < MAX_FILTER_LEN; i++){
        
        switch(filter_type) {
            case lowpass:
                sum += samples[j] * lowpass_hw[i];
                break;
            case highpass:
                sum += samples[j] * highpass_hw[i];
                break;
            case bandpass:
                sum += samples[j] * bandpass_hw[i];
                break;
            default:
                break;
        }
        j = (j + 1) % MAX_FILTER_LEN;
    }

    sum = (sum > 4095) ? (4095) : (sum);

    return sum;
}

void output_signal (int sum, channel_sel_t channel, filter_type_t filter_type){

  // all tasks fight for output, only enabled type can output
  if (enabled_filter == filter_type){
    // output the sum to the DAC
    MCF_QSPI_QAR = 0x0000;
    MCF_QSPI_QDR = ((int)sum | 0x3000);
  }
}