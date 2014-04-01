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

uint_32 uartRegisterMask = 0;

VMCF5225_STRUCT_PTR reg_ptr = (VMCF5225_STRUCT_PTR)BSP_IPSBAR;

TD_STRUCT_PTR lowpass_td_ptr;

#define pi 3.1415962

float lowpass_hw[MAX_FILTER_LEN] = {
 0.001532,
 0.016102,
-0.017852,
-0.014600,
 0.058082,
-0.034230,
-0.104202,
 0.288619,
 0.624880,
 0.288619,
-0.104202,
-0.034230,
 0.058082,
-0.014600,
-0.017852,
 0.016102,
 0.001532
};

_task_id main_tid = 0;
_task_id lowpass_tid = 0;

int samples[MAX_FILTER_LEN] = {0, };
int sample_start = 0;
int sample_end = 0;

TASK_TEMPLATE_STRUCT  MQX_template_list[] = 
{ 
    {MAIN_TASK,     main_task,     700, 3, "main",     MQX_TIME_SLICE_TASK | MQX_AUTO_START_TASK, 0, 0},
    {LOWPASS_TASK,  lowpass_task,  700, 8, "lowpass",  MQX_TIME_SLICE_TASK, 0, 0},
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
    
    while(!((reg_ptr -> ADC.ADSTAT) & 0x0001));
    /*if(((reg_ptr -> ADC.ADSTAT) & 0x0001)) {
        MCF_QSPI_QAR = 0x0000;
        MCF_QSPI_QDR = (((reg_ptr -> ADC.ADRSLT[0]) >> 3) | 0x3000);
    }*/

    add_sample(((reg_ptr -> ADC.ADRSLT[0]) >> 3));

    _task_ready(lowpass_td_ptr);

    _int_enable();
}

void setupISR() {
    _int_disable();

    uartRegisterMask = MCF_UART_UIMR_FFULL_RXRDY;
	MCF_UART0_UMR2 |= MCF_UART_UMR_CM_NORMAL;
    
    // Initialize PIT0 interrupt
    /*MCF_INTC0_IMRH &= ~MCF_INTC_IMRL_MASKALL;
    MCF_INTC0_IMRH &= ~MCF_INTC_IMRH_INT_MASK55;
    MCF_INTC0_IMRL &= ~MCF_INTC_IMRL_MASKALL;
    MCF_INTC0_ICR55 |= MCF_INTC_ICR_IP(0x3) | MCF_INTC_ICR_IL(0x5);
    _int_install_isr(64+55, &adcISR, NULL);*/

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

extern void main_task(uint_32 initial_data) {

    lowpass_tid = _task_create(0, LOWPASS_TASK, 0);
    if (lowpass_tid == MQX_NULL_TASK_ID){
        printf("Unable to create lowpass task!\n");
        _task_block();
    }

    lowpass_td_ptr = _task_get_td(lowpass_tid);

    adc_init();
    init_pins();
    init_qspi();
    init_pit0();
    setupISR();

    while(1) {
        _task_block();
    }
}

extern void lowpass_task(uint_32 initial_data) {
    int i, j, sum;

    while(1) {
        //_int_disable();
        sum = 0;
        j = sample_start;
        for (i = 0; i < MAX_FILTER_LEN; i++){
            sum += samples[j] * lowpass_hw[i];
            j = (j + 1) % MAX_FILTER_LEN;
        }

        sum = (sum > 4095) ? (4095) : ((int)sum);
        MCF_QSPI_QAR = 0x0000;
        MCF_QSPI_QDR = ((int)sum | 0x3000);
        //MCF_QSPI_QDR = ((samples[(sample_end - 1) % MAX_FILTER_LEN]) | 0x3000);
        //_int_enable();
        _task_block();
    }
}