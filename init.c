#include "MCF52259.h"
#include "init.h"
#include <mqx.h>
/*void init_adc(void) {
    reg_ptr -> ADC.POWER &= ~(MCF_ADC_POWER_APD | MCF_ADC_POWER_PD0);
    reg_ptr -> ADC.POWER |= MCF_ADC_POWER_PD1;
    while(MCF_ADC_POWER & MCF_ADC_POWER_PSTS0);
    reg_ptr -> ADC.CTRL2 = 0x0008;
    reg_ptr -> ADC.ADSDIS = 0x00FE;
    reg_ptr -> ADC.CTRL1 = 0x2802;
}*/

//Set up the interrupt service routine for the UART
//Set the UART to generate an interrupt whenever a character is received from the keyboard.


void init_qspi() {
    int i;
    // Configure initial Baud Rate to the lowest possible =833.33kbits per second
    // Configure number of bits per transfer to 16
    MCF_QSPI_QMR = MCF_QSPI_QMR_MSTR      |
                   MCF_QSPI_QMR_BITS(0x0) |
                   MCF_QSPI_QMR_BAUD(5);

    //Baud rate = system clock / 2 * baud value
    
    // Configure suitable delays for the external DAC
    MCF_QSPI_QDLYR = MCF_QSPI_QDLYR_QCD(0x4) |
                     MCF_QSPI_QDLYR_DTL(0x4);
	
	// Enable wraparound mode
	// Use the whole 15 memory location in the transmit RAM                    
    MCF_QSPI_QWR = MCF_QSPI_QWR_WREN |
                   MCF_QSPI_QWR_ENDQP(0x0) |
                   MCF_QSPI_QWR_NEWQP(0x0);

    MCF_QSPI_QAR=0x0020;
	MCF_QSPI_QWR |= MCF_QSPI_QWR_CSIV;
	MCF_QSPI_QDR = 0x4000;				// Command RAM starting Address
	for(i=1;i<16;i++)
		MCF_QSPI_QDR=0x4000;

    // Run QSPI
    MCF_QSPI_QDLYR |= MCF_QSPI_QDLYR_SPE;
}

void init_pins() {
    MCF_GPIO_PANPAR = MCF_GPIO_PANPAR_PANPAR0;
    MCF_GPIO_DDRQS = 0;
    MCF_GPIO_PQSPAR = MCF_GPIO_PQSPAR_PQSPAR6(0x1) |
                      MCF_GPIO_PQSPAR_PQSPAR5(0x1) |
                      MCF_GPIO_PQSPAR_PQSPAR4(0x1) |
                      MCF_GPIO_PQSPAR_PQSPAR3(0x1) |
                      MCF_GPIO_PQSPAR_PQSPAR2(0x1) |
                      MCF_GPIO_PQSPAR_PQSPAR1(0x1) |
                      MCF_GPIO_PQSPAR_PQSPAR0(0x1);
}

/*void init_gpio() {
    MCF_ADC_POWER = MCF_ADC_POWER_PD1;	 // power down ADC channel B

    while((MCF_ADC_POWER & MCF_ADC_POWER_PSTS0) != 0);	// waits for the ADC channel A power on

    MCF_ADC_CTRL1 |= MCF_ADC_CTRL1_STOP0;	 // stops the ADC by setting the STOP0 bit in the CTRL1 register to 1
    MCF_ADC_ADSDIS = MCF_ADC_ADSDIS_DS0;	 // disabling all but sample0 by setting ADSDIS to 11111110
    MCF_GPIO_PANPAR = MCF_GPIO_PANPAR_PANPAR0;	 // set the pin to its primary function for AN0

    MCF_ADC_CTRL1 = MCF_ADC_CTRL1_START0 |      // start scanning for input
                    MCF_ADC_CTRL1_EOSIE0 |      // End of Scan Interrupt Enable 
                    MCF_ADC_CTRL1_SMODE(0x2);   // set mode to loop sequential
}*/

/*void init_pit0(void) {
    MCF_PIT0_PCSR = 0x0;
    MCF_PIT0_PCSR = MCF_PIT_PCSR_OVW | // set overwrite bit so first PMR update is immediate
    MCF_PIT_PCSR_PRE(0x8) | // set timer frequency
    MCF_PIT_PCSR_PIE |	 // enable overflow interrupt
    MCF_PIT_PCSR_EN;	 // enable timer
    MCF_PIT0_PMR = 0xFFFF;	 // Count down from max value
}*/