/*
 * isr.c
 *
 *  Created on: 10/03/2014
 *      Author: Rogerio
 */

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "hw_memmap.h"
#include "hw_types.h"
#include "sysctl.h"
#include "gpio.h"
#include "debug.h"
#include "pwm.h"
#include "pin_map.h"
#include "hw_gpio.h"
#include "rom.h"
#include "interrupt.h"
#include "timer.h"
#include "uart.h"
#include "hw_uart.h"
#include "pso_init.h"
#include "pso_uart.h"
// #include "diskio.h" /* FatFs timer - disk_timerproc () */
#include "pso_pwm.h" /* Function generator - inc/dec funcs */
#include "pso_timing.h"
#include "fifo.h"
#include "pso_data.h"
#include "ulink.h"
#include "ulink_pso.h"
#include "ulink_types.h"


extern uart_raw_data_t g_uart0_data; /*Defined in "pos_uart.c" */
uint8_t g_timer_a0_scan_flag = 0U;   /* Main: 500k/4 = 125 kHz scan rate */
uint8_t g_timer_a3_scan_flag = 0U;   /* RPM: 10 Hz scan rate */

volatile uint32_t adc0_buffer[3];      /* Ax - Thr - V_m */
volatile uint32_t adc1_buffer[3];      /* Ay -  Az - I_m */
uint32_t delta;
uint32_t wt1cpp0_tav_buffer;  /* RPM */

volatile uint32_t g_rpm_raw_count = 0;
volatile uint32_t g_rpm_ready_flag = 0U;
volatile uint32_t g_rpm_value = 0;  

/* Debug */
uint16_t discard_0, discard_1;

void UART0IntHandler(void)
{
    uint32_t ui32Status;

    ui32Status = UARTIntStatus(UART0_BASE, true); //get interrupt status
    UARTIntClear(UART0_BASE, ui32Status);         //clear the asserted interrupts

    while(UARTCharsAvail(UART0_BASE)) //loop while there are chars
    {
    	g_uart0_data.rx_buffer[g_uart0_data.rx_index++] = HWREG(UART0_BASE + UART_O_DR);
    }

    g_uart0_data.new_data = 1;
}


void Timer0AIntHandler(void)
{
	// Clear the timer interrupt
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	// Read the current state of the GPIO pin and
	// write back the opposite state
//	if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3))
//	{
//		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
//	}
//	else
//	{
//		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
//	}

	// g_timer_a0_scan_flag = 1U;
}


void WTimer1AIntHandler(void)
{
	volatile uint32_t ui32Timer, ui32Timer_prev, diff;

    /**************************************************************************
     * 1.11) Poll the CnMRISbit in the GPTMRIS register or wait for the
     *       interrupt to be generated (if enabled). In both cases, the status
     *       flags are cleared by writing a 1 to the CnMCINT bit of the GPTM
     *       Interrupt Clear (GPTMICR) register.
     *
     *  GPTM Interrupt Clear (GPTMICR, page 751)
     *************************************************************************/
    WTIMER1_ICR_R |= TIMER_ICR_CAMCINT;
}


void WTimer1BIntHandler(void)
{
	// Clear the timer interrupt
	TimerIntClear(WTIMER1_BASE, TIMER_CAPB_EVENT);
}

void WTimer5AIntHandler(void)
{
	// Clear the timer interrupt
	TimerIntClear(WTIMER5_BASE, TIMER_CAPA_EVENT);
}


void WTimer5BIntHandler(void)
{
	// Clear the timer interrupt
	TimerIntClear(WTIMER5_BASE, TIMER_CAPB_EVENT);

}

void Timer3AIntHandler(void)
{
	/**************************************************************************
	 * 16/32-Bit General-Purpose Timer 3 is configured as a Count-Up 32-bit
	 * timer with the upper value set to 0x02625A00 (TIMER3_TAILR_R) which
	 * corresponds to 40.000.000 in decimal, that is the System Clock frequency.
	 * And running through this value, the ISR is called every 1 second.
	 * So this timer is used to generate interrupts at rate of 1 Hz in order to
	 * get the value of the counted positive input edges on the pin PC6
	 * configured as Input Edge-Count timer over the WT1CCP0.
	 *
	 * Timer3A current set to 100 ms period interrupt.
	 *
	 *************************************************************************/
    static uint32_t tav_1 = 0U; /* Previous edge count */
    static uint32_t last_rpm_count = 0;
    uint32_t current_count;


    /**************************************************************************
     * Clear the timer interrupt GPTM Timer A Time-Out Raw Interrupt
     *  GPTM Interrupt Clear (GPTMICR, page 751)
     *************************************************************************/
	TIMER3_ICR_R |= TIMER_ICR_TATOCINT;

	/**************************************************************************
	 * Reads the current value of the positive input edges counted on the pin
	 * PC6. This value is cumulative and shall be subtracted from the previous
	 * value read.
	 *************************************************************************/
	wt1cpp0_tav_buffer = WTIMER1_TAV_R;

	delta = wt1cpp0_tav_buffer - tav_1;
	tav_1 = wt1cpp0_tav_buffer;

    /* Calcula RPM */
    current_count = WTIMER1_TAV_R;
    g_rpm_value = (current_count - last_rpm_count) * 60 * 10;
    last_rpm_count = current_count;

    /* Incrementa contador de ms */
    g_system_ms_counter++;

	g_timer_a3_scan_flag ^= 0xFF;



    // disk_timerproc (); /* FatFs timer */
    increment ();      /* Used in PWM */

//	GPIO_PORTF_DATA_R ^= GPIO_PIN_2;
}

void ADC0SS1IntHandler(void)
{

	uint8_t k = 0U;

	//for (delay = 0U; delay < 50; delay++); /* Delay */
    /**************************************************************************
     * Retrieve data from sample sequence 1 FIFO. The data, if HW averaged is
     * enabled, are available in the FIFO.
     *
     *  ADC Sample Sequence Result FIFO 1 (ADCSSFIFO1, page 857)
     *************************************************************************/
	for (k = 0U; k < 3U; k++)
    {
        adc0_buffer[k] = ADC0_SSFIFO1_R;    /* k = 0: PD1_AIN6_Ax      */
											/* k = 1: PD0_AIN7_StrGage */
											/* k = 2: PE1_AIN2_V_motor */

        adc1_buffer[k] = ADC1_SSFIFO1_R;    /* k = 0: PD2_AIN5_Ay      */
											/* k = 1: PD3_AIN4_Az      */
											/* k = 2: PE2_AIN1_I_motor */
    }
	/* Codigo anterior */
//	adc0_buffer[2] = ADC0_SSFIFO1_R;	/* PE1_AIN2_V_motor */
//	adc0_buffer[0] = ADC0_SSFIFO1_R;    /* PD1_AIN6_Ax      */
//	adc0_buffer[1] = ADC0_SSFIFO1_R;    /* PD0_AIN7_StrGage */
//	discard_0        = ADC0_SSFIFO1_R;    /* Disrcard */

//	adc1_buffer[2] = ADC1_SSFIFO1_R;    /* PE2_AIN1_I_motor */
//	adc1_buffer[0] = ADC1_SSFIFO1_R;    /* PD2_AIN5_Ay      */
//	adc1_buffer[1] = ADC1_SSFIFO1_R;    /* PD3_AIN4_Az      */
//	discard_1        = ADC1_SSFIFO1_R;    /* Disrcard */

	/**************************************************************************
	 *  ADC0 - Acknowledge Sample Sequencer 1 Interrupt
	 *
	 *  ADC Interrupt Status and Clear (ADCISC, page 825)
	 *************************************************************************/
	ADC0_ISC_R = ADC_ISC_IN1;

	g_timer_a0_scan_flag = 1U;

//	GPIO_PORTF_DATA_R ^= GPIO_PIN_2;    /* Blue LED on PF2 */
}

void ADC1SS1IntHandler(void)
{
	/**************************************************************************
	 *  ADC1 - Acknowledge Sample Sequencer 1 Interrupt
	 *
	 *  ADC Interrupt Status and Clear (ADCISC, page 825)
	 *************************************************************************/
	ADC1_ISC_R = ADC_ISC_IN1;

    /**************************************************************************
     * Retrieve data from sample sequence 1 FIFO. The data, if HW averaged is
     * enabled, are available in the FIFO.
     *
     *  ADC Sample Sequence Result FIFO 1 (ADCSSFIFO1, page 857)
     *************************************************************************/
	adc1_buffer[0] = ADC1_SSFIFO1_R;
	adc1_buffer[1] = ADC1_SSFIFO1_R;
	adc1_buffer[2] = ADC1_SSFIFO1_R;

}
