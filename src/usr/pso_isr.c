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
#include "pso_pwm.h" /* Function generator - inc/dec funcs */
#include "pso_timing.h"
#include "fifo.h"
#include "pso_data.h"
#include "ulink.h"
#include "ulink_pso.h"
#include "ulink_types.h"
#include "pso_debug.h"
#include "pso_rpm.h"


extern uart_raw_data_t g_uart0_data;
uint8_t g_timer_a0_scan_flag = 0U;
volatile uint32_t g_timer_a3_scan_flag = 0U;    /* ADICIONAR volatile */

volatile uint32_t adc0_buffer[3];
volatile uint32_t adc1_buffer[3];
// volatile uint32_t delta = 0;                    /* ADICIONAR volatile */
uint32_t wt1cpp0_tav_buffer;
uint32_t g_pulse_diff;


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

}

/*
 * @brief Empty WTimer1A handler - should not be called
 * 
 * This handler should NOT be called because we disabled WTimer1 interrupts.
 * If this gets called, there's a configuration error.
 */
void WTimer1AIntHandler(void)
{
    /* Clear any spurious interrupt */
    WTIMER1_ICR_R = TIMER_ICR_CAMCINT;
    
    /* Optional: Set error flag if this gets called unexpectedly */
    // g_wtimer1_error_count++;
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


/**
 * @brief Timer3A ISR - Called every 100ms to calculate RPM
 * 
 * This function:
 * 1. Reads the current edge count from WTimer1A
 * 2. Calculates the difference from last reading (handles overflow)
 * 3. Converts pulse count to RPM based on motor configuration
 * 
 * RPM Calculation:
 *   pulses_per_100ms = current_count - last_count
 *   pulses_per_second = pulses_per_100ms * 10
 *   pulses_per_minute = pulses_per_second * 60
 *   RPM = pulses_per_minute / pulses_per_revolution
 *   
 *   Simplified: RPM = (pulses_per_100ms * 600) / BLADE_NUMBER
 */


void Timer3AIntHandler(void)
{
    static uint32_t last_rpm_count = 0;
    uint32_t current_count;
    uint32_t pulse_diff;

    
    /* 1. Clear the timer interrupt flag FIRST */
    TIMER3_ICR_R |= TIMER_ICR_TATOCINT;
    
    /* 2. Read current pulse count from Wide Timer 1A */
    current_count = WTIMER1_TAV_R;
    
    /* 3. Calculate pulse difference (handle 32-bit overflow) */
    if (current_count >= last_rpm_count)
    {
        /* Normal case - no overflow */
        pulse_diff = current_count - last_rpm_count;
    }
    else
    {
        /* Counter overflowed (wrapped from 0xFFFFFFFF to 0) */
        pulse_diff = (0xFFFFFFFF - last_rpm_count) + current_count + 1;
    }

    
     g_pulse_diff = pulse_diff;  /* Store for main loop RPM calculation */

    /* 5. Store current count for next calculation */
    last_rpm_count = current_count;
    
    /* 6. Set flag to indicate new RPM value is available */
    g_timer_a3_scan_flag ^= 0xFF;  /* Toggle flag */
    
    /* 7. Optional: Visual indicator (toggle LED) */
//    DEBUG_ADC_TOGGLE(); /* PD6 */
//    DEBUG_STATE_TOGGLE(); /* PD7 */
    
    /* 8. Call other functions that need 10Hz timing */
    increment();  /* PWM control function */
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


	/**************************************************************************
	 *  ADC0 - Acknowledge Sample Sequencer 1 Interrupt
	 *
	 *  ADC Interrupt Status and Clear (ADCISC, page 825)
	 *************************************************************************/
	ADC0_ISC_R = ADC_ISC_IN1;

	g_timer_a0_scan_flag = 1U;

	// DEBUG_STATE_TOGGLE(); /* PD7 - State indicator */

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

