/*
 * myinit.c
 *
 *  Created on: 07/03/2014
 *      Author: Rogerio
 */

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "hw_memmap.h"
#include "hw_types.h"
#include "hw_sysctl.h"
#include "hw_uart.h"
#include "sysctl.h"
#include "gpio.h"
#include "debug.h"
#include "pwm.h"
#include "pin_map.h"
#include "hw_gpio.h"
#include "rom.h"
#include "interrupt.h"
#include "timer.h"
#include "pso_init.h"
#include "uart.h"

/******************************************************************************
 * All the initilization procedure should follow strictly the steps suggested
 * in the datasheet.
 *
 *****************************************************************************/

/******************************************************************************
 * Interrupt Service Routine (ISR) Configuration
 * ----------------------------------------------------------------------------
 * UART 0 -> Port A -> Pins PA0(Tx) & PA1(Rx)
 * 9600-8N1
 *
 ******************************************************************************/
void myISR_Config()
{

}



/******************************************************************************
 * UART-0 Configuration VCP (Virtual Com Port)
 * ----------------------------------------------------------------------------
 * UART 0 -> Port A -> Pins PA0(Tx) & PA1(Rx)
 * 9600-8N1
 *
 ******************************************************************************/
void PSO_UART0Config()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);	/* Enable UART Module 0 */
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);	/* Enable GPIO Port A for UART-0 use */

	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
	                    UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
	                    UART_CONFIG_PAR_NONE);

	/* UART-0 interrupt configuration */
	UARTDisable(UART0_BASE);
	UARTFIFOEnable(UART0_BASE);

	IntEnable(INT_UART0);

	UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX1_8, UART_FIFO_RX7_8);

	UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);  // UART_INT_RT : Timeout interrupt
	UARTEnable(UART0_BASE);
	UARTFIFOEnable(UART0_BASE);
}


/******************************************************************************
 * UART-2 Configuration
 * ----------------------------------------------------------------------------
 * UART 2 -> Port B -> Pins PD6 & PD7
 * SysClk = 16 MHz
 * 9600-8N1
 *
 ******************************************************************************/
void myUART2Config_Init()
{
	HWREG(SYSCTL_RCGCUART) = SYSCTL_RCGCUART_R2;        /* UART 2 activated  */
	HWREG(SYSCTL_RCGCGPIO) = SYSCTL_RCGCGPIO_R3;        /* Port D activated  */
	HWREG(UART2_BASE + UART_O_CTL) &= ~UART_CTL_UARTEN; /* Disable UART      */
	/*-------------------------------------------------------------------------
	 * Baud-Rate Generation
	 * BRD  =  BRDI  +  BRDF  =  UARTSysClk  /  (ClkDiv  *  Baud  Rate)
	 *         BRDI -> Integer number
	 *         BRDF -> Fractional number = round(BRDF * 64 + 0.5)
	 *
	 * Baud-Rate = 9600 bps
	 * BRDI + BRDF = 16000000/(5 * 9600) = 3333.3333
	 *     BRDI = 3333 = 0x0D05
	 *     BRDF = (0.3333 * 64 + 0.5) = 26
	 *
	 * --------------------------------------------------------------------- */
	HWREG(UART2_BASE + UART_O_IBRD) = 0x00000D05;
    HWREG(UART2_BASE + UART_O_FBRD) = 0x0000001A;

    /* UART Line Control */
    HWREG(UART2_BASE + UART_O_LCRH) = UART_CONFIG_WLEN_8|UART_CONFIG_PAR_NONE|UART_CONFIG_STOP_ONE;

    HWREG(UART2_BASE + UART_O_CTL) = UART_CTL_UARTEN; /* Enable UART */

    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK)   = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR)    |= (GPIO_PIN_6 | GPIO_PIN_7);
    HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) |= (GPIO_PIN_6 | GPIO_PIN_7);
    HWREG(GPIO_PORTD_BASE + GPIO_O_DR2R)  |= GPIO_STRENGTH_2MA;
    HWREG(GPIO_PORTD_BASE + GPIO_O_DEN)   |= (GPIO_PIN_6 | GPIO_PIN_7);
    HWREG(GPIO_PORTD_BASE + GPIO_O_PCTL)  |= 0x11000000;


}



void PSO_PeripheralEnable()
{
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);		// Enable PWM Module 0
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);		// Enable PWM Module 1
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1);	// Enable Wide Timer 1
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER5);	// Enable Wide Timer 5
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);	// Enable GPIO Port C [PWM:PC4, PC5],[Timer: PC6, PC7]
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);	// Enable GPIO Port D [PWM:PD0, PD1],[Timer: PD6, PD7]
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);	// Enable GPIO Port F

}


void PSO_GPIOConfig()
{

	/*** Port F Configuration ***/
//	HWREG(SYSCTL_RCGCUART) = SYSCTL_RCGCGPIO_R5;        /* GPIO Port F Run Mode Clock  */
	                                                    /* Gating Control              */
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x1F;          /* Allow changes to PF4-0 */

	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGC2_GPIOF;           /* GPIO Port F Run Mode Clock  */
														/* Gating Control              */
	HWREG(GPIO_PORTF_BASE + GPIO_O_DIR) |= 0x0E;        /* xxx0-1110 -> 0x0E */
	HWREG(GPIO_PORTF_BASE + GPIO_O_AFSEL) = 0x00;       /* No alternate function */
	HWREG(GPIO_PORTF_BASE + GPIO_O_DR2R) = GPIO_STRENGTH_2MA;
	HWREG(GPIO_PORTF_BASE + GPIO_O_PUR) |= 0x11;         /* Enable pull up resistors for PF0 & PF4 */
	HWREG(GPIO_PORTF_BASE + GPIO_O_DEN) |= 0x1F;         /* Enable digital pins PF4-0 */
	HWREG(GPIO_PORTF_BASE + GPIO_O_AMSEL) = 0x00;       /* Disable analog function */
	HWREG(GPIO_PORTF_BASE + GPIO_O_PCTL) = 0x00000000;  /* GPIO clear bit PCTL */
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;


}


void myPWM_Init()
{
	GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_4);	// Set Port PC4 as PWM
	GPIOPinConfigure(GPIO_PC4_M0PWM6);

	GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_5);	// Set Port PC5 as PWM
	GPIOPinConfigure(GPIO_PC5_M0PWM7);

	GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);	// Set Port PD0 as PWM
	GPIOPinConfigure(GPIO_PD0_M1PWM0);

	GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_1);	// Set Port PD1 as PWM
	GPIOPinConfigure(GPIO_PD1_M1PWM1);
}


void PSO_Timers()
{
	uint32_t ui32Period;
	uint16_t freq = 5000;    /* Desired frequency, [Hz] */
	                        /* 500 Hz is the maximum allowable for 20 bytes
	                         * over serial @ 115kbps                           */

	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

	ui32Period = (SysCtlClockGet() / freq);
	TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period -1);

    TimerControlTrigger(TIMER0_BASE, TIMER_A, true); /* Trigger ADC */

	IntEnable(INT_TIMER0A);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	IntMasterEnable();

	TimerEnable(TIMER0_BASE, TIMER_A);

}


/******************************************************************************
 * ADC Configuration
 * ----------------------------------------------------------------------------
 * ADC XXXXXXXXXXXX
 * XXXXXXXXXXXXX
 *
 ******************************************************************************/
void PSO_ADCConfig()
{
	uint16_t delay;

	/*** 13.4.1 Module Initialization ***/

	/**************************************************************************
	 * 1.1) Enable the ADC clock using the  RCGADC register (page 350)
	 *************************************************************************/
	SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;	/* Enable ADC Module 0   */
	SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R1;  /* Enable ADC Module 1   */

	for (delay = 0U; delay < 100; delay++); /* Delay */

	/**************************************************************************
	 * 1.2) Enable the clock to the appropriate GPIO modules via the RCGCGPIO
	 *    register (page 338). To find out which GPIO port to enable, refer to
	 *    "Signal Description" on page 801.
	 *    ---------------------------------------------------------------------
	 *    Port D:
	 *             PD0 -> Strain gage
	 *             PD1 -> Ax
	 *             PD2 -> Ay
	 *             PD3 -> Az
	 *    Port E:
	 *             PE1 -> V_motor
	 *             PE2 -> I_motor
	 *************************************************************************/
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;    /* Enable Port D */
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;    /* Enable Port E */

	/**************************************************************************
	 *  1.3) Set the GPIO 'AFSEL' bits for the ADC input pins (page 668). To
	 *     determine which GPIOs to configure, see Table 23-4 on page 1337.
	 *     -------------------------------------------------------------------
	 *     Inputs:
	 *             PD0
	 *             PD1
	 *             PD2
	 *             PD3
	 *             PE1
	 *             PE2
	 *             PE0 (Discard)
	 *             PE5 (Discard)
	 *************************************************************************/
	GPIO_PORTD_DIR_R &=  ~(GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
	GPIO_PORTD_AFSEL_R |= (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
	GPIO_PORTE_DIR_R &=  ~(GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_5);
	GPIO_PORTE_AFSEL_R |= (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_5);;
	//GPIO_PORTD_PCTL_R


	/**************************************************************************
	 * 1.4) Configure the AINx signals to be analog inputs by clearing the
	 *    corresponding DEN bit in the GPIO Digital Enable (GPIODEN) register
	 *    (page 682).
	 *************************************************************************/
	GPIO_PORTD_DEN_R &= ~(GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
	GPIO_PORTE_DEN_R &= ~(GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_5);

	/**************************************************************************
	 * 1.5) Disable the analog isolation circuit for all ADC input pins that
	 *      are to be used by writing a 1 to
	 *************************************************************************/
	GPIO_PORTD_AMSEL_R |= (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
	GPIO_PORTE_AMSEL_R |= (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_5);

    /**************************************************************************
     * 1.6) Configure sequence priority: order (highest to lowest)= 3, 2, 0, 1
     *      ADC Sample Sequencer Priority register
     *      SS3 = 0x3 (lowest)
     *      SS2 = 0x2 (mid-low)
     *      SS1 = 0x0 (highest)
     *      SS0 = 0x1 (mid-high)
     *************************************************************************/
	ADC0_SSPRI_R = 0x3201;
	ADC1_SSPRI_R = 0x3201;

    /**************************************************************************
     * 1.7) Configure the ADC to sample at 250 ksps
     *
     * ADC Peripheral Configuration (ADCPC, page 888)
     *************************************************************************/
    ADC0_PC_R = ADC_PP_MSR_250K;
    ADC1_PC_R = ADC_PP_MSR_250K;

    /*** 13.4.2 Sample Sequencer Configuration ***/

	/**************************************************************************
	 * 2.1) Ensure that the sample sequencer is disabled by clearing the
	 *      corresponding "ASENn" bit in the "ADCACTSS" register. Programming
	 *      of the sample sequencers is allowed without having them enabled.
	 *************************************************************************/
	ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN1;   /* ADC0 SS1 Disable */
	ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN1;   /* ADC1 SS1 Disable */

	/**************************************************************************
     * 2.2) Configure the trigger event for the sample sequencer in the
     *      "ADCEMUX" register.
     *************************************************************************/
	ADC0_EMUX_R |= ADC_EMUX_EM1_TIMER;
	ADC1_EMUX_R |= ADC_EMUX_EM1_TIMER;

	/**************************************************************************
	 * 2.3) Coincident sampling of different signals. The sample sequence steps
	 *      run coincidentally in both converters (see page 802).
	 *
	 * ADC Sample Phase Control (ADCSPC, page 840)
	 *************************************************************************/
	ADC0_SPC_R = ADC_SPC_PHASE_0;
	ADC1_SPC_R = ADC_SPC_PHASE_0;

    /**************************************************************************
     * 2.4) Configure ADC Sample Sequence Input Multiplexer Select 1 register
     *
     *      ADC0:         Ax    Thrust    V_m
     *              -------|-------|-------|-------> t(s)
     *
     *      ADC1:         Ay      Az      I_m
     *              -------|-------|-------|-------> t(s)
     *
     *
     *      ADC0:
     *            - Step 0: AIN6 - PD1 - Accel-x
	 *            - Step 1: AIN7 - PD0 - Thrust
	 *            - Step 2: AIN2 - PE1 - V_motor
	 *            - Step 3: AIN3 - PE0 - DISCARD
	 *
	 *      ADC1:
     *            - Step 0: AIN5 - PD2 - Accel-y
	 *            - Step 1: AIN4 - PD3 - Accel-z
	 *            - Step 2: AIN1 - PE2 - I_motor
	 *            - Step 3: AIN8 - PE5 - DISCARD
     *************************************************************************/
	ADC0_SSMUX1_R = ((0x06 << ADC_SSMUX1_MUX0_S) |
			         (0x07 << ADC_SSMUX1_MUX1_S) |
			         (0x02 << ADC_SSMUX1_MUX2_S));

	ADC1_SSMUX1_R = ((0x05 << ADC_SSMUX1_MUX0_S) |
                     (0x04 << ADC_SSMUX1_MUX1_S) |
                     (0x01 << ADC_SSMUX1_MUX2_S));

    /**************************************************************************
     * 2.5) Configure Sample Sequence 1 Control
     *
     *      ADC0:
     *            - Step 0: Single-ended, No temp sensor, No interrupt
     *            - Step 1: Single-ended, No temp sensor, No interrupt
     *            - Step 2: Single-ended, No temp sensor, Interrupt, End of sequence
     *      ADC1:
     *            - Step 0: Single-ended, No temp sensor, No interrupt
     *            - Step 1: Single-ended, No temp sensor, No interrupt
     *            - Step 2: Single-ended, No temp sensor, End of sequence
     *
     * ADC Sample Sequence Control 1 register (ADCn_SSCTL1_R)
     *************************************************************************/
    ADC0_SSCTL1_R = (ADC_SSCTL1_END2 | ADC_SSCTL1_IE2);

    ADC1_SSCTL1_R = ADC_SSCTL1_END2;              /* Interrupt only in ADC0 */

    /**************************************************************************
     * 2.6) Enable Hardware Averaging Circuit for 4x./
     *
     *
     *  ADC Sample Averaging Control (ADCSAC, page 844)
     *************************************************************************/
    ADC0_SAC_R |= ADC_SAC_AVG_4X;                /* 4x hardware oversampling */
    ADC1_SAC_R |= ADC_SAC_AVG_4X;                /* 4x hardware oversampling */

    /**************************************************************************
     *  2.7) Enable the interrupt for sample sequence 1 (only for ADC0 here)
     *
     *  ADC Interrupt Mask (ADCIM - page 822)
     *************************************************************************/
    ADC0_IM_R = ADC_IM_MASK1;

    /**************************************************************************
     * 2.8) Enable Sample Sequencer 1
     *
     *
     * ADC Active Sample Sequencer (ADCACTSS, page 818)
     *************************************************************************/
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN1;   /* ADC1 SS0 Enable */
    ADC1_ACTSS_R |= ADC_ACTSS_ASEN1;   /* ADC1 SS1 Enable */

    /**************************************************************************
     * 2.9) Enable Master Interrupt for ADC0 Sequencer 1.
     *         ADC0SS1 = 31 -> bit 15 in EN0
     *         ADC1SS1 = 65 -> bit 02 in EN2
     *
     *  Interrupt 0-31 Set Enable (EN0, page 140)
     *  Interrupt 64-95 Set Enable (EN2, page 140),
     *************************************************************************/
    NVIC_EN0_R |= 0x8000;
    NVIC_EN2_R |= 0x0002;
}


/*******************************************************************************
 * FILENAME: pso_rpm_config() - FIXED VERSION
 * 
 * CRITICAL FIXES:
 * 1. Changed WTIMER1 from CAPTURE mode to proper EDGE-COUNT configuration
 * 2. Removed unnecessary match register configuration
 * 3. Simplified interrupt configuration
 * 
 * COPY THIS FUNCTION to replace the existing pso_rpm_config() in pso_init.c
 ******************************************************************************/

void pso_rpm_config(void)
{
    /**************************************************************************
     * RPM MEASUREMENT SYSTEM
     * -------------------------------------------------------------------------
     * Uses TWO timers:
     * 1. Timer3A (16/32-bit) - Periodic interrupt every 100ms to calculate RPM
     * 2. WTimer1A (32/64-bit) - Edge counter to count pulses on PC6
     * 
     * Connection: PC6 = WT1CCP0 = RPM sensor input
     *************************************************************************/

    /* ===================================================================== */
    /* PART 1: Configure Timer3A - Periodic 100ms interrupt                 */
    /* ===================================================================== */
    
    /**************************************************************************
     * 1.0) Enable Timer3 clock
     *************************************************************************/
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R3;
    while(!(SYSCTL_PRTIMER_R & SYSCTL_PRTIMER_R3)) {}  /* Wait until ready */

    /**************************************************************************
     * 1.1) Disable timer before configuration
     *************************************************************************/
    TIMER3_CTL_R &= ~TIMER_CTL_TAEN;

    /**************************************************************************
     * 1.2) Configure as 32-bit timer
     *************************************************************************/
    TIMER3_CFG_R = TIMER_CFG_32_BIT_TIMER;  /* 0x00000000 */

    /**************************************************************************
     * 1.3) Configure as Periodic mode, Count UP
     *************************************************************************/
    TIMER3_TAMR_R = TIMER_TAMR_TAMR_PERIOD | TIMER_TAMR_TACDIR;

    /**************************************************************************
     * 1.4) Load interval for 100ms (0.1 second)
     *      System Clock = 40 MHz
     *      Count = 40,000,000 / 10 = 4,000,000 = 0x003D0900
     *************************************************************************/
    TIMER3_TAILR_R = 4000000 - 1;  /* 0x003D08FF */

    /**************************************************************************
     * 1.5) Enable timeout interrupt
     *************************************************************************/
    TIMER3_IMR_R |= TIMER_IMR_TATOIM;

    /**************************************************************************
     * 1.6) Enable Timer3A
     *************************************************************************/
    TIMER3_CTL_R |= TIMER_CTL_TAEN;

    /**************************************************************************
     * 1.7) Enable Timer3A interrupt in NVIC
     *      Timer3A = IRQ 35 = bit 3 in EN1
     *************************************************************************/
    NVIC_EN1_R |= (1 << 3);  /* Enable interrupt 35 */


    /* ===================================================================== */
    /* PART 2: Configure WTimer1A - Edge Counter on PC6                     */
    /* ===================================================================== */

    /**************************************************************************
     * 2.0) Configure GPIO PC6 for WT1CCP0 input
     *************************************************************************/
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2;  /* Enable Port C clock */
    while(!(SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R2)) {}  /* Wait until ready */
    
    GPIO_PORTC_DIR_R &= ~GPIO_PIN_6;         /* PC6 as input */
    GPIO_PORTC_AFSEL_R |= GPIO_PIN_6;        /* Enable alternate function */
    GPIO_PORTC_DEN_R |= GPIO_PIN_6;          /* Enable digital */
    GPIO_PORTC_PUR_R |= GPIO_PIN_6;          /* Enable pull-up resistor */
    GPIO_PORTC_AMSEL_R &= ~GPIO_PIN_6;       /* Disable analog */
    
    /* Configure PCTL for WT1CCP0 (encoding 7 for PC6) */
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC6_WT1CCP0;       /* Port Mux Control 6 */

    /**************************************************************************
     * 2.1) Enable Wide Timer1 clock
     *************************************************************************/
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R1;
    while(!(SYSCTL_PRWTIMER_R & SYSCTL_PRWTIMER_R1)) {}  /* Wait until ready */

    /**************************************************************************
     * 2.2) Disable Wide Timer1A before configuration
     *************************************************************************/
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;

    /**************************************************************************
     * 2.3) Configure as 32-bit wide timer
     *      IMPORTANT: For Wide Timers, 0x0 = 32-bit, 0x4 = 64-bit
     *************************************************************************/
    WTIMER1_CFG_R = 0x00000004;  /* 32-bit wide timer */

    /**************************************************************************
     * 2.4) Configure EDGE-COUNT mode (NOT capture mode!)
     *      
     *      CRITICAL: This is different from CAPTURE mode!
     *      - TAMR bits [1:0] = 11b (0x3) = Capture mode
     *      - TACMR bit = 0 means Edge-Count mode (counts edges)
     *      - TACMR bit = 1 means Edge-Time mode (captures time)
     * 
     *      For edge counting, we need:
     *      - Capture mode (TAMR = 0x3)
     *      - Edge-count (TACMR = 0)
     *      - Count up (TACDIR = 1)
     *************************************************************************/
    WTIMER1_TAMR_R = (TIMER_TAMR_TAMR_CAP |      /* Capture mode (0x3) */
                      TIMER_TAMR_TACDIR);         /* Count up */
    /* TACMR = 0 by default (Edge-Count mode) */

    /**************************************************************************
     * 2.5) Configure to count on POSITIVE edges
     *      Change to TIMER_CTL_TAEVENT_NEG if your sensor uses negative edges
     *************************************************************************/
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEVENT_M;       /* Clear event bits */
    WTIMER1_CTL_R |= TIMER_CTL_TAEVENT_POS;      /* Count on rising edges */

    /**************************************************************************
     * 2.6) No prescaler - count every edge
     *************************************************************************/
    WTIMER1_TAPR_R = 0;

    /**************************************************************************
     * 2.7) Load maximum count value
     *      Timer will count from 0 to 0xFFFFFFFF and rollover
     *************************************************************************/
    WTIMER1_TAILR_R = 0xFFFFFFFF;

    /**************************************************************************
     * 2.8) Clear any pending interrupts (though we won't use them)
     *************************************************************************/
    WTIMER1_ICR_R = TIMER_ICR_CAMCINT;

    /**************************************************************************
     * 2.9) DO NOT enable interrupts for WTimer1
     *      We only need Timer3 interrupt to read the counter periodically
     *************************************************************************/
    WTIMER1_IMR_R = 0;  /* No interrupts from WTimer1 */

    /**************************************************************************
     * 2.10) Enable Wide Timer1A to start counting
     *************************************************************************/
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;

    /**************************************************************************
     * Configuration complete!
     * 
     * How it works:
     * 1. WTimer1A counts edges on PC6 continuously
     * 2. Every 100ms, Timer3A generates an interrupt
     * 3. In Timer3A ISR, read WTIMER1_TAV_R to get pulse count
     * 4. Calculate RPM from pulse difference
     *************************************************************************/
}


void pso_pwm_config()
{
	/**************************************************************************
	 * PWM is implemented to drive the brushless motor through the Electronic
	 * Speed Controller (ESC). It's send to the ESC... WTimer-1B
	 *
	 * Period     = 20 ms (50 Hz)
	 * Duty Cycle =  1 ms (minimum) to 2 ms (maximum) linearly displaced.
	 *
	 *************************************************************************/

	/*************************************************************************
     * 0.1) Port configuration for the proper use as PWM (WT1CCP1-PC7).
     *
     *      It's choose the alternate function for the PC6.
     *
     *      Refer to Table 23-5 (page 1344) to check the appropriate encoding
     *      for the bit fields in the register GPIOPCTL.
     *
     * GPIO Alternate Function Select (GPIOAFSEL, page 668)
     * GPIO Port Control (GPIOPCTL, page 685)
     *************************************************************************/
   // GPIO_PORTC_LOCK_R  = GPIO_LOCK_KEY;      /* Unlocks the GPIO_CR register */
   // GPIO_PORTC_CR_R   |= GPIO_PIN_7;         /* Allow changes to PC6         */
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2; /* GPIO Port C Run Mode Clock   */
    										 /* Gating Control               */
    GPIO_PORTC_DIR_R  |=  GPIO_PIN_7;        /* 1: PC7 as output     */
    GPIO_PORTC_AFSEL_R |= GPIO_PIN_7;        /* Alternate function for PC7   */
//	    GPIO_PORTC_PUR_R  |=  GPIO_PIN_7;    /* Enable pull up resistors for PC7 */
    GPIO_PORTC_DEN_R  |= GPIO_PIN_7;              /* Enable digital pins PC7 */
    GPIO_PORTC_AMSEL_R &= ~(GPIO_PIN_7);          /* Disable analog function */
    GPIO_PORTC_PCTL_R  |= GPIO_PCTL_PC7_WT1CCP1;       /* Port Mux Control 7 */
//    GPIO_PORTC_LOCK_R = GPIO_LOCK_UNLOCKED; /* The GPIOCR register is locked */

    /**************************************************************************
	 * 1.0) Enable the Wide Timer 0 using the  RCGCWTIMER register.
	 *
	 * 32/64-Bit Wide General-Purpose Timer Run Mode Clock Gating Control
	 * (RCGCWTIMER, page 355)
	 *************************************************************************/
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R1;
	/**************************************************************************
	 * 1.1) Ensure the timer is disabled (the TAEN bit in the GPTMCTL register
	 *      is cleared) before making any changes.
	 *
	 * GPTM Control (GPTMCTL, page 734)
	 *************************************************************************/
    WTIMER1_CTL_R &= ~(TIMER_CTL_TBEN);  /* GPTM Timer B Disable */
	/**************************************************************************
	 * 1.2)  Write the GPTM Configuration (GPTMCFG) register with a value of
	 *       0x0000.0004. For a 32/64-bit wide timer, this value selects the
	 *       32-bit timer configuration.
	 *
	 * GPTM Configuration (GPTMCFG, page 724)
	 *************************************************************************/
    WTIMER1_CFG_R |= TIMER_CFG_16_BIT;  // |= TIMER_CFG_16_BIT;
	/**************************************************************************
	 * 1.3) In the GPTM Timer Mode (GPTMTBMR) register, set the TnAMS bit to
	 *      0x1, the TnCMR bit to 0x0, and the TnMR field to 0x2.
	 *
	 * TBAMS = 0x1 : PWM mode is enabled.
	 * TBCMR = 0x0 : Edge-Count mode
	 * TBMR  = 0x2 : Periodic Timer mode
	 *
	 *  GPTM Timer B Mode (GPTMTAMR, page 730)
	 *************************************************************************/
    WTIMER1_TBMR_R |= (TIMER_TBMR_TBAMS | TIMER_TBMR_TBMR_PERIOD);
    /**************************************************************************
     * 1.4) Configure the output state of the PWM signal (whether or not it is
     *      inverted) in the TnPWML field of the GPTM Control (GPTMCTL)
     *      register.
     *
     * TBPWML = 0x1 : Output is inverted.
     *
     * GPTM Control (GPTMCTL, page 734)
     *************************************************************************/
    WTIMER1_CTL_R |= TIMER_CTL_TBPWML;

    /**************************************************************************
     * 1.5) Not intended to use the GPTM Timer B Prescale
     *
     *    GPTM Timer B Prescale (GPTMTBPR, page 758)
     *************************************************************************/

    /**************************************************************************
     * 1.6) If PWM interrupts are used, configure the interrupt condition in
     *      the TnEVENT field in the GPTMCTL register and enable the interrupts
     *      by setting the TnPWMIE bit in the GPTMTnMR register. Note that edge
     *      detect interrupt behavior is reversed when the PWM output is
     *      inverted (see page 734).
     *
     *  TBEVENT = 0x0 : Positive edge
     *
     * GPTM Control (GPTMCTL, page 734)
     *************************************************************************/
    //WTIMER1_CTL_R |= ???

	/**************************************************************************
    * 1.7)  Load the timer start value into the GPTM Timer n Interval Load
    *       (GPTMTnILR) register.
    *
    *       Considering the system clock frequency of 40 MHz (T = 25 ns), to
    *       generate a period of 20 ms (50 Hz), the start value that should
    *       be counted down is
    *
    *           Count = 20ms / 25ns
    *                 = 800000
    *                 = 0x000C.3500
    *
    *  GPTM Timer B Interval Load (GPTMTBILR, page 754)
    *************************************************************************/
    WTIMER1_TBILR_R = 0x000C3500;

    /**************************************************************************
    * 1.8)  Load the GPTM Timer n Match (GPTMTnMATCHR) register with the match
    *       value.
    *
    * Minimum -> 1.0 ms : 1.0ms / 25ns = 40000 = 0x0000.9C40
    * Mid     -> 1.5 ms : 1.5ms / 25ns = 60000 = 0x0000.EA60
    * Maximum -> 2.0 ms : 2.0ms / 25ns = 80000 = 0x0001.3880
    *
    *  GPTM Timer B Match (GPTMTBMATCHR, 756)
    *************************************************************************/
    WTIMER1_TBMATCHR_R = 0x00009C40;        /* Starts in minimum for safety */

    /**************************************************************************
    * 1.9) Set the TnEN bit in the GPTM Control (GPTMCTL) register to enable
    *      the timer and begin generation of the output PWM signal.
    *
    *  GPTM Control (GPTMCTL, page 734)
    *************************************************************************/
    WTIMER1_CTL_R |= TIMER_CTL_TBEN;                /* GPTM Timer B Enable */





    /**************************************************************************
     * 1.9) If interrupts are required, set the CnMIM bit in the GPTM Interrupt
     *      Mask (GPTMIMR) register. Generates an interrupt only when were
     *      counted from 0xFFFF.FFFF down to 0x0000.0000 in order to set all
     *      needed flags to keep the timer (edge-count) counting.
     *
     *  GPTM Interrupt Mask (GPTMIMR, page 742)
     *************************************************************************/
//    WTIMER1_IMR_R |= TIMER_IMR_CAMIM;
	/**************************************************************************
	 * 1.10) Set the TnEN bit in the GPTMCTL register to enable the timer and
	 *      start counting.
	 *
	 * GPTM Control (GPTMCTL, page 734)
	 *************************************************************************/
//    WTIMER1_CTL_R |= TIMER_CTL_TAEN; /* GPTM Timer A Enable */
    /**************************************************************************
    * 1.12) Enable Master Interrupt for 16/32-Bit Timer 3A.
    *        WT1CCP0 = 112 -> bit 00 in EN3
    *
    *   Interrupt 96-127 Set Enable (EN3, page 140)
    *************************************************************************/
//    NVIC_EN3_R = 0x00000001;

}

/******************************************************************************
 * SPI 0 Configuration
 * ----------------------------------------------------------------------------
 * SPI Peripheral 0 (SSI0) configured as master to write/read data to/from
 * external memory (SD card).
 *
 * Master
 *
 * Pinout:
 *   PA2: SSI0Clk
 *   PA3: SSI0Fss
 *   PA4: SSI0Rx
 *   PA5: SSI0Tx
 *
 * f_clk = 10 MHz
 *
 ******************************************************************************/
void pso_spi0_config()
{
	volatile uint32_t delay;

	/**************************************************************************
	 * Use SSI2, PB2 to communicate with the SDC.
	 *
	 * CS is PB2
	 * CS can be implemented with any other GPIO pin. It's not been used the
	 * SSI0Fss because sometimes its state changes arbitrarily.
	 *************************************************************************/
	  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;              /* Enable Port B */
	  delay = SYSCTL_RCGCGPIO_R;
	  GPIO_PORTB_PUR_R |= GPIO_PIN_2;           /* Enable weak pullup on PB2 */
	  GPIO_PORTB_DIR_R |= GPIO_PIN_2;                     /* Make PB2 output */
	  GPIO_PORTB_DR4R_R |= GPIO_PIN_2;              /* 4mA output on outputs */
	  //GPIO_PORTA_DATA_BITS_R |= GPIO_PIN_3;
	  GPIO_PORTB_PCTL_R &= ~GPIO_PCTL_PB2_M;
	  GPIO_PORTB_AMSEL_R &= ~GPIO_PIN_2;     /* Disable analog funct. on PB2 */
	  GPIO_PORTB_DEN_R |= GPIO_PIN_2;           /* Enable digital I/O on PB2 */


	/*** 15.4 Module Initialization ***/

	/**************************************************************************
	 * 1.1) Enable the ADC clock using the RCGCSSI register (see page 344).
	 *
	 * Synchronous Serial Interface Run Mode Clock Gating Control (RCGCSSI,344)
	 *************************************************************************/
	SYSCTL_RCGCSSI_R |= SYSCTL_SCGCSSI_S0;          /* Enable SSI Module 0   */

	/**************************************************************************
	 * 1.2) Enable the clock to the appropriate GPIO modules via the RCGCGPIO
	 *    register (page 338). To find out which GPIO port to enable, refer to
	 *    "Signal Description" on page 801.
	 *    ---------------------------------------------------------------------
	 *    Port A:
	 *             PA2 -> SSI0Clk
	 *             PA3 -> SSI0Fss
	 *             PA4 -> SSI0Rx
	 *             PA5 -> SSI0Tx
	 *
	 * General-Purpose Input/Output Run Mode Clock Gating Control (RCGCGPIO,
	 * page 338)
	 *************************************************************************/
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;                /* Enable Port A */
    delay = SYSCTL_RCGC2_R;               /* allow time to finish activating */

	/**************************************************************************
	 *  1.3) Set the GPIO 'AFSEL' bits for the ADC input pins (page 668). To
	 *     determine which GPIOs to configure, see Table 23-4 on page 1337.
	 *     -------------------------------------------------------------------
	 *     DIRECTION
	 *     Inputs (0):
	 *             PA4 <- SSI0Rx
	 *
	 *     Outputs (1):
	 *             PA2 -> SSI0Clk
	 *             PA3 -> SSI0Fss
	 *             PA5 -> SSI0Tx
	 *
	 *     ALTERNATE FUNCTION: Enabled pins with 1
	 *
	 *     DIGITAL PINS: Enabled pins with 1
	 *
	 *************************************************************************/
	//GPIO_PORTA_LOCK_R = GPIO_LOCK_KEY;       /* Unlocks the GPIO_CR register */
	GPIO_PORTA_DIR_R &=  ~(GPIO_PIN_4);                            /* Input  */
	GPIO_PORTA_DIR_R |=  (GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5);   /* Output */
	GPIO_PORTA_AFSEL_R |= (GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);
	GPIO_PORTA_DEN_R |= (GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);
	GPIO_PORTA_PUR_R |= (GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5);/* Pull-up on */
	/*  GPIO 4-mA Drive Select  */
	GPIO_PORTA_DR4R_R |= (GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);
	/* Disable analog function */
	GPIO_PORTA_AMSEL_R &= ~(GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);

	/**************************************************************************
	 * 1.4) Configure the PMCnfields in the GPIOPCTL register to assign the SSI
	 *      signals to the appropriate pins. See page 685 and Table 23-5 on
	 *      page 1344.
	 *
	 *      Port A:
	 *             PA2 -> SSI0Clk
	 *             PA3 -> SSI0Fss
	 *             PA4 -> SSI0Rx
	 *             PA5 -> SSI0Tx
	 *
	 *  GPIO Port Control (GPIOPCTL, page 685)
	 *  Table 23-5. GPIO Pins and Alternate Functions (page, 1344)
	 *************************************************************************/
	GPIO_PORTA_PCTL_R  |= GPIO_PCTL_PA2_SSI0CLK;           /* SSI0CLK on PA2 */
	GPIO_PORTA_PCTL_R  |= GPIO_PCTL_PA3_SSI0FSS;           /* SSI0FSS on PA3 */
	GPIO_PORTA_PCTL_R  |= GPIO_PCTL_PA4_SSI0RX;            /* SSI0RX on PA4  */
	GPIO_PORTA_PCTL_R  |= GPIO_PCTL_PA5_SSI0TX;            /* SSI0TX on PA5  */
    //GPIO_PORTA_LOCK_R = GPIO_LOCK_UNLOCKED; /* The GPIOCR register is locked */

    /**************************************************************************
     * 1.5.1) Ensure that the SSEbit in the SSICR1 register is clear before
     *        making any configuration changes.
     *
     *
     * SSI Control 1 (SSICR1, page 966)
     *************************************************************************/
    SSI0_CR1_R &= ~(SSI_CR1_SSE);              /* SSI0 operation is disabled */
//    SSI0_CR1_R |= SSI_CR1_LBM; /* Loopback mode enabled only for debug phase */

    /**************************************************************************
     * 1.5.2) Select whether the SSI is a master or slave:
     *
     *        a) For master operations, set the SSICR1 register to 0x0000.0000.
     *
     * SSI Control 1 (SSICR1, page 966)
     *************************************************************************/
    SSI0_CR1_R &= ~SSI_CR1_MS;                                     /* Master */

    /**************************************************************************
	 * 1.5.3) Configure the SSI clock source by writing to the SSICC register
	 *        System clock (based on clock source and divisor factor)
	 *
	 * SSI Clock Configuration (SSICC, page 979)
	 *************************************************************************/
    SSI0_CC_R = SSI_CC_CS_SYSPLL;                            /* System clock */

    /**************************************************************************
	 * 1.5.4) Configure the clock prescale divisor (CPSDVSR) by writing to the
	 *        SSICPSR register.
	 *
	 *       SSIClk  =  SysClk  /  (CPSDVSR  *  (1  +  SCR))
	 *               =   40MHz /   (  50     *  (1  +   1 ))
	 *               =   40MHz / 100
	 *
	 *      [SSIClk  =    400 kHz]
	 *
	 *                       ->   CPSDVSR = 50
	 *                       ->   SCR     = 1
	 *
	 *       This value must be an even number from 2 to 254, depending on the
     *       frequency of SSIClk. The LSB always returns 0 on reads.
	 *
	 * SSI Clock Prescale (SSICPSR, page 971)
	 *************************************************************************/
    SSI0_CPSR_R |= 0x00000032;                      /* CPSDVSR = 50 (0x32=50)*/

    /**************************************************************************
	 * 1.5.5) Write the SSICR0 register with the following configuration:
	 *
	 *       - Serial clock rate (SCR)
	 *           BR = SysClk/(CPSDVSR * (1 + SCR))
	 *              = 40e6/(50 * (1 + 1))
	 *              -> SCR = 1
	 *
	 *       - Desired clock phase/polarity (SPH and SPO)
	 *          SPO:
	 *            0: Low steady state clk's line; <--
	 *            1: High steady state clk's line.
	 *
	 *          SPH:
	 *            0: Data captured on the 1st clk edge; <--
	 *            1: Data captured on the 2nd clk edge.
	 *
	 *       - The protocol mode: Freescale SPI, TI SSF, MICROWIRE (FRF)
	 *
	 *       - The data size (DSS): 8 bits
	 *
	 * Excerpt from FatFS: "SPI mode 0 (CPHA=0, CPOL=0) is the proper setting
	 *                      to control MMC/SDC"
	 *
	 * SSI Control 0 (SSICR0, page 964)
	 *************************************************************************/
    SSI0_CR0_R = (SSI0_CR0_R & ~SSI_CR0_SCR_M) + 0x00000100;      /* SCR = 1 */
    SSI0_CR0_R |= (SSI_CR0_FRF_MOTO |                    /* Freescale SPI    */
    		       SSI_CR0_DSS_8);                       /* 8-bits data byte */

    SSI0_CR0_R &= ~(SSI_CR0_SPO | SSI_CR0_SPH);            /* CPOL=0, CPHA=0 */

    /**************************************************************************
     * 1.5.6) Ensure that the SSEbit in the SSICR1 register is clear before
     *        making any configuration changes.
     *
     *
     * SSI Control 1 (SSICR1, page 966)
     *************************************************************************/
    SSI0_CR1_R |= SSI_CR1_SSE;                               /* SSI0 enabled */

}

void pso_spi2_config()
{
	volatile uint32_t delay;

	/**************************************************************************
	 * Use SSI2, PA3 to communicate with the SDC.
	 *
	 * CS is PA3
	 * CS can be implemented with any other GPIO pin. It's not been used the
	 * SSI2Fss because sometimes its state changes arbitrarily.
	 *************************************************************************/
	  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;              /* Enable Port A */
	  delay = SYSCTL_RCGCGPIO_R;
	  GPIO_PORTA_PUR_R |= GPIO_PIN_3;           /* Enable weak pullup on PA3 */
	  GPIO_PORTA_DIR_R |= GPIO_PIN_3;                     /* Make PA3 output */
	  GPIO_PORTA_DR4R_R |= GPIO_PIN_3;              /* 4mA output on outputs */
	  //GPIO_PORTA_DATA_BITS_R |= GPIO_PIN_3;
	  GPIO_PORTA_PCTL_R &= ~GPIO_PCTL_PA3_M;
	  GPIO_PORTA_AMSEL_R &= ~GPIO_PIN_3;     /* Disable analog funct. on PA3 */
	  GPIO_PORTA_DEN_R |= GPIO_PIN_3;           /* Enable digital I/O on PA3 */

	/*** 15.4 Module Initialization ***/

	/**************************************************************************
	 * 1.1) Enable the ADC clock using the RCGCSSI register (see page 344).
	 *
	 * Synchronous Serial Interface Run Mode Clock Gating Control (RCGCSSI,344)
	 *************************************************************************/
	SYSCTL_RCGCSSI_R |= SYSCTL_SCGCSSI_S2;          /* Enable SSI Module 2   */

	/**************************************************************************
	 * 1.2) Enable the clock to the appropriate GPIO modules via the RCGCGPIO
	 *    register (page 338). To find out which GPIO port to enable, refer to
	 *    "Signal Description" on page 801.
	 *    ---------------------------------------------------------------------
	 *    Port B:
	 *             PB4 -> SSI0Clk
	 *             PB5 -> SSI0Fss
	 *             PB6 -> SSI0Rx
	 *             PB7 -> SSI0Tx
	 *
	 * General-Purpose Input/Output Run Mode Clock Gating Control (RCGCGPIO,
	 * page 338)
	 *************************************************************************/
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;                /* Enable Port B */
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R1) == 0){};            /* Ready? */

	/**************************************************************************
	 *  1.3) Set the GPIO 'AFSEL' bits for the ADC input pins (page 668). To
	 *     determine which GPIOs to configure, see Table 23-4 on page 1337.
	 *     -------------------------------------------------------------------
	 *     DIRECTION
	 *     Inputs (0):
	 *             PB6 <- SSI0Rx
	 *
	 *     Outputs (1):
	 *             PB4 -> SSI0Clk
	 *             PB5 -> SSI0Fss
	 *             PB7 -> SSI0Tx
	 *
	 *     ALTERNATE FUNCTION: Enabled pins with 1
	 *
	 *     DIGITAL PINS: Enabled pins with 1
	 *
	 *************************************************************************/
	//GPIO_PORTA_LOCK_R = GPIO_LOCK_KEY;       /* Unlocks the GPIO_CR register */
	GPIO_PORTB_DIR_R &=  ~(GPIO_PIN_6);                            /* Input  */
	GPIO_PORTB_DIR_R |=  (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7);   /* Output */
	GPIO_PORTB_AFSEL_R |= (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
	GPIO_PORTB_DEN_R |= (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
	GPIO_PORTB_PUR_R |= (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7);/* Pull-up on */
	/*  GPIO 4-mA Drive Select  */
	GPIO_PORTB_DR4R_R |= (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
	/* Disable analog function */
	GPIO_PORTB_AMSEL_R &= ~(GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

	/**************************************************************************
	 * 1.4) Configure the PMCnfields in the GPIOPCTL register to assign the SSI
	 *      signals to the appropriate pins. See page 685 and Table 23-5 on
	 *      page 1344.
	 *
	 *      Port A:
	 *             PB4 -> SSI0Clk
	 *             PB5 -> SSI0Fss
	 *             PB6 -> SSI0Rx
	 *             PB7 -> SSI0Tx
	 *
	 *  GPIO Port Control (GPIOPCTL, page 685)
	 *  Table 23-5. GPIO Pins and Alternate Functions (page, 1344)
	 *************************************************************************/
	GPIO_PORTB_PCTL_R  |= GPIO_PCTL_PB4_SSI2CLK;           /* SSI0CLK on PB4 */
	GPIO_PORTB_PCTL_R  |= GPIO_PCTL_PB5_SSI2FSS;           /* SSI0FSS on PB5 */
	GPIO_PORTB_PCTL_R  |= GPIO_PCTL_PB6_SSI2RX;            /* SSI0RX on PB6  */
	GPIO_PORTB_PCTL_R  |= GPIO_PCTL_PB7_SSI2TX;            /* SSI0TX on PB7  */
    //GPIO_PORTA_LOCK_R = GPIO_LOCK_UNLOCKED; /* The GPIOCR register is locked */

    /**************************************************************************
     * 1.5.1) Ensure that the SSEbit in the SSICR1 register is clear before
     *        making any configuration changes.
     *
     *
     * SSI Control 1 (SSICR1, page 966)
     *************************************************************************/
    SSI2_CR1_R &= ~(SSI_CR1_SSE);              /* SSI2 operation is disabled */
//    SSI0_CR1_R |= SSI_CR1_LBM; /* Loopback mode enabled only for debug phase */

    /**************************************************************************
     * 1.5.2) Select whether the SSI is a master or slave:
     *
     *        a) For master operations, set the SSICR1 register to 0x0000.0000.
     *
     * SSI Control 1 (SSICR1, page 966)
     *************************************************************************/
    SSI2_CR1_R &= ~SSI_CR1_MS;                                     /* Master */

    /**************************************************************************
	 * 1.5.3) Configure the SSI clock source by writing to the SSICC register
	 *        System clock (based on clock source and divisor factor)
	 *
	 * SSI Clock Configuration (SSICC, page 979)
	 *************************************************************************/
    SSI2_CC_R = SSI_CC_CS_SYSPLL;                            /* System clock */

    /**************************************************************************
	 * 1.5.4) Configure the clock prescale divisor by writing the SSICPSR
	 *        register.
	 *
	 *       SSIClk  =  SysClk  /  (CPSDVSR  *  (1  +  SCR))
	 *               =   40MHz /   (  10     *  (1  +   9 ))
	 *               =   40MHz / 100
	 *
	 *      [SSIClk  =    400 kHz]
	 *
	 *                       ->   CPSDVSR = 10
	 *                       ->   SCR     = 9
	 *
	 *       This value must be an even number from 2 to 254, depending on the
     *       frequency of SSIClk. The LSB always returns 0 on reads.
	 *
	 * SSI Clock Prescale (SSICPSR, page 971)
	 *************************************************************************/
    SSI2_CPSR_R |= 0x0000000A;                      /* CPSDVSR = 10 (0x0A=10)*/

    /**************************************************************************
	 * 1.5.5) Write the SSICR0 register with the following configuration:
	 *
	 *       - Serial clock rate (SCR)
	 *           BR = SysClk/(CPSDVSR * (1 + SCR))
	 *              = 40e6/(4 * (1 + 9))
	 *              -> SCR = 9
	 *
	 *       - Desired clock phase/polarity (SPH and SPO)
	 *          SPO:
	 *            0: Low steady state clk's line; <--
	 *            1: High steady state clk's line.
	 *
	 *          SPH:
	 *            0: Data captured on the 1st clk edge; <--
	 *            1: Data captured on the 2nd clk edge.
	 *
	 *       - The protocol mode: Freescale SPI, TI SSF, MICROWIRE (FRF)
	 *
	 *       - The data size (DSS): 8 bits
	 *
	 * Excerpt from FatFS: "SPI mode 0 (CPHA=0, CPOL=0) is the proper setting
	 *                      to control MMC/SDC"
	 *
	 * SSI Control 0 (SSICR0, page 964)
	 *************************************************************************/
    SSI2_CR0_R |= ((SSI_CR0_SCR_M & 0x00000900) |        /* (0x00000900 : 9) */
    		       SSI_CR0_FRF_MOTO |                    /* Freescale SPI    */
    		       SSI_CR0_DSS_8);                       /* 8-bits data byte */

    SSI2_CR0_R &= ~(SSI_CR0_SPO | SSI_CR0_SPH);            /* CPOL=0, CPHA=0 */

    /**************************************************************************
     * 1.5.6) Ensure that the SSEbit in the SSICR1 register is clear before
     *        making any configuration changes.
     *
     *
     * SSI Control 1 (SSICR1, page 966)
     *************************************************************************/
    SSI2_CR1_R |= SSI_CR1_SSE;                               /* SSI0 enabled */
}


