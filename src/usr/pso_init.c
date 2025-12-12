/*******************************************************************************
 * FILE:        pso_init.c
 *
 * DESCRIPTION:
 *     PSO System Initialization Module.
 *     Contains hardware initialization functions for TM4C123 peripherals:
 *     - UART0/2 configuration (115200 baud, 8N1)
 *     - GPIO configuration for debug pins and buttons
 *     - Timer0 configuration for ADC triggering (5 kHz)
 *     - ADC0/1 configuration for 6-channel data acquisition
 *     - PWM configuration for motor control (50 Hz, 1-2ms pulse)
 *     - SPI0/2 configuration for SD card communication (400 kHz)
 *     - RPM measurement system (Timer3 + WTimer1 edge counting)
 *
 * DOCUMENTATION STYLE:
 *     - Technical and functional
 *     - No functional or logical modifications
 *     - Improved comments and formatting only
 *
 * AUTHOR:      Rogerio Lima
 * DATE:        07/03/2014
 * REFORMAT:    2025 (Documentation and formatting only)
 *
 *******************************************************************************/

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

/*******************************************************************************
 * FUNCTION: myISR_Config
 * 
 * DESCRIPTION:
 *     Placeholder for Interrupt Service Routine configuration.
 *     Currently empty - intended for future interrupt setup.
 * 
 * PARAMETERS:
 *     None
 * 
 * RETURNS:
 *     void
 *******************************************************************************/
void myISR_Config()
{
    /* Function intentionally left empty for future implementation */
    /* Placeholder for interrupt service routine configuration */
}

/*******************************************************************************
 * FUNCTION: PSO_UART0Config
 * 
 * DESCRIPTION:
 *     Configures UART0 for Virtual COM Port communication at 115200 baud.
 *     Used for debugging and data streaming to host computer.
 * 
 * PARAMETERS:
 *     None
 * 
 * RETURNS:
 *     void
 * 
 * HARDWARE CONFIGURATION:
 *     - Pins: PA0 (RX), PA1 (TX)
 *     - Baud Rate: 115200
 *     - Data Format: 8-bit data, no parity, 1 stop bit (8N1)
 *     - Interrupts: RX and RX timeout interrupts enabled
 *     - FIFO: Enabled with 1/8 TX and 7/8 RX watermark levels
 *******************************************************************************/
void PSO_UART0Config()
{
    /* Enable UART0 and GPIO Port A peripherals */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);    /* Enable UART Module 0 */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);    /* Enable GPIO Port A */
    
    /* Configure GPIO pins for UART function */
    GPIOPinConfigure(GPIO_PA0_U0RX);                /* PA0 as UART0 RX */
    GPIOPinConfigure(GPIO_PA1_U0TX);                /* PA1 as UART0 TX */
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    
    /* Configure UART communication parameters */
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                        UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                        UART_CONFIG_PAR_NONE);
    
    /* UART interrupt configuration */
    UARTDisable(UART0_BASE);                        /* Disable before config */
    UARTFIFOEnable(UART0_BASE);                     /* Enable hardware FIFO */
    
    IntEnable(INT_UART0);                           /* Enable UART0 interrupt */
    
    /* Set FIFO trigger levels: TX at 1/8, RX at 7/8 */
    UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX1_8, UART_FIFO_RX7_8);
    
    /* Enable RX and RX timeout interrupts */
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    
    /* Enable UART and FIFO */
    UARTEnable(UART0_BASE);
    UARTFIFOEnable(UART0_BASE);
}

/*******************************************************************************
 * FUNCTION: myUART2Config_Init
 * 
 * DESCRIPTION:
 *     Configures UART2 for alternative serial communication at 9600 baud.
 *     Uses direct register access for educational purposes.
 * 
 * PARAMETERS:
 *     None
 * 
 * RETURNS:
 *     void
 * 
 * HARDWARE CONFIGURATION:
 *     - Pins: PD6 (RX), PD7 (TX)
 *     - Baud Rate: 9600
 *     - Data Format: 8-bit data, no parity, 1 stop bit (8N1)
 *     - System Clock: 16 MHz (assuming from configuration)
 *     - Baud Calculation: BRDI = 3333, BRDF = 26
 * 
 * NOTES:
 *     Uses direct hardware register access rather than driverlib functions
 *******************************************************************************/
void myUART2Config_Init()
{
    /* Enable UART2 and GPIO Port D clocks */
    HWREG(SYSCTL_RCGCUART) = SYSCTL_RCGCUART_R2;    /* Enable UART2 clock */
    HWREG(SYSCTL_RCGCGPIO) = SYSCTL_RCGCGPIO_R3;    /* Enable Port D clock */
    
    /* Disable UART before configuration */
    HWREG(UART2_BASE + UART_O_CTL) &= ~UART_CTL_UARTEN;
    
    /**************************************************************************
     * BAUD RATE CALCULATION (16 MHz system clock):
     * BRDI + BRDF = UARTSysClk / (ClkDiv * Baud Rate)
     *             = 16,000,000 / (5 * 9600)
     *             = 3333.3333
     * 
     * BRDI (Integer part): 3333 = 0x0D05
     * BRDF (Fractional part): (0.3333 * 64 + 0.5) = 26 = 0x1A
     *************************************************************************/
    HWREG(UART2_BASE + UART_O_IBRD) = 0x00000D05;   /* Integer baud divisor */
    HWREG(UART2_BASE + UART_O_FBRD) = 0x0000001A;   /* Fractional baud divisor */
    
    /* Configure line control: 8-bit, no parity, 1 stop bit */
    HWREG(UART2_BASE + UART_O_LCRH) = UART_CONFIG_WLEN_8 | 
                                      UART_CONFIG_PAR_NONE | 
                                      UART_CONFIG_STOP_ONE;
    
    /* Enable UART */
    HWREG(UART2_BASE + UART_O_CTL) = UART_CTL_UARTEN;
    
    /* Configure GPIO pins PD6 and PD7 for UART function */
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;   /* Unlock commit */
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= (GPIO_PIN_6 | GPIO_PIN_7);
    HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) |= (GPIO_PIN_6 | GPIO_PIN_7);
    HWREG(GPIO_PORTD_BASE + GPIO_O_DR2R) |= GPIO_STRENGTH_2MA;
    HWREG(GPIO_PORTD_BASE + GPIO_O_DEN) |= (GPIO_PIN_6 | GPIO_PIN_7);
    
    /* Configure pin mux for UART function (bits 24-31 for PD7, bits 16-23 for PD6) */
    HWREG(GPIO_PORTD_BASE + GPIO_O_PCTL) |= 0x11000000;
}

/*******************************************************************************
 * FUNCTION: PSO_PeripheralEnable
 * 
 * DESCRIPTION:
 *     Enables clock gating for required peripherals.
 *     Must be called before configuring any peripherals.
 * 
 * PARAMETERS:
 *     None
 * 
 * RETURNS:
 *     void
 * 
 * PERIPHERALS ENABLED:
 *     - GPIOC: PWM and Timer pins (PC4-PC7)
 *     - GPIOD: PWM and Timer pins (PD0-PD1, PD6-PD7)
 *     - GPIOF: User interface (LEDs and buttons)
 * 
 * NOTES:
 *     PWM and Timer peripherals commented out - may be enabled elsewhere
 *******************************************************************************/
void PSO_PeripheralEnable()
{
    /* PWM and Timer peripherals (commented out, may be enabled elsewhere) */
    // SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);     /* Enable PWM Module 0 */
    // SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);     /* Enable PWM Module 1 */
    // SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1);  /* Enable Wide Timer 1 */
    // SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER5);  /* Enable Wide Timer 5 */
    
    /* Enable required GPIO ports */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);    /* GPIO Port C: PWM/Timer */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);    /* GPIO Port D: PWM/Timer */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);    /* GPIO Port F: LEDs/Buttons */
}

/*******************************************************************************
 * FUNCTION: PSO_GPIOConfig
 * 
 * DESCRIPTION:
 *     Configures GPIO Port F for user interface:
 *     - PF1, PF2, PF3: Outputs (LEDs)
 *     - PF0, PF4: Inputs with pull-up (buttons SW2 and SW1)
 * 
 * PARAMETERS:
 *     None
 * 
 * RETURNS:
 *     void
 * 
 * PIN CONFIGURATION:
 *     - PF0: SW2 button (input with pull-up)
 *     - PF1: Red LED (output)
 *     - PF2: Blue LED (output)
 *     - PF3: Green LED (output)
 *     - PF4: SW1 button (input with pull-up)
 * 
 * NOTES:
 *     - Uses direct register access for educational purposes
 *     - PF0 and PF4 require commit control unlock sequence
 *******************************************************************************/
void PSO_GPIOConfig()
{
    /*** Port F Configuration ***/
    
    /* Unlock commit control for PF0-PF4 */
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x1F;      /* Allow changes to PF4-0 */
    
    /* Enable GPIO Port F clock */
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGC2_GPIOF;
    
    /* Configure pin directions: PF1-PF3 as outputs, PF0/PF4 as inputs */
    HWREG(GPIO_PORTF_BASE + GPIO_O_DIR) |= 0x0E;     /* 0000 1110: PF1,PF2,PF3 out */
    
    /* No alternate function (pure GPIO) */
    HWREG(GPIO_PORTF_BASE + GPIO_O_AFSEL) = 0x00;
    
    /* Set drive strength to 2mA */
    HWREG(GPIO_PORTF_BASE + GPIO_O_DR2R) = GPIO_STRENGTH_2MA;
    
    /* Enable pull-up resistors for PF0 and PF4 (buttons) */
    HWREG(GPIO_PORTF_BASE + GPIO_O_PUR) |= 0x11;     /* 0001 0001: PF0 and PF4 */
    
    /* Enable digital function for all 5 pins */
    HWREG(GPIO_PORTF_BASE + GPIO_O_DEN) |= 0x1F;
    
    /* Disable analog function */
    HWREG(GPIO_PORTF_BASE + GPIO_O_AMSEL) = 0x00;
    
    /* Clear port control (GPIO function) */
    HWREG(GPIO_PORTF_BASE + GPIO_O_PCTL) = 0x00000000;
    
    /* Lock registers for safety */
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
}

/*******************************************************************************
 * FUNCTION: myPWM_Init
 * 
 * DESCRIPTION:
 *     Configures GPIO pins for PWM output functionality.
 *     Maps PWM signals to specific GPIO pins for motor control.
 * 
 * PARAMETERS:
 *     None
 * 
 * RETURNS:
 *     void
 * 
 * PIN MAPPINGS:
 *     - PC4: M0PWM6 (PWM Module 0, Generator 3, PWM6)
 *     - PC5: M0PWM7 (PWM Module 0, Generator 3, PWM7)
 *     - PD0: M1PWM0 (PWM Module 1, Generator 0, PWM0)
 *     - PD1: M1PWM1 (PWM Module 1, Generator 0, PWM1)
 * 
 * NOTES:
 *     Only configures GPIO for PWM - PWM module configuration is separate
 *******************************************************************************/
void myPWM_Init()
{
    /* Configure Port C pins for PWM */
    GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_4);    /* PC4 as PWM */
    GPIOPinConfigure(GPIO_PC4_M0PWM6);              /* PC4 = M0PWM6 */
    
    GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_5);    /* PC5 as PWM */
    GPIOPinConfigure(GPIO_PC5_M0PWM7);              /* PC5 = M0PWM7 */
    
    /* Configure Port D pins for PWM */
    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);    /* PD0 as PWM */
    GPIOPinConfigure(GPIO_PD0_M1PWM0);              /* PD0 = M1PWM0 */
    
    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_1);    /* PD1 as PWM */
    GPIOPinConfigure(GPIO_PD1_M1PWM1);              /* PD1 = M1PWM1 */
}

/*******************************************************************************
 * FUNCTION: PSO_Timers
 * 
 * DESCRIPTION:
 *     Configures Timer0A for periodic interrupts at 5 kHz.
 *     Used to trigger ADC conversions at regular intervals.
 * 
 * PARAMETERS:
 *     None
 * 
 * RETURNS:
 *     void
 * 
 * TIMER CONFIGURATION:
 *     - Mode: Periodic
 *     - Frequency: 5000 Hz (200μs period)
 *     - System Clock: 40 MHz (from system_init)
 *     - Load Value: (40,000,000 / 5000) - 1 = 7999
 *     - Trigger: ADC (TimerControlTrigger for ADC triggering)
 *     - Interrupt: Timer0A timeout interrupt enabled
 * 
 * NOTES:
 *     Maximum sustainable UART rate at 115200 baud is ~500 Hz for 20-byte packets
 *******************************************************************************/
void PSO_Timers()
{
    uint32_t ui32Period;
    uint16_t freq = 5000;    /* Desired frequency: 5000 Hz (5 kHz) */
    
    /* Enable Timer0 peripheral */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    
    /* Configure as periodic timer */
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    
    /* Calculate timer period for 5 kHz */
    ui32Period = (SysCtlClockGet() / freq);
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period - 1);
    
    /* Configure timer to trigger ADC conversions */
    TimerControlTrigger(TIMER0_BASE, TIMER_A, true);
    
    /* Enable Timer0A interrupt and global interrupts */
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntMasterEnable();
    
    /* Start the timer */
    TimerEnable(TIMER0_BASE, TIMER_A);
}

/*******************************************************************************
 * FUNCTION: PSO_ADCConfig
 * 
 * DESCRIPTION:
 *     Configures ADC0 and ADC1 for 6-channel data acquisition at 5 kHz.
 *     Uses Sample Sequencer 1 with hardware averaging (4x).
 *     Timer-triggered conversions with interrupts on ADC0 only.
 * 
 * PARAMETERS:
 *     None
 * 
 * RETURNS:
 *     void
 * 
 * CHANNEL CONFIGURATION:
 *     ADC0 (3 channels):
 *       - Step 0: AIN6 (PD1) - Acceleration X (Ax)
 *       - Step 1: AIN7 (PD0) - Strain Gauge
 *       - Step 2: AIN2 (PE1) - Motor Voltage (V_motor)
 *     
 *     ADC1 (3 channels):
 *       - Step 0: AIN5 (PD2) - Acceleration Y (Ay)
 *       - Step 1: AIN4 (PD3) - Acceleration Z (Az)
 *       - Step 2: AIN1 (PE2) - Motor Current (I_motor)
 * 
 * TIMING:
 *     - Sample Rate: 5 kHz (triggered by Timer0)
 *     - Hardware Oversampling: 4x
 *     - Effective Resolution: 12-bit + averaging
 * 
 * NOTES:
 *     - PE0 and PE5 are configured but marked as DISCARD
 *     - Only ADC0 generates interrupts (ADC1 data polled)
 *     - Coincident sampling between ADC0 and ADC1
 *******************************************************************************/
void PSO_ADCConfig()
{
    uint16_t delay;
    
    /*** 13.4.1 Module Initialization ***/
    
    /* Enable ADC0 and ADC1 clocks */
    SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;    /* Enable ADC Module 0 */
    SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R1;    /* Enable ADC Module 1 */
    
    /* Short delay for clock stabilization */
    for (delay = 0U; delay < 100; delay++);
    
    /* Enable GPIO Ports D and E (ADC input pins) */
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;    /* Enable Port D */
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;    /* Enable Port E */
    
    /* Configure Port D pins PD0-PD3 as analog inputs */
    GPIO_PORTD_DIR_R &= ~(GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    GPIO_PORTD_AFSEL_R |= (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    
    /* Configure Port E pins PE0, PE1, PE2, PE5 as analog inputs */
    GPIO_PORTE_DIR_R &= ~(GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_5);
    GPIO_PORTE_AFSEL_R |= (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_5);
    
    /* Disable digital function on analog pins */
    GPIO_PORTD_DEN_R &= ~(GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    GPIO_PORTE_DEN_R &= ~(GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_5);
    
    /* Enable analog mode (disable isolation circuit) */
    GPIO_PORTD_AMSEL_R |= (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    GPIO_PORTE_AMSEL_R |= (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_5);
    
    /* Configure sample sequencer priority (SS1 highest priority) */
    ADC0_SSPRI_R = 0x3201;    /* SS3=lowest(3), SS2=mid-low(2), SS1=highest(0), SS0=mid-high(1) */
    ADC1_SSPRI_R = 0x3201;    /* Same priority for ADC1 */
    
    /* Configure ADC for 250 ksps maximum sample rate */
    ADC0_PC_R = ADC_PP_MSR_250K;
    ADC1_PC_R = ADC_PP_MSR_250K;
    
    /*** 13.4.2 Sample Sequencer Configuration ***/
    
    /* Disable sample sequencer 1 before configuration */
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN1;   /* Disable ADC0 SS1 */
    ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN1;   /* Disable ADC1 SS1 */
    
    /* Configure trigger source: Timer trigger */
    ADC0_EMUX_R |= ADC_EMUX_EM1_TIMER;
    ADC1_EMUX_R |= ADC_EMUX_EM1_TIMER;
    
    /* Configure coincident sampling (same phase for both ADCs) */
    ADC0_SPC_R = ADC_SPC_PHASE_0;
    ADC1_SPC_R = ADC_SPC_PHASE_0;
    
    /*** Configure ADC0 Input Multiplexer (3 channels) ***/
    ADC0_SSMUX1_R = ((0x06 << ADC_SSMUX1_MUX0_S) |   /* Step 0: AIN6 (PD1) - Ax */
                     (0x07 << ADC_SSMUX1_MUX1_S) |   /* Step 1: AIN7 (PD0) - Strain Gauge */
                     (0x02 << ADC_SSMUX1_MUX2_S));   /* Step 2: AIN2 (PE1) - V_motor */
    
    /*** Configure ADC1 Input Multiplexer (3 channels) ***/
    ADC1_SSMUX1_R = ((0x05 << ADC_SSMUX1_MUX0_S) |   /* Step 0: AIN5 (PD2) - Ay */
                     (0x04 << ADC_SSMUX1_MUX1_S) |   /* Step 1: AIN4 (PD3) - Az */
                     (0x01 << ADC_SSMUX1_MUX2_S));   /* Step 2: AIN1 (PE2) - I_motor */
    
    /*** Configure Sample Sequence Control ***/
    /* ADC0: End sequence and interrupt on step 2 */
    ADC0_SSCTL1_R = (ADC_SSCTL1_END2 | ADC_SSCTL1_IE2);
    
    /* ADC1: End sequence on step 2 (no interrupt) */
    ADC1_SSCTL1_R = ADC_SSCTL1_END2;
    
    /* Enable 4x hardware oversampling for both ADCs */
    ADC0_SAC_R |= ADC_SAC_AVG_4X;
    ADC1_SAC_R |= ADC_SAC_AVG_4X;
    
    /* Enable interrupt for ADC0 sample sequencer 1 */
    ADC0_IM_R = ADC_IM_MASK1;
    
    /* Enable sample sequencer 1 for both ADCs */
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN1;   /* Enable ADC0 SS1 */
    ADC1_ACTSS_R |= ADC_ACTSS_ASEN1;   /* Enable ADC1 SS1 */
    
    /* Enable ADC interrupts in NVIC */
    NVIC_EN0_R |= 0x8000;    /* ADC0SS1 = interrupt 31 (bit 15 in EN0) */
    NVIC_EN2_R |= 0x0002;    /* ADC1SS1 = interrupt 65 (bit 2 in EN2) */
}


/*******************************************************************************
 * FUNCTION: pso_rpm_config
 * 
 * DESCRIPTION:
 *     Configures RPM measurement system using two timers:
 *     1. Timer3A: Periodic 100ms interrupt for RPM calculation
 *     2. WTimer1A: Edge counter on PC6 for pulse counting
 * 
 * PARAMETERS:
 *     None
 * 
 * RETURNS:
 *     void
 * 
 * HARDWARE CONFIGURATION:
 *     - RPM Sensor Input: PC6 (WT1CCP0)
 *     - Timer3A: 100ms periodic interrupt (40MHz / 4,000,000 = 100ms)
 *     - WTimer1A: Edge counter mode, counts rising edges on PC6
 * 
 * RPM CALCULATION:
 *     RPM = (pulse_count * 60 * 1000) / (BLADE_NUMBER * measurement_period_ms)
 * 
 * NOTES:
 *     - WTimer1 configured in EDGE-COUNT mode (not CAPTURE mode)
 *     - Timer3 interrupt reads WTimer1 counter every 100ms
 *     - PC6 has pull-up resistor enabled for open-collector sensors
 *******************************************************************************/
void pso_rpm_config(void)
{
    /**************************************************************************
     * PART 1: Configure Timer3A - Periodic 100ms interrupt for RPM calculation
     *************************************************************************/
    
    /* Enable Timer3 clock and wait for readiness */
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R3;
    while(!(SYSCTL_PRTIMER_R & SYSCTL_PRTIMER_R3)) {}  /* Wait until ready */

    /* Disable timer before configuration */
    TIMER3_CTL_R &= ~TIMER_CTL_TAEN;

    /* Configure as 32-bit periodic timer counting up */
    TIMER3_CFG_R = TIMER_CFG_32_BIT_TIMER;          /* 32-bit configuration */
    TIMER3_TAMR_R = TIMER_TAMR_TAMR_PERIOD | TIMER_TAMR_TACDIR; /* Periodic, count up */

    /* Load interval for 100ms (40MHz / 4,000,000 = 100ms) */
    TIMER3_TAILR_R = 4000000 - 1;                   /* 0x003D08FF */

    /* Enable timeout interrupt */
    TIMER3_IMR_R |= TIMER_IMR_TATOIM;

    /* Clear any pending interrupts */
    TIMER3_ICR_R = TIMER_ICR_TATOCINT;

    /* Start timer */
    TIMER3_CTL_R |= TIMER_CTL_TAEN;

    /* Enable Timer3A interrupt in NVIC (IRQ 35) */
    NVIC_EN1_R |= (1 << 3);                         /* Enable interrupt 35 */

    /**************************************************************************
     * PART 2: Configure WTimer1A - Edge Capture Mode on PC6
     *************************************************************************/
    
    /* Configure GPIO PC6 for WT1CCP0 input */
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2;        /* Enable Port C clock */
    while(!(SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R2)) {} /* Wait until ready */
    
    GPIO_PORTC_DIR_R &= ~GPIO_PIN_6;                /* PC6 as input */
    GPIO_PORTC_AFSEL_R |= GPIO_PIN_6;               /* Enable alternate function */

    /* CORREÇÃO CRÍTICA: Configurar PCTL corretamente */
    GPIO_PORTC_PCTL_R = (GPIO_PORTC_PCTL_R & ~GPIO_PCTL_PC6_M) | GPIO_PCTL_PC6_WT1CCP0;

    GPIO_PORTC_DEN_R |= GPIO_PIN_6;                 /* Enable digital */
    GPIO_PORTC_PUR_R |= GPIO_PIN_6;                 /* Enable pull-up resistor */
    GPIO_PORTC_AMSEL_R &= ~GPIO_PIN_6;              /* Disable analog */
    
    /* Enable Wide Timer1 clock */
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R1;
    while(!(SYSCTL_PRWTIMER_R & SYSCTL_PRWTIMER_R1)) {} /* Wait until ready */
    
    /* Disable timer before configuration */
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    
    /* Configure as 32-bit wide timer */
    WTIMER1_CFG_R = 0x00000004;                     /* 32-bit wide timer (mesmo da versão funcional) */
    
    /* CORREÇÃO CRÍTICA: Configurar modo CAPTURE com TACDIR */
    WTIMER1_TAMR_R = TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; /* Capture mode, count up */
    
    /* Configure to capture on RISING edges */
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEVENT_M;          /* Clear event bits */
    WTIMER1_CTL_R |= TIMER_CTL_TAEVENT_POS;         /* Capture on rising edge */
    
    /* Set maximum count value */
    WTIMER1_TAILR_R = 0xFFFFFFFF;                   /* Count to max */

    /* No prescaler - maximum resolution */
    WTIMER1_TAPR_R = 0;
    
    /* REMOVER estas linhas (não são necessárias para capture mode) */
    /* WTIMER1_CTL_R |= TIMER_CTL_TAOTE; */         /* Não necessário */
    /* WTIMER1_CTL_R |= TIMER_CTL_TAPWML; */        /* Não necessário */
    
    /* Enable capture interrupt */
    WTIMER1_IMR_R |= TIMER_IMR_CAEIM;               /* Enable capture event interrupt */
    
    /* Clear any pending interrupts */
    WTIMER1_ICR_R = TIMER_ICR_CAECINT;

    /* Enable Wide Timer1A */
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;
    
    /* Habilitar no NVIC - WTimer1A é IRQ 96 */
    NVIC_EN3_R |= (1 << 0);  // Habilita IRQ 96 (WTimer1A)

    /**************************************************************************
     * DEBUG: Configure test pins for verification
     *************************************************************************/
    /* Habilitar Port D para debug */
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;
    while(!(SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R3)) {}

    /* PD6 para indicar Timer3 interrupções */
    GPIO_PORTD_DIR_R |= GPIO_PIN_6;
    GPIO_PORTD_DEN_R |= GPIO_PIN_6;
    GPIO_PORTD_DATA_R &= ~GPIO_PIN_6;

    /* PD7 para indicar WTimer1 interrupções */
    GPIO_PORTD_DIR_R |= GPIO_PIN_7;
    GPIO_PORTD_DEN_R |= GPIO_PIN_7;
    GPIO_PORTD_DATA_R &= ~GPIO_PIN_7;
}


/*******************************************************************************
 * FUNCTION: pso_pwm_config
 * 
 * DESCRIPTION:
 *     Configures PWM output for brushless motor control via ESC.
 *     Generates 50Hz PWM signal (20ms period) with 1-2ms pulse width range.
 *     Uses WTimer1B on PC7 (WT1CCP1).
 * 
 * PARAMETERS:
 *     None
 * 
 * RETURNS:
 *     void
 * 
 * PWM CHARACTERISTICS:
 *     - Frequency: 50 Hz (20ms period)
 *     - Pulse Width Range: 1ms (min) to 2ms (max)
 *     - Inverted Output: High when timer matches, low otherwise
 *     - System Clock: 40 MHz (25ns period)
 * 
 * CALCULATIONS:
 *     - 20ms period: 20ms / 25ns = 800,000 counts (0x000C3500)
 *     - 1ms pulse: 1ms / 25ns = 40,000 counts (0x00009C40)
 *     - 1.5ms pulse: 60,000 counts (0x0000EA60)
 *     - 2ms pulse: 80,000 counts (0x00013880)
 * 
 * SAFETY:
 *     Starts at minimum pulse width (1ms) for safety
 *******************************************************************************/
void pso_pwm_config()
{
    /**************************************************************************
     * 0. GPIO Configuration for PWM output on PC7 (WT1CCP1)
     *************************************************************************/
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2;        /* Enable Port C clock */
    GPIO_PORTC_DIR_R |= GPIO_PIN_7;                 /* PC7 as output */
    GPIO_PORTC_AFSEL_R |= GPIO_PIN_7;               /* Alternate function */
    GPIO_PORTC_DEN_R |= GPIO_PIN_7;                 /* Enable digital */
    GPIO_PORTC_AMSEL_R &= ~(GPIO_PIN_7);            /* Disable analog */
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC7_WT1CCP1;     /* Port mux for WT1CCP1 */
    
    /**************************************************************************
     * 1. PWM Timer Configuration (WTimer1B)
     *************************************************************************/
    
    /* Enable Wide Timer1 clock */
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R1;
    
    /* Disable timer before configuration */
    WTIMER1_CTL_R &= ~(TIMER_CTL_TBEN);
    
    /* Configure as 16-bit timer (for PWM generation) */
    // WTIMER1_CFG_R |= TIMER_CFG_16_BIT;
    
    /* Configure PWM mode and periodic timer mode */
    WTIMER1_TBMR_R |= (TIMER_TBMR_TBAMS |           /* PWM mode enabled */
                       TIMER_TBMR_TBMR_PERIOD);     /* Periodic timer mode */
    
    /* Configure inverted output (high when match, low otherwise) */
    WTIMER1_CTL_R |= TIMER_CTL_TBPWML;
    
    /* Set PWM period: 20ms = 800,000 counts at 40MHz */
    WTIMER1_TBILR_R = 0x000C3500;                   /* 800,000 = 0x000C3500 */
    
    /* Set initial pulse width to minimum (1ms = 40,000 counts) for safety */
    WTIMER1_TBMATCHR_R = 0x00009C40;                /* 40,000 = 0x00009C40 */
    
    /* Enable Timer B to start PWM generation */
    WTIMER1_CTL_R |= TIMER_CTL_TBEN;
}

/*******************************************************************************
 * FUNCTION: pso_spi0_config
 * 
 * DESCRIPTION:
 *     Configures SPI0 (SSI0) as master for SD card communication.
 *     Uses SPI mode 0 (CPOL=0, CPHA=0) at 400 kHz.
 *     Manual chip select on PB2 (not using SSI0Fss).
 * 
 * PARAMETERS:
 *     None
 * 
 * RETURNS:
 *     void
 * 
 * PIN CONFIGURATION:
 *     - PA2: SSI0Clk (SCK)
 *     - PA3: SSI0Fss (CS - configured but not used)
 *     - PA4: SSI0Rx (MISO)
 *     - PA5: SSI0Tx (MOSI)
 *     - PB2: Manual chip select (CS)
 * 
 * SPI CONFIGURATION:
 *     - Mode: Master
 *     - Clock: 400 kHz (40MHz / (50 * (1+1)))
 *     - Data: 8-bit
 *     - Phase: 0 (data captured on first clock edge)
 *     - Polarity: 0 (clock low when idle)
 *     - Frame Format: Motorola (Freescale) SPI
 * 
 * NOTES:
 *     - Uses manual chip select on PB2 instead of SSI0Fss
 *     - FatFS requires SPI mode 0 for MMC/SDC cards
 *******************************************************************************/
void pso_spi0_config()
{
    volatile uint32_t delay;
    
    /**************************************************************************
     * Manual Chip Select Configuration (PB2)
     *************************************************************************/
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;        /* Enable Port B */
    delay = SYSCTL_RCGCGPIO_R;                      /* Clock stabilization */
    
    GPIO_PORTB_PUR_R |= GPIO_PIN_2;                 /* Enable weak pullup on PB2 */
    GPIO_PORTB_DIR_R |= GPIO_PIN_2;                 /* PB2 as output (CS) */
    GPIO_PORTB_DR4R_R |= GPIO_PIN_2;                /* 4mA drive strength */
    GPIO_PORTB_AMSEL_R &= ~GPIO_PIN_2;              /* Disable analog */
    GPIO_PORTB_DEN_R |= GPIO_PIN_2;                 /* Enable digital I/O */
    
    /**************************************************************************
     * SPI0 Module Initialization
     *************************************************************************/
    
    /* Enable SPI0 clock */
    SYSCTL_RCGCSSI_R |= SYSCTL_SCGCSSI_S0;          /* Enable SSI Module 0 */
    
    /* Enable GPIO Port A (SPI pins) */
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;        /* Enable Port A */
    delay = SYSCTL_RCGC2_R;                         /* Clock stabilization */
    
    /**************************************************************************
     * GPIO Configuration for SPI0 pins
     *************************************************************************/
    GPIO_PORTA_DIR_R &= ~(GPIO_PIN_4);              /* PA4 (MISO) as input */
    GPIO_PORTA_DIR_R |= (GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5); /* Others as output */
    
    GPIO_PORTA_AFSEL_R |= (GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);
    GPIO_PORTA_DEN_R |= (GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);
    GPIO_PORTA_PUR_R |= (GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5); /* Pull-up on outputs */
    GPIO_PORTA_DR4R_R |= (GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);
    GPIO_PORTA_AMSEL_R &= ~(GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);
    
    /* Configure pin mux for SPI function */
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA2_SSI0CLK;     /* PA2 = SSI0Clk */
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA3_SSI0FSS;     /* PA3 = SSI0FSS */
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA4_SSI0RX;      /* PA4 = SSI0Rx (MISO) */
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA5_SSI0TX;      /* PA5 = SSI0Tx (MOSI) */
    
    /**************************************************************************
     * SPI0 Module Configuration
     *************************************************************************/
    
    /* Disable SPI before configuration */
    SSI0_CR1_R &= ~(SSI_CR1_SSE);
    
    /* Configure as master */
    SSI0_CR1_R &= ~SSI_CR1_MS;
    
    /* Use system PLL clock */
    SSI0_CC_R = SSI_CC_CS_SYSPLL;
    
    /* Set clock prescaler: CPSDVSR = 50 */
    SSI0_CPSR_R |= 0x00000032;                      /* 50 = 0x32 */
    
    /* Configure control register:
     * - SCR = 1 (serial clock rate)
     * - FRF = Motorola (Freescale) SPI
     * - DSS = 8-bit data
     * - SPO = 0, SPH = 0 (SPI mode 0)
     */
    SSI0_CR0_R = (SSI0_CR0_R & ~SSI_CR0_SCR_M) + 0x00000100; /* SCR = 1 */
    SSI0_CR0_R |= (SSI_CR0_FRF_MOTO |              /* Freescale SPI */
                   SSI_CR0_DSS_8);                 /* 8-bit data */
    SSI0_CR0_R &= ~(SSI_CR0_SPO | SSI_CR0_SPH);    /* CPOL=0, CPHA=0 */
    
    /* Enable SPI0 */
    SSI0_CR1_R |= SSI_CR1_SSE;
}

/*******************************************************************************
 * FUNCTION: pso_spi2_config
 * 
 * DESCRIPTION:
 *     Configures SPI2 (SSI2) as master for alternative SD card communication.
 *     Uses SPI mode 0 (CPOL=0, CPHA=0) at 400 kHz.
 *     Manual chip select on PA3 (not using SSI2Fss).
 * 
 * PARAMETERS:
 *     None
 * 
 * RETURNS:
 *     void
 * 
 * PIN CONFIGURATION:
 *     - PB4: SSI2Clk (SCK)
 *     - PB5: SSI2Fss (CS - configured but not used)
 *     - PB6: SSI2Rx (MISO)
 *     - PB7: SSI2Tx (MOSI)
 *     - PA3: Manual chip select (CS)
 * 
 * SPI CONFIGURATION:
 *     - Mode: Master
 *     - Clock: 400 kHz (40MHz / (10 * (1+9)))
 *     - Data: 8-bit
 *     - Phase: 0 (data captured on first clock edge)
 *     - Polarity: 0 (clock low when idle)
 *     - Frame Format: Motorola (Freescale) SPI
 * 
 * NOTES:
 *     - Alternative configuration to SPI0
 *     - Different clock divider settings (CPSDVSR=10, SCR=9)
 *******************************************************************************/
void pso_spi2_config()
{
    volatile uint32_t delay;
    
    /**************************************************************************
     * Manual Chip Select Configuration (PA3)
     *************************************************************************/
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;        /* Enable Port A */
    delay = SYSCTL_RCGCGPIO_R;                      /* Clock stabilization */
    
    GPIO_PORTA_PUR_R |= GPIO_PIN_3;                 /* Enable weak pullup on PA3 */
    GPIO_PORTA_DIR_R |= GPIO_PIN_3;                 /* PA3 as output (CS) */
    GPIO_PORTA_DR4R_R |= GPIO_PIN_3;                /* 4mA drive strength */
    GPIO_PORTA_PCTL_R &= ~GPIO_PCTL_PA3_M;          /* Clear port control */
    GPIO_PORTA_AMSEL_R &= ~GPIO_PIN_3;              /* Disable analog */
    GPIO_PORTA_DEN_R |= GPIO_PIN_3;                 /* Enable digital I/O */
    
    /**************************************************************************
     * SPI2 Module Initialization
     *************************************************************************/
    
    /* Enable SPI2 clock */
    SYSCTL_RCGCSSI_R |= SYSCTL_SCGCSSI_S2;          /* Enable SSI Module 2 */
    
    /* Enable GPIO Port B (SPI pins) */
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;        /* Enable Port B */
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R1) == 0){}; /* Wait for ready */
    
    /**************************************************************************
     * GPIO Configuration for SPI2 pins
     *************************************************************************/
    GPIO_PORTB_DIR_R &= ~(GPIO_PIN_6);              /* PB6 (MISO) as input */
    GPIO_PORTB_DIR_R |= (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7); /* Others as output */
    
    GPIO_PORTB_AFSEL_R |= (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
    GPIO_PORTB_DEN_R |= (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
    GPIO_PORTB_PUR_R |= (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7); /* Pull-up on outputs */
    GPIO_PORTB_DR4R_R |= (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
    GPIO_PORTB_AMSEL_R &= ~(GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
    
    /* Configure pin mux for SPI function */
    GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB4_SSI2CLK;     /* PB4 = SSI2Clk */
    GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB5_SSI2FSS;     /* PB5 = SSI2Fss */
    GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB6_SSI2RX;      /* PB6 = SSI2Rx (MISO) */
    GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB7_SSI2TX;      /* PB7 = SSI2Tx (MOSI) */
    
    /**************************************************************************
     * SPI2 Module Configuration
     *************************************************************************/
    
    /* Disable SPI before configuration */
    SSI2_CR1_R &= ~(SSI_CR1_SSE);
    
    /* Configure as master */
    SSI2_CR1_R &= ~SSI_CR1_MS;
    
    /* Use system PLL clock */
    SSI2_CC_R = SSI_CC_CS_SYSPLL;
    
    /* Set clock prescaler: CPSDVSR = 10 */
    SSI2_CPSR_R |= 0x0000000A;                      /* 10 = 0x0A */
    
    /* Configure control register:
     * - SCR = 9 (serial clock rate)
     * - FRF = Motorola (Freescale) SPI
     * - DSS = 8-bit data
     * - SPO = 0, SPH = 0 (SPI mode 0)
     */
    SSI2_CR0_R |= ((SSI_CR0_SCR_M & 0x00000900) |   /* SCR = 9 */
                   SSI_CR0_FRF_MOTO |               /* Freescale SPI */
                   SSI_CR0_DSS_8);                  /* 8-bit data */
    SSI2_CR0_R &= ~(SSI_CR0_SPO | SSI_CR0_SPH);     /* CPOL=0, CPHA=0 */
    
    /* Enable SPI2 */
    SSI2_CR1_R |= SSI_CR1_SSE;
}
