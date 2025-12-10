/*******************************************************************************
 * FILE:        pso_isr.c
 *
 * DESCRIPTION:
 *     PSO Interrupt Service Routines (ISRs) Module.
 *     Contains interrupt handlers for UART, timers, and ADCs used in the
 *     Propeller Speed Optimizer (PSO) system. Handles real-time data
 *     acquisition, RPM calculation, and system timing.
 *
 * DOCUMENTATION STYLE:
 *     - Technical and functional
 *     - No functional or logical modifications
 *     - Improved comments and formatting only
 *
 * AUTHOR:      Rogerio Lima
 * REFORMAT:    2025 (Documentation and formatting only)
 *
 *******************************************************************************/

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
#include "pso_pwm.h" /* Function generator - inc/dec functions */
#include "pso_timing.h"
#include "fifo.h"
#include "pso_data.h"
#include "ulink.h"
#include "ulink_pso.h"
#include "ulink_types.h"
#include "pso_debug.h"
#include "pso_rpm.h"

/*******************************************************************************
 * GLOBAL VARIABLES (External Declarations)
 *******************************************************************************/
extern g_led_toggle_flag;                    /* Toggle LED flag - external */
extern uart_raw_data_t g_uart0_data;         /* UART0 receive buffer - external */

/*******************************************************************************
 * MODULE-SPECIFIC GLOBAL VARIABLES
 *******************************************************************************/
uint8_t g_timer_a0_scan_flag = 0U;           /* Timer0A scan completion flag */
volatile uint32_t g_timer_a3_scan_flag = 0U; /* Timer3A scan flag (RPM ready) */

/*******************************************************************************
 * ADC DATA BUFFERS
 *     These buffers store raw ADC values from both ADC modules.
 *     Buffer indices correspond to specific sensor channels:
 *       0: Ax (Acceleration X-axis) / Ay (Acceleration Y-axis)
 *       1: Strain Gauge / Az (Acceleration Z-axis)
 *       2: Motor Voltage / Motor Current
 *******************************************************************************/
volatile uint32_t adc0_buffer[3];            /* ADC0 channel data buffer */
volatile uint32_t adc1_buffer[3];            /* ADC1 channel data buffer */

/*******************************************************************************
 * TIMING AND MEASUREMENT VARIABLES
 *******************************************************************************/
uint32_t wt1cpp0_tav_buffer;                 /* Wide Timer 1 capture buffer */
uint32_t g_pulse_diff;                       /* Pulse difference for RPM calc */

/*******************************************************************************
 * FUNCTION: UART0IntHandler
 *
 * DESCRIPTION:
 *     Interrupt handler for UART0 receive operations.
 *     Reads available characters from UART0 receive FIFO and stores them
 *     in the global UART data buffer. Sets new_data flag when data is received.
 *
 * PARAMETERS:
 *     None
 *
 * RETURNS:
 *     void
 *
 * OPERATION:
 *     1. Reads and clears UART interrupt status
 *     2. Reads all available characters from receive FIFO
 *     3. Stores characters in circular buffer
 *     4. Sets new_data flag for main loop processing
 *
 * NOTES:
 *     - Non-blocking operation
 *     - Buffer overflow protection needed in main loop
 *     - Interrupt-driven for minimal CPU usage
 *******************************************************************************/
void UART0IntHandler(void)
{
    uint32_t ui32Status;

    /* 1. Get interrupt status and clear interrupts */
    ui32Status = UARTIntStatus(UART0_BASE, true);
    UARTIntClear(UART0_BASE, ui32Status);

    /* 2. Read all available characters from receive FIFO */
    while (UARTCharsAvail(UART0_BASE))
    {
        /* Store character in buffer and increment index */
        g_uart0_data.rx_buffer[g_uart0_data.rx_index++] = HWREG(UART0_BASE + UART_O_DR);
    }

    /* 3. Set flag to indicate new data is available */
    g_uart0_data.new_data = 1;
}

/*******************************************************************************
 * FUNCTION: Timer0AIntHandler
 *
 * DESCRIPTION:
 *     Timer0A interrupt handler (currently minimal implementation).
 *     Clears the timer interrupt flag. Can be expanded for periodic tasks.
 *
 * PARAMETERS:
 *     None
 *
 * RETURNS:
 *     void
 *
 * NOTES:
 *     - Basic implementation - can be extended for system timing
 *     - Ensure Timer0 is configured with appropriate period
 *******************************************************************************/
void Timer0AIntHandler(void)
{
    /* Clear the timer interrupt flag */
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

/*******************************************************************************
 * FUNCTION: WTimer1AIntHandler
 *
 * DESCRIPTION:
 *     Wide Timer 1A interrupt handler - error handler only.
 *     This handler should NOT be called as WTimer1 interrupts are disabled.
 *     If triggered, indicates a configuration error or spurious interrupt.
 *
 * PARAMETERS:
 *     None
 *
 * RETURNS:
 *     void
 *
 * ERROR HANDLING:
 *     - Clears any spurious interrupt
 *     - Can be extended to set error flag/counter
 *
 * NOTES:
 *     - This is a safety handler only
 *     - Consider adding error logging if called
 *******************************************************************************/
void WTimer1AIntHandler(void)
{
    /* Clear any spurious interrupt */
    WTIMER1_ICR_R = TIMER_ICR_CAMCINT;
    
    /* Optional: Add error handling if this gets called */
    /* Example: g_wtimer1_error_count++; */
}

/*******************************************************************************
 * FUNCTION: WTimer1BIntHandler
 *
 * DESCRIPTION:
 *     Wide Timer 1B interrupt handler.
 *     Clears the capture event interrupt flag. Used for pulse timing/capture.
 *
 * PARAMETERS:
 *     None
 *
 * RETURNS:
 *     void
 *
 * NOTES:
 *     - Ensure WTimer1B is properly configured for capture mode
 *     - Consider adding pulse measurement logic here
 *******************************************************************************/
void WTimer1BIntHandler(void)
{
    /* Clear the timer capture interrupt */
    TimerIntClear(WTIMER1_BASE, TIMER_CAPB_EVENT);
}

/*******************************************************************************
 * FUNCTION: WTimer5AIntHandler
 *
 * DESCRIPTION:
 *     Wide Timer 5A interrupt handler.
 *     Clears the capture event interrupt flag.
 *
 * PARAMETERS:
 *     None
 *
 * RETURNS:
 *     void
 *
 * NOTES:
 *     - Basic implementation
 *     - Can be extended for system timing/capture functions
 *******************************************************************************/
void WTimer5AIntHandler(void)
{
    /* Clear the timer capture interrupt */
    TimerIntClear(WTIMER5_BASE, TIMER_CAPA_EVENT);
}

/*******************************************************************************
 * FUNCTION: WTimer5BIntHandler
 *
 * DESCRIPTION:
 *     Wide Timer 5B interrupt handler.
 *     Clears the capture event interrupt flag.
 *
 * PARAMETERS:
 *     None
 *
 * RETURNS:
 *     void
 *
 * NOTES:
 *     - Basic implementation
 *     - Can be extended for system timing/capture functions
 *******************************************************************************/
void WTimer5BIntHandler(void)
{
    /* Clear the timer capture interrupt */
    TimerIntClear(WTIMER5_BASE, TIMER_CAPB_EVENT);
}

/*******************************************************************************
 * FUNCTION: Timer3AIntHandler
 *
 * DESCRIPTION:
 *     Timer3A interrupt handler - called every 100ms (10 Hz).
 *     Primary function is RPM calculation based on pulse counts from WTimer1A.
 *     Also handles periodic system tasks like PWM control and LED indication.
 *
 * PARAMETERS:
 *     None
 *
 * RETURNS:
 *     void
 *
 * RPM CALCULATION:
 *     Formula: RPM = (pulse_diff * 600) / BLADE_NUMBER
 *     Where:
 *       - pulse_diff = pulses counted in 100ms interval
 *       - BLADE_NUMBER = pulses per revolution
 *       - 600 = conversion factor (100ms → min: *10 *60)
 *
 * OPERATION:
 *     1. Clear timer interrupt flag
 *     2. Read current pulse count from WTimer1A
 *     3. Calculate pulse difference (handles 32-bit overflow)
 *     4. Store pulse difference for main loop RPM calculation
 *     5. Update last count reference
 *     6. Set flag indicating new RPM data available
 *     7. Execute periodic system functions
 *
 * NOTES:
 *     - Runs at 10Hz (100ms intervals)
 *     - Handles 32-bit counter overflow correctly
 *     - RPM calculation completed in main loop using g_pulse_diff
 *     - Toggles LED and debug pins for visual feedback
 *******************************************************************************/
void Timer3AIntHandler(void)
{
    static uint32_t last_rpm_count = 0;  /* Previous pulse count */
    uint32_t current_count;              /* Current pulse count */
    uint32_t pulse_diff;                 /* Pulses in last 100ms */

    /* 1. Clear the timer interrupt flag FIRST (critical) */
    TIMER3_ICR_R |= TIMER_ICR_TATOCINT;

    /* 2. Read current pulse count from Wide Timer 1A */
    current_count = WTIMER1_TAV_R;

    /* 3. Calculate pulse difference with 32-bit overflow handling */
    if (current_count >= last_rpm_count)
    {
        /* Normal case: no counter overflow */
        pulse_diff = current_count - last_rpm_count;
    }
    else
    {
        /* Counter overflow occurred (wrapped from 0xFFFFFFFF to 0) */
        pulse_diff = (0xFFFFFFFF - last_rpm_count) + current_count + 1;
    }

    /* 4. Store pulse difference for main loop RPM calculation */
    g_pulse_diff = pulse_diff;

    /* 5. Update reference count for next calculation */
    last_rpm_count = current_count;

    /* 6. Set flag to indicate new RPM value is available */
    g_timer_a3_scan_flag ^= 0xFF;  /* Toggle flag for main loop detection */

    /* 7. Visual feedback: toggle LED indicator */
    g_led_toggle_flag ^= 0xFF;

    /* 8. Optional debug pin toggling (scope visualization) */
    /* DEBUG_ADC_TOGGLE(); */   /* PD6 - ADC timing indicator */
    /* DEBUG_STATE_TOGGLE(); */  /* PD7 - State machine indicator */

    /* 9. Execute periodic PWM control function */
    increment();  /* PWM duty cycle adjustment */
}

/*******************************************************************************
 * FUNCTION: ADC0SS1IntHandler
 *
 * DESCRIPTION:
 *     ADC0 Sample Sequencer 1 interrupt handler.
 *     Reads 3-channel ADC data from FIFO into buffer.
 *     Channels: PD1 (AIN6 - Ax), PD0 (AIN7 - Strain Gauge), PE1 (AIN2 - Vmotor)
 *
 * PARAMETERS:
 *     None
 *
 * RETURNS:
 *     void
 *
 * CHANNEL MAPPING:
 *     adc0_buffer[0]: PD1 (AIN6) - Ax acceleration
 *     adc0_buffer[1]: PD0 (AIN7) - Strain Gauge
 *     adc0_buffer[2]: PE1 (AIN2) - Motor Voltage
 *
 * OPERATION:
 *     1. Read 3 ADC samples from FIFO
 *     2. Simultaneously read from ADC1 for synchronization
 *     3. Clear ADC interrupt flag
 *     4. Set scan completion flag for main loop
 *
 * NOTES:
 *     - Hardware averaging may be enabled in ADC configuration
 *     - Data is raw ADC values (0-4095 for 12-bit)
 *     - Channel order matches ADC sequence configuration
 *******************************************************************************/
void ADC0SS1IntHandler(void)
{
    uint8_t k = 0U;

    /* 1. Read 3-channel data from both ADC FIFOs */
    for (k = 0U; k < 3U; k++)
    {
        /* ADC0 channels */
        adc0_buffer[k] = ADC0_SSFIFO1_R;  /* Channel order: Ax, Strain, Vmotor */

        /* ADC1 channels (read simultaneously for time alignment) */
        adc1_buffer[k] = ADC1_SSFIFO1_R;  /* Channel order: Ay, Az, Imotor */
    }

    /* 2. Acknowledge Sample Sequencer 1 interrupt */
    ADC0_ISC_R = ADC_ISC_IN1;

    /* 3. Set flag to indicate ADC scan is complete */
    g_timer_a0_scan_flag = 1U;

    /* 4. Optional debug pin toggle for timing measurement */
    /* DEBUG_STATE_TOGGLE(); */  /* PD7 - State indicator */

    /* 5. Optional LED indicator */
    /* GPIO_PORTF_DATA_R ^= GPIO_PIN_2; */  /* Blue LED on PF2 */
}

/*******************************************************************************
 * FUNCTION: ADC1SS1IntHandler
 *
 * DESCRIPTION:
 *     ADC1 Sample Sequencer 1 interrupt handler.
 *     Reads 3-channel ADC data from FIFO into buffer.
 *     Channels: PD2 (AIN5 - Ay), PD3 (AIN4 - Az), PE2 (AIN1 - Imotor)
 *
 * PARAMETERS:
 *     None
 *
 * RETURNS:
 *     void
 *
 * CHANNEL MAPPING:
 *     adc1_buffer[0]: PD2 (AIN5) - Ay acceleration
 *     adc1_buffer[1]: PD3 (AIN4) - Az acceleration
 *     adc1_buffer[2]: PE2 (AIN1) - Motor Current
 *
 * NOTES:
 *     - This handler is called but data may also be read in ADC0 handler
 *     - Ensure proper interrupt clearing to prevent stuck interrupts
 *     - Consider merging with ADC0 handler if simultaneous sampling needed
 *******************************************************************************/
void ADC1SS1IntHandler(void)
{
    /* 1. Acknowledge Sample Sequencer 1 interrupt FIRST */
    ADC1_ISC_R = ADC_ISC_IN1;

    /* 2. Read 3-channel data from ADC1 FIFO */
    adc1_buffer[0] = ADC1_SSFIFO1_R;  /* Ay acceleration */
    adc1_buffer[1] = ADC1_SSFIFO1_R;  /* Az acceleration */
    adc1_buffer[2] = ADC1_SSFIFO1_R;  /* Motor Current */
}

/*******************************************************************************
 * INTERRUPT HANDLER SUMMARY:
 *
 * Priority (Typical, depends on NVIC configuration):
 *     1. ADC0/1 (High priority - time-critical data acquisition)
 *     2. Timer3A (Medium priority - RPM calculation at 10Hz)
 *     3. UART0 (Medium priority - command processing)
 *     4. Other timers (Low priority - system timing)
 *
 * Safety Considerations:
 *     - Keep ISRs short and efficient
 *     - Clear interrupt flags early in handler
 *     - Use volatile for shared variables
 *     - Consider interrupt nesting and priority
 *
 * Debug Features:
 *     - Debug pins PD6/PD7 can be toggled in ISRs for timing measurement
 *     - LED indicators provide visual system status
 *     - Consider adding ISR execution time measurement
 *******************************************************************************/