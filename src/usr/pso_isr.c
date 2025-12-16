/*******************************************************************************
 * FILE:        pso_isr.c
 *
 * DESCRIPTION:
 *     PSO Interrupt Service Routines (ISRs) Module.
 *     Contains interrupt handlers for UART, timers, and ADCs used in the
 *     Propeller Speed Optimizer (PSO) system. Handles real-time data
 *     acquisition, RPM calculation, and system timing.
 *
 *     Supports two RPM measurement methods selectable via compile-time directive:
 *       - RPM_EDGE_COUNT_METHOD: Counting method (default)
 *       - RPM_EDGE_PERIOD_METHOD: Period measurement method
 *
 * AUTHOR:      Rogerio Lima
 * CONSOLIDATED: December 2025
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
#include "pso_pwm.h"
#include "pso_timing.h"
#include "fifo.h"
#include "pso_data.h"
#include "ulink.h"
#include "ulink_pso.h"
#include "ulink_types.h"
#include "pso_debug.h"
#include "pso_rpm.h"

/*******************************************************************************
 * COMPILE-TIME METHOD SELECTION
 ******************************************************************************/
#if !defined(RPM_EDGE_COUNT_METHOD) && !defined(RPM_EDGE_PERIOD_METHOD)
    #define RPM_EDGE_COUNT_METHOD  /* Default to edge counting */
#endif

#if defined(RPM_EDGE_COUNT_METHOD) && defined(RPM_EDGE_PERIOD_METHOD)
    #error "Only one RPM measurement method can be defined!"
#endif

/*******************************************************************************
 * GLOBAL VARIABLES (External Declarations)
 *******************************************************************************/
extern uint8_t g_led_toggle_flag;            /* Toggle LED flag */
extern uart_raw_data_t g_uart0_data;         /* UART0 receive buffer */

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

#ifdef RPM_EDGE_COUNT_METHOD
/*-----------------------------------------------------------------------------
 * EDGE COUNT METHOD - VARIABLES
 *---------------------------------------------------------------------------*/
extern uint32_t g_pulse_diff;                       /* Pulse difference for RPM calc */

#else /* RPM_EDGE_PERIOD_METHOD */
/*-----------------------------------------------------------------------------
 * EDGE PERIOD METHOD - VARIABLES
 *---------------------------------------------------------------------------*/
extern uint32_t g_edge_interval_us;          /* Period between edges in μs */
extern uint32_t g_last_edge_time_us;         /* Time of last edge in μs */
extern uint32_t g_last_capture_value;        /* Last timer capture value */
extern uint32_t g_edge_valid_count;          /* Valid edges counter */
extern uint32_t g_edge_timeout_counter;      /* Timeout for stopped motor */

#endif /* RPM_EDGE_COUNT_METHOD / RPM_EDGE_PERIOD_METHOD */

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
 * RPM MEASUREMENT ISRs - METHOD-DEPENDENT IMPLEMENTATION
 ******************************************************************************/

#ifdef RPM_EDGE_COUNT_METHOD
/*=============================================================================
 * EDGE COUNT METHOD - ISR IMPLEMENTATION
 *===========================================================================*/

/*******************************************************************************
 * FUNCTION: WTimer1AIntHandler
 *
 * DESCRIPTION:
 *     Wide Timer 1A interrupt handler - ERROR HANDLER ONLY.
 *     This handler should NOT be called as WTimer1A interrupts are DISABLED
 *     in edge count mode. If triggered, indicates a configuration error.
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
    WTIMER1_ICR_R = TIMER_ICR_CAECINT;
    
    /* Optional: Add error handling if this gets called */
    /* Example: g_wtimer1_error_count++; */
}

/*******************************************************************************
 * FUNCTION: Timer3AIntHandler (EDGE COUNT METHOD)
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
 *       - 600 = conversion factor (100ms → min: ×10 ×60)
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

#else /* RPM_EDGE_PERIOD_METHOD */
/*=============================================================================
 * EDGE PERIOD METHOD - ISR IMPLEMENTATION
 *===========================================================================*/

/*******************************************************************************
 * FUNCTION: WTimer1AIntHandler (EDGE PERIOD METHOD)
 *
 * DESCRIPTION:
 *     Wide Timer 1A edge capture interrupt handler for RPM measurement.
 *     Triggered on each rising edge at PC6 (WT1CCP0).
 *     Calculates period between consecutive edges with 25ns resolution.
 *
 * PARAMETERS:
 *     None
 *
 * RETURNS:
 *     void
 *
 * OPERATION:
 *     1. Read current timer capture value
 *     2. Calculate period since last edge (handles overflow)
 *     3. Convert timer ticks to microseconds (40 ticks = 1 μs)
 *     4. Validate period (noise filter + range check)
 *     5. Calculate RPM: 60,000,000 / (period_μs × BLADE_NUMBER)
 *     6. Update filter and set ready flag
 *     7. Reset timeout counter
 *     8. Clear interrupt flag
 *
 * RPM FORMULA:
 *     RPM = 60,000,000 / (period_μs × BLADE_NUMBER)
 *     
 *     Example (1000 RPM, BLADE_NUMBER=2):
 *       Period = 60,000,000 / (1000 × 2) = 30,000 μs = 30 ms
 *       Verify: 60,000,000 / (30,000 × 2) = 1000 RPM ✓
 *
 * NOTES:
 *     - ISR execution time: 5-8 μs
 *     - Instant RPM update (no 100ms delay)
 *     - Handles 32-bit timer overflow
 *     - Filters noise with MIN_EDGE_INTERVAL_US
 *******************************************************************************/
void WTimer1AIntHandler(void)
{
    uint32_t current_capture;
    uint32_t period_ticks;
    uint32_t period_us;
    
    /* 1. Get current timer capture value (32-bit, free-running) */
    current_capture = WTIMER1_TAR_R;
    
    /* 2. Calculate period since last edge (handle timer overflow) */
    if (g_last_capture_value > 0)
    {
        if (current_capture >= g_last_capture_value)
        {
            /* Normal case - no overflow */
            period_ticks = current_capture - g_last_capture_value;
        }
        else
        {
            /* Timer overflow occurred (wrapped from 0xFFFFFFFF to 0) */
            period_ticks = (0xFFFFFFFF - g_last_capture_value) + current_capture + 1;
        }
        
        /* 3. Convert ticks to microseconds */
        /* 40 MHz clock = 25 ns/tick = 40 ticks/μs */
        /* Add 20 for rounding: (ticks + 20) / 40 */
        period_us = (period_ticks + 20) / 40;
        
        /* 4. Validate period (filter noise and invalid readings) */
        if (period_us >= MIN_EDGE_INTERVAL_US && 
            period_us <= (MAX_EDGE_INTERVAL_MS * 1000))
        {
            /* Store valid period */
            g_edge_interval_us = period_us;
            g_edge_valid_count++;
            g_edge_timeout_counter = 0;  /* Reset timeout counter */
            
            /* 5. Calculate RPM directly */
            /* RPM = 60,000,000 / (period_μs × BLADE_NUMBER) */
            if (period_us > 0)
            {
                g_rpm_value = 60000000UL / (period_us * BLADE_NUMBER);
                
                /* 6. Update moving average filter */
                rpm_update_filter(g_rpm_value);
                
                /* Signal new RPM available */
                g_rpm_ready_flag ^= 0xFF;
            }
        }
    }
    
    /* 7. Update last capture value for next edge */
    g_last_capture_value = current_capture;
    
    /* 8. Clear interrupt flag */
    WTIMER1_ICR_R = TIMER_ICR_CAECINT;
}

/*******************************************************************************
 * FUNCTION: Timer3AIntHandler (EDGE PERIOD METHOD)
 *
 * DESCRIPTION:
 *     Timer3A interrupt handler - called every 100ms (10 Hz).
 *     In edge-period mode, this handler only performs timeout detection
 *     and periodic system tasks. RPM calculation is done in WTimer1A ISR.
 *
 * PARAMETERS:
 *     None
 *
 * RETURNS:
 *     void
 *
 * OPERATION:
 *     1. Clear timer interrupt flag
 *     2. Increment timeout counter
 *     3. Check for motor stopped condition (no edges detected)
 *     4. Execute periodic PWM control
 *     5. Toggle LED for visual feedback
 *
 * NOTES:
 *     - Simpler than edge count version
 *     - Only handles timeout and periodic tasks
 *     - RPM calculation moved to WTimer1A ISR
 *******************************************************************************/
void Timer3AIntHandler(void)
{
    /* 1. Clear the timer interrupt flag FIRST */
    TIMER3_ICR_R |= TIMER_ICR_TATOCINT;
    
    /* 2. Increment timeout counter (100ms per interrupt) */
    g_edge_timeout_counter += RPM_CALC_PERIOD_MS;
    
    /* 3. Check for motor stopped condition */
    if (g_edge_timeout_counter >= RPM_STOP_TIMEOUT_MS)
    {
        /* No edges detected for timeout period - motor stopped */
        g_rpm_value = 0;
        g_edge_interval_us = 0;
        g_rpm_ready_flag ^= 0xFF;  /* Signal RPM update (to 0) */
    }
    
    /* 4. Execute periodic PWM control */
    increment();
    
    /* 5. Toggle LED for visual feedback */
    g_led_toggle_flag ^= 0xFF;
    
    /* 6. Optional debug pin toggling */
    /* DEBUG_STATE_TOGGLE(); */
}

#endif /* RPM_EDGE_COUNT_METHOD / RPM_EDGE_PERIOD_METHOD */

/*******************************************************************************
 * COMMON ISR HANDLERS (Independent of RPM Method)
 ******************************************************************************/

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
 * ISR SUMMARY AND NOTES
 ******************************************************************************/

/**
 * INTERRUPT PRIORITY (typical NVIC configuration):
 * 
 * Priority 0 (Highest):
 *   - SysTick (1ms system tick)
 * 
 * Priority 1 (High):
 *   - ADC0 SS1 (time-critical data acquisition)
 *   - ADC1 SS1 (time-critical data acquisition)
 * 
 * Priority 2 (Medium):
 *   - WTimer1A (RPM edge capture - PERIOD METHOD ONLY)
 *   - Timer3A (RPM calculation or timeout detection)
 * 
 * Priority 3 (Medium):
 *   - UART0 (command processing)
 * 
 * Priority 4+ (Low):
 *   - Other timers (system timing)
 * 
 * 
 * METHOD-SPECIFIC ISR BEHAVIOR:
 * 
 * EDGE COUNT METHOD:
 *   - WTimer1A: NO interrupt (polling-based counter)
 *   - Timer3A: Calculates RPM every 100ms
 *   - CPU load: Low (~0.01%)
 *   - Latency: 100ms fixed
 * 
 * EDGE PERIOD METHOD:
 *   - WTimer1A: Interrupt per edge, calculates RPM
 *   - Timer3A: Timeout detection only
 *   - CPU load: Variable (~0.5% @ 1000 RPM)
 *   - Latency: Instant (per edge)
 * 
 * 
 * SAFETY CONSIDERATIONS:
 *   - Keep ISRs short and efficient
 *   - Clear interrupt flags early in handler
 *   - Use volatile for shared variables
 *   - Consider interrupt nesting and priority
 *   - Test with worst-case interrupt rates
 * 
 * 
 * DEBUG FEATURES:
 *   - Debug pins PD4-PD7 can be toggled in ISRs
 *   - LED indicators provide visual system status
 *   - Consider adding ISR execution time measurement
 *   - Use oscilloscope for timing verification
 */
