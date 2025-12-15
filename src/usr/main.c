/*******************************************************************************
 * FILE:        main.c
 *
 * DESCRIPTION:
 *     PSO Embedded System Module.
 *     This file is part of the PSO real-time data acquisition and PWM control
 *     firmware for the TM4C123 platform.
 *
 * DOCUMENTATION STYLE:
 *     - Balanced (technical + objective)
 *     - No functional or logical modifications were introduced
 *     - Only comments and formatting were improved
 *
 * AUTHOR:      Rogerio Lima
 * REFORMAT:    2025 (Documentation and formatting only)
 *
 *******************************************************************************
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
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
#include "timer.h"
#include "uart.h"
#include "interrupt.h"
#include "pso_init.h"
#include "pso_uart.h"
#include "pso_led.h"
#include "pso_data.h"
#include "pso_timing.h"
#include "adc.h"
#include "pso_pwm.h"
#include "fifo.h"
#include "ulink.h"
#include "ulink_types.h"
#include "systick.h"            // SysTick functions
#include "hw_nvic.h"            // NVIC registers
#include "pso_debug.h"
#include "pso_rpm.h"
#include "pso_system.h"


/*******************************************************************************
 * COMPILING OPTIONS
 ******************************************************************************/
// #define PWM_PROFILE_NONE_SELECTED            /* NOT TESTED */

#define PWM_PROFILE_TRAPEZOID_SELECTED       /* IMPLEMENTED */
//  #define PWM_PROFILE_LINEAR_SELECTED          /* IMPLEMENTED */
//#define PWM_PROFILE_STEP_SELECTED            /* IMPLEMENTED */

// #define PWM_PROFILE_CUSTOM_SELECTED          /* NOT IMPLEMENTED */
// #define PWM_PROFILE_SINE_SELECTED            /* NOT IMPLEMENTED */
// #define PWM_PROFILE_EXPONENTIAL_SELECTED     /* NOT IMPLEMENTED */



/*******************************************************************************
 * EXTERNAL VARIABLES
 ******************************************************************************/
extern uart_raw_data_t g_uart0_data;            // UART0 raw data buffer
extern uint8_t g_tx_buffer_uart;                // UART transmit buffer
extern uint16_t uart_tx_buffer[ULINK_MAX_PACKET_LEN]; // UART packet buffer
extern uint8_t g_timer_a0_scan_flag;            // Timer A0 scan completion flag
extern ulink_pso_data_t dp;                     // Data packet structure
extern fifo_t g_fifo_ping;                      // Ping FIFO for data buffering
extern fifo_t g_fifo_pong;                      // Pong FIFO for data buffering
extern uint8_t pwm_throttle;                    // PWM throttle value
extern uint8_t fix_rpm_start_acq;               // Fixed RPM acquisition start flag
extern uint32_t g_pulse_diff;                   // Pulse difference for RPM calculation
extern uint8_t g_pwm_value;                     // Global PWM duty cycle value

/*******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************/
uint16_t scan_period_actual;                    // Actual scan period in use
pwm_profile_t current_profile = PWM_PROFILE_NONE; // Currently active PWM profile
uint8_t profile_complete = 0U;                  // Flag indicating profile completion
uint32_t g_profile_start_time = 0U;             // Start time of current profile (ms)
uint32_t g_scaled_rpm;                          // Scaled RPM value

/* PWM Profile Configuration */
static linear_config_t linear_config;        // Active linear profile configuration
static step_config_t step_config;            // Active step profile configuration
static trapezoid_config_t trapezoid_config;  // Active trapezoid profile configuration

/*******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/
static void system_init(void);                  // Initialize system peripherals
static void stream_data_uart(void);             // Stream data via UART
static void handle_data_capture(void);          // Handle data capture and buffering

/* SysTick Function Prototypes */
void SysTick_Handler(void);                     // SysTick interrupt handler
uint32_t get_systick_ms(void);                  // Get milliseconds since system start
static void configure_systick(void);            // Configure SysTick timer

/* State Machine Handler Prototypes */
static sys_state_t state_idle(void);            // Idle state handler
static sys_state_t state_init(void);            // Initialization state handler
static sys_state_t state_timing(void);          // Timing control state handler
static sys_state_t state_processing(void);      // Data processing state handler
static sys_state_t state_streaming(void);       // Data streaming state handler
static sys_state_t state_pwm_control(void);     // PWM control state handler
static sys_state_t state_stopping(void);        // Stopping state handler
static sys_state_t state_error(void);           // Error state handler

/*******************************************************************************
 * FUNCTION: main
 * 
 * DESCRIPTION:
 *       Main application entry point. Initializes system and runs the main
 *       state machine loop for data acquisition and streaming.
 * 
 * PARAMETERS:
 *       None
 * 
 * RETURNS:
 *       int (never returns - infinite loop)
 ******************************************************************************/
int main(void)
{
    sys_state_t sys_state = SYS_STATE_INIT;     // Initial system state
    
    /* Initialize system peripherals and configurations */
    system_init();

    /* Initialize PWM profile system */
    pwm_profile_init();

#ifdef PWM_PROFILE_LINEAR_SELECTED
    /* Configure custom linear profile */
    linear_config = (linear_config_t){
        .duration_ms = 10000,      /* Total profile duration in ms */
        .start_value = 0,      /* Start PWM value (0-100%) */
        .end_value = 70,        /* End PWM value (0-100%) */
        .cycles = 1,           /* Number of cycles to repeat */
        .bidirectional = 0,    /* Ramp up then down if true */
        .slew_rate = 0.01        /* Rate of change (%/ms) - if non-zero overrides duration */
    };
#elif defined(PWM_PROFILE_STEP_SELECTED)
    /* Configure custom step profile */
    step_config = (step_config_t){
        .step_interval_ms = 5000, /* Time between step changes (5 seconds) */
        .num_steps = 9,           /* Number of steps in the sequence */
        .steps = {0, 25, 0, 50, 0, 75, 0, 100, 0}, /* Step values (0-100%) */
        .cycles = 1,              /* Number of cycles to repeat */
        .ping_pong = false        /* Ping-pong (forward then reverse) if true */
    };
#elif defined(PWM_PROFILE_TRAPEZOID_SELECTED)
    /* Configure custom trapezoidal profile */
    trapezoid_config = (trapezoid_config_t){
        .duration_ms = 20000,                   // 20 seconds total duration
        .ramp_up_ms = 5000,                     // 5 seconds ramp up
        .hold_ms = 10000,                       // 10 seconds hold at maximum
        .ramp_down_ms = 5000,                   // 5 seconds ramp down
        .min_value = 0,                         // Minimum 0% duty cycle
        .max_value = 60,                       // Maximum 100% duty cycle
        .cycles = 2,                            // Repeat 2 times
        .auto_repeat = false                    // No auto-repeat
    };
#else
    /* No profile selected */   
#endif


    /***************************************************************************
     * MAIN LOOP
     * 
     * The main loop follows a priority-based execution model:
     * 1. High-priority ADC sample processing (interrupt-driven)
     * 2. Main state machine for system control
     **************************************************************************/
    while (1)
    {
        /* DEBUG: Pulse ADC timing pin (commented out in production) */
        // DEBUG_STATE_TOGGLE();

        /* 1. HIGH-PRIORITY ADC SAMPLE PROCESSING */
        if (g_timer_a0_scan_flag)
        {
            g_timer_a0_scan_flag = 0U;          // Clear flag after processing
        }
        
        /* 2. MAIN CONTROL LOOP - STATE MACHINE */
        switch (sys_state)
        {
            case SYS_STATE_INIT:
                sys_state = state_init();
                break;

            case SYS_STATE_IDLE:
                sys_state = state_idle();
                break;

            case SYS_STATE_TIMING:
                sys_state = state_timing();
                break;
                
            case SYS_STATE_DATA_PROCESSING:
                sys_state = state_processing(); // Data scaling + filtering
                break;

            case SYS_STATE_STREAMING:
                sys_state = state_streaming();  // Continuous UART streaming
                break;
                
            case SYS_STATE_PWM_CONTROL:
                sys_state = state_pwm_control(); // PWM profile execution
                break;
                
            case SYS_STATE_STOPPING:
                sys_state = state_stopping();
                break;
                
            case SYS_STATE_ERROR:
                sys_state = state_error();
                break;
                
            default:
                sys_state = SYS_STATE_IDLE;     // Default fallback state
                break;
        }
    }
}

/*******************************************************************************
 * FUNCTION: system_init
 * 
 * DESCRIPTION:
 *       Initializes all system peripherals, clocks, and configurations.
 *       Enables required modules and sets up interrupts.
 * 
 * PARAMETERS:
 *       None
 * 
 * RETURNS:
 *       void
 ******************************************************************************/
static void system_init(void)
{
    uint16_t i;

    /* Configure system clock: 16MHz -> PLL -> 400MHz -> /10 = 40MHz */
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | 
                   SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    
    /* CONFIGURE SysTick for 1ms interrupts */
    SysTickPeriodSet(SysCtlClockGet() / 1000);  // 1ms period
    SysTickIntRegister(&SysTick_Handler);       // Register interrupt handler
    SysTickEnable();                            // Enable SysTick counter
    SysTickIntEnable();                         // Enable SysTick interrupts

    /* INITIALIZE DEBUG GPIO PINS */
    debug_gpio_init();

    /* Enable and configure peripherals */
    PSO_PeripheralEnable();                     // Enable system peripherals
    PSO_GPIOConfig();                           // Configure GPIO pins
    PSO_UART0Config();                          // Configure UART0
    PSO_Timers();                               // Configure system timers
    PSO_ADCConfig();                            // Configure ADC

    /* Stabilization delay for hardware to settle */
    for (i = 0U; i < 10000U; i++);

    /* Configure additional peripherals */
    pso_rpm_config();                           // Configure RPM measurement
    pso_pwm_config();                           // Configure PWM output
    pso_spi0_config();                          // Configure SPI0 (if needed)

    /* Initialize FIFOs for data buffering */
    fifo_init(&g_fifo_ping);
    fifo_init(&g_fifo_pong);

    /* Enable global interrupts */
    IntMasterEnable();

    /* Turn off all LEDs initially */
    led_all_off();
}

/*******************************************************************************
 * FUNCTION: stream_data_uart
 * 
 * DESCRIPTION:
 *       Transmits data packet via UART0 when streaming is active.
 *       Includes LED indication for streaming activity.
 * 
 * PARAMETERS:
 *       None
 * 
 * RETURNS:
 *       void
 ******************************************************************************/
static void stream_data_uart(void)
{
    if (streaming_active && enable_data_capture)
    {
        /* Transmit data packet via UART */
        uartBatchWrite(UART0_BASE, uart_tx_buffer, PACKET_LENGTH);
        
        /* Toggle LED to indicate streaming activity (commented out) */
        // led_green_toggle();
    }
}

/*******************************************************************************
 * FUNCTION: handle_data_capture
 * 
 * DESCRIPTION:
 *       Manages data capture and FIFO operations during active streaming.
 *       Prevents data loss by checking FIFO capacity before writing.
 * 
 * PARAMETERS:
 *       None
 * 
 * RETURNS:
 *       void
 ******************************************************************************/
static void handle_data_capture(void)
{
    uint8_t i;
    
    if (fix_rpm_start_acq && enable_data_capture)
    {
        /* Write data to FIFO for buffering */
        for (i = 0U; i < PACKET_LENGTH; i++)
        {
            if (!fifo_put(&g_fifo_ping, (uint8_t)uart_tx_buffer[i]))
            {
                /* FIFO full - prevent data loss by breaking early */
                break;
            }
        }
    }
}

/*******************************************************************************
 * STATE MACHINE: INIT STATE
 * 
 * DESCRIPTION:
 *       Initializes streaming parameters and prepares system for data
 *       transmission. Configures timing, resets counters, and sets up
 *       PWM profiles.
 * 
 * PARAMETERS:
 *       None
 * 
 * RETURNS:
 *       sys_state_t - Next system state (SYS_STATE_IDLE)
 ******************************************************************************/
static sys_state_t state_init(void)
{
    /* Reset streaming flags to default state */
    fix_rpm_start_acq = 1U;
    enable_data_capture = 1U;
    streaming_active = 1U;
    
    /* Configure default PWM profile */
#if defined(PWM_PROFILE_TRAPEZOID_SELECTED)
    current_profile = PWM_PROFILE_TRAPEZOID;            // Default trapezoidal profile
    pwm_set_trapezoid_config(&trapezoid_config); // Configure active trapezoidal profile
#elif defined(PWM_PROFILE_LINEAR_SELECTED)
    current_profile = PWM_PROFILE_LINEAR;               // Default linear profile   
    pwm_set_linear_config(&linear_config);      // Configure active linear profile
#elif defined(PWM_PROFILE_STEP_SELECTED)
    current_profile = PWM_PROFILE_STEP;             // Default trapezoidal profile
    pwm_set_step_config(&step_config);       // Configure active step profile
#else
    current_profile = PWM_PROFILE_NONE;                 // No profile selected 
#endif
    
    /* Reset profile start time */
    g_profile_start_time = 0U;    

    /* Configure timing rates */
    timing_configure(RATE_1000_HZ, 0);          // Main loop at 1 kHz
    
    /* Reset counters and timers */
    timing_reset();
    sample_counter = 0;
    
    /* Clear FIFOs for fresh start */
    fifo_init(&g_fifo_ping);
    fifo_init(&g_fifo_pong);
    
    /* Configure active trapezoidal profile */
    // pwm_set_trapezoid_config(&active_trapezoid_config);
    
    return SYS_STATE_IDLE;
}

/*******************************************************************************
 * STATE MACHINE: IDLE STATE
 * 
 * DESCRIPTION:
 *       Waits for user input (SW1 button press) to start streaming.
 *       Provides visual standby indication.
 * 
 * PARAMETERS:
 *       Nonepso_data
 * 
 * RETURNS:
 *       sys_state_t - Next system state
 *                     (SYS_STATE_IDLE or SYS_STATE_TIMING)
 ******************************************************************************/
static sys_state_t state_idle(void)
{
    indicate_standby();                         // Visual indication of standby mode
    
    /* Check if SW1 (PF4) is pressed to start streaming */
    if (!(GPIO_PORTF_DATA_R & GPIO_PIN_4))
    {
        led_all_off();                          // Turn off all LEDs
        
        // Optional: Reset profile start time here if needed
        // g_profile_start_time = get_systick_ms();
        
        return SYS_STATE_TIMING;                // Transition to timing state
    }

    return SYS_STATE_IDLE;                      // Remain in idle state
}

/*******************************************************************************
 * STATE MACHINE: TIMING STATE
 * 
 * DESCRIPTION:
 *       Controls the main loop rate at 500 Hz (2ms intervals). 
 *       Provides timing synchronization for the data processing pipeline.
 * 
 * PARAMETERS:
 *       None
 * 
 * RETURNS:
 *       sys_state_t - Next system state
 *                     (SYS_STATE_TIMING or SYS_STATE_DATA_PROCESSING)
 ******************************************************************************/
static sys_state_t state_timing(void)
{
    /* Control loop rate at 500 Hz (2ms intervals) */
    if (check_interval_ms(2))
    {
        DEBUG_STATE_TOGGLE();                   /* PD7 - Yellow channel debug */
        
        return SYS_STATE_DATA_PROCESSING;       // Proceed to data processing
    }

    return SYS_STATE_TIMING;                    // Wait for next timing interval
}

/*******************************************************************************
 * STATE MACHINE: DATA PROCESSING STATE
 * 
 * DESCRIPTION:
 *       Performs scaling and filtering of acquired data before streaming.
 *       Calculates RPM and prepares data packet for transmission.
 * 
 * PARAMETERS:
 *       None
 * 
 * RETURNS:
 *       sys_state_t - Next system state (SYS_STATE_STREAMING)
 ******************************************************************************/
static sys_state_t state_processing(void)
{
    uint32_t current_rpm;
    uint32_t edge_interval_us;
    
    /* Get RPM directly from ISR calculation (period-based) */
    current_rpm = rpm_get_value();
    
    /* Optional: Apply moving average filter */
    rpm_update_filter(current_rpm);
    g_scaled_rpm = rpm_get_filtered();
    
    /* Debug information */
    edge_interval_us = rpm_get_edge_interval_us();
    
    /* ... rest of existing code ... */
    packet_data(&dp);
    copy_data(uart_tx_buffer, &dp);
    
    return SYS_STATE_STREAMING;
}

/*******************************************************************************
 * STATE MACHINE: STREAMING STATE
 * 
 * DESCRIPTION:
 *       Active data streaming via UART at 500 Hz.
 *       Manages UART data transmission and provides streaming indication.
 *       Monitors for manual stop command (SW2 button).
 * 
 * PARAMETERS:
 *       None
 * 
 * RETURNS:
 *       sys_state_t - pso_dataNext system state
 *                     (SYS_STATE_STREAMING, SYS_STATE_STOPPING, or SYS_STATE_PWM_CONTROL)
 ******************************************************************************/
static sys_state_t state_streaming(void)
{
    static uint32_t stream_packet_count = 0;    // Packet counter for debugging
    uint32_t current_time = get_systick_ms();   // Current system time

    indicate_streaming();                       // Visual streaming indication
    
    if (enable_data_capture)
    {
        /* Explicitly call uartBatchWrite for data transmission */
        uartBatchWrite(UART0_BASE, uart_tx_buffer, PACKET_LENGTH);
        
        stream_packet_count++;                  // Increment packet counter
        
        /* Toggle debug pin to indicate streaming activity */
        DEBUG_ADC_TOGGLE();                     /* PD6 - ADC debug pin */
    }

    /* Check if SW2 (PF0) is pressed for manual stop */
    if (!(GPIO_PORTF_DATA_R & GPIO_PIN_0))
    {
        return SYS_STATE_STOPPING;              // Transition to stopping state
    }

    return SYS_STATE_PWM_CONTROL;               // Proceed to PWM control
}

/*******************************************************************************
 * STATE MACHINE: PWM CONTROL STATE
 * 
 * DESCRIPTION:
 *       Executes PWM control profiles (trapezoid, linear, step, custom).
 *       Automatically resets profile timing on first entry.
 *       Updates throttle value in data packet and monitors profile completion.
 *       Provides manual stop capability via SW2 button.
 * 
 * PARAMETERS:
 *       None
 * 
 * RETURNS:
 *       sys_state_t - Next system state
 *                     (SYS_STATE_TIMING or SYS_STATE_STOPPING)
 ******************************************************************************/
static sys_state_t state_pwm_control(void)
{
    static uint32_t last_pwm_update = 0;        // Last PWM update time
    static uint8_t first_entry = 1;             // Flag for first entry into state
    uint32_t current_time = get_systick_ms();   // Current system time
    uint8_t profile_result = FUNCTION_RUNNING;  // Profile execution status

    indicate_pwm_active();                      // Visual PWM active indication

    /* RESET start time on first entry into this state */
    if (first_entry)
    {
        g_profile_start_time = current_time;    // Reset profile start time
        first_entry = 0;                        // Clear first entry flag
    }

    last_pwm_update = current_time;
    
    /* Calculate elapsed time since profile start */
    uint32_t elapsed_ms = current_time - g_profile_start_time;
    
    /* Execute selected profile type */
    switch (current_profile)
    {
        case PWM_PROFILE_TRAPEZOID:
            profile_result = execute_trapezoid_profile(elapsed_ms, &trapezoid_config);
            break;
            
        case PWM_PROFILE_LINEAR:
            {
                profile_result = execute_linear_profile(elapsed_ms, &linear_config);
            }
            break;
            
        case PWM_PROFILE_STEP:
            {
                profile_result = execute_step_profile(elapsed_ms, &step_config);
            }
            break;
            
        case PWM_PROFILE_CUSTOM:
            profile_result = execute_custom_profile(elapsed_ms);
            break;
            
        default:
            /* Fallback to trapezoidal profile */
            profile_result = execute_trapezoid_profile(elapsed_ms, &trapezoid_config);
            break;
    }
    
    /* Update throttle value in data packet */
    dp.throttle = g_pwm_value;
    
    /* Check if profile has completed */
    if (profile_result == FUNCTION_COMPLETE)
    {
        profile_complete = 1U;                  // Set completion flag
        pwm_profile_stop();                     // Stop profile execution
        first_entry = 1;                        // Prepare for next execution
        return SYS_STATE_STOPPING;              // Transition to stopping state
    }

    /* Check if SW2 (PF0) is pressed for manual stop */
    if (!(GPIO_PORTF_DATA_R & GPIO_PIN_0))
    {
        pwm_profile_stop();                     // Stop PWM profile
        first_entry = 1;                        // Prepare for next execution
        return SYS_STATE_STOPPING;              // Transition to stopping state
    }

    return SYS_STATE_TIMING;                    // Return to timing control
}

/*******************************************************************************
 * STATE MACHINE: STOPPING STATE
 * 
 * DESCRIPTION:
 *       Gracefully stops streaming and performs cleanup operations.
 *       Flushes remaining data from FIFOs, stops PWM, and provides
 *       visual feedback.
 * 
 * PARAMETERS:
 *       None
 * 
 * RETURNS:
 *       sys_state_t - Next system state (SYS_STATE_INIT)
 ******************************************************************************/
static sys_state_t state_stopping(void)
{
    /* Disable all streaming and data capture */
    streaming_active = 0U;
    enable_data_capture = 0U;
    fix_rpm_start_acq = 0U;

    /* Ensure PWM is fully stopped */
    pwm_profile_stop();                         // Stop PWM profile execution
    pwm_set_throttle(0);                        /* Ensure PWM at 0% duty cycle */

    /* Flush any remaining data in FIFOs */
    while (!fifo_is_empty(&g_fifo_ping))
    {
        uint8_t data = fifo_get(&g_fifo_ping);
        /* Transmit remaining data */
        UARTCharPut(UART0_BASE, data);
    }

    indicate_finish();                          // Visual finish indication

    return SYS_STATE_INIT;                      // Return to initialization state
}

/*******************************************************************************
 * STATE MACHINE: ERROR STATE
 * 
 * DESCRIPTION:
 *       Handles error conditions by disabling all operations,
 *       stopping PWM, and providing visual error feedback.
 * 
 * PARAMETERS:
 *       None
 * 
 * RETURNS:
 *       sys_state_t - Next system state (SYS_STATE_IDLE)
 ******************************************************************************/
static sys_state_t state_error(void)
{
    /* Disable all operations */
    streaming_active = 0U;
    enable_data_capture = 0U;
    
    /* Stop PWM immediately */
    pwm_profile_stop();

    indicate_error();                           // Visual error indication

    return SYS_STATE_IDLE;                      // Return to idle state
}
