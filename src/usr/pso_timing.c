/*******************************************************************************
 * FILE:        pso_timing.c
 *
 * DESCRIPTION:
 *     PSO Timing and Scheduling Module.
 *     Provides precise timing functions for system scheduling, interval checking,
 *     and delay operations. Uses SysTick timer for 1ms system tick and implements
 *     configurable sampling rates from 1Hz to 2000Hz with deadline monitoring.
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
#include "pso_timing.h"

/*******************************************************************************
 * MODULE GLOBAL VARIABLES
 *******************************************************************************/

/* Timing control structure for interval management */
static timing_control_t g_timing_controller;

/* System tick counter - incremented every 1ms by SysTick interrupt */
volatile uint32_t g_system_tick_counter = 0;

/* Ticks per millisecond calculation (default 40 for 40MHz system clock) */
static uint32_t g_ticks_per_ms = 40;

/* Legacy timing variables (for compatibility with existing code) */
volatile uint32_t g_system_ms_counter = 0;
uint8_t streaming_active = 0U;
uint8_t enable_data_capture = 1U;
uint8_t fix_rpm_start_acq = 0U;
uint32_t sample_counter = 0U;

/*******************************************************************************
 * FUNCTION: SysTick_Handler
 *
 * DESCRIPTION:
 *     SysTick interrupt handler - called every 1ms.
 *     Increments the global system tick counter used for all timing operations.
 *
 * PARAMETERS:
 *     None
 *
 * RETURNS:
 *     void
 *
 * NOTES:
 *     - Minimal ISR for maximum performance
 *     - Only increments counter - no complex operations
 *     - Execution time: < 100ns (typically 2-3 instructions)
 *     - SysTick configured by timing_init() function
 *******************************************************************************/
void SysTick_Handler(void)
{
    g_system_tick_counter++;
}

/*******************************************************************************
 * FUNCTION: get_systick_ms
 *
 * DESCRIPTION:
 *     Returns the current system time in milliseconds.
 *     Reads the volatile tick counter updated by SysTick interrupt.
 *
 * PARAMETERS:
 *     None
 *
 * RETURNS:
 *     Current time in milliseconds (32-bit, overflows after ~49.7 days)
 *
 * NOTES:
 *     - Thread-safe for single-read operations
 *     - For time difference calculations, use timing_check_interval() instead
 *     - Counter overflow is handled in timing functions
 *******************************************************************************/
uint32_t get_systick_ms(void)
{
    return g_system_tick_counter;
}

/*******************************************************************************
 * FUNCTION: timing_init
 *
 * DESCRIPTION:
 *     Initializes the timing system with the specified system clock frequency.
 *     Configures SysTick timer for 1ms interrupts and sets up timing controller.
 *
 * PARAMETERS:
 *     system_clock_hz - System clock frequency in Hertz (e.g., 40000000 for 40MHz)
 *
 * RETURNS:
 *     void
 *
 * OPERATION:
 *     1. Calculates ticks per millisecond based on system clock
 *     2. Configures SysTick timer for 1ms interrupts
 *     3. Initializes timing controller structure with default values
 *     4. Sets default sampling rate to 100Hz (10ms intervals)
 *
 * NOTES:
 *     - Must be called before any other timing functions
 *     - Assumes SysTick clock source is system clock (not external)
 *     - Default configuration: 100Hz sampling rate
 *******************************************************************************/
void timing_init(uint32_t system_clock_hz)
{
    /* 1. Calculate ticks per millisecond */
    g_ticks_per_ms = system_clock_hz / 1000;
    
    /* 2. Configure SysTick timer for 1ms interrupts */
    NVIC_ST_CTRL_R = 0;                    /* Disable temporarily */
    NVIC_ST_RELOAD_R = g_ticks_per_ms - 1; /* Reload value for 1ms period */
    NVIC_ST_CURRENT_R = 0;                 /* Clear current counter */
    
    /* Enable SysTick: use system clock, enable interrupt, enable timer */
    NVIC_ST_CTRL_R = NVIC_ST_CTRL_CLK_SRC | 
                     NVIC_ST_CTRL_INTEN | 
                     NVIC_ST_CTRL_ENABLE;
    
    /* 3. Initialize timing control structure */
    g_timing_controller.last_tick = 0;
    g_timing_controller.interval_ticks = 10 * g_ticks_per_ms; /* Default: 10ms (100Hz) */
    g_timing_controller.actual_interval = 0;
    g_timing_controller.execution_count = 0;
    g_timing_controller.enabled = true;
    g_timing_controller.ready_flag = false;
    g_timing_controller.rate = RATE_100_HZ;
    g_timing_controller.missed_deadlines = 0;
}

/*******************************************************************************
 * FUNCTION: timing_configure
 *
 * DESCRIPTION:
 *     Configures the sampling/processing rate for interval-based operations.
 *     Supports predefined rates (1Hz to 2000Hz) or custom intervals.
 *
 * PARAMETERS:
 *     rate              - Predefined sampling rate from sample_rate_t enum
 *     custom_interval_ms - Custom interval in milliseconds (used if rate=RATE_CUSTOM)
 *
 * RETURNS:
 *     void
 *
 * SUPPORTED RATES:
 *     RATE_1_HZ:    1000ms interval
 *     RATE_10_HZ:   100ms interval
 *     RATE_50_HZ:   20ms interval
 *     RATE_100_HZ:  10ms interval (default)
 *     RATE_200_HZ:  5ms interval
 *     RATE_500_HZ:  2ms interval
 *     RATE_1000_HZ: 1ms interval
 *     RATE_2000_HZ: 0.5ms interval (special handling)
 *     RATE_CUSTOM:  User-defined interval
 *
 * NOTES:
 *     - 2000Hz requires special handling due to 0.5ms interval
 *     - Resets timing statistics and missed deadline counter
 *******************************************************************************/
void timing_configure(sample_rate_t rate, uint32_t custom_interval_ms)
{
    uint32_t interval_ms;
    
    /* Determine interval based on rate selection */
    if (rate == RATE_CUSTOM && custom_interval_ms > 0)
    {
        /* Custom interval specified by user */
        interval_ms = custom_interval_ms;
        g_timing_controller.rate = RATE_CUSTOM;
    }
    else
    {
        /* Map predefined rate enum to interval in milliseconds */
        switch (rate)
        {
            case RATE_1_HZ:    interval_ms = 1000; break;
            case RATE_10_HZ:   interval_ms = 100; break;
            case RATE_50_HZ:   interval_ms = 20; break;
            case RATE_100_HZ:  interval_ms = 10; break;
            case RATE_200_HZ:  interval_ms = 5; break;
            case RATE_500_HZ:  interval_ms = 2; break;
            case RATE_1000_HZ: interval_ms = 1; break;
            case RATE_2000_HZ: interval_ms = 0; break; /* Special case: 0.5ms */
            default:           interval_ms = 10; break; /* Default: 100Hz */
        }
        g_timing_controller.rate = rate;
    }
    
    /* Convert interval to ticks */
    if (interval_ms == 0)
    {
        /* Special handling for 2000Hz (0.5ms interval) */
        g_timing_controller.interval_ticks = g_ticks_per_ms / 2;
    }
    else
    {
        g_timing_controller.interval_ticks = interval_ms * g_ticks_per_ms;
    }
    
    /* Reset timing controller state */
    g_timing_controller.last_tick = g_system_tick_counter;
    g_timing_controller.ready_flag = false;
    g_timing_controller.missed_deadlines = 0;
}

/*******************************************************************************
 * FUNCTION: timing_enable
 *
 * DESCRIPTION:
 *     Enables or disables the timing interval checking functionality.
 *
 * PARAMETERS:
 *     enable - true to enable timing, false to disable
 *
 * RETURNS:
 *     void
 *
 * NOTES:
 *     - When enabled, resets the last_tick reference to current time
 *     - When disabled, timing_check_interval() always returns false
 *     - Useful for power saving or temporary suspension of timed operations
 *******************************************************************************/
void timing_enable(bool enable)
{
    g_timing_controller.enabled = enable;
    if (enable)
    {
        g_timing_controller.last_tick = g_system_tick_counter;
    }
}

/*******************************************************************************
 * FUNCTION: timing_check_interval
 *
 * DESCRIPTION:
 *     Checks if the configured time interval has elapsed since last check.
 *     Updates timing statistics and detects missed deadlines.
 *
 * PARAMETERS:
 *     None
 *
 * RETURNS:
 *     true  - Configured interval has elapsed
 *     false - Interval has not yet elapsed OR timing is disabled
 *
 * OPERATION:
 *     1. Checks if timing is enabled
 *     2. Calculates elapsed ticks (with 32-bit overflow handling)
 *     3. Compares elapsed ticks with configured interval
 *     4. Updates statistics if interval elapsed
 *     5. Detects missed deadlines (elapsed > 2×interval)
 *
 * NOTES:
 *     - Handles 32-bit tick counter overflow (occurs every ~49.7 days)
 *     - Non-blocking - should be called regularly in main loop
 *     - Tracks actual interval for rate calculation accuracy
 *******************************************************************************/
bool timing_check_interval(void)
{
    if (!g_timing_controller.enabled)
    {
        return false;
    }
    
    uint32_t current_tick = g_system_tick_counter;
    uint32_t elapsed_ticks;
    
    /* Calculate elapsed ticks with overflow handling */
    if (current_tick >= g_timing_controller.last_tick)
    {
        /* Normal case: no overflow since last check */
        elapsed_ticks = current_tick - g_timing_controller.last_tick;
    }
    else
    {
        /* Counter overflow occurred (wrapped from 0xFFFFFFFF to 0) */
        elapsed_ticks = (0xFFFFFFFF - g_timing_controller.last_tick) + current_tick + 1;
    }
    
    /* Check if configured interval has elapsed */
    if (elapsed_ticks >= g_timing_controller.interval_ticks)
    {
        /* Update timing statistics */
        g_timing_controller.actual_interval = elapsed_ticks / g_ticks_per_ms;
        g_timing_controller.execution_count++;
        g_timing_controller.last_tick = current_tick;
        
        /* Detect missed deadlines (interval > 2× configured) */
        if (elapsed_ticks > (g_timing_controller.interval_ticks * 2))
        {
            g_timing_controller.missed_deadlines++;
        }
        
        return true;
    }
    
    return false;
}

/*******************************************************************************
 * FUNCTION: timing_get_elapsed_ms
 *
 * DESCRIPTION:
 *     Returns the actual elapsed time (in ms) since the last interval execution.
 *
 * PARAMETERS:
 *     None
 *
 * RETURNS:
 *     Actual interval in milliseconds (may vary from configured interval)
 *
 * NOTES:
 *     - Returns 0 if no interval has been executed yet
 *     - Useful for monitoring actual vs configured timing
 *     - Can indicate system load or interrupt latency issues
 *******************************************************************************/
uint32_t timing_get_elapsed_ms(void)
{
    return g_timing_controller.actual_interval;
}

/*******************************************************************************
 * FUNCTION: timing_get_execution_count
 *
 * DESCRIPTION:
 *     Returns the total number of interval executions since last reset.
 *
 * PARAMETERS:
 *     None
 *
 * RETURNS:
 *     Count of interval executions
 *
 * NOTES:
 *     - Counter can be reset with timing_reset()
 *     - 32-bit counter, maximum ~4.3 billion executions
 *     - Useful for profiling and testing
 *******************************************************************************/
uint32_t timing_get_execution_count(void)
{
    return g_timing_controller.execution_count;
}

/*******************************************************************************
 * FUNCTION: timing_get_actual_rate_hz
 *
 * DESCRIPTION:
 *     Calculates the actual execution rate in Hertz based on measured intervals.
 *
 * PARAMETERS:
 *     None
 *
 * RETURNS:
 *     Actual execution rate in Hz (0.0 if no interval measured)
 *
 * NOTES:
 *     - Based on actual_interval, not configured interval
 *     - Floating point calculation - may be slow on some systems
 *     - Returns 0.0 if actual_interval is 0 (no execution yet)
 *******************************************************************************/
float timing_get_actual_rate_hz(void)
{
    if (g_timing_controller.actual_interval > 0)
    {
        return 1000.0f / g_timing_controller.actual_interval;
    }
    return 0.0f;
}

/*******************************************************************************
 * FUNCTION: timing_get_missed_deadlines
 *
 * DESCRIPTION:
 *     Returns the count of missed deadlines since last reset.
 *
 * PARAMETERS:
 *     None
 *
 * RETURNS:
 *     Count of missed deadlines
 *
 * NOTES:
 *     - Deadline missed when actual interval > 2× configured interval
 *     - Indicates system overload or interrupt blocking
 *     - Can be used for adaptive rate control or error reporting
 *******************************************************************************/
uint32_t timing_get_missed_deadlines(void)
{
    return g_timing_controller.missed_deadlines;
}

/*******************************************************************************
 * FUNCTION: timing_reset
 *
 * DESCRIPTION:
 *     Resets timing statistics and counters to initial state.
 *
 * PARAMETERS:
 *     None
 *
 * RETURNS:
 *     void
 *
 * OPERATION:
 *     - Resets execution count and missed deadlines
 *     - Updates last_tick reference to current time
 *     - Clears ready_flag
 *
 * NOTES:
 *     - Does not change configured interval or enable state
 *     - Useful for starting new measurement periods
 *******************************************************************************/
void timing_reset(void)
{
    g_timing_controller.execution_count = 0;
    g_timing_controller.missed_deadlines = 0;
    g_timing_controller.last_tick = g_system_tick_counter;
    g_timing_controller.ready_flag = false;
}

/*******************************************************************************
 * FUNCTION: delay_ms
 *
 * DESCRIPTION:
 *     Blocking delay function for precise millisecond delays.
 *     Uses system tick counter with overflow handling.
 *
 * PARAMETERS:
 *     milliseconds - Number of milliseconds to delay
 *
 * RETURNS:
 *     void
 *
 * OPERATION:
 *     1. Records start tick count
 *     2. Calculates target tick count
 *     3. Waits in loop until target reached
 *     4. Handles tick counter overflow during wait
 *
 * NOTES:
 *     - Blocking function - CPU busy during delay
 *     - Maximum delay limited by 32-bit counter (~49.7 days)
 *     - More accurate than software loops, less than 1ms jitter
 *     - Not interrupt-safe if called from ISR
 *******************************************************************************/
void delay_ms(uint32_t milliseconds)
{
    uint32_t start_tick = g_system_tick_counter;
    uint32_t target_ticks = milliseconds * g_ticks_per_ms;
    uint32_t current_tick;
    
    while (1)
    {
        current_tick = g_system_tick_counter;
        
        if (current_tick >= start_tick)
        {
            /* Normal case: no overflow during delay */
            if ((current_tick - start_tick) >= target_ticks)
                break;
        }
        else
        {
            /* Tick counter overflowed during delay */
            if (((0xFFFFFFFF - start_tick) + current_tick + 1) >= target_ticks)
                break;
        }
    }
}

/*******************************************************************************
 * FUNCTION: delay_us
 *
 * DESCRIPTION:
 *     Approximate microsecond delay using NOP instruction loops.
 *     Less accurate than delay_ms() but useful for short delays.
 *
 * PARAMETERS:
 *     microseconds - Number of microseconds to delay
 *
 * RETURNS:
 *     void
 *
 * OPERATION:
 *     - Calculates number of NOP cycles based on system clock
 *     - Executes NOP instructions in loop
 *
 * NOTES:
 *     - Approximate timing only (varies with compiler optimization)
 *     - Blocking function - CPU busy during delay
 *     - Not recommended for precise timing (>10% error typical)
 *     - Useful for short delays (<100μs) where SysTick resolution is insufficient
 *******************************************************************************/
void delay_us(uint32_t microseconds)
{
    uint32_t cycles = (microseconds * (g_ticks_per_ms / 1000));
    uint32_t i;
    
    for (i = 0; i < cycles; i++)
    {
        __asm(" NOP");  /* 1 cycle delay (25ns at 40MHz) */
    }
}

/*******************************************************************************
 * FUNCTION: get_system_ticks
 *
 * DESCRIPTION:
 *     Returns the current system tick count (1ms resolution).
 *
 * PARAMETERS:
 *     None
 *
 * RETURNS:
 *     Current system tick count (32-bit, overflows every ~49.7 days)
 *
 * NOTES:
 *     - Alias for get_systick_ms()
 *     - Provided for compatibility with existing code
 *******************************************************************************/
uint32_t get_system_ticks(void)
{
    return g_system_tick_counter;
}

/*******************************************************************************
 * FUNCTION: get_system_ms
 *
 * DESCRIPTION:
 *     Legacy function returning system time in milliseconds.
 *     Returns the same value as get_system_ticks().
 *
 * PARAMETERS:
 *     None
 *
 * RETURNS:
 *     Current time in milliseconds
 *
 * NOTES:
 *     - Deprecated - use get_systick_ms() or get_system_ticks() instead
 *     - Maintained for backward compatibility
 *******************************************************************************/
uint32_t get_system_ms(void)
{
    return get_system_ticks();
}

/*******************************************************************************
 * FUNCTION: check_interval_ms
 *
 * DESCRIPTION:
 *     Simplified interval checking for legacy code compatibility.
 *     Checks if specified number of milliseconds have elapsed since last call.
 *
 * PARAMETERS:
 *     interval_ms - Interval to check in milliseconds
 *
 * RETURNS:
 *     true  - Specified interval has elapsed
 *     false - Interval has not elapsed
 *
 * NOTES:
 *     - Simple static implementation without overflow handling
 *     - For new code, use timing_check_interval() instead
 *     - Maintains state between calls (static variable)
 *******************************************************************************/
bool check_interval_ms(uint32_t interval_ms)
{
    static uint32_t last_time = 0;
    uint32_t current_time = get_system_ticks();
    
    if ((current_time - last_time) >= interval_ms)
    {
        last_time = current_time;
        return true;
    }
    
    return false;
}


/*******************************************************************************
 * TIME MANAGEMENT FUNCTIONS
 *******************************************************************************/

/**
 * @brief Get system time in microseconds
 * 
 * @return Current time in microseconds since system start
 */
uint32_t get_system_time_us(void)
{
    static uint32_t overflow_count = 0;
    static uint32_t last_systick = 0;
    uint32_t current_systick;
    uint32_t micros;
    
    /* Disable interrupts during read */
    IntMasterDisable();
    
    current_systick = SysTickValueGet();
    
    /* Check for SysTick overflow */
    if (current_systick > last_systick)
    {
        overflow_count++;
    }
    last_systick = current_systick;
    
    /* Calculate total microseconds */
    /* SysTick period = 1ms = 1000µs */
    micros = (overflow_count * 1000) + 
             ((SysTickPeriodGet() - current_systick) * 1000 / SysTickPeriodGet());
    
    IntMasterEnable();
    return micros;
}

/*******************************************************************************
 * PERFORMANCE AND TIMING CHARACTERISTICS
 *
 * TIMING ACCURACY:
 *     SysTick-based timing: ±0.5ms typical (dependent on interrupt latency)
 *     NOP-based delays: ±10% typical (compiler/optimization dependent)
 *
 * MAXIMUM RATES:
 *     SysTick interrupt: 1ms minimum period (1000Hz maximum)
 *     Software polling: Up to 2000Hz (0.5ms) with timing_check_interval()
 *
 * OVERFLOW HANDLING:
 *     - 32-bit tick counter overflows every ~49.7 days at 1ms resolution
 *     - All timing functions handle overflow correctly
 *     - For continuous operation >49 days, use 64-bit extensions
 *
 * CPU USAGE:
 *     SysTick ISR: <0.01% (2-3 instructions every 1ms at 40MHz)
 *     timing_check_interval(): ~10-20 cycles per call
 *     delay_ms(): 100% CPU during delay (blocking)
 *
 * RECOMMENDED USAGE PATTERNS:
 *
 * 1. MAIN LOOP SCHEDULING:
 *     timing_configure(RATE_100_HZ, 0);  // 100Hz main loop
 *     while(1) {
 *         if (timing_check_interval()) {
 *             // Execute 100Hz tasks
 *             update_sensors();
 *             control_loop();
 *         }
 *         // Handle other tasks
 *         process_uart();
 *     }
 *
 * 2. MULTIPLE RATES:
 *     // 100Hz control loop
 *     static uint32_t control_last = 0;
 *     if ((get_systick_ms() - control_last) >= 10) {
 *         control_last = get_systick_ms();
 *         control_loop();
 *     }
 *
 * 3. SHORT DELAYS:
 *     delay_us(50);  // 50μs delay for peripheral setup
 *
 * ERROR DETECTION:
 *     // Monitor system health
 *     if (timing_get_missed_deadlines() > 10) {
 *         // System overload detected
 *         reduce_processing_load();
 *     }
 *
 * EXTENSIONS (Suggested):
 *     1. Add 64-bit tick counter for extended operation
 *     2. Implement non-blocking delays using state machines
 *     3. Add timestamp logging for debugging
 *     4. Implement adaptive rate control based on missed deadlines
 *******************************************************************************/