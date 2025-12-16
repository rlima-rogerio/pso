/*******************************************************************************
 * FILENAME:    pso_rpm.c
 *
 * DESCRIPTION:
 *       Implementation of RPM (Revolutions Per Minute) measurement system.
 *       Supports two measurement methods selectable via compile-time directive:
 *       
 *       RPM_EDGE_COUNT_METHOD (default):
 *         - Wide Timer 1A counts edges continuously (no interrupts)
 *         - Timer 3A (100ms) calculates RPM from pulse count
 *         - Formula: RPM = (pulses * 600) / BLADE_NUMBER
 *         - Update rate: 10 Hz (100ms fixed)
 *         - Range: ~60 RPM to 100,000 RPM
 *       
 *       RPM_EDGE_PERIOD_METHOD:
 *         - Wide Timer 1A triggers interrupt on each edge
 *         - ISR measures period between consecutive edges
 *         - Formula: RPM = 60,000,000 / (period_μs * BLADE_NUMBER)
 *         - Update rate: Per edge (instant)
 *         - Range: ~15 RPM to 300,000 RPM
 *
 * USAGE:
 *       Define one of the following in pso_config.h or project settings:
 *         #define RPM_EDGE_COUNT_METHOD    // Edge counting (default)
 *         #define RPM_EDGE_PERIOD_METHOD   // Edge period measurement
 *
 * HARDWARE:
 *       - PC6 (WT1CCP0) - Hall sensor or encoder input
 *       - Wide Timer 1A - Edge counter or capture mode
 *       - Timer 3A - 100ms periodic for RPM calc or timeout detection
 *
 * AUTHOR:      Rogerio Lima (Original)
 *              Consolidated: December 2025
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "hw_memmap.h"
#include "hw_types.h"
#include "sysctl.h"
#include "gpio.h"
#include "pin_map.h"
#include "hw_gpio.h"
#include "interrupt.h"
#include "timer.h"
#include "pso_rpm.h"

/*******************************************************************************
 * COMPILE-TIME METHOD SELECTION
 * 
 * If neither is defined, default to edge count method
 ******************************************************************************/
#if !defined(RPM_EDGE_COUNT_METHOD) && !defined(RPM_EDGE_PERIOD_METHOD)
    #define RPM_EDGE_COUNT_METHOD  /* Default to edge counting */
#endif

/* Ensure only one method is selected */
#if defined(RPM_EDGE_COUNT_METHOD) && defined(RPM_EDGE_PERIOD_METHOD)
    #error "Only one RPM measurement method can be defined!"
#endif

/*******************************************************************************
 * GLOBAL VARIABLE DEFINITIONS
 ******************************************************************************/

/* Common variables for both methods */
volatile uint32_t g_rpm_value = 0;          /* Current RPM value */
volatile uint32_t g_rpm_ready_flag = 0U;    /* New data flag */

#ifdef RPM_EDGE_COUNT_METHOD
/*-----------------------------------------------------------------------------
 * EDGE COUNT METHOD - GLOBAL VARIABLES
 *---------------------------------------------------------------------------*/
volatile uint32_t g_rpm_raw_count = 0;      /* Raw pulse count */
uint32_t g_pulse_diff = 0;                  /* Pulse difference */

#else /* RPM_EDGE_PERIOD_METHOD */
/*-----------------------------------------------------------------------------
 * EDGE PERIOD METHOD - GLOBAL VARIABLES
 *---------------------------------------------------------------------------*/
uint32_t g_edge_interval_us = 0;            /* Period between edges (μs) */
uint32_t g_last_edge_time_us = 0;           /* Time of last edge (μs) */
uint32_t g_last_capture_value = 0;          /* Last timer capture value */
uint32_t g_edge_valid_count = 0;            /* Valid edges counter */
uint32_t g_edge_timeout_counter = 0;        /* Timeout counter (ms) */

/* Moving average filter */
static uint32_t rpm_filter_buffer[RPM_FILTER_SAMPLES];
static uint8_t rpm_filter_index = 0;
static uint8_t rpm_filter_count = 0;

#endif /* RPM_EDGE_COUNT_METHOD / RPM_EDGE_PERIOD_METHOD */

/*******************************************************************************
 * COMMON PUBLIC FUNCTIONS (Both Methods)
 ******************************************************************************/

/**
 * @brief Get current RPM value
 * 
 * @return Current RPM (0 if stopped or invalid)
 */
uint32_t rpm_get_value(void)
{
    return g_rpm_value;
}

/**
 * @brief Check if new RPM measurement is available
 * 
 * @return true if new data available, false otherwise
 */
bool rpm_is_ready(void)
{
    return (g_rpm_ready_flag != 0);
}

/**
 * @brief Clear RPM ready flag
 * 
 * Thread-safe clear of ready flag.
 */
void rpm_clear_ready_flag(void)
{
    /* Disable interrupts during flag clear */
    IntMasterDisable();
    g_rpm_ready_flag = 0;
    IntMasterEnable();
}

/**
 * @brief Validate RPM reading
 * 
 * @param rpm RPM value to validate
 * @return true if valid, false if out of range
 */
bool rpm_is_valid(uint32_t rpm)
{
    /* Zero is valid (motor stopped) */
    if (rpm == 0)
    {
        return true;
    }
    
    return (rpm >= RPM_MIN_VALID && rpm <= RPM_MAX_VALID);
}

/**
 * @brief Convert frequency (Hz) to RPM
 * 
 * @param frequency_hz Pulse frequency in Hz
 * @param pulses_per_rev Pulses per revolution
 * @return Equivalent RPM
 */
uint32_t rpm_from_frequency(uint32_t frequency_hz, uint32_t pulses_per_rev)
{
    if (pulses_per_rev == 0)
    {
        return 0;
    }
    
    return (frequency_hz * 60) / pulses_per_rev;
}

/**
 * @brief Convert RPM to frequency (Hz)
 * 
 * @param rpm RPM value
 * @param pulses_per_rev Pulses per revolution
 * @return Equivalent frequency in Hz
 */
uint32_t rpm_to_frequency(uint32_t rpm, uint32_t pulses_per_rev)
{
    return (rpm * pulses_per_rev) / 60;
}

/*******************************************************************************
 * METHOD-SPECIFIC FUNCTIONS
 ******************************************************************************/

#ifdef RPM_EDGE_COUNT_METHOD
/*=============================================================================
 * EDGE COUNT METHOD - IMPLEMENTATION
 *===========================================================================*/

/**
 * @brief Get raw pulse count from Wide Timer 1A
 * 
 * @return Current cumulative pulse count
 */
uint32_t rpm_get_raw_count(void)
{
    return WTIMER1_TAV_R;
}

/**
 * @brief Calculate RPM from pulse difference
 * 
 * @param pulse_diff Number of pulses in measurement period
 * @param period_ms Measurement period in milliseconds
 * @param pulses_per_rev Pulses per complete revolution
 * 
 * @return Calculated RPM
 * 
 * Formula derivation:
 *   - pulse_diff pulses in period_ms milliseconds
 *   - pulses_per_second = (pulse_diff * 1000) / period_ms
 *   - pulses_per_minute = pulses_per_second * 60
 *   - RPM = pulses_per_minute / pulses_per_rev
 *   - Simplified: RPM = (pulse_diff * 60000) / (period_ms * pulses_per_rev)
 *   - For 100ms: RPM = (pulse_diff * 600) / pulses_per_rev
 */
uint32_t rpm_calculate(uint32_t pulse_diff, 
                       uint32_t period_ms, 
                       uint32_t pulses_per_rev)
{
    /* Avoid division by zero */
    if (period_ms == 0 || pulses_per_rev == 0)
    {
        return 0;
    }
    
    uint32_t rpm = (pulse_diff * 60000UL) / (period_ms * pulses_per_rev);
    
    return rpm;
}

/**
 * @brief Get pulse difference from last measurement
 * 
 * @return Number of pulses in last measurement period
 */
uint32_t rpm_get_delta(void)
{
    return g_pulse_diff;
}

/**
 * @brief Reset RPM measurement system
 * 
 * Clears counters and flags without disabling timers.
 */
void rpm_reset(void)
{
    /* Disable interrupts during reset */
    IntMasterDisable();
    
    g_rpm_value = 0;
    g_rpm_raw_count = 0;
    g_rpm_ready_flag = 0;
    g_pulse_diff = 0;
    
    IntMasterEnable();
}

#else /* RPM_EDGE_PERIOD_METHOD */
/*=============================================================================
 * EDGE PERIOD METHOD - IMPLEMENTATION
 *===========================================================================*/

/**
 * @brief Get raw pulse count (for compatibility)
 * 
 * @return Current timer count value
 */
uint32_t rpm_get_raw_count(void)
{
    return WTIMER1_TAV_R;
}

/**
 * @brief Calculate RPM from pulse difference (compatibility function)
 * 
 * Maintained for API compatibility with edge count method.
 * In period method, RPM is calculated directly in ISR.
 */
uint32_t rpm_calculate(uint32_t pulse_diff, 
                       uint32_t period_ms, 
                       uint32_t pulses_per_rev)
{
    if (period_ms == 0 || pulses_per_rev == 0)
    {
        return 0;
    }
    
    return (pulse_diff * 60000UL) / (period_ms * pulses_per_rev);
}

/**
 * @brief Calculate RPM from edge period
 * 
 * @param period_us Period between edges in microseconds
 * @param pulses_per_rev Pulses per revolution
 * 
 * @return Calculated RPM
 * 
 * Formula: RPM = 60,000,000 / (period_us * pulses_per_rev)
 * 
 * Example:
 *   period_us = 30,000 μs (30 ms)
 *   pulses_per_rev = 2
 *   RPM = 60,000,000 / (30,000 * 2) = 1000 RPM
 */
uint32_t rpm_from_period_us(uint32_t period_us, uint32_t pulses_per_rev)
{
    if (period_us == 0 || pulses_per_rev == 0)
    {
        return 0;
    }
    
    /* Use 64-bit arithmetic to prevent overflow */
    uint64_t denominator = (uint64_t)period_us * pulses_per_rev;
    
    return (uint32_t)(60000000UL / denominator);
}

/**
 * @brief Get edge interval in microseconds
 * 
 * @return Period between last two edges (μs)
 */
uint32_t rpm_get_edge_interval_us(void)
{
    return g_edge_interval_us;
}

/**
 * @brief Check if motor has stopped
 * 
 * @return true if no edges detected for timeout period
 */
bool rpm_is_stopped(void)
{
    return (g_edge_timeout_counter >= RPM_STOP_TIMEOUT_MS);
}

/**
 * @brief Get filtered RPM value
 * 
 * Returns moving average of last N RPM samples.
 * 
 * @return Filtered RPM
 */
uint32_t rpm_get_filtered(void)
{
    uint32_t sum = 0;
    uint8_t i;
    
    if (rpm_filter_count == 0)
    {
        return g_rpm_value;
    }
    
    for (i = 0; i < rpm_filter_count; i++)
    {
        sum += rpm_filter_buffer[i];
    }
    
    return sum / rpm_filter_count;
}

/**
 * @brief Update RPM filter with new value
 * 
 * @param new_rpm New RPM value to add to filter
 */
void rpm_update_filter(uint32_t new_rpm)
{
    rpm_filter_buffer[rpm_filter_index] = new_rpm;
    rpm_filter_index = (rpm_filter_index + 1) % RPM_FILTER_SAMPLES;
    
    if (rpm_filter_count < RPM_FILTER_SAMPLES)
    {
        rpm_filter_count++;
    }
}

/**
 * @brief Reset moving average filter
 */
void rpm_reset_filter(void)
{
    uint8_t i;
    
    for (i = 0; i < RPM_FILTER_SAMPLES; i++)
    {
        rpm_filter_buffer[i] = 0;
    }
    
    rpm_filter_index = 0;
    rpm_filter_count = 0;
}

/**
 * @brief Reset RPM measurement system
 */
void rpm_reset(void)
{
    /* Disable interrupts during reset */
    IntMasterDisable();
    
    g_rpm_value = 0;
    g_edge_interval_us = 0;
    g_edge_valid_count = 0;
    g_edge_timeout_counter = 0;
    g_rpm_ready_flag = 0;
    g_last_capture_value = 0;
    
    /* Reset filter */
    rpm_reset_filter();
    
    IntMasterEnable();
}

#endif /* RPM_EDGE_COUNT_METHOD / RPM_EDGE_PERIOD_METHOD */

/*******************************************************************************
 * CALIBRATION AND TESTING NOTES
 ******************************************************************************/

/**
 * EDGE COUNT METHOD:
 * ==================
 * Test with signal generator on PC6:
 * 
 * Frequency | Expected RPM (BLADE_NUMBER=2)
 * ----------|------------------------------
 * 50 Hz     | 1500 RPM
 * 100 Hz    | 3000 RPM
 * 1000 Hz   | 30000 RPM
 * 
 * Verification:
 *   - pulse_diff per 100ms = frequency / 10
 *   - RPM = (pulse_diff * 600) / 2
 * 
 * Example (50 Hz):
 *   - pulse_diff = 50/10 = 5 pulses per 100ms
 *   - RPM = (5 * 600) / 2 = 1500 RPM ✓
 * 
 * 
 * EDGE PERIOD METHOD:
 * ===================
 * Test with signal generator on PC6:
 * 
 * Frequency | Period    | Expected RPM (BLADE_NUMBER=2)
 * ----------|-----------|------------------------------
 * 1 Hz      | 1,000,000μs| 30 RPM
 * 16.67 Hz  | 60,000μs   | 500 RPM
 * 33.33 Hz  | 30,000μs   | 1000 RPM
 * 50 Hz     | 20,000μs   | 1500 RPM
 * 100 Hz    | 10,000μs   | 3000 RPM
 * 
 * Verification:
 *   - Period (μs) = 1,000,000 / frequency
 *   - RPM = 60,000,000 / (period_μs * 2)
 * 
 * Example (50 Hz):
 *   - Period = 1,000,000 / 50 = 20,000 μs
 *   - RPM = 60,000,000 / (20,000 * 2) = 1500 RPM ✓
 */

/*******************************************************************************
 * PERFORMANCE NOTES
 ******************************************************************************/

/**
 * EDGE COUNT METHOD:
 * ==================
 * Timer3A ISR execution: 8-12 μs
 * Update rate: 10 Hz (100ms fixed)
 * CPU usage: ~0.01%
 * Range: ~60 RPM to 100,000 RPM
 * Resolution: Poor at low RPM, good at high RPM
 * 
 * Advantages:
 *   + Simple, proven algorithm
 *   + Low CPU usage (no edge interrupts)
 *   + Fixed update rate (predictable)
 * 
 * Disadvantages:
 *   - 100ms latency
 *   - Poor resolution at low RPM
 *   - Limited minimum RPM (~60 RPM)
 * 
 * 
 * EDGE PERIOD METHOD:
 * ===================
 * WTimer1A ISR execution: 5-8 μs per edge
 * Update rate: Per edge (instant)
 * CPU usage: ~0.5% @ 1000 RPM (variable)
 * Range: ~15 RPM to 300,000 RPM
 * Resolution: Excellent at all RPM
 * 
 * Advantages:
 *   + Instant updates (no latency)
 *   + Excellent resolution at low RPM
 *   + Wide measurement range
 *   + High precision (25ns timer)
 * 
 * Disadvantages:
 *   - Higher CPU usage (ISR per edge)
 *   - Variable update rate
 *   - Requires edge interrupts
 */

/*******************************************************************************
 * BLADE_NUMBER CONFIGURATION GUIDE
 ******************************************************************************/

/**
 * BLADE_NUMBER = Number of pulses per complete motor revolution
 * 
 * Common configurations:
 * 
 * 1. Brushless Motor with Hall Sensor:
 *    - 2-pole motor (1 pole pair) → BLADE_NUMBER = 2
 *    - 4-pole motor (2 pole pairs) → BLADE_NUMBER = 4
 *    - 6-pole motor (3 pole pairs) → BLADE_NUMBER = 6
 *    - 8-pole motor (4 pole pairs) → BLADE_NUMBER = 8
 * 
 * 2. Optical Encoder:
 *    - 360 PPR (pulses per revolution) → BLADE_NUMBER = 360
 *    - 1024 PPR encoder → BLADE_NUMBER = 1024
 * 
 * 3. Magnetic Encoder:
 *    - 12 magnets → BLADE_NUMBER = 12
 *    - 24 magnets → BLADE_NUMBER = 24
 * 
 * How to determine BLADE_NUMBER:
 * 1. Rotate motor slowly by hand (1 complete revolution)
 * 2. Count pulses on PC6 using oscilloscope
 * 3. BLADE_NUMBER = total pulses counted
 */
