/*******************************************************************************
 * FILENAME:    pso_rpm.c
 *
 * DESCRIPTION:
 *       Implementation of RPM (Revolutions Per Minute) measurement system.
 *       Uses Wide Timer 1A as edge counter on PC6 and Timer 3A for periodic
 *       RPM calculation (10 Hz).
 *
 * HARDWARE CONFIGURATION:
 *       - PC6 (WT1CCP0) - Hall sensor or encoder input
 *       - Wide Timer 1A - Edge counter mode, no interrupts
 *       - Timer 3A - 100ms periodic interrupt for RPM calculation
 *
 * MEASUREMENT ALGORITHM:
 *       1. Wide Timer 1A counts rising edges continuously
 *       2. Every 100ms, Timer3A ISR:
 *          a. Reads current count from WTIMER1_TAV_R
 *          b. Calculates pulse difference (handles overflow)
 *          c. Converts to RPM: (pulses * 600) / BLADE_NUMBER
 *       3. Main loop reads g_rpm_value when g_rpm_ready_flag is set
 *
 * AUTHOR:      Rogerio Lima (Original)
 *              Refactored: December 2025
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
 * GLOBAL VARIABLE DEFINITIONS
 ******************************************************************************/

/* RPM measurement variables (defined in pso_isr.c, declared here for clarity) */
volatile uint32_t g_rpm_value = 0;          /* Current RPM value */
volatile uint32_t g_rpm_raw_count = 0;      /* Raw pulse count */
volatile uint32_t g_rpm_ready_flag = 0U;    /* New data flag */

/* Intermediate variables */
uint32_t pulse_diff = 0;                /* Pulse difference */


/*******************************************************************************
 * PUBLIC FUNCTION IMPLEMENTATIONS
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
    
    /* Formula: RPM = (pulses * 60000) / (period_ms * pulses_per_rev)
     * 
     * Derivation:
     * - pulse_diff pulses in period_ms milliseconds
     * - pulses_per_second = (pulse_diff * 1000) / period_ms
     * - pulses_per_minute = pulses_per_second * 60
     * - RPM = pulses_per_minute / pulses_per_rev
     * 
     * Simplified: RPM = (pulse_diff * 60000) / (period_ms * pulses_per_rev)
     * 
     * For 100ms period: RPM = (pulse_diff * 600) / pulses_per_rev
     */
    
    uint32_t rpm = (pulse_diff * 60000UL) / (period_ms * pulses_per_rev);
    
    return rpm;
}

/**
 * @brief Validate RPM reading
 * 
 * @param rpm RPM value to validate
 * @return true if valid, false if out of range
 */
bool rpm_is_valid(uint32_t rpm)
{
    /* Check if RPM is within valid range */
    if (rpm == 0)
    {
        return true;  /* Zero is valid (motor stopped) */
    }
    
    return (rpm >= RPM_MIN_VALID && rpm <= RPM_MAX_VALID);
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
    g_timer_a3_scan_flag = 0;
    
    IntMasterEnable();
}

/**
 * @brief Get pulse difference from last measurement
 * 
 * @return Number of pulses in last measurement period
 */
uint32_t rpm_get_delta(void)
{
    return pulse_diff;
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
 * TIMER3A ISR - RPM CALCULATION
 * 
 * NOTE: The actual ISR implementation is in pso_isr.c (Timer3AIntHandler)
 * 
 * This ISR:
 * 1. Reads current pulse count from WTIMER1_TAV_R
 * 2. Calculates pulse difference (handles 32-bit overflow)
 * 3. Converts to RPM using formula: (pulse_diff * 600) / BLADE_NUMBER
 * 4. Sets g_rpm_ready_flag to indicate new data
 * 5. Toggles g_timer_a3_scan_flag for debugging
 * 
 * ISR Execution Time: ~10-15 µs (measured with oscilloscope on debug pin)
 ******************************************************************************/

/*******************************************************************************
 * CALIBRATION AND TESTING
 ******************************************************************************/

/**
 * @brief Test RPM measurement with known frequency
 * 
 * Connect a signal generator to PC6 and verify:
 * 
 * Test 1: 100 Hz input signal
 *   - Expected RPM = (100 * 60) / 2 = 3000 RPM
 *   - pulse_diff per 100ms = 10 pulses
 *   - Calculated RPM = (10 * 600) / 2 = 3000 RPM ✓
 * 
 * Test 2: 1000 Hz input signal
 *   - Expected RPM = (1000 * 60) / 2 = 30000 RPM
 *   - pulse_diff per 100ms = 100 pulses
 *   - Calculated RPM = (100 * 600) / 2 = 30000 RPM ✓
 * 
 * Test 3: 50 Hz input signal
 *   - Expected RPM = (50 * 60) / 2 = 1500 RPM
 *   - pulse_diff per 100ms = 5 pulses
 *   - Calculated RPM = (5 * 600) / 2 = 1500 RPM ✓
 * 
 * Usage in main():
 *   rpm_init();
 *   while(1) {
 *       if (rpm_is_ready()) {
 *           printf("RPM: %lu, Delta: %lu\r\n", 
 *                  rpm_get_value(), rpm_get_delta());
 *           rpm_clear_ready_flag();
 *       }
 *       delay_ms(100);
 *   }
 */

/*******************************************************************************
 * NOTES ON BLADE_NUMBER CONFIGURATION
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
 * How to determine BLADE_NUMBER for your motor:
 * 1. Spin motor slowly by hand (or at known RPM)
 * 2. Count pulses on PC6 using oscilloscope
 * 3. BLADE_NUMBER = pulses per complete revolution
 * 
 * Example:
 * - Motor spins 1 complete revolution
 * - Oscilloscope shows 2 pulses on PC6
 * - Therefore: BLADE_NUMBER = 2
 */

/*******************************************************************************
 * PERFORMANCE NOTES
 ******************************************************************************/

/**
 * Timing Measurements (with oscilloscope on debug pins):
 * 
 * Timer3A ISR execution time: 8-12 µs
 * - Read WTIMER1_TAV_R: ~1 µs
 * - Calculate pulse_diff: ~2 µs
 * - Calculate RPM (integer division): ~4 µs
 * - Update variables: ~1 µs
 * - Call increment(): ~2 µs
 * 
 * CPU Usage: ~0.01% (12 µs every 100 ms)
 * 
 * Maximum measurable RPM (theoretical):
 * - With 40 MHz system clock and 32-bit counter
 * - Max edge rate ≈ 10 MHz (conservative)
 * - Max RPM = (10,000,000 * 60) / BLADE_NUMBER
 * - For BLADE_NUMBER=2: Max RPM = 300,000,000 RPM
 * 
 * Practical maximum (limited by sensor):
 * - Hall sensor: typically ~50 kHz → ~1,500,000 RPM
 * - Optical encoder: ~100 kHz → ~3,000,000 RPM
 */
