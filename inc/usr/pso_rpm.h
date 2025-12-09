/*******************************************************************************
 * FILENAME:    pso_rpm.h
 *
 * DESCRIPTION:
 *       Header file for RPM (Revolutions Per Minute) measurement system.
 *       Provides interface for edge-counting RPM measurement using Wide Timer 1
 *       configured as edge counter on PC6 input.
 *
 * HARDWARE:
 *       - TM4C123GH6PM microcontroller
 *       - Wide Timer 1A (WT1CCP0) on PC6 - Edge counter mode
 *       - Timer 3A - 100ms periodic interrupt for RPM calculation
 *       - Hall effect sensor or optical encoder on PC6
 *
 * MEASUREMENT METHOD:
 *       - Wide Timer 1A continuously counts rising edges on PC6
 *       - Timer 3A interrupts every 100ms (10 Hz)
 *       - ISR calculates pulse difference and converts to RPM
 *       - Formula: RPM = (pulses_per_100ms * 600) / BLADE_NUMBER
 *
 * AUTHOR:      Rogerio Lima (Original)
 *              Refactored: December 2025
 *
 * NOTES:
 *       - BLADE_NUMBER must match motor/encoder configuration
 *       - Maximum measurable RPM depends on sensor output frequency
 *       - Timer overflow is automatically handled in ISR
 ******************************************************************************/

#ifndef PSO_RPM_H_
#define PSO_RPM_H_

#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * CONFIGURATION DEFINES
 ******************************************************************************/

/**
 * @brief Number of pulses per complete motor revolution
 * 
 * Configure this based on your sensor:
 * - Hall effect sensor with 2-pole motor: 2 pulses/rev
 * - Hall effect sensor with 4-pole motor: 4 pulses/rev
 * - Optical encoder: depends on encoder resolution
 * 
 * Example: A 2-pole brushless motor with Hall sensor = 2
 */
#define BLADE_NUMBER            2U

/**
 * @brief RPM calculation period in milliseconds
 * 
 * Timer3A interrupt period for RPM calculation.
 * Default: 100ms (10 Hz update rate)
 */
#define RPM_CALC_PERIOD_MS      100U

/**
 * @brief Maximum valid RPM value
 * 
 * RPM values above this are considered invalid (noise/error).
 * Adjust based on your motor specifications.
 */
#define RPM_MAX_VALID           100000U

/**
 * @brief Minimum valid RPM for non-zero reading
 * 
 * RPM values below this but non-zero are considered noise.
 */
#define RPM_MIN_VALID           10U

/*******************************************************************************
 * GLOBAL VARIABLE DECLARATIONS
 ******************************************************************************/

/**
 * @brief Current RPM value (read by main loop)
 * 
 * Updated by Timer3A ISR every 100ms.
 * Thread-safe: read-only from main, written only by ISR.
 */
extern volatile uint32_t g_rpm_value;

/**
 * @brief Raw pulse count from Wide Timer 1A
 * 
 * Cumulative count of edges detected on PC6.
 * Used internally for RPM calculation.
 */
extern volatile uint32_t g_rpm_raw_count;

/**
 * @brief Flag indicating new RPM measurement available
 * 
 * Set by Timer3A ISR after calculating new RPM.
 * Clear this flag after reading RPM in main loop.
 * 
 * Values:
 *   0 = No new data
 *   Non-zero = New RPM available 
 */
extern volatile uint32_t g_rpm_ready_flag;

/**
 * @brief Timer3A scan flag (toggles every 100ms)
 * 
 * Toggles (XOR 0xFF) on each Timer3A interrupt.
 * Can be used to detect if Timer3A is running.
 */
extern volatile uint32_t g_timer_a3_scan_flag;



/**
 * @brief Wide Timer 1 value buffer
 * 
 * Stores snapshot of WTIMER1_TAV_R for debugging.
 * Not used in current implementation but kept for compatibility.
 */
extern uint32_t wt1cpp0_tav_buffer;

/*******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/

/**
 * @brief Initialize RPM measurement system
 * 
 * Configures:
 * - Wide Timer 1A as edge counter on PC6
 * - Timer 3A for 100ms periodic interrupts
 * - Enables interrupts
 * - Initializes global variables
 * 
 * @pre System clock must be configured
 * @pre GPIO Port C must be enabled
 * 
 * @note Call this once during system initialization
 */

uint32_t rpm_get_value(void);

/**
 * @brief Check if new RPM measurement is available
 * 
 * Checks the g_rpm_ready_flag to see if Timer3A ISR has
 * calculated a new RPM value since last check.
 * 
 * @return true if new RPM available, false otherwise
 * 
 * @note Does NOT clear the flag - use rpm_clear_ready_flag()
 */
bool rpm_is_ready(void);

/**
 * @brief Clear the RPM ready flag
 * 
 * Call this after reading a new RPM value to acknowledge
 * that the data has been processed.
 * 
 * @note Thread-safe: disables interrupts during clear
 */
void rpm_clear_ready_flag(void);

/**
 * @brief Get raw pulse count from Wide Timer 1A
 * 
 * Reads the current cumulative pulse count directly from
 * the Wide Timer 1A register.
 * 
 * @return Current pulse count (32-bit, wraps at 0xFFFFFFFF)
 * 
 * @note This is a snapshot - counter continues incrementing
 */
uint32_t rpm_get_raw_count(void);

/**
 * @brief Calculate RPM from pulse difference
 * 
 * Converts pulse count to RPM based on measurement period.
 * Utility function for manual RPM calculation.
 * 
 * @param pulse_diff Number of pulses counted
 * @param period_ms Measurement period in milliseconds
 * @param pulses_per_rev Pulses per complete revolution
 * 
 * @return Calculated RPM value
 * 
 * Formula: RPM = (pulse_diff * 60000) / (period_ms * pulses_per_rev)
 * 
 * @note Handles division by zero (returns 0)
 */
uint32_t rpm_calculate(uint32_t pulse_diff, 
                       uint32_t period_ms, 
                       uint32_t pulses_per_rev);

/**
 * @brief Validate RPM reading
 * 
 * Checks if RPM value is within valid range.
 * 
 * @param rpm RPM value to validate
 * 
 * @return true if valid, false if out of range
 * 
 * @note Uses RPM_MIN_VALID and RPM_MAX_VALID limits
 */
bool rpm_is_valid(uint32_t rpm);

/**
 * @brief Reset RPM measurement system
 * 
 * Resets all counters and flags to initial state.
 * Does NOT disable timers.
 * 
 * @note Use this to clear accumulated counts without full re-initialization
 */
void rpm_reset(void);

/**
 * @brief Get pulse difference from last measurement
 * 
 * Returns the number of pulses counted in the last
 * measurement period (typically 100ms).
 * 
 * @return Pulse count from last period
 * 
 * @note This is the raw 'delta' value used for RPM calculation
 */
uint32_t rpm_get_delta(void);

/**
 * @brief Convert frequency (Hz) to RPM
 * 
 * Utility function to convert pulse frequency to RPM.
 * 
 * @param frequency_hz Pulse frequency in Hz
 * @param pulses_per_rev Pulses per revolution
 * 
 * @return Equivalent RPM value
 * 
 * Formula: RPM = (frequency_hz * 60) / pulses_per_rev
 */
uint32_t rpm_from_frequency(uint32_t frequency_hz, uint32_t pulses_per_rev);

/**
 * @brief Convert RPM to frequency (Hz)
 * 
 * Utility function to convert RPM to pulse frequency.
 * 
 * @param rpm RPM value
 * @param pulses_per_rev Pulses per revolution
 * 
 * @return Equivalent frequency in Hz
 * 
 * Formula: Hz = (rpm * pulses_per_rev) / 60
 */
uint32_t rpm_to_frequency(uint32_t rpm, uint32_t pulses_per_rev);

#endif /* PSO_RPM_H_ */