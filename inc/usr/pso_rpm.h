/*******************************************************************************
 * FILENAME:    pso_rpm.h
 *
 * DESCRIPTION:
 *     RPM measurement interface for the PSO firmware.
 *
 *     The firmware now uses the edge-period method only. WTimer1A captures
 *     both edges from the RPM sensor on PC6/WT1CCP0, the ISR computes the
 *     period between blade-pulse midpoints, and Timer3A provides timeout
 *     detection for stopped-motor handling.
 *
 *     This removes the older edge-count compile-time mode so the code,
 *     comments, and CCS project configuration all describe the same runtime
 *     behavior.
 *
 * TOOLCHAIN:
 *     Developed and built with Code Composer Studio 12.8.1.
 *
 * AUTHOR:      Rogerio Lima
 * UPDATED:     June 2026
 ******************************************************************************/

#ifndef PSO_RPM_H_
#define PSO_RPM_H_

#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * RPM CONFIGURATION
 ******************************************************************************/

/* Number of sensor pulses per complete revolution. */
#define BLADE_NUMBER            2U

/* Expected operating range used to reject spurious measurements. */
#define RPM_MIN_OPERATING       60U
#define RPM_MAX_OPERATING       15000U

/* Generic validity range used by rpm_is_valid(). */
#define RPM_MIN_VALID           1U
#define RPM_MAX_VALID           100000U

/* Timer3A timeout tick period. */
#define RPM_CALC_PERIOD_MS      100U

/* Period validation derived from the operating range. */
#define RPM_PERIOD_MIN_US       (60000000UL / (RPM_MAX_OPERATING * BLADE_NUMBER))
#define RPM_PERIOD_MAX_US       (60000000UL / (RPM_MIN_OPERATING * BLADE_NUMBER))

/* Sensor glitch rejection. */
#define MIN_EDGE_INTERVAL_US    500U
#define MAX_PERIOD_VARIATION_PCT 50U

/* Pulse-width validation. */
#define RPM_HIGH_MIN_US         50U
#define RPM_HIGH_MAX_US         200000UL
#define RPM_LOW_MIN_US          50U
#define RPM_LOW_MAX_US          200000UL

/* Motor is considered stopped when no edge is seen for this long. */
#define RPM_STOP_TIMEOUT_MS     2000U

/* Moving-average filter size. Keep at 1 for no extra filtering latency. */
#define RPM_FILTER_SAMPLES      1U

/*
 * Blade detection polarity:
 *   DIRECT   - blade presence produces a HIGH pulse
 *   INVERTED - blade presence produces a LOW pulse
 *
 * The current hardware uses the inverted logic path.
 */
/* #define BLADE_DETECTION_LOGIC_DIRECT */
#define BLADE_DETECTION_LOGIC_INVERTED

/*******************************************************************************
 * GLOBAL VARIABLE DECLARATIONS
 ******************************************************************************/

extern volatile uint32_t g_rpm_value;
extern volatile uint32_t g_rpm_ready_flag;
extern volatile uint32_t g_timer_a3_scan_flag;

extern uint32_t wt1cpp0_tav_buffer;
extern uint32_t g_edge_interval_us;
extern uint32_t g_last_edge_time_us;
extern uint32_t g_last_capture_value;
extern uint32_t g_edge_valid_count;
extern uint32_t g_edge_timeout_counter;

/*******************************************************************************
 * API
 ******************************************************************************/

uint32_t rpm_get_value(void);
bool rpm_is_ready(void);
void rpm_clear_ready_flag(void);

uint32_t rpm_get_raw_count(void);
bool rpm_is_valid(uint32_t rpm);
void rpm_reset(void);

uint32_t rpm_from_frequency(uint32_t frequency_hz, uint32_t pulses_per_rev);
uint32_t rpm_to_frequency(uint32_t rpm, uint32_t pulses_per_rev);
uint32_t rpm_from_period_us(uint32_t period_us, uint32_t pulses_per_rev);

uint32_t rpm_get_edge_interval_us(void);
bool rpm_is_stopped(void);

uint32_t rpm_get_filtered(void);
void rpm_update_filter(uint32_t new_rpm);
void rpm_reset_filter(void);

#endif /* PSO_RPM_H_ */
