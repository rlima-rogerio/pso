/*******************************************************************************
 * FILENAME:    pso_rpm.h
 *
 * DESCRIPTION:
 *       Header file for RPM measurement system using edge-period method.
 *
 ******************************************************************************/

#ifndef PSO_RPM_H_
#define PSO_RPM_H_

#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * CONFIGURATION DEFINES
 ******************************************************************************/

#define BLADE_NUMBER            2U      /* Pulses per revolution */
#define RPM_CALC_PERIOD_MS      100U    /* Timer3 interrupt period */
#define RPM_STOP_TIMEOUT_MS     2000U   /* Motor stopped detection timeout */
#define MIN_EDGE_INTERVAL_US    100U    /* Minimum valid edge interval */
#define MAX_EDGE_INTERVAL_MS    60000U  /* Maximum valid edge interval */
#define RPM_FILTER_SAMPLES      4U      /* Moving average filter size */

/* Mantenha as definições antigas para compatibilidade */
#define RPM_MIN_VALID           1U      /* Minimum valid RPM */
#define RPM_MAX_VALID           100000U /* Maximum valid RPM */

/*******************************************************************************
 * GLOBAL VARIABLE DECLARATIONS
 ******************************************************************************/

extern uint32_t g_rpm_value;           /* Current RPM */
extern uint32_t g_edge_interval_us;    /* Edge period in microseconds */
extern uint32_t g_last_edge_time_us;   /* Last edge timestamp */
extern uint32_t g_edge_valid_count;    /* Valid edges counter */
extern uint32_t g_edge_timeout_counter;/* Stopped motor timeout */
extern uint32_t g_rpm_ready_flag;      /* New RPM data flag */

/* Timer3A flag (declared in pso_isr.c) */
extern volatile uint32_t g_timer_a3_scan_flag;

/* Variável mantida para compatibilidade */
extern uint32_t wt1cpp0_tav_buffer;

/*******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/

/* Funções principais */
uint32_t rpm_get_value(void);
bool rpm_is_ready(void);
void rpm_clear_ready_flag(void);
uint32_t rpm_get_raw_count(void);  /* Mantida para compatibilidade */
uint32_t rpm_calculate(uint32_t pulse_diff, uint32_t period_ms, uint32_t pulses_per_rev);
bool rpm_is_valid(uint32_t rpm);
void rpm_reset(void);

/* Novas funções para medição de período */
uint32_t rpm_from_period_us(uint32_t period_us, uint32_t pulses_per_rev);
uint32_t rpm_get_edge_interval_us(void);
bool rpm_is_stopped(void);
uint32_t rpm_get_filtered(void);
void rpm_update_filter(uint32_t new_rpm);
void rpm_reset_filter(void);

/* Funções de conversão (mantidas para compatibilidade) */
uint32_t rpm_from_frequency(uint32_t frequency_hz, uint32_t pulses_per_rev);
uint32_t rpm_to_frequency(uint32_t rpm, uint32_t pulses_per_rev);

#endif /* PSO_RPM_H_ */
