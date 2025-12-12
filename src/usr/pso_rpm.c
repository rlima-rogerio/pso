/*******************************************************************************
 * FILENAME:    pso_rpm.c
 *
 * DESCRIPTION:
 *       Implementation of RPM measurement using edge-period method.
 *
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

/* RPM measurement variables */
uint32_t g_rpm_value = 0;
uint32_t g_edge_interval_us = 0;
uint32_t g_last_edge_time_us = 0;
uint32_t g_edge_valid_count = 0;
uint32_t g_edge_timeout_counter;
uint32_t g_rpm_ready_flag = 0;



/* Filtro de média móvel */
static uint32_t rpm_filter_buffer[RPM_FILTER_SAMPLES];
static uint8_t rpm_filter_index = 0;
static uint8_t rpm_filter_count = 0;

/*******************************************************************************
 * PUBLIC FUNCTION IMPLEMENTATIONS
 ******************************************************************************/

uint32_t rpm_get_value(void)
{
    return g_rpm_value;
}

bool rpm_is_ready(void)
{
    return (g_rpm_ready_flag != 0);
}

void rpm_clear_ready_flag(void)
{
    IntMasterDisable();
    g_rpm_ready_flag = 0;
    IntMasterEnable();
}

uint32_t rpm_get_raw_count(void)
{
    /* Retorna o contador do WTimer1 para compatibilidade */
    return WTIMER1_TAV_R;
}

uint32_t rpm_calculate(uint32_t pulse_diff, uint32_t period_ms, uint32_t pulses_per_rev)
{
    /* Mantida para compatibilidade - usa fórmula antiga */
    if (period_ms == 0 || pulses_per_rev == 0)
    {
        return 0;
    }
    
    return (pulse_diff * 60000UL) / (period_ms * pulses_per_rev);
}

bool rpm_is_valid(uint32_t rpm)
{
    /* Verificação simplificada - 0 RPM é válido */
    if (rpm == 0)
    {
        return true;
    }
    
    return (rpm >= RPM_MIN_VALID && rpm <= RPM_MAX_VALID);
}

void rpm_reset(void)
{
    IntMasterDisable();
    
    g_rpm_value = 0;
    g_edge_interval_us = 0;
    g_edge_valid_count = 0;
    g_edge_timeout_counter = 0;
    g_rpm_ready_flag = 0;
    
    /* Reset do filtro */
    rpm_reset_filter();
    
    IntMasterEnable();
}

/*******************************************************************************
 * NEW PERIOD-BASED FUNCTIONS
 ******************************************************************************/

uint32_t rpm_from_period_us(uint32_t period_us, uint32_t pulses_per_rev)
{
    if (period_us == 0 || pulses_per_rev == 0)
    {
        return 0;
    }
    
    /* RPM = 60,000,000 / (period_us * pulses_per_rev) */
    uint64_t denominator = (uint64_t)period_us * pulses_per_rev;
    
    if (denominator == 0)
    {
        return 0;
    }
    
    return (uint32_t)(60000000UL / denominator);
}

uint32_t rpm_get_edge_interval_us(void)
{
    return g_edge_interval_us;
}

bool rpm_is_stopped(void)
{
    return (g_edge_timeout_counter >= RPM_STOP_TIMEOUT_MS);
}

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

void rpm_update_filter(uint32_t new_rpm)
{
    rpm_filter_buffer[rpm_filter_index] = new_rpm;
    rpm_filter_index = (rpm_filter_index + 1) % RPM_FILTER_SAMPLES;
    
    if (rpm_filter_count < RPM_FILTER_SAMPLES)
    {
        rpm_filter_count++;
    }
}

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

/*******************************************************************************
 * COMPATIBILITY FUNCTIONS
 ******************************************************************************/

uint32_t rpm_from_frequency(uint32_t frequency_hz, uint32_t pulses_per_rev)
{
    if (pulses_per_rev == 0)
    {
        return 0;
    }
    
    return (frequency_hz * 60) / pulses_per_rev;
}

uint32_t rpm_to_frequency(uint32_t rpm, uint32_t pulses_per_rev)
{
    return (rpm * pulses_per_rev) / 60;
}
