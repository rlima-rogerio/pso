/*******************************************************************************
 * FILENAME:    pso_rpm.c
 *
 * DESCRIPTION:
 *     RPM measurement support for the PSO firmware.
 *
 *     RPM is measured with the edge-period method only. WTimer1A captures
 *     sensor edges on PC6/WT1CCP0 and the ISR in pso_isr.c calculates RPM
 *     from the elapsed time between blade-pulse midpoints.
 *
 * TOOLCHAIN:
 *     Developed and built with Code Composer Studio 12.8.1.
 *
 * AUTHOR:      Rogerio Lima
 * UPDATED:     June 2026
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "interrupt.h"
#include "pso_rpm.h"

/*******************************************************************************
 * GLOBAL VARIABLE DEFINITIONS
 ******************************************************************************/

volatile uint32_t g_rpm_value = 0U;
volatile uint32_t g_rpm_ready_flag = 0U;

uint32_t g_edge_interval_us = 0U;
uint32_t g_last_edge_time_us = 0U;
uint32_t g_last_capture_value = 0U;
uint32_t g_edge_valid_count = 0U;
uint32_t g_edge_timeout_counter = 0U;

static uint32_t rpm_filter_buffer[RPM_FILTER_SAMPLES];
static uint8_t rpm_filter_index = 0U;
static uint8_t rpm_filter_count = 0U;

/*******************************************************************************
 * PUBLIC FUNCTIONS
 ******************************************************************************/

uint32_t rpm_get_value(void)
{
    return g_rpm_value;
}

bool rpm_is_ready(void)
{
    return (g_rpm_ready_flag != 0U);
}

void rpm_clear_ready_flag(void)
{
    IntMasterDisable();
    g_rpm_ready_flag = 0U;
    IntMasterEnable();
}

uint32_t rpm_get_raw_count(void)
{
    return WTIMER1_TAV_R;
}

bool rpm_is_valid(uint32_t rpm)
{
    if (rpm == 0U)
    {
        return true;
    }

    return ((rpm >= RPM_MIN_VALID) && (rpm <= RPM_MAX_VALID));
}

uint32_t rpm_from_frequency(uint32_t frequency_hz, uint32_t pulses_per_rev)
{
    if (pulses_per_rev == 0U)
    {
        return 0U;
    }

    return (frequency_hz * 60U) / pulses_per_rev;
}

uint32_t rpm_to_frequency(uint32_t rpm, uint32_t pulses_per_rev)
{
    return (rpm * pulses_per_rev) / 60U;
}

uint32_t rpm_from_period_us(uint32_t period_us, uint32_t pulses_per_rev)
{
    if ((period_us == 0U) || (pulses_per_rev == 0U))
    {
        return 0U;
    }

    return (uint32_t)(60000000ULL / ((uint64_t)period_us * pulses_per_rev));
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
    uint32_t sum = 0U;
    uint8_t i;

    if (rpm_filter_count == 0U)
    {
        return g_rpm_value;
    }

    for (i = 0U; i < rpm_filter_count; i++)
    {
        sum += rpm_filter_buffer[i];
    }

    return sum / rpm_filter_count;
}

void rpm_update_filter(uint32_t new_rpm)
{
    rpm_filter_buffer[rpm_filter_index] = new_rpm;
    rpm_filter_index = (uint8_t)((rpm_filter_index + 1U) % RPM_FILTER_SAMPLES);

    if (rpm_filter_count < RPM_FILTER_SAMPLES)
    {
        rpm_filter_count++;
    }
}

void rpm_reset_filter(void)
{
    uint8_t i;

    for (i = 0U; i < RPM_FILTER_SAMPLES; i++)
    {
        rpm_filter_buffer[i] = 0U;
    }

    rpm_filter_index = 0U;
    rpm_filter_count = 0U;
}

void rpm_reset(void)
{
    IntMasterDisable();

    g_rpm_value = 0U;
    g_rpm_ready_flag = 0U;
    g_edge_interval_us = 0U;
    g_last_edge_time_us = 0U;
    g_last_capture_value = 0U;
    g_edge_valid_count = 0U;
    g_edge_timeout_counter = 0U;
    rpm_reset_filter();

    IntMasterEnable();
}
