/*******************************************************************************
 * FILENAME:    pso_scaling_int16.c
 *
 * DESCRIPTION:
 *       Implementation of voltage/current scaling for int16_t protocol.
 *
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "pso_scaling_int16.h"

/*******************************************************************************
 * PRIMARY SCALING FUNCTIONS (FOR PROTOCOL)
 ******************************************************************************/

/**
 * @brief Convert raw ADC value to voltage in decivolts (dV)
 */
int16_t voltage_adc_to_dv(uint32_t adc_value)
{
    /* Validate and clamp input */
    if (adc_value > ADC_MAX_VALUE)
    {
        adc_value = ADC_MAX_VALUE;
    }
    
    /*
     * Formula: V(dV) = (adc_value × 334) / 4095
     * 
     * Maximum value: (4095 × 334) / 4095 = 334 dV
     * This safely fits in int16_t (max = 32767)
     */
    uint32_t voltage_dv = (adc_value * VOLTAGE_SCALE_NUM_DV) / VOLTAGE_SCALE_DEN;
    
    return (int16_t)voltage_dv;
}

/**
 * @brief Convert raw ADC value to current in amperes (A)
 */
int16_t current_adc_to_a(uint32_t adc_value)
{
    /* Validate and clamp input */
    if (adc_value > ADC_MAX_VALUE)
    {
        adc_value = ADC_MAX_VALUE;
    }
    
    /*
     * Formula: I(A) = (adc_value × 6600) / 4095
     * 
     * Maximum value: (4095 × 6600) / 4095 = 6600 A
     * This safely fits in int16_t (max = 32767)
     */
    uint32_t current_a = (adc_value * CURRENT_SCALE_NUM_A) / CURRENT_SCALE_DEN;
    
    return (int16_t)current_a;
}

/*******************************************************************************
 * ALTERNATIVE SCALING (HIGHER PRECISION FOR LOW CURRENTS)
 ******************************************************************************/

/**
 * @brief Convert raw ADC value to current in centiamperes (cA)
 */
int16_t current_adc_to_ca(uint32_t adc_value)
{
    /* Validate and clamp input */
    if (adc_value > ADC_MAX_VALUE)
    {
        adc_value = ADC_MAX_VALUE;
    }
    
    /*
     * Formula: I(cA) = (adc_value × 660000) / 4095
     * 
     * WARNING: Result can exceed int16_t range!
     * Maximum safe ADC value: ~2030 (gives ~32000 cA = 320 A)
     * 
     * For higher currents, saturate at INT16_MAX
     */
    uint32_t current_ca = (adc_value * CURRENT_SCALE_NUM_CA) / CURRENT_SCALE_DEN;
    
    /* Saturate at int16_t maximum */
    if (current_ca > 32767)
    {
        current_ca = 32767;
    }
    
    return (int16_t)current_ca;
}

/*******************************************************************************
 * REVERSE CONVERSION FUNCTIONS (FOR TESTING/VALIDATION)
 ******************************************************************************/

/**
 * @brief Convert voltage in decivolts back to ADC value
 */
uint32_t voltage_dv_to_adc(int16_t voltage_dv)
{
    /* Validate input */
    if (voltage_dv < 0)
    {
        voltage_dv = 0;
    }
    if (voltage_dv > VBAT_MAX_DV)
    {
        voltage_dv = VBAT_MAX_DV;
    }
    
    /*
     * Formula: ADC = (voltage_dv × 4095) / 334
     */
    uint32_t adc_value = ((uint32_t)voltage_dv * VOLTAGE_SCALE_DEN) / VOLTAGE_SCALE_NUM_DV;
    
    return adc_value;
}

/**
 * @brief Convert current in amperes back to ADC value
 */
uint32_t current_a_to_adc(int16_t current_a)
{
    /* Validate input */
    if (current_a < 0)
    {
        current_a = 0;
    }
    if (current_a > IMAX_A)
    {
        current_a = IMAX_A;
    }
    
    /*
     * Formula: ADC = (current_a × 4095) / 6600
     */
    uint32_t adc_value = ((uint32_t)current_a * CURRENT_SCALE_DEN) / CURRENT_SCALE_NUM_A;
    
    return adc_value;
}

/*******************************************************************************
 * VALIDATION FUNCTIONS
 ******************************************************************************/

/**
 * @brief Validate voltage reading is in safe operating range
 */
bool voltage_dv_is_valid(int16_t voltage_dv)
{
    return (voltage_dv >= 0 && voltage_dv <= VBAT_MAX_DV);
}

/**
 * @brief Validate current reading is in safe operating range
 */
bool current_a_is_valid(int16_t current_a)
{
    return (current_a >= 0 && current_a <= IMAX_A);
}
