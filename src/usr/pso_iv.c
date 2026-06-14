/*******************************************************************************
 * FILENAME:    pso_iv.c (Based on INA169)
 *
 * DESCRIPTION:
 *       Voltage and current scaling implementation.
 *       Current measurement via INA169 + 0.5mΩ shunt.
 *
 ******************************************************************************/

#include "pso_iv.h"

/*******************************************************************************
 * VOLTAGE SCALING
 ******************************************************************************/

uint16_t voltage_adc_to_mv(uint32_t adc_value)
{
    if (adc_value > ADC_MAX_VALUE) {
        adc_value = ADC_MAX_VALUE;
    }
    
    /*
     * Formula: V(mV) = (ADC × 33400) / 4095
     * 
     * Voltage divider: R3=1.5kΩ, R4=13.7kΩ
     * V_div = 1500 / (1500 + 13700) = 0.09868
     * Vmax_adc = 0.09868 × 33.4V = 3.296V
     * 
     * Max: 33400 mV (fits in uint16_t ✓)
     */
    uint32_t voltage_mv = (adc_value * VBAT_MAX_MV) / ADC_MAX_VALUE;
    
    return (uint16_t)voltage_mv;
}

/*******************************************************************************
 * CURRENT SCALING (INA169)
 ******************************************************************************/

uint16_t current_adc_to_ma(uint32_t adc_value)
{
    /* Saturate ADC value */
    if (adc_value > ADC_MAX_VALUE)
    {
        adc_value = ADC_MAX_VALUE;
    }

    /*
     * Empirical conversion (calibrated in hardware):
     *
     * After reducing RL to 55 kΩ, measurements show:
     *   Vout / I ≈ 48.7 mV/A
     *
     * This corresponds to:
     *   I(mA) ≈ ADC × 16.6
     *
     * A small offset (~3.5 mV ≈ 4 ADC counts) is removed
     * to improve low-current accuracy.
     */

    /* Remove measured offset */
    if (adc_value > ADC_CURRENT_OFFSET)
    {
        adc_value -= ADC_CURRENT_OFFSET;
    }
    else
    {
        adc_value = 0U;
    }

    /* Apply calibrated gain */
    uint32_t current_ma =
        (adc_value * CURRENT_MA_PER_COUNT) / CURRENT_SCALE_DIV;

    if (current_ma > IMAX_MA)
    {
        current_ma = IMAX_MA;
    }

    return (uint16_t)current_ma;
}

/*******************************************************************************
 * REVERSE CONVERSIONS (FOR TESTING/CALIBRATION)
 ******************************************************************************/

uint32_t voltage_mv_to_adc(uint16_t voltage_mv)
{
    if (voltage_mv > VBAT_MAX_MV) {
        voltage_mv = VBAT_MAX_MV;
    }
    
    /*
     * ADC = (V_mV × 4095) / 33400
     */
    uint32_t adc_value = ((uint32_t)voltage_mv * ADC_MAX_VALUE) / VBAT_MAX_MV;
    
    return adc_value;
}

uint32_t current_ma_to_adc(uint16_t current_ma)
{
    if (current_ma > IMAX_MA) {
        current_ma = IMAX_MA;
    }
    
    /*
     * Inverse of current_adc_to_ma():
     *   I(mA) = max(ADC - offset, 0) * CURRENT_MA_PER_COUNT / scale
     *   ADC   = (I(mA) * scale / CURRENT_MA_PER_COUNT) + offset
     */
    uint32_t adc_value =
        (((uint32_t)current_ma * CURRENT_SCALE_DIV) / CURRENT_MA_PER_COUNT) +
        ADC_CURRENT_OFFSET;

    if (adc_value > ADC_MAX_VALUE)
    {
        adc_value = ADC_MAX_VALUE;
    }
    
    return adc_value;
}

/*******************************************************************************
 * VALIDATION FUNCTIONS
 ******************************************************************************/

bool voltage_mv_is_valid(uint16_t voltage_mv)
{
    return (voltage_mv <= VBAT_MAX_MV);
}

bool current_ma_is_valid(uint16_t current_ma)
{
    return (current_ma <= IMAX_MA);
}
