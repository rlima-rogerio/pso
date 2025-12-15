/*******************************************************************************
 * FILENAME:    pso_iv.c (CORRETO - COM INA169)
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
    if (adc_value > ADC_MAX_VALUE) {
        adc_value = ADC_MAX_VALUE;
    }
    
    /*
     * Formula: I(mA) = (ADC × 60000) / 4095
     * 
     * INA169 Transfer Function:
     *   Vout = (Is × Rs) × gm × RL
     *   Vout = Is × (0.0005 × 0.001 × 110000)
     *   Vout = Is × 0.055 [V/A]
     * 
     * Solving for Is:
     *   Is = Vout / 0.055
     * 
     * Where: Vout = (ADC / 4095) × 3.3
     * 
     * Therefore:
     *   Is = [(ADC / 4095) × 3.3] / 0.055
     *   Is = (ADC × 3.3) / (4095 × 0.055)
     *   Is = (ADC × 3.3) / 225.225
     *   Is(A) = ADC × 0.014652
     *   Is(mA) = ADC × 14.652
     * 
     * Or equivalently:
     *   Is(mA) = (ADC × 60000) / 4095
     * 
     * Maximum current:
     *   ADC = 4095 → 60000 mA (60.0 A)
     * 
     * Example values:
     *   ADC = 682  → 10000 mA (10.0 A)
     *   ADC = 2048 → 30015 mA (30.0 A)
     *   ADC = 4095 → 60000 mA (60.0 A)
     */
    uint32_t current_ma = (adc_value * IMAX_MA) / ADC_MAX_VALUE;
    
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
     * ADC = (I_mA × 4095) / 60000
     */
    uint32_t adc_value = ((uint32_t)current_ma * ADC_MAX_VALUE) / IMAX_MA;
    
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
