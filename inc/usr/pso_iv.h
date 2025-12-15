/*******************************************************************************
 * FILENAME:    pso_iv.h (CORRETO - COM INA169)
 *
 * DESCRIPTION:
 *       Voltage and current scaling for PSO system with INA169.
 *       
 *       VOLTAGE: Millivolts (mV) - via divisor resistivo
 *       CURRENT: Milliamperes (mA) - via INA169 + shunt 0.5mΩ
 *
 * HARDWARE:
 *       Voltage Divider: R3=1.5kΩ, R4=13.7kΩ
 *       Current Monitor: INA169 + Rshunt=0.5mΩ + RL=110kΩ
 *
 * FORMULAS:
 *       Voltage:  V(mV) = (ADC × 33400) / 4095
 *       Current:  I(mA) = (ADC × 60000) / 4095
 *
 ******************************************************************************/

#ifndef PSO_IV_H_
#define PSO_IV_H_

#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * HARDWARE CONSTANTS
 ******************************************************************************/

/* Voltage Divider */
#define R3_OHM               1500U        /* Upper resistor: 1.5kΩ */
#define R4_OHM               13700U       /* Lower resistor: 13.7kΩ */
#define VBAT_MAX_MV          33400UL      /* Max battery voltage: 33.4V */

/* Current Monitor - INA169 Configuration */
#define RSHUNT_MOHM          0.5f         /* Shunt resistance: 0.5 mΩ */
#define RL_OHM               110000UL     /* Load resistor: 110 kΩ */
#define INA169_GM            0.001f       /* Transconductance: 1000 μA/V */

/* ADC Configuration */
#define ADC_VREF_MV          3300U        /* ADC reference: 3.3V */
#define ADC_MAX_VALUE        4095U        /* 12-bit ADC: 2^12 - 1 */
#define ADC_RESOLUTION       4096U        /* 12-bit: 2^12 */

/* uint16_t limits */
#define UINT16_MAX_VAL       65535U       /* Maximum uint16_t value */

/*******************************************************************************
 * DERIVED CONSTANTS
 ******************************************************************************/

/*
 * VOLTAGE CALCULATION:
 * -------------------
 * V_div = R3 / (R3 + R4) = 1500 / (1500 + 13700) = 0.09868
 * Vmax_adc = V_div × Vbat_max = 0.09868 × 33.4V = 3.296V
 * 
 * Formula: V(mV) = (ADC × 33400) / 4095
 * 
 * 
 * CURRENT CALCULATION (INA169):
 * -----------------------------
 * Transfer function: Vout = (Is × Rs) × gm × RL
 * 
 * Where:
 *   Is = shunt current
 *   Rs = shunt resistance = 0.5 mΩ = 0.0005 Ω
 *   gm = transconductance = 1000 μA/V = 0.001 A/V
 *   RL = load resistor = 110 kΩ = 110000 Ω
 * 
 * Vout = (Is × 0.0005) × 0.001 × 110000
 * Vout = Is × 0.055
 * 
 * Solving for Is:
 * Is = Vout / 0.055
 * 
 * Maximum current (Vout = 3.3V):
 * Is_max = 3.3 / 0.055 = 60 A
 * 
 * ADC to current:
 * Vadc = (ADC / 4095) × 3.3
 * Is = Vadc / 0.055
 * Is = (ADC × 3.3) / (4095 × 0.055)
 * Is = (ADC × 3.3) / 225.225
 * Is(mA) = (ADC × 3300) / 225.225
 * Is(mA) = ADC × 14.652
 * 
 * Or more precisely:
 * Is(mA) = (ADC × 60000) / 4095
 */

/* Maximum current in mA */
#define IMAX_MA              60000UL      /* 60 A = 60000 mA */

/* Scaling factor: Vout/Is = Rs × gm × RL */
#define VOUT_PER_AMP         0.055f       /* 0.055 V/A */

/*******************************************************************************
 * PRIMARY FUNCTIONS
 ******************************************************************************/

/**
 * @brief Convert ADC to voltage in millivolts
 * 
 * @param adc_value Raw ADC (0-4095)
 * @return Voltage in mV (0-33400)
 * 
 * Formula: V(mV) = (ADC × 33400) / 4095
 * 
 * Example:
 *   ADC = 2048 → 16704 mV (16.704V)
 *   ADC = 4095 → 33400 mV (33.400V)
 * 
 * MATLAB: V = voltage_mv / 1000
 */
uint16_t voltage_adc_to_mv(uint32_t adc_value);

/**
 * @brief Convert ADC to current in milliamperes
 * 
 * @param adc_value Raw ADC (0-4095)
 * @return Current in mA (0-60000)
 * 
 * Formula: I(mA) = (ADC × 60000) / 4095
 * 
 * Based on INA169 transfer function:
 *   Vout = (Is × Rs) × gm × RL
 *   Vout = Is × 0.055 [V/A]
 *   Is = Vout / 0.055
 * 
 * Where Vout = (ADC / 4095) × 3.3
 * 
 * Example:
 *   ADC = 682  → 10000 mA (10.0 A)
 *   ADC = 2048 → 30015 mA (30.0 A)
 *   ADC = 4095 → 60000 mA (60.0 A)
 * 
 * MATLAB: A = current_ma / 1000
 */
uint16_t current_adc_to_ma(uint32_t adc_value);

/*******************************************************************************
 * REVERSE CONVERSION FUNCTIONS (FOR TESTING)
 ******************************************************************************/

/**
 * @brief Convert voltage in mV back to expected ADC value
 */
uint32_t voltage_mv_to_adc(uint16_t voltage_mv);

/**
 * @brief Convert current in mA back to expected ADC value
 */
uint32_t current_ma_to_adc(uint16_t current_ma);

/*******************************************************************************
 * VALIDATION FUNCTIONS
 ******************************************************************************/

/**
 * @brief Check if voltage reading is within valid range
 */
bool voltage_mv_is_valid(uint16_t voltage_mv);

/**
 * @brief Check if current reading is within valid range
 */
bool current_ma_is_valid(uint16_t current_ma);

/*******************************************************************************
 * CONVERSION MACROS
 ******************************************************************************/

/* mV → V */
#define MV_TO_V(mv)   ((float)(mv) / 1000.0f)

/* mA → A */
#define MA_TO_A(ma)   ((float)(ma) / 1000.0f)

/* V → mV */
#define V_TO_MV(v)    ((uint16_t)((v) * 1000.0f))

/* A → mA */
#define A_TO_MA(a)    ((uint16_t)((a) * 1000.0f))

/*******************************************************************************
 * REFERENCE TABLE
 ******************************************************************************/

/*
 * ADC → VOLTAGE AND CURRENT CONVERSION TABLE:
 * 
 * ADC  | Voltage (mV) | Voltage (V) | Current (mA) | Current (A)
 * -----|--------------|-------------|--------------|------------
 * 0    | 0            | 0.000       | 0            | 0.00
 * 682  | 5562         | 5.562       | 10000        | 10.00
 * 1024 | 8352         | 8.352       | 15015        | 15.01
 * 2048 | 16704        | 16.704      | 30015        | 30.01
 * 3072 | 25056        | 25.056      | 45023        | 45.02
 * 4095 | 33400        | 33.400      | 60000        | 60.00
 * 
 * ✓ Voltage: uses full 0-33400 mV range (fits in uint16_t)
 * ✓ Current: uses 0-60000 mA range (fits in uint16_t)
 * ✓ Both conversions are DIRECT - no scaling factor needed
 * ✓ MATLAB: V = mV/1000, A = mA/1000 (simple!)
 */

/*******************************************************************************
 * IMPORTANT NOTES
 ******************************************************************************/

/*
 * CIRCUIT ANALYSIS:
 * ================
 * 
 * INA169 Configuration:
 *   - Shunt: 0.5 mΩ
 *   - RL: 110 kΩ
 *   - gm: 1000 μA/V
 * 
 * Transfer Function:
 *   Vout = (Is × Rs) × gm × RL
 *   Vout = Is × (0.0005 × 0.001 × 110000)
 *   Vout = Is × 0.055 [V/A]
 * 
 * Current Range:
 *   Vadc_max = 3.3V
 *   Is_max = 3.3 / 0.055 = 60 A
 * 
 * Resolution:
 *   12-bit ADC → 4096 steps
 *   Current resolution = 60A / 4096 = 14.65 mA/step
 * 
 * MATLAB Conversion:
 *   V = dp.v_motor / 1000;  % mV → V
 *   A = dp.i_motor / 1000;  % mA → A
 */

#endif /* PSO_IV_H_ */
