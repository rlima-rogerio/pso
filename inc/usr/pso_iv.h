/*******************************************************************************
 * FILENAME:    pso_iv.h (Based on INA169)
 *
 * DESCRIPTION:
 *       Voltage and current scaling for PSO system with INA169.
 *       
 *       VOLTAGE: Millivolts (mV) - via divisor resistivo
 *       CURRENT: Milliamperes (mA) - via INA169 + shunt 0.5mΩ
 *
 * HARDWARE:
 *       Voltage Divider: R3=1.5kΩ, R4=13.7kΩ
 *       Current Monitor: INA169 + Rshunt=0.5 mOhm + calibrated RL=55 kOhm
 *
 * FORMULAS:
 *       Voltage:  V(mV) = (ADC × 33400) / 4095
 *       Current:  I(mA) = max(ADC - 4, 0) × 16.6
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
#define RSHUNT_MOHM          0.5f         /* Shunt resistance: 0.5 mOhm */
#define RL_OHM               55000UL      /* Calibrated load resistor: 55 kOhm */
#define INA169_GM            0.001f       /* Transconductance: 1000 uA/V */
/* Empirical INA169 calibration (RL = 55 kOhm, measured) */
#define CURRENT_MA_PER_COUNT   166U   /* 16.6 mA per ADC count (scaled ×10) */
#define CURRENT_SCALE_DIV      10U
#define ADC_CURRENT_OFFSET     4U     /* Offset to remove (in ADC counts) */

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
 * The current channel uses the measured hardware calibration implemented in
 * pso_iv.c. The small ADC offset is removed before applying the gain:
 *
 *   I(mA) = max(ADC - ADC_CURRENT_OFFSET, 0) * 16.6
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
 * Formula: I(mA) = max(ADC - 4, 0) × 16.6
 * 
 * Example:
 *   ADC = 4    → 0 mA
 *   ADC = 606  → 9993 mA (approximately 10.0 A)
 *   ADC = 1811 → 29996 mA (approximately 30.0 A)
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
 * 606  | 4943         | 4.943       | 9993         | 9.99
 * 1024 | 8352         | 8.352       | 16932        | 16.93
 * 1811 | 14774        | 14.774      | 29996        | 30.00
 * 3072 | 25056        | 25.056      | 50928        | 50.93
 * 4095 | 33400        | 33.400      | 60000*       | 60.00*
 * 
 * ✓ Voltage: uses full 0-33400 mV range (fits in uint16_t)
 * *4095 is above the nominal calibrated operating limit and saturates at 60 A.
 * ✓ Current: calibrated empirical conversion with low-current offset removal
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
 *   - RL: 55 kOhm calibrated hardware value
 *   - gm: 1000 uA/V
 *
 * Firmware calibration:
 *   I(mA) = max(ADC - 4, 0) * 166 / 10
 * 
 * MATLAB Conversion:
 *   V = dp.v_motor / 1000;  % mV → V
 *   A = dp.i_motor / 1000;  % mA → A
 */

#endif /* PSO_IV_H_ */
