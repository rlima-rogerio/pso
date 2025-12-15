/*******************************************************************************
 * FILENAME:    pso_scaling_int16.h
 *
 * DESCRIPTION:
 *       Voltage and current scaling functions optimized for int16_t protocol.
 *       Returns values in units that fit within 16-bit signed range.
 *
 * UNITS:
 *       - Voltage: DECIVOLTS (dV = V × 10)  →  Range: 0-334 dV (0-33.4V)
 *       - Current: AMPERES (A)               →  Range: 0-6600 A
 *       - Alternative Current: CENTIAMPERES (cA = A × 100) for precision
 *
 * PROTOCOL INTEGRATION:
 *       These functions are designed to work with ulink_pso_data_t structure
 *       where fields are int16_t.
 *
 ******************************************************************************/

#ifndef PSO_SCALING_INT16_H_
#define PSO_SCALING_INT16_H_

#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * HARDWARE CONFIGURATION CONSTANTS
 ******************************************************************************/

/* Voltage Divider Configuration */
#define R3_OHM                  1500U       /* Resistor R3 in ohms */
#define R4_OHM                  13700U      /* Resistor R4 in ohms */
#define VBAT_MAX_DV             334         /* Maximum battery voltage in dV (33.4V) */

/* Current Shunt Configuration */
#define RSHUNT_MICROOHM         500U        /* Shunt resistance in µΩ (0.5 mΩ) */
#define IMAX_A                  6600        /* Maximum current in A (6600 A) */

/* ADC Configuration */
#define ADC_VREF_MV             3300U       /* ADC reference voltage (mV) */
#define ADC_RESOLUTION          4096U       /* 12-bit ADC: 2^12 = 4096 */
#define ADC_MAX_VALUE           4095U       /* Maximum ADC reading */

/*******************************************************************************
 * SCALING FORMULAS FOR INT16_T
 ******************************************************************************/

/*
 * VOLTAGE SCALING:
 * ----------------
 * Goal: Convert ADC value to decivolts (dV = V × 10) to fit in int16_t
 * 
 * Range: 0 - 334 dV (0 - 33.4V) → fits in int16_t (-32768 to +32767)
 * 
 * Formula:
 *   Vbat(V) = (adc_value × 33.4) / 4095
 *   Vbat(dV) = (adc_value × 334) / 4095
 * 
 * Example:
 *   adc_value = 2048  → V = 16.7V = 167 dV
 *   adc_value = 4095  → V = 33.4V = 334 dV
 */

#define VOLTAGE_SCALE_NUM_DV    334U        /* Numerator: Vmax in decivolts */
#define VOLTAGE_SCALE_DEN       4095U       /* Denominator: ADC max value */

/*
 * CURRENT SCALING - OPTION 1: Amperes (A)
 * ----------------------------------------
 * Goal: Convert ADC value to amperes (A) to fit in int16_t
 * 
 * Range: 0 - 6600 A → fits in int16_t (-32768 to +32767)
 * 
 * Formula:
 *   I(A) = (adc_value × 6600) / 4095
 * 
 * Example:
 *   adc_value = 2048  → I = 3300 A
 *   adc_value = 4095  → I = 6600 A
 */

#define CURRENT_SCALE_NUM_A     6600U       /* Numerator: Imax in amperes */
#define CURRENT_SCALE_DEN       4095U       /* Denominator: ADC max value */

/*
 * CURRENT SCALING - OPTION 2: Centiamperes (cA = A × 100)
 * --------------------------------------------------------
 * Goal: Higher precision for lower currents
 * 
 * Range: 0 - 32767 cA (0 - 327.67 A) → fits in int16_t
 * 
 * Use this if you need better resolution for currents below ~300A
 * For high currents (>300A), use Option 1 (Amperes)
 */

#define CURRENT_SCALE_NUM_CA    660000U     /* Numerator: Imax × 100 */

/*******************************************************************************
 * FUNCTION PROTOTYPES - PRIMARY (FOR PROTOCOL)
 ******************************************************************************/

/**
 * @brief Convert raw ADC value to voltage in decivolts (dV)
 * 
 * @param adc_value Raw ADC reading (0-4095)
 * @return Voltage in decivolts (0-334 dV = 0-33.4V)
 * 
 * This is the PRIMARY function for protocol integration.
 * Result fits in int16_t and can be directly assigned to dp->v_motor.
 * 
 * Receiver side conversion: V = dV / 10.0
 * 
 * Example:
 *   adc_value = 2048  → result = 167 dV (16.7V)
 *   adc_value = 4095  → result = 334 dV (33.4V)
 */
int16_t voltage_adc_to_dv(uint32_t adc_value);

/**
 * @brief Convert raw ADC value to current in amperes (A)
 * 
 * @param adc_value Raw ADC reading (0-4095)
 * @return Current in amperes (0-6600 A)
 * 
 * This is the PRIMARY function for protocol integration.
 * Result fits in int16_t and can be directly assigned to dp->i_motor.
 * 
 * Receiver side conversion: A = value (already in amperes)
 * 
 * Example:
 *   adc_value = 2048  → result = 3300 A
 *   adc_value = 4095  → result = 6600 A
 */
int16_t current_adc_to_a(uint32_t adc_value);

/*******************************************************************************
 * FUNCTION PROTOTYPES - ALTERNATIVE (HIGHER PRECISION)
 ******************************************************************************/

/**
 * @brief Convert raw ADC value to current in centiamperes (cA = A × 100)
 * 
 * @param adc_value Raw ADC reading (0-4095)
 * @return Current in centiamperes (limited to ±32767 cA = ±327.67 A)
 * 
 * Use this for higher precision when measuring low to medium currents.
 * WARNING: Saturates at 327.67 A (int16_t max)
 * 
 * Receiver side conversion: A = cA / 100.0
 * 
 * Example:
 *   adc_value = 512   → result = 8252 cA (82.52 A)
 *   adc_value = 2048  → result = 32767 cA (saturated, actual = 3300 A)
 */
int16_t current_adc_to_ca(uint32_t adc_value);

/*******************************************************************************
 * FUNCTION PROTOTYPES - REVERSE CONVERSION (FOR TESTING)
 ******************************************************************************/

/**
 * @brief Convert voltage in decivolts back to ADC value
 * 
 * @param voltage_dv Voltage in decivolts (0-334)
 * @return Expected ADC reading (0-4095)
 */
uint32_t voltage_dv_to_adc(int16_t voltage_dv);

/**
 * @brief Convert current in amperes back to ADC value
 * 
 * @param current_a Current in amperes (0-6600)
 * @return Expected ADC reading (0-4095)
 */
uint32_t current_a_to_adc(int16_t current_a);

/*******************************************************************************
 * CONVENIENCE MACROS
 ******************************************************************************/

/* Convert decivolts to volts (for display/receiver) */
#define DV_TO_V(dv)             ((float)(dv) / 10.0f)

/* Convert amperes (already in A, no conversion needed) */
#define A_TO_A(a)               ((float)(a))

/* Convert centiamperes to amperes (for display/receiver) */
#define CA_TO_A(ca)             ((float)(ca) / 100.0f)

/* Voltage in range check (100 - 350 dV = 10V - 35V) */
#define VOLTAGE_DV_IN_RANGE(dv) ((dv) >= 100 && (dv) <= 350)

/* Current in range check (0 - 5000 A) */
#define CURRENT_A_IN_RANGE(a)   ((a) >= 0 && (a) <= 5000)

/*******************************************************************************
 * INTEGRATION EXAMPLE
 ******************************************************************************/

/*
 * BEFORE (raw ADC):
 * -----------------
 * dp->v_motor = (int16_t)adc0_buffer[2];  // Raw ADC value (0-4095)
 * dp->i_motor = (int16_t)adc1_buffer[2];  // Raw ADC value (0-4095)
 * 
 * 
 * AFTER (scaled values):
 * ----------------------
 * #include "pso_scaling_int16.h"
 * 
 * dp->v_motor = voltage_adc_to_dv(adc0_buffer[2]);  // In decivolts (0-334)
 * dp->i_motor = current_adc_to_a(adc1_buffer[2]);   // In amperes (0-6600)
 * 
 * 
 * MATLAB RECEIVER:
 * ----------------
 * % Old (with raw ADC):
 * v_motor_V = (dp.v_motor / 4095) * (33.4 / 0.09868);
 * 
 * % New (with decivolts):
 * v_motor_V = dp.v_motor / 10;  % Simple!
 * 
 * % Current (already in amperes):
 * i_motor_A = dp.i_motor;  % No conversion needed!
 */

/*******************************************************************************
 * REFERENCE TABLES
 ******************************************************************************/

/*
 * VOLTAGE CONVERSION TABLE (ADC → decivolts):
 * 
 * ADC Value | Voltage (V) | Voltage (dV) | int16_t Range
 * ----------|-------------|--------------|---------------
 * 0         | 0.00 V      | 0 dV         | ✓ Fits
 * 512       | 4.18 V      | 42 dV        | ✓ Fits
 * 1024      | 8.35 V      | 84 dV        | ✓ Fits
 * 2048      | 16.70 V     | 167 dV       | ✓ Fits
 * 3072      | 25.06 V     | 251 dV       | ✓ Fits
 * 4095      | 33.40 V     | 334 dV       | ✓ Fits (max: 32767)
 * 
 * 
 * CURRENT CONVERSION TABLE (ADC → amperes):
 * 
 * ADC Value | Current (A) | int16_t Range
 * ----------|-------------|---------------
 * 0         | 0 A         | ✓ Fits
 * 512       | 825 A       | ✓ Fits
 * 1024      | 1650 A      | ✓ Fits
 * 2048      | 3300 A      | ✓ Fits
 * 3072      | 4950 A      | ✓ Fits
 * 4095      | 6600 A      | ✓ Fits (max: 32767)
 * 
 * 
 * CURRENT CONVERSION TABLE (ADC → centiamperes) - ALTERNATIVE:
 * 
 * ADC Value | Current (A) | Current (cA) | int16_t Range
 * ----------|-------------|--------------|---------------
 * 0         | 0.00 A      | 0 cA         | ✓ Fits
 * 512       | 82.52 A     | 8252 cA      | ✓ Fits
 * 1024      | 165.04 A    | 16504 cA     | ✓ Fits
 * 2048      | 330.08 A    | 32767 cA     | ✓ Saturated (max)
 * 3072      | 495.12 A    | 32767 cA     | ✗ Saturated
 * 4095      | 660.00 A    | 32767 cA     | ✗ Saturated
 * 
 * Note: Use cA only for currents < 300A
 */

#endif /* PSO_SCALING_INT16_H_ */
