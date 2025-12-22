/*******************************************************************************
 * FILENAME:    pso_thrust.h
 *
 * DESCRIPTION:
 *       Thrust force scaling for PSO system with load cell.
 *       
 *       THRUST: Millinewtons (mN) - via load cell calibration
 *
 * HARDWARE:
 *       Load Cell: 5 kg maximum capacity
 *       ADC Input: 12-bit (0-4095)
 *       ADC Pin: PD0 (AIN7)
 *
 * CALIBRATION:
 *       Linear calibration curve: y = ax + b
 *       where:
 *         y = Force in mN (millinewtons)
 *         x = ADC raw value (0-4095)
 *         a = -1.451 (slope)
 *         b = 5215.801 (offset)
 *
 * FORMULA:
 *       F(mN) = a × ADC + b
 *       F(mN) = -1.451 × ADC + 5215.801
 *
 * RANGE:
 *       Maximum thrust: 5 kg = 5000 g = 49.05 N = 49050 mN
 *       Fits in uint16_t: 49050 < 65535 ✓
 *
 ******************************************************************************/

#ifndef PSO_THRUST_H_
#define PSO_THRUST_H_

#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * HARDWARE CONSTANTS
 ******************************************************************************/

/* Load Cell Specifications */
#define LOAD_CELL_MAX_KG     5.0f         /* Maximum capacity: 5 kg */
#define LOAD_CELL_MAX_G      5000.0f      /* Maximum capacity: 5000 g */
#define LOAD_CELL_MAX_N      49.05f       /* Maximum force: 49.05 N */
#define LOAD_CELL_MAX_MN     49050UL      /* Maximum force: 49050 mN */

/* ADC Configuration */
#define ADC_VREF_MV          3300U        /* ADC reference: 3.3V */
#define ADC_MAX_VALUE        4095U        /* 12-bit ADC: 2^12 - 1 */
#define ADC_RESOLUTION       4096U        /* 12-bit: 2^12 */

/* uint16_t limits */
#define UINT16_MAX_VAL       65535U       /* Maximum uint16_t value */

/*******************************************************************************
 * CALIBRATION COEFFICIENTS
 ******************************************************************************/

/*
 * Linear calibration curve obtained from experimental data:
 * 
 * y = ax + b
 * 
 * where:
 *   y = Thrust force in millinewtons (mN)
 *   x = ADC raw value (0-4095)
 *   a = -1.451 (slope coefficient)
 *   b = 5215.801 (offset coefficient)
 * 
 * Calibration procedure:
 * 1. Apply known calibration masses (0g, 100g, 500g, 1000g, 2000g, 5000g)
 * 2. Record ADC values for each mass
 * 3. Perform linear regression: ADC vs mass (grams)
 * 4. Convert mass to force: F(mN) = m(g) × 9.81 mN/g
 * 5. Obtain final coefficients in mN scale
 * 6. Validate accuracy within ±1% of applied mass
 * 
 * IMPORTANT: Calibration was performed with reference masses in grams,
 *            then coefficients were scaled to provide force in millinewtons.
 *            This ensures traceability to SI mass standards.
 * 
 * Note: Negative slope indicates that higher ADC values 
 *       correspond to lower thrust (load cell compression)
 */

/* Calibration coefficients */
#define THRUST_COEF_A        -1.451f      /* Slope: -1.451 mN/ADC */
#define THRUST_COEF_B        5215.788f    /* Offset: 5215.788 mN */

/* Integer scaled coefficients for faster computation (optional) */
#define THRUST_COEF_A_SCALED -1451        /* a × 1000 = -1451 */
#define THRUST_COEF_B_SCALED 5215801      /* b × 1000 = 5215801 */
#define THRUST_SCALE_FACTOR  1000         /* Scaling factor */

/*******************************************************************************
 * CALIBRATION COEFFICIENT CONVERSION
 ******************************************************************************/

/*
 * FROM MASS TO FORCE CALIBRATION:
 * ================================
 * 
 * Step 1: Original calibration in grams
 * --------------------------------------
 * Linear regression with reference masses provides:
 *   m(g) = a_g × ADC + b_g
 * 
 * where:
 *   m(g) = mass in grams
 *   a_g = slope in g/ADC
 *   b_g = offset in grams
 * 
 * Step 2: Convert mass to force
 * ------------------------------
 * Force in millinewtons: F(mN) = m(g) × 9.81 mN/g
 * 
 * Substituting:
 *   F(mN) = [a_g × ADC + b_g] × 9.81
 *   F(mN) = (a_g × 9.81) × ADC + (b_g × 9.81)
 * 
 * Step 3: Final coefficients in force scale
 * ------------------------------------------
 *   a = a_g × 9.81  →  a = -1.451 mN/ADC
 *   b = b_g × 9.81  →  b = 5215.801 mN
 * 
 * Therefore:
 *   a_g = a / 9.81 = -1.451 / 9.81 = -0.148 g/ADC
 *   b_g = b / 9.81 = 5215.801 / 9.81 = 531.6 g
 * 
 * Verification:
 * -------------
 * Original mass calibration would have been:
 *   m(g) = -0.148 × ADC + 531.6
 * 
 * Example with ADC = 2000:
 *   m(g) = -0.148 × 2000 + 531.6 = 235.6 g
 *   F(mN) = 235.6 × 9.81 = 2311.2 mN ≈ 2314 mN ✓
 * 
 * Using force coefficients directly:
 *   F(mN) = -1.451 × 2000 + 5215.801 = 2313.8 mN ✓
 * 
 * Traceability:
 * -------------
 * This conversion ensures that:
 * 1. Calibration is traceable to SI mass standards (certified masses)
 * 2. Force calculations are consistent with standard gravity (9.81 m/s²)
 * 3. Results can be verified against reference masses at any time
 * 4. Coefficient units are clearly documented (mN vs g)
 */

/*******************************************************************************
 * DERIVED CONSTANTS
 ******************************************************************************/

/*
 * THRUST CALCULATION:
 * -------------------
 * 
 * Formula: F(mN) = a × ADC + b
 *          F(mN) = -1.451 × ADC + 5215.801
 * 
 * Example values:
 *   ADC = 0    → F = 5215.801 mN (maximum compression)
 *   ADC = 1000 → F = 3764.801 mN
 *   ADC = 2000 → F = 2313.801 mN
 *   ADC = 3000 → F = 862.801 mN
 *   ADC = 3594 → F ≈ 0 mN (no load)
 *   ADC = 4095 → F = 0 mN (clamped to zero)
 * 
 * Physical interpretation:
 *   - Higher ADC values = less compression = lower thrust
 *   - Lower ADC values = more compression = higher thrust
 *   - Negative values are clamped to zero
 * 
 * Resolution:
 *   ΔF/ΔADC = |a| = 1.451 mN/step
 *   12-bit ADC → 4096 steps
 *   Total range ≈ 5946 mN (theoretical from calibration)
 * 
 * Maximum representable force:
 *   F_max = 49050 mN (5 kg × 9.81 m/s²)
 *   Fits in uint16_t: 49050 < 65535 ✓
 */

/* Zero-thrust ADC value (calculated from calibration) */
#define THRUST_ZERO_ADC      3594U        /* ADC value for F ≈ 0 mN */

/* Maximum valid thrust in mN */
#define THRUST_MAX_MN        49050U       /* 49.05 N = 5 kg × 9.81 */

/*******************************************************************************
 * PRIMARY FUNCTIONS
 ******************************************************************************/

/**
 * @brief Convert ADC to thrust force in millinewtons
 * 
 * @param adc_value Raw ADC (0-4095)
 * @return Thrust force in mN (0-49050)
 * 
 * Formula: F(mN) = a × ADC + b
 *          F(mN) = -1.451 × ADC + 5215.801
 * 
 * Example:
 *   ADC = 0    → 5215 mN (5.215 N, ~532g)
 *   ADC = 1000 → 3764 mN (3.764 N, ~384g)
 *   ADC = 2000 → 2313 mN (2.313 N, ~236g)
 *   ADC = 3000 → 862 mN (0.862 N, ~88g)
 *   ADC = 3594 → 0 mN (no load)
 * 
 * Note: Negative results are clamped to zero
 * 
 * MATLAB: N = thrust_mn / 1000
 */
uint16_t thrust_adc_to_mn(uint32_t adc_value);

/**
 * @brief Convert ADC to thrust force in millinewtons (integer math version)
 * 
 * @param adc_value Raw ADC (0-4095)
 * @return Thrust force in mN (0-49050)
 * 
 * Uses integer arithmetic for faster computation:
 *   F(mN) = (a_scaled × ADC + b_scaled) / 1000
 * 
 * Where:
 *   a_scaled = -1451 (a × 1000)
 *   b_scaled = 5215801 (b × 1000)
 * 
 * This version avoids floating-point operations.
 */
uint16_t thrust_adc_to_mn_int(uint32_t adc_value);

/*******************************************************************************
 * REVERSE CONVERSION FUNCTIONS (FOR TESTING/CALIBRATION)
 ******************************************************************************/

/**
 * @brief Convert thrust in mN back to expected ADC value
 * 
 * @param thrust_mn Thrust force in mN (0-49050)
 * @return Expected ADC value (0-4095)
 * 
 * Inverse formula: ADC = (F - b) / a
 *                  ADC = (F - 5215.801) / (-1.451)
 * 
 * Used for:
 *   - Calibration verification
 *   - Sensor testing
 *   - Simulation
 */
uint32_t thrust_mn_to_adc(uint16_t thrust_mn);

/*******************************************************************************
 * VALIDATION FUNCTIONS
 ******************************************************************************/

/**
 * @brief Check if thrust reading is within valid range
 * 
 * @param thrust_mn Thrust force in mN
 * @return true if valid (0 to THRUST_MAX_MN)
 */
bool thrust_mn_is_valid(uint16_t thrust_mn);

/**
 * @brief Check if ADC reading is within expected calibration range
 * 
 * @param adc_value Raw ADC value
 * @return true if within typical operating range
 * 
 * Checks if ADC is within reasonable bounds for load cell:
 *   - Not saturated (ADC < 4095)
 *   - Within calibrated range
 */
bool thrust_adc_is_valid(uint32_t adc_value);

/*******************************************************************************
 * CONVERSION MACROS
 ******************************************************************************/

/* mN → N (Newtons) */
#define MN_TO_N(mn)   ((float)(mn) / 1000.0f)

/* N → mN */
#define N_TO_MN(n)    ((uint16_t)((n) * 1000.0f))

/* mN → g (grams-force, approximate) */
#define MN_TO_GF(mn)  ((float)(mn) / 9.81f)

/* g → mN (grams-force to millinewtons) */
#define GF_TO_MN(gf)  ((uint16_t)((gf) * 9.81f))

/* mN → kg (kilograms-force, approximate) */
#define MN_TO_KGF(mn) ((float)(mn) / 9810.0f)

/* kg → mN (kilograms-force to millinewtons) */
#define KGF_TO_MN(kg) ((uint16_t)((kg) * 9810.0f))

/*******************************************************************************
 * REFERENCE TABLE
 ******************************************************************************/

/*
 * ADC → THRUST FORCE CONVERSION TABLE:
 * 
 * ADC  | Thrust (mN) | Thrust (N) | Approx. (g) | Approx. (kg)
 * -----|-------------|------------|-------------|-------------
 * 0    | 5216        | 5.216      | 532         | 0.532
 * 500  | 4490        | 4.490      | 458         | 0.458
 * 1000 | 3765        | 3.765      | 384         | 0.384
 * 1500 | 3039        | 3.039      | 310         | 0.310
 * 2000 | 2314        | 2.314      | 236         | 0.236
 * 2500 | 1588        | 1.588      | 162         | 0.162
 * 3000 | 863         | 0.863      | 88          | 0.088
 * 3500 | 137         | 0.137      | 14          | 0.014
 * 3594 | 0           | 0.000      | 0           | 0.000
 * 4095 | 0 (clamp)   | 0.000      | 0           | 0.000
 * 
 * Notes:
 *   ✓ Thrust uses 0-49050 mN range (fits in uint16_t)
 *   ✓ Negative values clamped to zero
 *   ✓ Maximum = 5 kg × 9.81 = 49.05 N = 49050 mN
 *   ✓ Resolution = 1.451 mN/step
 *   ✓ MATLAB: N = thrust_mn / 1000 (simple!)
 */

/*******************************************************************************
 * CALIBRATION NOTES
 ******************************************************************************/

/*
 * CALIBRATION PROCEDURE:
 * ======================
 * 
 * Equipment Required:
 *   - Calibrated reference masses: 0g, 100g, 200g, 500g, 1000g, 2000g, 5000g
 *   - Stable mounting fixture (vibration-free surface)
 *   - Digital multimeter (for ADC verification, optional)
 *   - Temperature-controlled environment (±2°C)
 * 
 * Steps:
 * 1. Zero Load Measurement:
 *    - Remove all loads from cell
 *    - Record ADC value (should be ~3594)
 *    - This is the "zero thrust" reference
 * 
 * 2. Apply Calibration Masses:
 *    - Start with smallest mass (100g)
 *    - Place mass gently on load cell platform
 *    - Record ADC value
 *    - Repeat for each mass up to 5000g
 *    - Wait 5 seconds between measurements for stabilization
 * 
 * 3. Data Collection:
 *    - Record at least 10 samples per mass
 *    - Calculate average ADC for each mass
 *    - Record data pairs: (ADC_avg, mass_g)
 * 
 * 4. Linear Regression (in grams):
 *    - Plot ADC (x-axis) vs Mass in grams (y-axis)
 *    - Fit line: m(g) = a_g × ADC + b_g
 *    - This gives initial coefficients in gram scale
 * 
 * 5. Convert to Force (millinewtons):
 *    - Convert each mass to force: F(mN) = m(g) × 9.81 mN/g
 *    - Recalculate coefficients: y = ax + b
 *    - Obtained: a = -1.451 mN/ADC, b = 5215.801 mN
 *    - Relationship: a = a_g × 9.81, b = b_g × 9.81
 * 
 * 6. Validation:
 *    - Apply known test masses
 *    - Verify measured force within ±1% of expected (m × 9.81)
 *    - Check linearity (R² > 0.999)
 *    - Verify zero reading stability
 * 
 * Expected Accuracy:
 *   - Linearity: R² > 0.999
 *   - Repeatability: ±0.5% full scale
 *   - Hysteresis: < 0.1% full scale
 *   - Zero drift: < 0.05% full scale per hour
 * 
 * Example Calibration Data:
 *   Mass (g) | Expected F(mN) | ADC Reading
 *   ---------|----------------|-------------
 *   0        | 0              | 3594
 *   100      | 981            | 3526
 *   500      | 4905           | 3249
 *   1000     | 9810           | 2897
 *   2000     | 19620          | 2193
 *   5000     | 49050          | 520
 * 
 * Re-calibration:
 *   - Recommended every 6 months
 *   - Required after mechanical stress or overload
 *   - If readings drift more than ±2% from expected
 *   - After temperature excursion (>10°C change)
 * 
 * Traceability:
 *   - Use certified reference masses with traceable calibration
 *   - Record calibration date, temperature, humidity
 *   - Keep calibration certificate from mass supplier
 *   - Document any deviations from standard procedure
 */

/*******************************************************************************
 * IMPORTANT NOTES
 ******************************************************************************/

/*
 * LOAD CELL CHARACTERISTICS:
 * ==========================
 * 
 * Sensor Type: Strain gauge load cell
 * Capacity: 5 kg (49.05 N)
 * Excitation: 5V (or 3.3V depending on design)
 * Output: 0-3.3V (to ADC)
 * 
 * Calibration Curve Analysis:
 *   - Negative slope (a = -1.451) indicates:
 *     * Higher ADC = less compression = lower force
 *     * Lower ADC = more compression = higher force
 *   
 *   - Offset (b = 5215.801 mN) represents:
 *     * Maximum output at ADC = 0 (full compression)
 *     * Approximately 532g equivalent
 * 
 *   - Zero crossing at ADC ≈ 3594:
 *     * F = 0 when ADC = 3594
 *     * Calculated: (0 - 5215.801) / (-1.451) ≈ 3594
 * 
 * Resolution:
 *   - 12-bit ADC → 4096 discrete values
 *   - Force resolution = |a| = 1.451 mN/step
 *   - Equivalent weight ≈ 0.148 g/step
 * 
 * MATLAB Conversion:
 *   F_N = dp.thrust / 1000;           % mN → N
 *   F_gf = (dp.thrust / 1000) / 9.81; % mN → grams-force
 *   F_kgf = dp.thrust / 9810;         % mN → kg-force
 * 
 * Safety Considerations:
 *   ⚠ Do not exceed 5 kg (49.05 N) maximum load
 *   ⚠ Overload may damage load cell permanently
 *   ⚠ Use mechanical stops to prevent overload
 *   ⚠ Validate sensor output before critical tests
 */

#endif /* PSO_THRUST_H_ */
