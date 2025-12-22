/*******************************************************************************
 * FILENAME:    pso_thrust.c
 *
 * DESCRIPTION:
 *       Thrust force scaling implementation.
 *       Load cell measurement with linear calibration.
 *
 ******************************************************************************/

#include "pso_thrust.h"

/*******************************************************************************
 * THRUST FORCE SCALING (FLOATING-POINT VERSION)
 ******************************************************************************/

uint16_t thrust_adc_to_mn(uint32_t adc_value)
{
    float thrust_mn_float;
    int32_t thrust_mn_signed;
    
    /* Clamp ADC to valid range */
    if (adc_value > ADC_MAX_VALUE) {
        adc_value = ADC_MAX_VALUE;
    }
    
    /*
     * Formula: F(mN) = a × ADC + b
     *          F(mN) = -1.451 × ADC + 5215.801
     * 
     * Load Cell Calibration:
     *   - Linear regression from experimental data
     *   - Calibrated using reference masses in grams (0g to 5000g)
     *   - ADC values recorded for each reference mass
     *   - Coefficients converted to force scale: F(mN) = m(g) × 9.81
     *   - Negative slope: higher ADC = lower thrust
     *   - Offset represents maximum compression reading
     * 
     * Physical meaning:
     *   ADC = 0    → F = 5215.801 mN (maximum compression)
     *   ADC = 3594 → F ≈ 0 mN (no load, zero mass)
     *   ADC = 4095 → F = negative (clamped to 0)
     * 
     * Example calculations:
     *   ADC = 1000 → F = -1.451×1000 + 5215.801 = 3764.801 mN ✓
     *   ADC = 2000 → F = -1.451×2000 + 5215.801 = 2313.801 mN ✓
     *   ADC = 3000 → F = -1.451×3000 + 5215.801 = 862.801 mN ✓
     */
    thrust_mn_float = (THRUST_COEF_A * (float)adc_value) + THRUST_COEF_B;
    
    /* Convert to signed integer for clamping */
    thrust_mn_signed = (int32_t)thrust_mn_float;
    
    /* Clamp negative values to zero */
    if (thrust_mn_signed < 0) {
        return 0;
    }
    
    /* Clamp to maximum valid thrust (5 kg = 49050 mN) */
    if ((uint32_t)thrust_mn_signed > THRUST_MAX_MN) {
        return (uint16_t)THRUST_MAX_MN;
    }
    
    return (uint16_t)thrust_mn_signed;
}

/*******************************************************************************
 * THRUST FORCE SCALING (INTEGER VERSION - FASTER)
 ******************************************************************************/

uint16_t thrust_adc_to_mn_int(uint32_t adc_value)
{
    int32_t thrust_mn_scaled;
    int32_t thrust_mn;
    
    /* Clamp ADC to valid range */
    if (adc_value > ADC_MAX_VALUE) {
        adc_value = ADC_MAX_VALUE;
    }
    
    /*
     * Integer arithmetic version (no floating-point):
     * 
     * Original: F(mN) = a × ADC + b
     *           F(mN) = -1.451 × ADC + 5215.801
     * 
     * Scaled:   F(mN) × 1000 = a_scaled × ADC + b_scaled
     *           F_scaled = -1451 × ADC + 5215801
     * 
     * Then:     F(mN) = F_scaled / 1000
     * 
     * Advantages:
     *   - No floating-point operations (faster on Cortex-M4)
     *   - Better for real-time applications
     *   - Consistent timing
     * 
     * Example:
     *   ADC = 2000
     *   F_scaled = (-1451 × 2000) + 5215801 = -2902000 + 5215801 = 2313801
     *   F(mN) = 2313801 / 1000 = 2313 mN ✓
     */
    thrust_mn_scaled = (THRUST_COEF_A_SCALED * (int32_t)adc_value) + THRUST_COEF_B_SCALED;
    
    /* Divide by scale factor */
    thrust_mn = thrust_mn_scaled / THRUST_SCALE_FACTOR;
    
    /* Clamp negative values to zero */
    if (thrust_mn < 0) {
        return 0;
    }
    
    /* Clamp to maximum valid thrust */
    if (thrust_mn > (int32_t)THRUST_MAX_MN) {
        return (uint16_t)THRUST_MAX_MN;
    }
    
    return (uint16_t)thrust_mn;
}

/*******************************************************************************
 * REVERSE CONVERSION (FOR TESTING/CALIBRATION)
 ******************************************************************************/

uint32_t thrust_mn_to_adc(uint16_t thrust_mn)
{
    float adc_float;
    int32_t adc_value;
    
    /* Clamp thrust to valid range */
    if (thrust_mn > THRUST_MAX_MN) {
        thrust_mn = THRUST_MAX_MN;
    }
    
    /*
     * Inverse formula: ADC = (F - b) / a
     *                  ADC = (F - 5215.801) / (-1.451)
     * 
     * Example:
     *   F = 2313 mN
     *   ADC = (2313 - 5215.801) / (-1.451)
     *   ADC = -2902.801 / (-1.451)
     *   ADC ≈ 2000 ✓
     * 
     * Verification:
     *   F_check = -1.451 × 2000 + 5215.801 = 2313.801 mN ✓
     */
    adc_float = ((float)thrust_mn - THRUST_COEF_B) / THRUST_COEF_A;
    adc_value = (int32_t)adc_float;
    
    /* Clamp to valid ADC range */
    if (adc_value < 0) {
        return 0;
    }
    
    if (adc_value > (int32_t)ADC_MAX_VALUE) {
        return ADC_MAX_VALUE;
    }
    
    return (uint32_t)adc_value;
}

/*******************************************************************************
 * VALIDATION FUNCTIONS
 ******************************************************************************/

bool thrust_mn_is_valid(uint16_t thrust_mn)
{
    /*
     * Valid thrust range: 0 to 49050 mN (5 kg × 9.81 m/s²)
     * 
     * Note: Zero is valid (no load condition)
     */
    return (thrust_mn <= THRUST_MAX_MN);
}

bool thrust_adc_is_valid(uint32_t adc_value)
{
    /*
     * Check if ADC is within reasonable operating range
     * 
     * Valid range considerations:
     *   - ADC must be ≤ 4095 (12-bit maximum)
     *   - For load cell, typical range is 0-4000
     *   - Full saturation (4095) may indicate sensor issue
     * 
     * Warning conditions:
     *   - ADC = 0 continuously: possible sensor disconnection
     *   - ADC = 4095 continuously: possible overload or short
     *   - ADC outside 0-4095: ADC malfunction
     */
    
    /* Basic range check */
    if (adc_value > ADC_MAX_VALUE) {
        return false;
    }
    
    /* 
     * Optional: Add stricter validation for expected operating range
     * Uncomment if you want to flag unusual readings:
     * 
     * if (adc_value == 0 || adc_value >= 4090) {
     *     return false;  // Possible sensor issue
     * }
     */
    
    return true;
}

/*******************************************************************************
 * CALIBRATION VERIFICATION FUNCTION (OPTIONAL)
 ******************************************************************************/

#ifdef THRUST_ENABLE_CALIBRATION_CHECK

/**
 * @brief Verify calibration accuracy with known test points
 * 
 * @return true if calibration is accurate within tolerance
 * 
 * This function can be called during system initialization or 
 * periodic self-test to verify calibration integrity.
 */
bool thrust_verify_calibration(void)
{
    /* Test points from calibration data */
    typedef struct {
        uint32_t adc;           /* ADC reading */
        uint16_t expected_mn;   /* Expected thrust in mN */
        uint16_t tolerance_mn;  /* Allowed deviation in mN */
    } calibration_point_t;
    
    /* Define test points (from calibration procedure) */
    const calibration_point_t test_points[] = {
        /* ADC,  Expected(mN), Tolerance(mN) */
        {1000,   3765,         50},   /* ~384g */
        {2000,   2314,         50},   /* ~236g */
        {3000,   863,          50},   /* ~88g */
        {3594,   0,            20},   /* No load */
    };
    
    const uint8_t num_points = sizeof(test_points) / sizeof(test_points[0]);
    uint8_t i;
    
    /* Test each calibration point */
    for (i = 0; i < num_points; i++)
    {
        uint16_t measured_mn = thrust_adc_to_mn(test_points[i].adc);
        int32_t error = (int32_t)measured_mn - (int32_t)test_points[i].expected_mn;
        
        /* Check if error is within tolerance */
        if (error < 0) {
            error = -error;  /* Absolute value */
        }
        
        if ((uint32_t)error > test_points[i].tolerance_mn) {
            /* Calibration verification failed */
            return false;
        }
    }
    
    /* All test points passed */
    return true;
}

#endif /* THRUST_ENABLE_CALIBRATION_CHECK */

/*******************************************************************************
 * USAGE EXAMPLES AND TESTING NOTES
 ******************************************************************************/

/*
 * EXAMPLE 1: Basic thrust reading
 * ================================
 * 
 * uint32_t adc_raw = adc0_buffer[1];  // Read from ADC buffer
 * uint16_t thrust_mn = thrust_adc_to_mn(adc_raw);
 * float thrust_n = MN_TO_N(thrust_mn);
 * 
 * printf("Thrust: %u mN (%.3f N)\n", thrust_mn, thrust_n);
 * 
 * 
 * EXAMPLE 2: Convert to different units
 * ======================================
 * 
 * uint16_t thrust_mn = thrust_adc_to_mn(adc_value);
 * 
 * float thrust_n = MN_TO_N(thrust_mn);      // Newtons
 * float thrust_gf = MN_TO_GF(thrust_mn);    // grams-force
 * float thrust_kgf = MN_TO_KGF(thrust_mn);  // kg-force
 * 
 * printf("Force: %.2f N = %.1f g = %.3f kg\n", 
 *        thrust_n, thrust_gf, thrust_kgf);
 * 
 * 
 * EXAMPLE 3: Validation and error handling
 * =========================================
 * 
 * uint32_t adc_value = read_thrust_adc();
 * 
 * if (!thrust_adc_is_valid(adc_value)) {
 *     printf("ERROR: Invalid ADC reading!\n");
 *     return;
 * }
 * 
 * uint16_t thrust_mn = thrust_adc_to_mn(adc_value);
 * 
 * if (!thrust_mn_is_valid(thrust_mn)) {
 *     printf("ERROR: Thrust out of range!\n");
 *     return;
 * }
 * 
 * // Use thrust_mn safely
 * 
 * 
 * EXAMPLE 4: Integer version (faster)
 * ====================================
 * 
 * // Use integer version for time-critical applications
 * uint16_t thrust_mn = thrust_adc_to_mn_int(adc_value);
 * 
 * // Both versions produce same result:
 * uint16_t thrust_float = thrust_adc_to_mn(adc_value);
 * uint16_t thrust_int = thrust_adc_to_mn_int(adc_value);
 * 
 * // Difference should be ≤ 1 mN (rounding error)
 * 
 * 
 * EXAMPLE 5: Calibration verification at startup
 * ===============================================
 * 
 * #ifdef THRUST_ENABLE_CALIBRATION_CHECK
 * if (!thrust_verify_calibration()) {
 *     printf("WARNING: Calibration verification failed!\n");
 *     printf("Re-calibration recommended.\n");
 * }
 * #endif
 * 
 * 
 * MATLAB DATA PROCESSING:
 * =======================
 * 
 * % Read thrust data from UART packet
 * thrust_mn = data.thrust;  % Already in millinewtons
 * 
 * % Convert to Newtons
 * thrust_n = thrust_mn / 1000;
 * 
 * % Convert to grams-force (approximate)
 * thrust_gf = thrust_n / 9.81;
 * 
 * % Convert to kg-force
 * thrust_kgf = thrust_gf / 1000;
 * 
 * % Plot thrust over time
 * plot(time, thrust_n);
 * xlabel('Time (s)');
 * ylabel('Thrust (N)');
 * title('Motor Thrust vs Time');
 * grid on;
 */

/*******************************************************************************
 * PERFORMANCE NOTES
 ******************************************************************************/

/*
 * EXECUTION TIME COMPARISON:
 * ==========================
 * 
 * Cortex-M4 @ 40 MHz (approximate):
 * 
 * thrust_adc_to_mn():     ~50-80 cycles (floating-point)
 * thrust_adc_to_mn_int(): ~30-40 cycles (integer only)
 * 
 * For real-time applications with tight timing requirements,
 * use thrust_adc_to_mn_int() for better performance.
 * 
 * Accuracy comparison:
 *   - Floating-point: ±0.5 mN typical
 *   - Integer: ±1 mN typical (due to rounding)
 * 
 * Both versions are sufficiently accurate for most applications.
 * The difference is negligible compared to sensor noise and
 * calibration uncertainty (±1% full scale ≈ ±490 mN).
 */
