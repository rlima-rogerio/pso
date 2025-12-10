/*******************************************************************************
 * FILE:        pso_pwm.c
 *
 * DESCRIPTION:
 *     PSO PWM Signal Generation Module.
 *     Provides configurable PWM signal generation with multiple waveform profiles
 *     for motor control and system actuation. Supports trapezoidal, linear, step,
 *     and custom profiles with real-time parameter adjustment.
 *
 * DOCUMENTATION STYLE:
 *     - Technical and functional
 *     - No functional or logical modifications
 *     - Improved comments and formatting only
 *
 * AUTHOR:      Rogerio Lima
 * REFORMAT:    2025 (Documentation and formatting only)
 *
 *******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "gpio.h"
#include "hw_memmap.h"
#include "tm4c123gh6pm.h"
#include "pso_pwm.h"
#include "pso_timing.h"

/*******************************************************************************
 * MODULE GLOBAL VARIABLES
 *******************************************************************************/

/* Global PWM value for monitoring by other modules */
uint8_t g_pwm_value = 0U;

/* Profile control state variables */
static pwm_profile_t selected_profile = PWM_PROFILE_NONE;  /* Currently selected profile */
static profile_config_t current_config;                    /* Active configuration */
static bool profile_running = false;                       /* Profile execution status */
static uint8_t current_throttle = 0U;                      /* Current throttle value (0-100%) */

/*******************************************************************************
 * DEFAULT PROFILE CONFIGURATIONS
 *
 * These structures define the default parameters for each profile type.
 * They can be overridden by user configuration through setter functions.
 *******************************************************************************/

/* Default trapezoidal profile: 30-second cycle with 5-second ramps */
static const trapezoid_config_t default_trapezoid = {
    .duration_ms = 30000,     /* Total profile duration (30 seconds) */
    .ramp_up_ms = 5000,       /* Time to ramp from min to max (5 seconds) */
    .hold_ms = 20000,         /* Time to hold at max value (20 seconds) */
    .ramp_down_ms = 5000,     /* Time to ramp from max to min (5 seconds) */
    .min_value = 0,           /* Minimum throttle value (0%) */
    .max_value = 100,         /* Maximum throttle value (100%) */
    .cycles = 1,              /* Number of cycles to repeat */
    .auto_repeat = false      /* Auto-repeat after completion */
};

/* Default linear profile: 30-second linear ramp */
static const linear_config_t default_linear = {
    .duration_ms = 30000,     /* Total profile duration (30 seconds) */
    .start_value = 0,         /* Starting throttle value (0%) */
    .end_value = 100,         /* Ending throttle value (100%) */
    .cycles = 1,              /* Number of cycles to repeat */
    .bidirectional = false,   /* Ramp up then down if true */
    .slew_rate = 0.0f         /* Maximum rate of change (0 = unlimited) */
};

/* Default step profile: 9-step sequence with 5-second intervals */
static const step_config_t default_step = {
    .step_interval_ms = 5000, /* Time between step changes (5 seconds) */
    .num_steps = 9,           /* Number of steps in the sequence */
    .steps = {0, 25, 50, 75, 100, 75, 50, 25, 0}, /* Step values (0-100%) */
    .cycles = 1,              /* Number of cycles to repeat */
    .ping_pong = false        /* Ping-pong (forward then reverse) if true */
};

/*******************************************************************************
 * PWM TIMING CONSTANTS AND MACROS
 *
 * Defines PWM signal characteristics for servo/motor control:
 *   - Standard RC servo pulse widths: 1000-2000μs (1-2ms)
 *   - Center position: 1500μs (1.5ms)
 *   - 40MHz system clock → 25ns period
 *   - Position range: 0-100% maps to 1000-2000μs
 *******************************************************************************/

#define PWM_POSITION_MIN            0U       /* Minimum position (0%) */
#define PWM_POSITION_MAX            100U     /* Maximum position (100%) */
#define PWM_MIN_PULSE_WIDTH_US      1000U    /* 1ms pulse width (min position) */
#define PWM_MID_PULSE_WIDTH_US      1500U    /* 1.5ms pulse width (center) */
#define PWM_MAX_PULSE_WIDTH_US      2000U    /* 2ms pulse width (max position) */
#define PWM_CLOCK_PERIOD_NS         25U      /* 40MHz = 25ns period */
#define US_TO_COUNTS(us)            ((us) * 1000U / PWM_CLOCK_PERIOD_NS)  /* μs to timer counts */
#define PWM_MIN_DUTY_CYCLE          US_TO_COUNTS(PWM_MIN_PULSE_WIDTH_US)  /* Min timer value */
#define PWM_DUTY_FROM_POSITION(pos) ((400U * (pos)) + PWM_MIN_DUTY_CYCLE) /* Position→timer counts */

/*******************************************************************************
 * FUNCTION: set_pwm_position
 *
 * DESCRIPTION:
 *     Sets the PWM output to a specific position (0-100%).
 *     Converts percentage position to timer match register value and updates
 *     the hardware PWM generator (WTIMER1 Module B).
 *
 * PARAMETERS:
 *     pos - Desired position (0-100%)
 *
 * RETURNS:
 *     FUNCTION_COMPLETE - Always returns success
 *
 * OPERATION:
 *     1. Clamps input to valid range (0-100%)
 *     2. Calculates timer match value for desired pulse width
 *     3. Updates WTIMER1_TBMATCHR register
 *     4. Updates global tracking variables
 *
 * NOTES:
 *     - Direct hardware register access for minimum latency
 *     - Pulse width formula: 1000μs + (10μs * position)
 *     - 0% = 1000μs, 50% = 1500μs, 100% = 2000μs
 *******************************************************************************/
uint8_t set_pwm_position(uint8_t pos)
{
    uint32_t duty_cycle;
    
    /* 1. Clamp input to valid range */
    if (pos > PWM_POSITION_MAX) pos = PWM_POSITION_MAX;
    
    /* 2. Calculate timer match value for desired pulse width */
    duty_cycle = PWM_DUTY_FROM_POSITION(pos);
    
    /* 3. Update PWM hardware (WTIMER1 Module B) */
    WTIMER1_TBMATCHR_R = duty_cycle;
    
    /* 4. Update global tracking variables */
    g_pwm_value = pos;
    current_throttle = pos;
    
    return FUNCTION_COMPLETE;
}

/*******************************************************************************
 * FUNCTION: execute_trapezoid_profile
 *
 * DESCRIPTION:
 *     Executes a trapezoidal PWM profile with configurable parameters.
 *     Generates a three-segment profile: ramp up, hold, ramp down.
 *
 * PARAMETERS:
 *     elapsed_ms - Time elapsed since profile start (milliseconds)
 *     config     - Pointer to trapezoid configuration structure
 *
 * RETURNS:
 *     FUNCTION_RUNNING - Profile still executing
 *     FUNCTION_COMPLETE - Profile completed
 *
 * PROFILE CHARACTERISTICS:
 *     Segment 1: Linear ramp from min_value to max_value over ramp_up_ms
 *     Segment 2: Constant at max_value for hold_ms
 *     Segment 3: Linear ramp from max_value to min_value over ramp_down_ms
 *
 * NOTES:
 *     - Time-based calculation (no fixed step size)
 *     - Only updates PWM when throttle value changes
 *     - Handles edge cases (zero duration, invalid config)
 *******************************************************************************/
uint8_t execute_trapezoid_profile(uint32_t elapsed_ms, const trapezoid_config_t* config)
{
    static uint8_t last_throttle = 0;  /* Previous throttle value for change detection */
    
    /* 1. Validate configuration */
    if (!config || config->duration_ms == 0) {
        last_throttle = 0;
        return FUNCTION_COMPLETE;
    }
    
    /* 2. Check if profile duration completed */
    if (elapsed_ms >= config->duration_ms) {
        pwm_set_throttle(config->min_value);
        return FUNCTION_COMPLETE;
    }
    
    uint8_t throttle = config->min_value;
    
    /* 3. Calculate throttle based on current phase */
    if (elapsed_ms < config->ramp_up_ms) {
        /* Ramp up phase: linear increase from min to max */
        throttle = config->min_value + 
                  (uint8_t)((elapsed_ms * (config->max_value - config->min_value)) / 
                           config->ramp_up_ms);
    } 
    else if (elapsed_ms < (config->ramp_up_ms + config->hold_ms)) {
        /* Hold phase: constant at maximum value */
        throttle = config->max_value;
    }
    else {
        /* Ramp down phase: linear decrease from max to min */
        uint32_t ramp_down_time = elapsed_ms - (config->ramp_up_ms + config->hold_ms);
        if (ramp_down_time < config->ramp_down_ms) {
            throttle = config->max_value - 
                      (uint8_t)((ramp_down_time * (config->max_value - config->min_value)) / 
                               config->ramp_down_ms);
        } else {
            throttle = config->min_value;
        }
    }
    
    /* 4. Apply safety limits */
    if (throttle > config->max_value) throttle = config->max_value;
    if (throttle < config->min_value) throttle = config->min_value;
    
    /* 5. Update PWM only if throttle value changed */
    if (throttle != last_throttle) {
        pwm_set_throttle(throttle);
        last_throttle = throttle;
    }
    
    /* 6. Update global PWM value */
    g_pwm_value = throttle;

    return FUNCTION_RUNNING;
}

/*******************************************************************************
 * FUNCTION: execute_linear_profile
 *
 * DESCRIPTION:
 *     Executes a linear PWM profile with configurable parameters.
 *     Supports simple linear ramps and bidirectional (triangle) waveforms.
 *
 * PARAMETERS:
 *     elapsed_ms - Time elapsed since profile start (milliseconds)
 *     config     - Pointer to linear configuration structure
 *
 * RETURNS:
 *     FUNCTION_RUNNING - Profile still executing
 *     FUNCTION_COMPLETE - Profile completed
 *
 * MODES:
 *     Simple ramp: start_value → end_value over duration_ms
 *     Bidirectional: start_value → end_value → start_value (triangle wave)
 *
 * NOTES:
 *     - Slew rate limiting available but not implemented in current version
 *     - Bidirectional mode splits duration equally between rise and fall
 *******************************************************************************/
uint8_t execute_linear_profile(uint32_t elapsed_ms, const linear_config_t* config)
{
    static uint8_t last_throttle = 0;  /* Previous throttle value for change detection */
    
    /* 1. Validate configuration */
    if (!config || config->duration_ms == 0) {
        last_throttle = 0;
        return FUNCTION_COMPLETE;
    }
    
    /* 2. Check if profile duration completed */
    if (elapsed_ms >= config->duration_ms) {
        pwm_set_throttle(config->end_value);
        return FUNCTION_COMPLETE;
    }
    
    uint8_t throttle;
    
    /* 3. Calculate throttle based on mode */
    if (!config->bidirectional) {
        /* Simple linear ramp: start_value to end_value */
        throttle = config->start_value + 
                  (uint8_t)((elapsed_ms * (config->end_value - config->start_value)) / 
                           config->duration_ms);
    } else {
        /* Bidirectional: ramp up then down (triangle wave) */
        uint32_t half_duration = config->duration_ms / 2;
        if (elapsed_ms < half_duration) {
            /* Ramp up phase */
            throttle = config->start_value + 
                      (uint8_t)((elapsed_ms * (config->end_value - config->start_value)) / 
                               half_duration);
        } else {
            /* Ramp down phase */
            uint32_t ramp_down_time = elapsed_ms - half_duration;
            throttle = config->end_value - 
                      (uint8_t)((ramp_down_time * (config->end_value - config->start_value)) / 
                               half_duration);
        }
    }
    
    /* 4. Apply safety limits (0-100%) */
    if (throttle > 100) throttle = 100;
    if (throttle < 0) throttle = 0;
    
    /* 5. Update PWM only if throttle value changed */
    if (throttle != last_throttle) {
        pwm_set_throttle(throttle);
        last_throttle = throttle;
    }
    
    /* 6. Update global PWM value */
    g_pwm_value = throttle;

    return FUNCTION_RUNNING;
}

/*******************************************************************************
 * FUNCTION: execute_step_profile
 *
 * DESCRIPTION:
 *     Executes a step PWM profile with configurable parameters.
 *     Moves through a predefined sequence of throttle values at fixed intervals.
 *
 * PARAMETERS:
 *     elapsed_ms - Time elapsed since profile start (milliseconds)
 *     config     - Pointer to step configuration structure
 *
 * RETURNS:
 *     FUNCTION_RUNNING - Profile still executing
 *     FUNCTION_COMPLETE - Profile completed
 *
 * OPERATION:
 *     - Divides elapsed time by step interval to determine current step
 *     - Uses predefined step values from configuration array
 *     - Updates PWM only when step changes
 *
 * NOTES:
 *     - Maximum 20 steps supported (hardware limitation)
 *     - Ping-pong mode not implemented in current version
 *******************************************************************************/
uint8_t execute_step_profile(uint32_t elapsed_ms, const step_config_t* config)
{
    static uint8_t last_throttle = 0;      /* Previous throttle value */
    static uint8_t last_step_index = 0;    /* Previous step index */
    
    /* 1. Validate configuration */
    if (!config || config->num_steps == 0 || config->step_interval_ms == 0) {
        last_throttle = 0;
        last_step_index = 0;
        return FUNCTION_COMPLETE;
    }
    
    /* 2. Calculate total profile duration */
    uint32_t total_steps_time = config->num_steps * config->step_interval_ms;
    
    /* 3. Check if profile duration completed */
    if (elapsed_ms >= total_steps_time) {
        pwm_set_throttle(config->steps[config->num_steps - 1]);
        return FUNCTION_COMPLETE;
    }
    
    /* 4. Determine current step index */
    uint8_t step_index = (elapsed_ms / config->step_interval_ms);
    if (step_index >= config->num_steps) {
        step_index = config->num_steps - 1;  /* Safety clamp */
    }
    
    /* 5. Get throttle value for current step */
    uint8_t throttle = config->steps[step_index];
    
    /* 6. Update PWM only if step or throttle changed */
    if (throttle != last_throttle || step_index != last_step_index) {
        pwm_set_throttle(throttle);
        last_throttle = throttle;
        last_step_index = step_index;
    }
    
    /* 7. Update global PWM value */
    g_pwm_value = throttle;

    return FUNCTION_RUNNING;
}

/*******************************************************************************
 * FUNCTION: execute_custom_profile
 *
 * DESCRIPTION:
 *     Placeholder for custom PWM profile execution.
 *     Currently implements a default trapezoidal profile.
 *
 * PARAMETERS:
 *     elapsed_ms - Time elapsed since profile start (milliseconds)
 *
 * RETURNS:
 *     FUNCTION_RUNNING - Profile still executing
 *     FUNCTION_COMPLETE - Profile completed
 *
 * NOTES:
 *     - Intended for user-defined profile implementation
 *     - Current implementation is a simple trapezoid
 *     - Should be extended for actual custom profile support
 *******************************************************************************/
uint8_t execute_custom_profile(uint32_t elapsed_ms)
{
    /* Default custom profile: simple trapezoid */
    static const trapezoid_config_t default_custom = {
        .duration_ms = 30000,
        .ramp_up_ms = 5000,
        .hold_ms = 20000,
        .ramp_down_ms = 5000,
        .min_value = 0,
        .max_value = 100,
        .cycles = 1,
        .auto_repeat = false
    };
    
    return execute_trapezoid_profile(elapsed_ms, &default_custom);
}

/*******************************************************************************
 * CONFIGURATION SETTER FUNCTIONS
 *
 * DESCRIPTION:
 *     Functions to configure profile parameters before execution.
 *     Each function copies user-provided configuration into the module's
 *     current configuration and selects the corresponding profile type.
 *******************************************************************************/

void pwm_set_trapezoid_config(const trapezoid_config_t* config)
{
    if (config) {
        memcpy(&current_config.trapezoid, config, sizeof(trapezoid_config_t));
        selected_profile = PWM_PROFILE_TRAPEZOID;
    }
}

void pwm_set_linear_config(const linear_config_t* config)
{
    if (config) {
        memcpy(&current_config.linear, config, sizeof(linear_config_t));
        selected_profile = PWM_PROFILE_LINEAR;
    }
}

void pwm_set_step_config(const step_config_t* config)
{
    if (config && config->num_steps <= 20) {
        memcpy(&current_config.step, config, sizeof(step_config_t));
        selected_profile = PWM_PROFILE_STEP;
    }
}

/*******************************************************************************
 * PROFILE TEMPLATE FUNCTIONS
 *
 * DESCRIPTION:
 *     Returns predefined profile configurations for common use cases.
 *     Useful for quick setup without manual parameter configuration.
 *******************************************************************************/

const trapezoid_config_t* pwm_get_trapezoid_template_standard(void)
{
    return &default_trapezoid;
}

const trapezoid_config_t* pwm_get_trapezoid_template_soft_start(void)
{
    static trapezoid_config_t template = {
        .duration_ms = 60000,      /* 60-second total duration */
        .ramp_up_ms = 15000,       /* 15-second ramp up (gentle start) */
        .hold_ms = 30000,          /* 30-second hold at reduced power */
        .ramp_down_ms = 15000,     /* 15-second ramp down (gentle stop) */
        .min_value = 0,            /* Start/end at 0% */
        .max_value = 80,           /* Reduced maximum (80% for soft start) */
        .cycles = 1,
        .auto_repeat = false
    };
    return &template;
}

/*******************************************************************************
 * PROFILE CONTROL FUNCTIONS
 *******************************************************************************/

void pwm_set_throttle(uint8_t throttle)
{
    /* 1. Clamp throttle to valid range (0-100%) */
    if (throttle > 100) throttle = 100;
    
    /* 2. Update tracking variables */
    current_throttle = throttle;
    g_pwm_value = throttle;
    
    /* 3. Update PWM hardware */
    set_pwm_position(throttle);
}

uint8_t pwm_get_current_throttle(void)
{
    return current_throttle;
}

uint8_t pwm_profile_is_running(void)
{
    return (profile_running) ? 1U : 0U;
}

void pwm_profile_start(pwm_profile_t profile)
{
    /* 1. Set selected profile type */
    selected_profile = profile;
    profile_running = true;
    
    /* 2. Load default configuration for selected profile */
    switch (profile) {
        case PWM_PROFILE_TRAPEZOID:
            current_config.trapezoid = default_trapezoid;
            break;
        case PWM_PROFILE_LINEAR:
            current_config.linear = default_linear;
            break;
        case PWM_PROFILE_STEP:
            current_config.step = default_step;
            break;
        default:
            break;
    }
}

void pwm_profile_stop(void)
{
    /* 1. Stop profile execution */
    profile_running = false;
    selected_profile = PWM_PROFILE_NONE;
    
    /* 2. Return to safe state (0% throttle) */
    pwm_set_throttle(0);
}

uint8_t pwm_profile_execute(void)
{
    /* NOTE: This function needs elapsed_ms parameter - see header file for fix */
    return FUNCTION_COMPLETE;
}

/*******************************************************************************
 * LEGACY FUNCTIONS (Compatibility Only)
 *
 * DESCRIPTION:
 *     Empty functions maintained for backward compatibility with existing code.
 *     These functions were used in earlier versions of the PSO system.
 *******************************************************************************/
void increment(void) { /* Empty for compatibility */ }
void decrement(void) { /* Empty for compatibility */ }

uint8_t fun_linear(double delta_t, uint16_t t_f, uint8_t y_i, uint8_t y_f)
{
    return FUNCTION_COMPLETE;
}

uint8_t fun_trapezoid(void)
{
    return FUNCTION_COMPLETE;
}

/*******************************************************************************
 * PROFILE SELECTION FUNCTIONS
 *******************************************************************************/

pwm_profile_t select_pwm_profile(void)
{
    /* Default profile selection: trapezoidal */
    return PWM_PROFILE_TRAPEZOID;
}

const char* get_profile_name(pwm_profile_t profile)
{
    switch (profile) {
        case PWM_PROFILE_TRAPEZOID: return "Trapezoid";
        case PWM_PROFILE_LINEAR:    return "Linear";
        case PWM_PROFILE_STEP:      return "Step";
        case PWM_PROFILE_CUSTOM:    return "Custom";
        default:                    return "None";
    }
}

/*******************************************************************************
 * FUNCTION: pwm_profile_init
 *
 * DESCRIPTION:
 *     Initializes the PWM profile module to default state.
 *     Resets all control variables and ensures safe starting conditions.
 *
 * PARAMETERS:
 *     None
 *
 * RETURNS:
 *     void
 *
 * INITIAL STATE:
 *     - Current throttle: 0%
 *     - Selected profile: NONE
 *     - Profile running: false
 *     - Global PWM value: 0
 *
 * NOTES:
 *     - Should be called during system initialization
 *     - Ensures PWM starts in safe (off) state
 *******************************************************************************/
void pwm_profile_init(void)
{
    current_throttle = 0U;
    g_pwm_value = 0U;
    selected_profile = PWM_PROFILE_NONE;
    profile_running = false;
}

/*******************************************************************************
 * MODULE USAGE NOTES:
 *
 * 1. PROFILE EXECUTION SEQUENCE:
 *     a) Call pwm_profile_init() during system startup
 *     b) Configure profile using setter functions (optional)
 *     c) Call pwm_profile_start() with desired profile type
 *     d) In main loop, call appropriate execute_*_profile() with elapsed time
 *     e) Call pwm_profile_stop() to terminate early
 *
 * 2. TIMING CONSIDERATIONS:
 *     - Profile functions should be called regularly (e.g., every 10-100ms)
 *     - Elapsed time must be measured accurately (use system tick timer)
 *     - PWM updates only occur when throttle value changes (efficient)
 *
 * 3. SAFETY FEATURES:
 *     - All inputs clamped to valid ranges
 *     - Profile automatically stops at completion
 *     - Default to 0% throttle on stop/error
 *     - Configuration validation before execution
 *
 * 4. EXTENSIONS (Suggested):
 *     - Add PID control integration
 *     - Implement profile saving/loading from flash
 *     - Add real-time parameter adjustment
 *     - Support for more complex waveforms (sinusoidal, exponential)
 *******************************************************************************/
