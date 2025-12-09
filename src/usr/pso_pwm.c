/******************************************************************************
* FILENAME:    pso_pwm.c
*
* DESCRIPTION:
*       Functions to generate PWM signals with configurable profiles.
*
******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "gpio.h"
#include "hw_memmap.h"
#include "tm4c123gh6pm.h"
#include "pso_pwm.h"
#include "pso_timing.h"

/* Global PWM value for monitoring */
uint8_t g_pwm_value = 0U;

/* Current profile configuration */
static pwm_profile_t selected_profile = PWM_PROFILE_NONE;
static profile_config_t current_config;
static bool profile_running = false;
static uint8_t current_throttle = 0U;

/* Default configurations */
static const trapezoid_config_t default_trapezoid = {
    .duration_ms = 30000,
    .ramp_up_ms = 5000,
    .hold_ms = 20000,
    .ramp_down_ms = 5000,
    .min_value = 0,
    .max_value = 100,
    .cycles = 1,
    .auto_repeat = false
};

static const linear_config_t default_linear = {
    .duration_ms = 30000,
    .start_value = 0,
    .end_value = 100,
    .cycles = 1,
    .bidirectional = false,
    .slew_rate = 0.0f
};

static const step_config_t default_step = {
    .step_interval_ms = 5000,
    .num_steps = 9,
    .steps = {0, 25, 50, 75, 100, 75, 50, 25, 0},
    .cycles = 1,
    .ping_pong = false
};

/* PWM timing macros */
#define PWM_POSITION_MIN            0U
#define PWM_POSITION_MAX            100U
#define PWM_MIN_PULSE_WIDTH_US      1000U
#define PWM_MID_PULSE_WIDTH_US      1500U
#define PWM_MAX_PULSE_WIDTH_US      2000U
#define PWM_CLOCK_PERIOD_NS         25U
#define US_TO_COUNTS(us)            ((us) * 1000U / PWM_CLOCK_PERIOD_NS)
#define PWM_MIN_DUTY_CYCLE          US_TO_COUNTS(PWM_MIN_PULSE_WIDTH_US)
#define PWM_DUTY_FROM_POSITION(pos) ((400U * (pos)) + PWM_MIN_DUTY_CYCLE)

/*******************************************************************************
* Function Name  : set_pwm_position
*******************************************************************************/
uint8_t set_pwm_position(uint8_t pos)
{
    uint32_t duty_cycle;
    
    if (pos > PWM_POSITION_MAX) pos = PWM_POSITION_MAX;
    
    duty_cycle = PWM_DUTY_FROM_POSITION(pos);
    WTIMER1_TBMATCHR_R = duty_cycle;
    
    /* Update global PWM value */
    g_pwm_value = pos;
    current_throttle = pos;
    
    return FUNCTION_COMPLETE;
}

/*******************************************************************************
* Trapezoid Profile with configurable parameters
*******************************************************************************/
uint8_t execute_trapezoid_profile(uint32_t elapsed_ms, const trapezoid_config_t* config)
{
    static uint8_t last_throttle = 0;
    
    if (!config || config->duration_ms == 0) {
        last_throttle = 0;
        return FUNCTION_COMPLETE;
    }
    
    /* Check if duration reached */
    if (elapsed_ms >= config->duration_ms) {
        pwm_set_throttle(config->min_value);
        return FUNCTION_COMPLETE;
    }
    
    uint8_t throttle = config->min_value;
    
    if (elapsed_ms < config->ramp_up_ms) {
        /* Ramp up */
        throttle = config->min_value + 
                  (uint8_t)((elapsed_ms * (config->max_value - config->min_value)) / 
                           config->ramp_up_ms);
    } 
    else if (elapsed_ms < (config->ramp_up_ms + config->hold_ms)) {
        /* Hold at max */
        throttle = config->max_value;
    }
    else {
        /* Ramp down */
        uint32_t ramp_down_time = elapsed_ms - (config->ramp_up_ms + config->hold_ms);
        if (ramp_down_time < config->ramp_down_ms) {
            throttle = config->max_value - 
                      (uint8_t)((ramp_down_time * (config->max_value - config->min_value)) / 
                               config->ramp_down_ms);
        } else {
            throttle = config->min_value;
        }
    }
    
    /* Apply limits */
    if (throttle > config->max_value) throttle = config->max_value;
    if (throttle < config->min_value) throttle = config->min_value;
    
    if (throttle != last_throttle) {
        pwm_set_throttle(throttle);
        last_throttle = throttle;
    }
    

    g_pwm_value = throttle;

    return FUNCTION_RUNNING;
}

/*******************************************************************************
* Linear Profile with configurable parameters
*******************************************************************************/
uint8_t execute_linear_profile(uint32_t elapsed_ms, const linear_config_t* config)
{
    static uint8_t last_throttle = 0;
    
    if (!config || config->duration_ms == 0) {
        last_throttle = 0;
        return FUNCTION_COMPLETE;
    }
    
    /* Check if duration reached */
    if (elapsed_ms >= config->duration_ms) {
        pwm_set_throttle(config->end_value);
        return FUNCTION_COMPLETE;
    }
    
    uint8_t throttle;
    
    if (!config->bidirectional) {
        /* Simple ramp */
        throttle = config->start_value + 
                  (uint8_t)((elapsed_ms * (config->end_value - config->start_value)) / 
                           config->duration_ms);
    } else {
        /* Bidirectional: ramp up then down */
        uint32_t half_duration = config->duration_ms / 2;
        if (elapsed_ms < half_duration) {
            /* Ramp up */
            throttle = config->start_value + 
                      (uint8_t)((elapsed_ms * (config->end_value - config->start_value)) / 
                               half_duration);
        } else {
            /* Ramp down */
            uint32_t ramp_down_time = elapsed_ms - half_duration;
            throttle = config->end_value - 
                      (uint8_t)((ramp_down_time * (config->end_value - config->start_value)) / 
                               half_duration);
        }
    }
    
    /* Apply limits */
    if (throttle > 100) throttle = 100;
    if (throttle < 0) throttle = 0;
    
    if (throttle != last_throttle) {
        pwm_set_throttle(throttle);
        last_throttle = throttle;
    }
    
    g_pwm_value = throttle;

    return FUNCTION_RUNNING;
}

/*******************************************************************************
* Step Profile with configurable parameters
*******************************************************************************/
uint8_t execute_step_profile(uint32_t elapsed_ms, const step_config_t* config)
{
    static uint8_t last_throttle = 0;
    static uint8_t last_step_index = 0;
    
    if (!config || config->num_steps == 0 || config->step_interval_ms == 0) {
        last_throttle = 0;
        last_step_index = 0;
        return FUNCTION_COMPLETE;
    }
    
    uint32_t total_steps_time = config->num_steps * config->step_interval_ms;
    
    /* Check if duration reached */
    if (elapsed_ms >= total_steps_time) {
        pwm_set_throttle(config->steps[config->num_steps - 1]);
        return FUNCTION_COMPLETE;
    }
    
    /* Determine current step */
    uint8_t step_index = (elapsed_ms / config->step_interval_ms);
    if (step_index >= config->num_steps) {
        step_index = config->num_steps - 1;
    }
    
    uint8_t throttle = config->steps[step_index];
    
    if (throttle != last_throttle || step_index != last_step_index) {
        pwm_set_throttle(throttle);
        last_throttle = throttle;
        last_step_index = step_index;
    }
    
    g_pwm_value = throttle;

    return FUNCTION_RUNNING;
}

/*******************************************************************************
* Custom Profile
*******************************************************************************/
uint8_t execute_custom_profile(uint32_t elapsed_ms)
{
    /* For now, just use a simple trapezoid */
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
* Configuration Setters
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
* Profile Templates
*******************************************************************************/
const trapezoid_config_t* pwm_get_trapezoid_template_standard(void)
{
    return &default_trapezoid;
}

const trapezoid_config_t* pwm_get_trapezoid_template_soft_start(void)
{
    static trapezoid_config_t template = {
        .duration_ms = 60000,
        .ramp_up_ms = 15000,
        .hold_ms = 30000,
        .ramp_down_ms = 15000,
        .min_value = 0,
        .max_value = 80,
        .cycles = 1,
        .auto_repeat = false
    };
    return &template;
}

/*******************************************************************************
* Profile Control Functions
*******************************************************************************/
void pwm_set_throttle(uint8_t throttle)
{
    if (throttle > 100) throttle = 100;
    current_throttle = throttle;
    g_pwm_value = throttle;
    
    /* Update PWM hardware */
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
    selected_profile = profile;
    profile_running = true;
    
    /* Set default configuration based on profile type */
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
    profile_running = false;
    selected_profile = PWM_PROFILE_NONE;
    pwm_set_throttle(0);
}

uint8_t pwm_profile_execute(void)
{
    /* This function needs elapsed_ms parameter - fixed in header */
    return FUNCTION_COMPLETE;
}

/*******************************************************************************
* Legacy functions (kept for compatibility)
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
* Profile Selection
*******************************************************************************/
pwm_profile_t select_pwm_profile(void)
{
    /* Default to trapezoid */
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
* Initialization
*******************************************************************************/
void pwm_profile_init(void)
{
    current_throttle = 0U;
    g_pwm_value = 0U;
    selected_profile = PWM_PROFILE_NONE;
    profile_running = false;
}
