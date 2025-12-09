#ifndef PSO_PWM_H
#define PSO_PWM_H

#include <stdint.h>
#include <stdbool.h>

#define TIMER3_10HZ     0.1

/* Return value macros */
#define FUNCTION_RUNNING            0U
#define FUNCTION_COMPLETE           1U
#define FUNCTION_FAIL               0U

/* PWM Profile Types */
typedef enum {
    PWM_PROFILE_NONE = 0U,
    PWM_PROFILE_TRAPEZOID = 1U,
    PWM_PROFILE_LINEAR = 2U,
    PWM_PROFILE_STEP = 3U,
    PWM_PROFILE_CUSTOM = 4U,
    PWM_PROFILE_SINE = 5U,
    PWM_PROFILE_EXPONENTIAL = 6U
} pwm_profile_t;

/* Profile Configuration Structures */
typedef struct {
    uint32_t duration_ms;      /* Total profile duration in ms */
    uint32_t ramp_up_ms;       /* Ramp up time in ms */
    uint32_t hold_ms;          /* Hold time at max in ms */
    uint32_t ramp_down_ms;     /* Ramp down time in ms */
    uint8_t  min_value;        /* Minimum PWM value (0-100%) */
    uint8_t  max_value;        /* Maximum PWM value (0-100%) */
    uint8_t  cycles;           /* Number of cycles to repeat (0 = infinite) */
    bool     auto_repeat;      /* Auto-repeat when complete */
} trapezoid_config_t;

typedef struct {
    uint32_t duration_ms;      /* Total profile duration in ms */
    uint8_t  start_value;      /* Start PWM value (0-100%) */
    uint8_t  end_value;        /* End PWM value (0-100%) */
    uint8_t  cycles;           /* Number of cycles to repeat */
    bool     bidirectional;    /* Ramp up then down if true */
    float    slew_rate;        /* Rate of change (%/ms) - if non-zero overrides duration */
} linear_config_t;

typedef struct {
    uint32_t step_interval_ms; /* Time per step in ms */
    uint8_t  num_steps;        /* Number of steps */
    uint8_t  steps[20];        /* Step values (0-100%) - max 20 steps */
    uint8_t  cycles;           /* Number of cycles to repeat */
    bool     ping_pong;        /* Go forward then backward through steps */
} step_config_t;

typedef struct {
    uint32_t duration_ms;      /* Total profile duration in ms */
    uint8_t  amplitude;        /* Amplitude (0-100%) */
    uint8_t  offset;           /* DC offset (0-100%) */
    uint32_t period_ms;        /* Period in ms */
    uint8_t  cycles;           /* Number of cycles to repeat */
    float    phase_deg;        /* Phase shift in degrees */
} sine_config_t;

typedef struct {
    uint32_t duration_ms;      /* Total profile duration in ms */
    uint8_t  start_value;      /* Start PWM value (0-100%) */
    uint8_t  end_value;        /* End PWM value (0-100%) */
    float    time_constant_ms; /* Time constant for exponential (ms) */
    uint8_t  cycles;           /* Number of cycles to repeat */
    bool     rise_fall;        /* true = rise, false = fall exponential */
} exponential_config_t;

/* Main Profile Configuration Union */
typedef union {
    trapezoid_config_t   trapezoid;
    linear_config_t      linear;
    step_config_t        step;
    sine_config_t        sine;
    exponential_config_t exponential;
} profile_config_t;

/* Function Prototypes */
uint8_t set_pwm_position(uint8_t pos);

/* New PWM Profile Functions with configurable parameters */
uint8_t execute_trapezoid_profile(uint32_t elapsed_ms, const trapezoid_config_t* config);
uint8_t execute_linear_profile(uint32_t elapsed_ms, const linear_config_t* config);
uint8_t execute_step_profile(uint32_t elapsed_ms, const step_config_t* config);
uint8_t execute_sine_profile(uint32_t elapsed_ms, const sine_config_t* config);
uint8_t execute_exponential_profile(uint32_t elapsed_ms, const exponential_config_t* config);
uint8_t execute_custom_profile(uint32_t elapsed_ms);

/* Configuration helpers */
void pwm_set_trapezoid_config(const trapezoid_config_t* config);
void pwm_set_linear_config(const linear_config_t* config);
void pwm_set_step_config(const step_config_t* config);
void pwm_set_sine_config(const sine_config_t* config);
void pwm_set_exponential_config(const exponential_config_t* config);

/* Profile templates (pre-configured common profiles) */
const trapezoid_config_t* pwm_get_trapezoid_template_standard(void);
const trapezoid_config_t* pwm_get_trapezoid_template_soft_start(void);
const linear_config_t* pwm_get_linear_template_slow_ramp(void);
const linear_config_t* pwm_get_linear_template_fast_ramp(void);
const step_config_t* pwm_get_step_template_5steps(void);
const step_config_t* pwm_get_step_template_10steps(void);
const sine_config_t* pwm_get_sine_template_slow(void);
const sine_config_t* pwm_get_sine_template_fast(void);

/* PWM Configuration */
void pwm_profile_init(void);
void pwm_set_throttle(uint8_t throttle);
uint8_t pwm_get_current_throttle(void);

/* Global PWM value for monitoring */
extern uint8_t g_pwm_value;

/* Profile execution control */
uint8_t pwm_profile_is_running(void);
void pwm_profile_start(pwm_profile_t profile);
void pwm_profile_start_with_config(pwm_profile_t profile, const profile_config_t* config);
void pwm_profile_stop(void);
uint8_t pwm_profile_execute(void);
pwm_profile_t pwm_get_current_profile(void);

#endif  // PSO_PWM_H