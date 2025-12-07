#ifndef PSO_PWM_H
#define PSO_PWM_H

#include <stdint.h>

#define TIMER3_10HZ     0.1


/* PWM Profile Types */
typedef enum {
    PWM_PROFILE_NONE = 0U,
    PWM_PROFILE_TRAPEZOID = 1U,
    PWM_PROFILE_LINEAR = 2U,
    PWM_PROFILE_STEP = 3U,
    PWM_PROFILE_CUSTOM = 4U
} pwm_profile_t;

/* Function Prototypes */
uint8_t set_pwm_position(uint8_t pos);
uint8_t fun_linear(double delta_t, uint16_t t_f, uint8_t y_i, uint8_t y_f);
void increment(void);
void decrement(void);
uint8_t fun_trapezoid(void);

/* New PWM Profile Functions */
uint8_t execute_trapezoid_profile(uint32_t elapsed_ms);
uint8_t execute_linear_profile(uint32_t elapsed_ms);
uint8_t execute_step_profile(uint32_t elapsed_ms);
uint8_t execute_custom_profile(uint32_t elapsed_ms);
pwm_profile_t select_pwm_profile(void);
const char* get_profile_name(pwm_profile_t profile);

/* PWM Configuration */
void pwm_profile_init(void);
void pwm_set_throttle(uint8_t throttle);
uint8_t pwm_get_current_throttle(void);

#endif  // PSO_PWM_H
