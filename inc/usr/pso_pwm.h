#ifndef PSO_PWM_H
#define PSO_PWM_H

#define TIMER3_10HZ     0.1

uint8_t set_pwm_position (uint8_t pos);
uint8_t fun_linear (double delta_t, uint16_t t_f, uint8_t  y_i, uint8_t  y_f);
void increment (void);
void decrement (void);
uint8_t fun_trapezoid (void);

#endif  // PSO_PWM_H
