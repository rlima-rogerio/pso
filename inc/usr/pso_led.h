/*******************************************************************************
 * FILE:        pso_led.h
 *
 * DESCRIPTION:
 *      LED control API using direct register access (no DriverLib).
 *      Naming convention: led_<color>_<action>()
 *
 * LED PIN MAP (TM4C123GH6PM – LaunchPad):
 *      PF1 – Red
 *      PF2 – Blue
 *      PF3 – Green
 *
 ******************************************************************************/

#ifndef PSO_LED_H_
#define PSO_LED_H_

/* Single-color LED control */
void led_red_on(void);
void led_red_off(void);
void led_green_on(void);
void led_green_off(void);
void led_blue_on(void);
void led_blue_off(void);

/* Toggles */
void led_red_toggle(void);
void led_blue_toggle(void);
void led_green_toggle(void);

/* Multi-color combinations */
void led_white_on(void);
void led_white_off(void);
void led_white_toggle(void);

void led_cyan_on(void);
void led_cyan_off(void);
void led_cyan_toggle(void);

void led_purple_on(void);
void led_purple_off(void);
void led_purple_toggle(void);

void led_yellow_on(void);
void led_yellow_off(void);
void led_yellow_toggle(void);

/* Global control */
void led_all_off(void);

#endif /* PSO_LED_H_ */
