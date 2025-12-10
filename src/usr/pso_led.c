/*******************************************************************************
 * FILE:        pso_led.c
 *
 * DESCRIPTION:
 *     PSO LED Control Module.
 *     Low-level LED driver implemented through direct GPIO register access
 *     for maximum performance and minimal overhead. No DriverLib functions
 *     are used. Provides individual and multi-color LED control for system
 *     status indication and debugging.
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
#include "tm4c123gh6pm.h"

/*******************************************************************************
 * LED PIN MAPPING AND BIT MASKS
 *
 * TM4C123 LaunchPad LED Configuration:
 *     PF1 – Red LED   (Active HIGH)
 *     PF2 – Blue LED  (Active HIGH)
 *     PF3 – Green LED (Active HIGH)
 *
 * NOTES:
 *     - All LEDs are active HIGH (1 = ON, 0 = OFF)
 *     - Direct register access for minimum latency
 *     - Port F is also used for other functions (unlock required for PF0)
 *******************************************************************************/
#define LED_RED     (1U << 1)   /* PF1 = 0x02 */
#define LED_BLUE    (1U << 2)   /* PF2 = 0x04 */
#define LED_GREEN   (1U << 3)   /* PF3 = 0x08 */
#define LED_ALL     (LED_RED | LED_BLUE | LED_GREEN)

/*******************************************************************************
 * FUNCTION GROUP: RED LED CONTROL
 *
 * DESCRIPTION:
 *     Functions for controlling the Red LED (PF1) individually.
 *     Provides ON, OFF, and TOGGLE operations through direct register access.
 *
 * OPERATION:
 *     - ON: Sets PF1 to HIGH level (LED illuminated)
 *     - OFF: Sets PF1 to LOW level (LED extinguished)
 *     - TOGGLE: Inverts PF1 state (HIGH→LOW or LOW→HIGH)
 *
 * NOTES:
 *     - Non-blocking operations with single instruction execution
 *     - Does not affect other Port F pins
 *     - Typically used for error indication or critical alerts
 *******************************************************************************/

void led_red_on(void)
{
    /* Set PF1 HIGH - Red LED ON */
    GPIO_PORTF_DATA_R |= LED_RED;
}

void led_red_off(void)
{
    /* Set PF1 LOW - Red LED OFF */
    GPIO_PORTF_DATA_R &= ~LED_RED;
}

void led_red_toggle(void)
{
    /* Invert PF1 state - Red LED TOGGLE */
    GPIO_PORTF_DATA_R ^= LED_RED;
}

/*******************************************************************************
 * FUNCTION GROUP: GREEN LED CONTROL
 *
 * DESCRIPTION:
 *     Functions for controlling the Green LED (PF3) individually.
 *     Provides ON, OFF, and TOGGLE operations.
 *
 * NOTES:
 *     - Typically used for normal operation indication
 *     - Fast execution for timing-sensitive applications
 *******************************************************************************/

void led_green_on(void)
{
    /* Set PF3 HIGH - Green LED ON */
    GPIO_PORTF_DATA_R |= LED_GREEN;
}

void led_green_off(void)
{
    /* Set PF3 LOW - Green LED OFF */
    GPIO_PORTF_DATA_R &= ~LED_GREEN;
}

void led_green_toggle(void)
{
    /* Invert PF3 state - Green LED TOGGLE */
    GPIO_PORTF_DATA_R ^= LED_GREEN;
}

/*******************************************************************************
 * FUNCTION GROUP: BLUE LED CONTROL
 *
 * DESCRIPTION:
 *     Functions for controlling the Blue LED (PF2) individually.
 *     Provides ON, OFF, and TOGGLE operations.
 *
 * NOTES:
 *     - Typically used for status or mode indication
 *     - Fixed bug in original code: used wrong mask (0x08 instead of LED_BLUE)
 *******************************************************************************/

void led_blue_on(void)
{
    /* Set PF2 HIGH - Blue LED ON */
    GPIO_PORTF_DATA_R |= LED_BLUE;
}

void led_blue_off(void)
{
    /* Set PF2 LOW - Blue LED OFF */
    GPIO_PORTF_DATA_R &= ~LED_BLUE;  /* FIXED: Original code used 0x08 (green) */
}

void led_blue_toggle(void)
{
    /* Invert PF2 state - Blue LED TOGGLE */
    GPIO_PORTF_DATA_R ^= LED_BLUE;
}

/*******************************************************************************
 * FUNCTION GROUP: MULTI-COLOR LED COMBINATIONS (ON/OFF)
 *
 * DESCRIPTION:
 *     Functions for controlling multi-color LED combinations.
 *     Each function turns ON or OFF specific color combinations by
 *     manipulating multiple bits in the Port F data register simultaneously.
 *
 * COLOR COMBINATIONS:
 *     WHITE  = Red + Blue + Green (all LEDs ON)
 *     CYAN   = Blue + Green
 *     PURPLE = Red + Blue
 *     YELLOW = Red + Green
 *
 * NOTES:
 *     - Single register operation for atomic color change
 *     - No intermediate states during color transitions
 *     - Useful for system status coding (different colors = different states)
 *******************************************************************************/

/* WHITE = Red + Blue + Green (all LEDs ON) */
void led_white_on(void)
{
    GPIO_PORTF_DATA_R |= LED_ALL;
}

void led_white_off(void)
{
    GPIO_PORTF_DATA_R &= ~LED_ALL;
}

/* CYAN = Blue + Green */
void led_cyan_on(void)
{
    GPIO_PORTF_DATA_R |= (LED_BLUE | LED_GREEN);
}

void led_cyan_off(void)
{
    GPIO_PORTF_DATA_R &= ~(LED_BLUE | LED_GREEN);
}

/* PURPLE = Red + Blue */
void led_purple_on(void)
{
    GPIO_PORTF_DATA_R |= (LED_RED | LED_BLUE);
}

void led_purple_off(void)
{
    GPIO_PORTF_DATA_R &= ~(LED_RED | LED_BLUE);
}

/* YELLOW = Red + Green */
void led_yellow_on(void)
{
    GPIO_PORTF_DATA_R |= (LED_RED | LED_GREEN);
}

void led_yellow_off(void)
{
    GPIO_PORTF_DATA_R &= ~(LED_RED | LED_GREEN);
}

/*******************************************************************************
 * FUNCTION GROUP: MULTI-COLOR LED TOGGLE FUNCTIONS
 *
 * DESCRIPTION:
 *     Toggle functions for multi-color LED combinations.
 *     Each function toggles ONLY the bits corresponding to the specified
 *     color combination without affecting other Port F pins.
 *
 * OPERATION:
 *     - XOR operation inverts the state of specified LED pins
 *     - Unaffected pins maintain their current state
 *     - Useful for blinking multi-color patterns
 *
 * NOTES:
 *     - Atomic operation - no race conditions
 *     - Preserves state of other Port F GPIOs (e.g., switches, debug pins)
 *     - Efficient single-instruction implementation
 *******************************************************************************/

/* WHITE = Red + Blue + Green */
void led_white_toggle(void)
{
    GPIO_PORTF_DATA_R ^= (LED_RED | LED_BLUE | LED_GREEN);
}

/* CYAN = Blue + Green */
void led_cyan_toggle(void)
{
    GPIO_PORTF_DATA_R ^= (LED_BLUE | LED_GREEN);
}

/* PURPLE = Red + Blue */
void led_purple_toggle(void)
{
    GPIO_PORTF_DATA_R ^= (LED_RED | LED_BLUE);
}

/* YELLOW = Red + Green */
void led_yellow_toggle(void)
{
    GPIO_PORTF_DATA_R ^= (LED_RED | LED_GREEN);
}

/*******************************************************************************
 * FUNCTION: led_all_off
 *
 * DESCRIPTION:
 *     Turns OFF all three LEDs (Red, Blue, Green) simultaneously.
 *     Clears only the LED bits in Port F data register, preserving
 *     the state of other Port F pins.
 *
 * PARAMETERS:
 *     None
 *
 * RETURNS:
 *     void
 *
 * USAGE:
 *     - System initialization
 *     - Error recovery
 *     - Power-saving mode entry
 *
 * NOTES:
 *     - Does not affect PF0 (SW2) or PF4 (SW1) switches
 *     - Atomic operation - all LEDs turn off simultaneously
 *******************************************************************************/
void led_all_off(void)
{
    GPIO_PORTF_DATA_R &= ~LED_ALL;
}