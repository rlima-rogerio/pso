/*******************************************************************************
 * FILE:        pso_system.c
 *
 * DESCRIPTION:
 *     PSO System Status and LED Indication Module.
 *     Provides visual feedback through LED patterns for different system states:
 *     standby, streaming, PWM control, completion, and error conditions.
 *     Uses timer-based LED control synchronized with system events.
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
#include "hw_memmap.h"
#include "hw_types.h"
#include "hw_sysctl.h"
#include "hw_uart.h"
#include "sysctl.h"
#include "gpio.h"
#include "pin_map.h"
#include "hw_gpio.h"
#include "rom.h"
#include "interrupt.h"
#include "timer.h"
#include "pso_init.h"
#include "pso_system.h"
#include "pso_led.h"

/*******************************************************************************
 * GLOBAL VARIABLE DEFINITIONS
 *******************************************************************************/

/* LED toggle flag - set by Timer3A ISR every 100ms */
g_led_toggle_flag = 0U;

/*******************************************************************************
 * FUNCTION: indicate_standby
 *
 * DESCRIPTION:
 *     Indicates system is in standby/ready state.
 *     Blue LED toggles at approximately 5 Hz (every 200ms) when the global
 *     LED toggle flag is set by Timer3A ISR.
 *
 * PARAMETERS:
 *     None
 *
 * RETURNS:
 *     void
 *
 * LED PATTERN:
 *     - Blue LED toggles ON/OFF every 200ms
 *     - Result: 5 Hz blinking blue LED
 *
 * NOTES:
 *     - Synchronized with Timer3A ISR (10 Hz)
 *     - Non-blocking operation
 *     - Alternate implementation (commented) provides independent timing
 *******************************************************************************/
void indicate_standby(void)
{
    static uint16_t blink_counter = 0U;
    
    /* Timer-synchronized toggle: toggles when ISR sets flag */
    if (g_led_toggle_flag)
    {
        led_blue_toggle();
    }

    /* ALTERNATE IMPLEMENTATION (independent timing, commented out):
     * Slow blue LED blink (approximately 1 Hz)
     * if (++blink_counter > 5000U) {
     *     led_blue_toggle();
     *     blink_counter = 0U;
     * }
     */
}

/*******************************************************************************
 * FUNCTION: indicate_streaming
 *
 * DESCRIPTION:
 *     Indicates active data streaming (UART or communication active).
 *     Green LED toggles at approximately 5 Hz (every 200ms) when the global
 *     LED toggle flag is set by Timer3A ISR.
 *
 * PARAMETERS:
 *     None
 *
 * RETURNS:
 *     void
 *
 * LED PATTERN:
 *     - Green LED toggles ON/OFF every 200ms
 *     - Result: 5 Hz blinking green LED
 *
 * NOTES:
 *     - Synchronized with Timer3A ISR (10 Hz)
 *     - Alternate implementation (commented) would keep LED solid ON
 *     - Distinct pattern differentiates from standby mode
 *******************************************************************************/
void indicate_streaming(void)
{
    /* Timer-synchronized toggle: toggles when ISR sets flag */
    if (g_led_toggle_flag)
    {
        led_green_toggle();
    }

    /* ALTERNATE IMPLEMENTATION (commented out):
     * Green LED solid ON during streaming
     * led_green_on();
     */
}

/*******************************************************************************
 * FUNCTION: indicate_pwm_active
 *
 * DESCRIPTION:
 *     Indicates PWM control is active and motor is being driven.
 *     Currently minimal implementation - can be expanded for more complex
 *     LED patterns during PWM operation.
 *
 * PARAMETERS:
 *     None
 *
 * RETURNS:
 *     void
 *
 * NOTES:
 *     - Current implementation does not control LEDs
 *     - Framework provided for future implementation
 *     - Suggested pattern: alternating green/blue at 5 Hz
 *
 * SUGGESTED IMPLEMENTATION (commented):
 *     - Fast alternating green/blue LEDs (5 Hz)
 *     - Clear visual distinction from other states
 *     - Independent timing from Timer3A ISR
 *******************************************************************************/
void indicate_pwm_active(void)
{
    static uint32_t last_blink = 0;
    uint32_t current_time = get_system_ticks();
    
    /* PLACEHOLDER FOR FUTURE IMPLEMENTATION:
     * Fast alternating green/blue LEDs during PWM control (5 Hz blink)
     * if ((current_time - last_blink) >= 200) {
     *     last_blink = current_time;
     *     static uint8_t blink_state = 0;
     *     
     *     if (blink_state == 0) {    
     *         led_green_toggle();
     *     }
     *     blink_state ^= 1U;
     * }
     */
}

/*******************************************************************************
 * FUNCTION: indicate_finish
 *
 * DESCRIPTION:
 *     Indicates successful completion of an operation (e.g., data stream,
 *     calibration, or test sequence). Provides visual confirmation to user.
 *
 * PARAMETERS:
 *     None
 *
 * RETURNS:
 *     void
 *
 * LED PATTERN:
 *     - White LED (all LEDs ON) blinks FINISH_BLINK_COUNT times
 *     - Each blink: 250ms ON, 250ms OFF
 *     - Total sequence: FINISH_BLINK_COUNT × 500ms
 *
 * TIMING:
 *     - Uses SysCtlDelay() for precise timing
 *     - Delay calculated based on system clock (40MHz typical)
 *     - SysCtlClockGet() / 12 ≈ 250ms at 40MHz
 *
 * NOTES:
 *     - Blocking function - CPU busy during LED sequence
 *     - Should be used for brief completion indications only
 *     - Distinct from error indication (different color/pattern)
 *******************************************************************************/
void indicate_finish(void)
{
    uint16_t i;
    
    /* Repeat blink sequence FINISH_BLINK_COUNT times */
    for (i = 0U; i < FINISH_BLINK_COUNT; i++)
    {
        /* White LED ON for 250ms */
        led_white_on();
        SysCtlDelay(SysCtlClockGet() / 12U);  /* 250ms delay at 40MHz */
        
        /* White LED OFF for 250ms */
        led_white_off();
        SysCtlDelay(SysCtlClockGet() / 12U);
    }
}

/*******************************************************************************
 * FUNCTION: indicate_error
 *
 * DESCRIPTION:
 *     Indicates an error or fault condition in the system.
 *     Provides clear visual feedback for debugging and user notification.
 *
 * PARAMETERS:
 *     None
 *
 * RETURNS:
 *     void
 *
 * LED PATTERN:
 *     - Red LED blinks ERROR_BLINK_COUNT times
 *     - Each blink: 500ms ON, 500ms OFF
 *     - Total sequence: ERROR_BLINK_COUNT × 1000ms
 *
 * TIMING:
 *     - Uses SysCtlDelay() for precise timing
 *     - Delay calculated based on system clock (40MHz typical)
 *     - SysCtlClockGet() / 6 ≈ 500ms at 40MHz
 *
 * NOTES:
 *     - Blocking function - CPU busy during LED sequence
 *     - Should be used for critical error notifications
 *     - Distinct pattern from finish indication (different timing/color)
 *     - Consider adding audible alarm for critical errors
 *******************************************************************************/
void indicate_error(void)
{
    uint16_t i;
    
    /* Repeat blink sequence ERROR_BLINK_COUNT times */
    for (i = 0U; i < ERROR_BLINK_COUNT; i++)
    {
        /* Red LED ON for 500ms */
        led_red_on();
        SysCtlDelay(SysCtlClockGet() / 6U);   /* 500ms delay at 40MHz */
        
        /* Red LED OFF for 500ms */
        led_red_off();
        SysCtlDelay(SysCtlClockGet() / 6U);
    }
}

/*******************************************************************************
 * SYSTEM STATUS INDICATION SUMMARY:
 *
 * MODE               LED PATTERN              TIMING/SYNCHRONIZATION
 * ----------------------------------------------------------------------------
 * Standby            Blue blink (toggle)      Timer3A ISR (10Hz) synchronized
 * Streaming          Green blink (toggle)     Timer3A ISR (10Hz) synchronized
 * PWM Active         (Not implemented)        (Future: independent 5Hz timing)
 * Finish             White blink sequence     Blocking, 250ms ON/OFF
 * Error              Red blink sequence       Blocking, 500ms ON/OFF
 *
 * TIMING REFERENCE (40MHz System Clock):
 *     SysCtlClockGet() / 12 = ~250ms delay
 *     SysCtlClockGet() / 6  = ~500ms delay
 *     Timer3A ISR period    = 100ms (10Hz)
 *
 * DESIGN CONSIDERATIONS:
 *     1. Non-blocking vs Blocking Patterns:
 *        - Standby/Streaming: Non-blocking, ISR-synchronized
 *        - Finish/Error: Blocking, precise timing required
 *
 *     2. Color Coding:
 *        - Blue: Standby/Ready
 *        - Green: Active/Streaming
 *        - White: Success/Completion
 *        - Red: Error/Fault
 *
 *     3. Pattern Differentiation:
 *        - Different colors for different states
 *        - Different blink rates/patterns
 *        - Blocking vs non-blocking operation
 *
 * EXTENSIONS (Suggested):
 *     1. Add indicate_warning() for non-critical alerts (yellow/orange)
 *     2. Implement indicate_calibrating() with specific pattern
 *     3. Add pattern sequences for multi-state operations
 *     4. Implement non-blocking error indication using state machine
 *     5. Add audible feedback (buzzer) for critical errors
 *******************************************************************************/
