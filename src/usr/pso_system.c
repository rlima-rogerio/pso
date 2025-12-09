
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
 * LED INDICATION FUNCTIONS
 ******************************************************************************/

/* Indicate system in standby mode */
void indicate_standby(void)
{
    static uint16_t blink_counter = 0U;
    
    led_blue_toggle ();
    /* Slow blue LED blink */
    if (++blink_counter > 5000U)
    {
        led_blue_toggle ();
        blink_counter = 0U;
    }
}


/* Indicate active streaming */
void indicate_streaming(void)
{
    /* Green LED solid on during streaming */
    PSO_LEDGreenOn();
    // PSO_LEDRedOff();
}

/* Indicate PWM control active */
void indicate_pwm_active(void)
{
    static uint32_t last_blink = 0;
    uint32_t current_time = get_system_ticks();
    
    /* Fast alternating green/blue LEDs during PWM control */
    if ((current_time - last_blink) >= 200)  // 5 Hz blink
    {
        last_blink = current_time;
        static uint8_t blink_state = 0;
        
        if (blink_state == 0)
        {
            PSO_LEDGreenOn();
            PSO_LEDBlueOff();
        }
        else
        {
            PSO_LEDGreenOff();
            PSO_LEDBlueOn();
        }
        blink_state ^= 1U;
    }
}

/* Indicate streaming finished successfully */
void indicate_finish(void)
{
    uint16_t i;
    
    for (i = 0U; i < FINISH_BLINK_COUNT; i++)
    {
        PSO_LEDWhiteOn();
        SysCtlDelay(SysCtlClockGet() / 12U);  /* ~250ms delay */
        PSO_LEDWhiteOff();
        SysCtlDelay(SysCtlClockGet() / 12U);
    }
}

/* Indicate error condition */
void indicate_error(void)
{
    uint16_t i;
    
    for (i = 0U; i < ERROR_BLINK_COUNT; i++)
    {
        PSO_LEDRedOn();
        SysCtlDelay(SysCtlClockGet() / 6U);   /* ~500ms delay */
        PSO_LEDRedOff();
        SysCtlDelay(SysCtlClockGet() / 6U);
    }

}