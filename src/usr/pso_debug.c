/*******************************************************************************
 * FILE:        pso_debug.c
 *
 * DESCRIPTION:
 *     PSO Debug and Diagnostic Module.
 *     Contains functions for GPIO debug pins initialization, timing pulse
 *     generation, and execution time measurement for system diagnostics.
 *     Provides hardware debugging capabilities via GPIO pins PD4-PD7.
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

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include "hw_memmap.h"
#include "hw_types.h"
#include "sysctl.h"
#include "gpio.h"
#include "debug.h"
#include "pwm.h"
#include "pin_map.h"
#include "hw_gpio.h"
#include "pso_data.h"
#include "pso_debug.h"

/*******************************************************************************
 * FUNCTION: debug_gpio_init
 * 
 * DESCRIPTION:
 *     Initializes GPIO Port D pins PD4-PD7 as debug output pins.
 *     These pins can be used for timing measurements, state indication,
 *     and other debugging purposes. The configuration follows TM4C123
 *     GPIO setup sequence with proper unlocking of protected pins.
 * 
 * PARAMETERS:
 *     None
 * 
 * RETURNS:
 *     void
 * 
 * PIN ASSIGNMENTS:
 *     PD4: Debug pin 4 (general purpose)
 *     PD5: Debug pin 5 (general purpose)
 *     PD6: Debug pin 6 (general purpose)
 *     PD7: Debug pin 7 (general purpose)
 * 
 * NOTES:
 *     - PD7 requires unlock sequence as it has commit control on TM4C123
 *     - All pins configured as digital outputs with 2mA drive strength
 *     - Initial state is LOW (0V) for all pins
 *     - Configuration is consistent with PSO_GPIOConfig() function
 *******************************************************************************/
void debug_gpio_init(void)
{
    /* 1. Enable clock for GPIO Port D */
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGC2_GPIOD;  /* Enable clock for Port D */
    
    /* 2. Short delay for clock stabilization */
    volatile uint32_t delay = SYSCTL_RCGCGPIO_R;
    (void)delay;  /* Prevent compiler warning for unused variable */
    
    /* 3. Unlock PD7 if necessary (PD7 has commit control on TM4C123) */
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;   /* Unlock register */
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0xF0;             /* Allow changes to PD7-PD4 */
    
    /* 4. Configure direction as output for PD4-PD7 */
    HWREG(GPIO_PORTD_BASE + GPIO_O_DIR) |= 0xF0;            /* PD7-PD4 as outputs */
    
    /* 5. Disable alternate function (use as GPIO) */
    HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) &= ~0xF0;
    
    /* 6. Set drive strength to 2mA */
    HWREG(GPIO_PORTD_BASE + GPIO_O_DR2R) |= 0xF0;
    
    /* 7. Disable pull-up/pull-down (not needed for outputs) */
    HWREG(GPIO_PORTD_BASE + GPIO_O_PUR) &= ~0xF0;
    HWREG(GPIO_PORTD_BASE + GPIO_O_PDR) &= ~0xF0;
    
    /* 8. Enable digital function */
    HWREG(GPIO_PORTD_BASE + GPIO_O_DEN) |= 0xF0;
    
    /* 9. Disable analog function */
    HWREG(GPIO_PORTD_BASE + GPIO_O_AMSEL) &= ~0xF0;
    
    /* 10. Clear PCTL register for GPIO function */
    HWREG(GPIO_PORTD_BASE + GPIO_O_PCTL) &= ~0xFFFF0000;
    
    /* 11. Lock registers again for safety */
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;
    
    /* 12. Initialize all pins to LOW state */
    HWREG(GPIO_PORTD_BASE + GPIO_O_DATA) &= ~0xF0;
    
    /* Optional: Send debug message via UART */
    // const char *msg = "Debug GPIO PD4-PD7 initialized\r\n";
    // while (*msg) UARTCharPut(UART0_BASE, *msg++);
}

/*******************************************************************************
 * FUNCTION: debug_timing_pulse
 * 
 * DESCRIPTION:
 *     Generates a short pulse (approximately 1-2 microseconds) on the
 *     specified debug pin. Useful for timing measurements and trigger
 *     signals when viewed with an oscilloscope or logic analyzer.
 * 
 * PARAMETERS:
 *     debug_pin - GPIO pin mask to generate pulse on (e.g., DEBUG_PIN_6)
 *                 Must be one of: GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7
 * 
 * RETURNS:
 *     void
 * 
 * PULSE CHARACTERISTICS:
 *     - Rise time: Immediate (limited by GPIO speed)
 *     - Pulse width: ~1-2μs (4 NOP instructions at 40MHz = 100ns each)
 *     - Fall time: Immediate (limited by GPIO speed)
 * 
 * USAGE:
 *     - Measure interrupt latency
 *     - Trigger oscilloscope for timing measurements
 *     - Debug state machine transitions
 *******************************************************************************/
void debug_timing_pulse(uint8_t debug_pin)
{
    /* Set pin HIGH to start pulse */
    GPIOPinWrite(DEBUG_GPIO_PORT, debug_pin, debug_pin);
    
    /* Short delay for pulse width (approximately 1-2μs) */
    __asm(" NOP");  /* 1 cycle delay at 40MHz = 25ns */
    __asm(" NOP");  /* 1 cycle delay */
    __asm(" NOP");  /* 1 cycle delay */
    __asm(" NOP");  /* 1 cycle delay */
    
    /* Set pin LOW to end pulse */
    GPIOPinWrite(DEBUG_GPIO_PORT, debug_pin, 0);
}

/*******************************************************************************
 * FUNCTION: debug_timing_measure
 * 
 * DESCRIPTION:
 *     Starts or stops a timing measurement on the specified debug pin.
 *     When called with a valid start_time pointer, it sets the pin HIGH
 *     and records the current time. When called with NULL start_time,
 *     it sets the pin LOW, allowing oscilloscope measurement of the interval.
 * 
 * PARAMETERS:
 *     debug_pin   - GPIO pin mask to use for timing (e.g., DEBUG_PIN_7)
 *     start_time  - Pointer to store start time, or NULL to stop measurement
 * 
 * RETURNS:
 *     void
 * 
 * USAGE EXAMPLE:
 *     uint32_t start;
 *     
 *     // Start timing measurement
 *     debug_timing_measure(DEBUG_PIN_7, &start);
 *     
 *     // Code to measure execution time
 *     some_function_to_measure();
 *     
 *     // Stop timing measurement
 *     debug_timing_measure(DEBUG_PIN_7, NULL);
 *     
 *     // Calculate elapsed time
 *     uint32_t elapsed = get_systick_ms() - start;
 * 
 * NOTES:
 *     - Requires SysTick timer to be configured and running
 *     - Measurement resolution is 1ms (SysTick period)
 *     - Pin remains HIGH during the measured interval
 *******************************************************************************/
void debug_timing_measure(uint8_t debug_pin, uint32_t *start_time)
{
    if (start_time)
    {
        /* START MEASUREMENT: Set pin HIGH and record start time */
        GPIOPinWrite(DEBUG_GPIO_PORT, debug_pin, debug_pin);
        *start_time = get_systick_ms();  /* Record current time in milliseconds */
    }
    else
    {
        /* STOP MEASUREMENT: Set pin LOW */
        GPIOPinWrite(DEBUG_GPIO_PORT, debug_pin, 0);
    }
}

/*******************************************************************************
 * ADDITIONAL DEBUG UTILITIES (Suggested for future implementation):
 * 
 * 1. debug_assert(condition, pin) - Assert with visual feedback
 * 2. debug_led_pattern(pattern) - Visual error codes via LEDs
 * 3. debug_dump_registers() - Dump critical registers via UART
 * 4. debug_performance_counter() - Measure function execution time
 * 
 * CURRENT PIN USAGE IN MAIN SYSTEM:
 *     PD6: ADC timing/debug (DEBUG_ADC_TOGGLE)
 *     PD7: State machine timing (DEBUG_STATE_TOGGLE)
 *******************************************************************************/