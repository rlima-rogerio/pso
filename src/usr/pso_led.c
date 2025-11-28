/******************************************************************************
* FILENAME:    pso_led.c
*
* DESCRIPTION:
*       Functions to turn on/off the three LEDs.
*
* FUNCTIONS:
*    void PSO_LEDRedOn    (void);
*    void PSO_LEDRedOff   (void);
*    void PSO_LEDGreenOn  (void);
*    void PSO_LEDGreenOff (void);
*    void PSO_LEDBlueOn   (void);
*    void PSO_LEDBlueOff  (void);
*    void PSO_LEDWhiteOn  (void);
*    void PSO_LEDWhiteOff (void);
*
* NOTES:
*       None.
*
* REFERENCES:
*       None.
*
* START DATE:    16 Aug 2015
*
* CHANGES :
*
* VERSION DATE        WHO                    DETAIL
* 1.0     16 Aug 2015 Rogerio Lima         Start-up coding.
*
* -----------------------------------------------------------------------------
* 2.0
******************************************************************************/


/*****************************************************************************
 * GENERAL INFORMATION
 *     PF0: SW2
 *     PF1: Red LED   (0x02)
 *     PF2: Blue LED  (0x04)
 *     PF3: Green LED (0x08)
 *     PF4: SW1
 *
 *****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "gpio.h"
#include "hw_memmap.h"      /* Macros defining the memory map of the device. */
#include "tm4c123gh6pm.h"

/* Turn on red LED */
void PSO_LEDRedOn (void)
{
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x02);
}

/* Turn off red LED */
void PSO_LEDRedOff (void)
{
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x00);
}

/* Turn on green LED */
void PSO_LEDGreenOn (void)
{
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x08);
}

/* Turn off green LED */
void PSO_LEDGreenOff (void)
{
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x00);
}

/* Turn on blue LED */
void PSO_LEDBlueOn (void)
{
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x04);
}

/* Turn off blue LED */
void PSO_LEDBlueOff (void)
{
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x08);
}
/* Toggle blue LED */
void led_blue_toggle (void)
{
	GPIO_PORTF_DATA_R ^= GPIO_PIN_2;    /* Turn on red LED */
}


/* Turn on white LED */
void PSO_LEDWhiteOn (void)
{
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x0E);
}

/* Turn off white LED */
void PSO_LEDWhiteOff (void)
{
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x00);
}

/* Turn on cyan (blue(pin2-0x04) + green(pin3-0x08)) LED */
void PSO_LEDCyanOn (void)
{
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2|GPIO_PIN_3, 0x0C);
}

/* Turn off cyan LED */
void PSO_LEDCyanOff (void)
{
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2|GPIO_PIN_3, 0x00);
}

/* Turn on purple (red(pin1-0x02) + blue(pin2-0x04)) LED */
void PSO_LEDPurpleOn (void)
{
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2, 0x06);
}

/* Turn off purple LED */
void PSO_LEDPurpleOff (void)
{
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2, 0x00);
}

/* Turn on yellow (red(pin1-0x02) + green(pin3-0x08)) LED */
void PSO_LEDYellowOn (void)
{
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_3, 0x0A);
}

/* Turn off yellow LED */
void PSO_LEDYellowOff (void)
{
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_3, 0x00);
}

/* Turn off all LEDs */
void PSO_LEDAllOff (void)
{
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x00);
}

