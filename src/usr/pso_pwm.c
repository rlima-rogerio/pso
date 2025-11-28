/******************************************************************************
* FILENAME:    led.c
*
* DESCRIPTION:
*       Functions to turn on/off the three LEDs.
*
* FUNCTIONS:
*    void LEDRedOn    (void);
*    void LEDRedOff   (void);
*    void LEDGreenOn  (void);
*    void LEDGreenOff (void);
*    void LEDBlueOn   (void);
*    void LEDBlueOff  (void);
*    void LEDWhiteOn  (void);
*    void LEDWhiteOff (void);
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
#include "tm4c123gh6pm.h"	/* Interrupt and register assignments on the Tiva C LauchPad board */
#include "pso_pwm.h"

uint8_t pwm_throttle = 0U;
extern uint8_t fix_rpm_start_acq;       /* Flag to fix RPM and start of acq. */

static volatile
uint16_t inc, dec;    /* 100Hz decrement timer */

/* Turn off buzzer */

/*******************************************************************************
* Function Name  : set_pwm_position
* Input          : uint8_t pos
* Output         : None
* Return         : 1 if success, 0 if fail
* Description    : Defines the position (servo) or throttle (ESC) in a percen-
*                  tual scale from 0 to 100.
*******************************************************************************/
uint8_t set_pwm_position (uint8_t pos)
{
	uint32_t duty_cycle;
	uint8_t returnval;
    /**************************************************************************
    * Minimum -> 1.0 ms : 1.0ms / 25ns = 40000 = 0x0000.9C40
    * Mid     -> 1.5 ms : 1.5ms / 25ns = 60000 = 0x0000.EA60
    * Maximum -> 2.0 ms : 2.0ms / 25ns = 80000 = 0x0001.3880
    **************************************************************************/

	if( pos >= 0U & pos <= 100U)
	{
	    duty_cycle = 400*pos + 40000;
	    WTIMER1_TBMATCHR_R = duty_cycle;
//	    GPIO_PORTF_DATA_R ^= GPIO_PIN_2;

	    returnval = 1U;
    }
	else
	{
        returnval = 0U;
	}

    return returnval;
}

/* This function must be called in period of 100ms                        */
void increment (void)
{
    uint16_t n;

    n = inc;                        /* 100Hz decrement timer */
    inc = ++n;

}

/* This function must be called in period of 100ms                        */
void decrement (void)
{
    uint16_t n;

    n = dec;                        /* 100Hz decrement timer */
    if (n) dec = --n;

}

/*******************************************************************************
* Function Name  : fun_linear
* Input          : double   delta_t [1/interrupt_period]
*                : uint16_t t_f     [> 0]
*                : uint8_t  y_i     [0,100]
*                : uint8_t  y_f     [0,100]
* Output         : None
* Return         : 1 if success, 0 if fail
* Description    : Generates a linear function based on the final time (tf) and
*                  the start (y_i) and final value (y_f).
*******************************************************************************/
uint8_t fun_linear (double delta_t, uint16_t t_f, uint8_t  y_i, uint8_t  y_f)
{
	double ang_coeff;
	static double lin_coeff;
	uint16_t num_points = 0U;
	uint8_t returnval = 0U;

	num_points = (uint16_t)(t_f / delta_t);
	ang_coeff = (y_f - y_i)/num_points;
	lin_coeff = y_i;

	if (inc >= num_points)      /* Function executed */
	{
	    returnval = 1U;
	    lin_coeff = (double)((inc * ang_coeff) + lin_coeff);
	}
	else// if (ang_coeff >= 0.0)   /* Positive angular coeff. */
	{
		pwm_throttle = (uint8_t)((inc * ang_coeff) + lin_coeff);
		set_pwm_position(pwm_throttle);
		returnval = 0U;
	}



    return returnval;
}

/*******************************************************************************
* Function Name  : fun_trapezoid
* Input          : double   delta_t [1/interrupt_period]
*                : uint16_t t_f     [> 0]
*                : uint8_t  y_i     [0,100]
*                : uint8_t  y_f     [0,100]
* Output         : None
* Return         : 1 if success, 0 if fail
* Description    : Generates a linear function based on the final time (tf) and
*                  the start (y_i) and final value (y_f).
*******************************************************************************/
uint8_t fun_trapezoid (void)
{

	uint8_t returnval = 0U;
	static uint8_t state = 0U;

    /* Finite State Machine (FSM) */
    switch (state)
    {
    /* State 0:  */
        case 0:
        	inc = 0U;
        	state = 1U;
        	returnval = 0U;
        	break;

	/* State 1:*/
		case 1:
			if (fun_linear (TIMER3_10HZ, 3, 0, 0))
			{
				state = 2U;        /* Goes to the next state */
				inc = 0U;          /* Counter reset          */
				fix_rpm_start_acq = 1U;
			}

			break;
	/* State 2: */
		case 2:
			if (fun_linear (TIMER3_10HZ, 2, 10, 10))
			{
				state = 3U;        /* Goes to the next state */
				inc = 0U;          /* Counter reset          */
				fix_rpm_start_acq = 1U;
			}
			break;
	/* State 3: */
		case 3:
			if (fun_linear (TIMER3_10HZ, 2, 20, 20))
			{
				state = 4U;        /* Goes to the next state */
				inc = 0U;          /* Counter reset          */
				fix_rpm_start_acq = 1U;
			}
			break;
	/* State 4: */
			case 4:
				if (fun_linear (TIMER3_10HZ, 2, 30, 30))
				{
					state = 5U;        /* Goes to the next state */
					inc = 0U;          /* Counter reset          */
					fix_rpm_start_acq = 1U;
				}
				break;
	/* State 5: */
		case 5:
			if (fun_linear (TIMER3_10HZ, 2, 40, 40))
			{
				state = 6U;        /* Goes to the next state */
				inc = 0U;          /* Counter reset          */
				fix_rpm_start_acq = 1U;
			}
			break;
	/* State 6: */
		case 6:
			if (fun_linear (TIMER3_10HZ, 2, 50, 50))
			{
				state = 7U;        /* Goes to the next state */
				inc = 0U;          /* Counter reset          */
				fix_rpm_start_acq = 1U;
			}
			break;
	/* State 7: */
		case 7:
			if (fun_linear (TIMER3_10HZ, 2, 60, 60))
			{
				state = 8U;        /* Goes to the next state */
				inc = 0U;          /* Counter reset          */
				fix_rpm_start_acq = 1U;
			}
			break;
	/* State 8: */
		case 8:
			if (fun_linear (TIMER3_10HZ, 2, 70, 70))
			{
				state = 9U;        /* Goes to the next state */
				inc = 0U;          /* Counter reset          */
				fix_rpm_start_acq = 1U;
			}
			break;
	/* State 9: */
		case 9:
			if (fun_linear (TIMER3_10HZ, 2, 80, 80))
			{
				state = 10U;       /* Goes to the next state */
				inc = 0U;          /* Counter reset          */
				fix_rpm_start_acq = 1U;
			}
			break;
	/* State 10: */
		case 10:
			if (fun_linear (TIMER3_10HZ, 2, 90, 90))
			{
				state = 11U;       /* Goes to the next state */
				inc = 0U;          /* Counter reset          */
				fix_rpm_start_acq = 1U;
			}
			break;
	/* State 11: */
		case 11:
			if (fun_linear (TIMER3_10HZ, 3, 100, 0))
			{
				state = 12U;       /* Goes to the next state */
				inc = 0U;          /* Counter reset          */
				fix_rpm_start_acq = 1U;
			}
			break;
	/* State 12: */
		case 12:
			state = 0U;
			returnval = 1U;
			//fix_rpm_start_acq = 1U;
			break;
		default:
			/* code */
			break;

	}

    return returnval;
}
