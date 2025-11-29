/******************************************************************************
* FILENAME:    pso_pwm.c
*
* DESCRIPTION:
*       Functions to turn on/off the three LEDs.
*
* FUNCTIONS:
*    uint8_t set_pwm_position(uint8_t pos);

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
* 1.1     29 Nov 2025 Rogerio Lima 		   Fixing code
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

/*****************************************************************************
 * PWM TIMING MACROS (25ns clock period)
 *****************************************************************************/
#define PWM_MIN_PULSE_WIDTH_US      1000U   /* 1.0 ms minimum pulse width */
#define PWM_MID_PULSE_WIDTH_US      1500U   /* 1.5 ms mid pulse width */
#define PWM_MAX_PULSE_WIDTH_US      2000U   /* 2.0 ms maximum pulse width */
#define PWM_CLOCK_PERIOD_NS         25U     /* 25 ns clock period */

/* Convert microseconds to timer counts */
#define US_TO_COUNTS(us)            ((us) * 1000U / PWM_CLOCK_PERIOD_NS)

#define PWM_MIN_DUTY_CYCLE          US_TO_COUNTS(PWM_MIN_PULSE_WIDTH_US)  /* 40000 */
#define PWM_MID_DUTY_CYCLE          US_TO_COUNTS(PWM_MID_PULSE_WIDTH_US)  /* 60000 */
#define PWM_MAX_DUTY_CYCLE          US_TO_COUNTS(PWM_MAX_PULSE_WIDTH_US)  /* 80000 */

/* PWM position limits */
#define PWM_POSITION_MIN            0U
#define PWM_POSITION_MAX            100U

/* Calculate duty cycle from position (0-100%) */
#define PWM_DUTY_FROM_POSITION(pos) ((400U * (pos)) + PWM_MIN_DUTY_CYCLE)

/*****************************************************************************
 * TRAPEZOID FUNCTION STATE MACHINE MACROS
 *****************************************************************************/
#define TRAP_STATE_INIT             0U
#define TRAP_STATE_RAMP_START       1U
#define TRAP_STATE_STEP_10          2U
#define TRAP_STATE_STEP_20          3U
#define TRAP_STATE_STEP_30          4U
#define TRAP_STATE_STEP_40          5U
#define TRAP_STATE_STEP_50          6U
#define TRAP_STATE_STEP_60          7U
#define TRAP_STATE_STEP_70          8U
#define TRAP_STATE_STEP_80          9U
#define TRAP_STATE_STEP_90          10U
#define TRAP_STATE_RAMP_DOWN        11U
#define TRAP_STATE_COMPLETE         12U

/* Trapezoid timing parameters */
#define TRAP_RAMP_START_TIME        3U      /* Initial ramp time */
#define TRAP_STEP_HOLD_TIME         2U      /* Time to hold each step */
#define TRAP_RAMP_DOWN_TIME         3U      /* Final ramp down time */

#define TRAP_THROTTLE_STEP          10U     /* Throttle increment per step */
#define TRAP_THROTTLE_MIN           0U
#define TRAP_THROTTLE_MAX           100U

/*****************************************************************************
 * RETURN VALUE MACROS
 *****************************************************************************/
#define FUNCTION_SUCCESS            1U
#define FUNCTION_IN_PROGRESS        0U
#define FUNCTION_FAIL               0U

/*****************************************************************************
 * HELPER MACROS
 *****************************************************************************/
#define RESET_COUNTER()             (inc = 0U)
#define SET_RPM_FLAG()              (fix_rpm_start_acq = 1U)
#define TRANSITION_TO_NEXT_STATE(next_state) \
    do { \
        state = (next_state); \
        RESET_COUNTER(); \
        SET_RPM_FLAG(); \
    } while(0)


uint8_t pwm_throttle = 0U;
extern uint8_t fix_rpm_start_acq;       /* Flag to fix RPM and start of acq. */

static volatile uint16_t inc, dec;      /* 100Hz increment/decrement timer */

/* Turn off buzzer */

/*******************************************************************************
* Function Name  : set_pwm_position
* Input          : uint8_t pos
* Output         : None
* Return         : 1 if success, 0 if fail
* Description    : Defines the position (servo) or throttle (ESC) in a percen-
*                  tual scale from 0 to 100.
*******************************************************************************/
uint8_t set_pwm_position(uint8_t pos)
{
    uint32_t duty_cycle;
    uint8_t returnval;

    if (pos >= PWM_POSITION_MIN && pos <= PWM_POSITION_MAX)
    {
        duty_cycle = PWM_DUTY_FROM_POSITION(pos);
        WTIMER1_TBMATCHR_R = duty_cycle;
        
        returnval = FUNCTION_SUCCESS;
    }
    else
    {
        returnval = FUNCTION_FAIL;
    }

    return returnval;
}

/* This function must be called in period of 100ms */
void increment(void)
{
    uint16_t n;

    n = inc;                        /* 100Hz increment timer */
    inc = ++n;
}

/* This function must be called in period of 100ms */
void decrement(void)
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
uint8_t fun_linear(double delta_t, uint16_t t_f, uint8_t y_i, uint8_t y_f)
{
    double ang_coeff;
    static double lin_coeff;
    uint16_t num_points = 0U;
    uint8_t returnval = FUNCTION_IN_PROGRESS;

    num_points = (uint16_t)(t_f / delta_t);
    ang_coeff = (y_f - y_i) / num_points;
    lin_coeff = y_i;

    if (inc >= num_points)      /* Function executed */
    {
        returnval = FUNCTION_SUCCESS;
        lin_coeff = (double)((inc * ang_coeff) + lin_coeff);
    }
    else
    {
        pwm_throttle = (uint8_t)((inc * ang_coeff) + lin_coeff);
        set_pwm_position(pwm_throttle);
        returnval = FUNCTION_IN_PROGRESS;
    }

    return returnval;
}

/*******************************************************************************
* Function Name  : fun_trapezoid
* Input          : None
* Output         : None
* Return         : 1 if success, 0 if fail
* Description    : Generates a trapezoidal throttle profile through a series
*                  of stepped increases from 0% to 100% and back to 0%.
*******************************************************************************/
uint8_t fun_trapezoid(void)
{
    uint8_t returnval = FUNCTION_IN_PROGRESS;
    static uint8_t state = TRAP_STATE_INIT;

    /* Finite State Machine (FSM) */
    switch (state)
    {
        case TRAP_STATE_INIT:
            RESET_COUNTER();
            state = TRAP_STATE_RAMP_START;
            returnval = FUNCTION_IN_PROGRESS;
            break;

        case TRAP_STATE_RAMP_START:
            if (fun_linear(TIMER3_10HZ, TRAP_RAMP_START_TIME, 
                          TRAP_THROTTLE_MIN, TRAP_THROTTLE_MIN))
            {
                TRANSITION_TO_NEXT_STATE(TRAP_STATE_STEP_10);
            }
            break;

        case TRAP_STATE_STEP_10:
            if (fun_linear(TIMER3_10HZ, TRAP_STEP_HOLD_TIME, 
                          1 * TRAP_THROTTLE_STEP, 1 * TRAP_THROTTLE_STEP))
            {
                TRANSITION_TO_NEXT_STATE(TRAP_STATE_STEP_20);
            }
            break;

        case TRAP_STATE_STEP_20:
            if (fun_linear(TIMER3_10HZ, TRAP_STEP_HOLD_TIME, 
                          2 * TRAP_THROTTLE_STEP, 2 * TRAP_THROTTLE_STEP))
            {
                TRANSITION_TO_NEXT_STATE(TRAP_STATE_STEP_30);
            }
            break;

        case TRAP_STATE_STEP_30:
            if (fun_linear(TIMER3_10HZ, TRAP_STEP_HOLD_TIME, 
                          3 * TRAP_THROTTLE_STEP, 3 * TRAP_THROTTLE_STEP))
            {
                TRANSITION_TO_NEXT_STATE(TRAP_STATE_STEP_40);
            }
            break;

        case TRAP_STATE_STEP_40:
            if (fun_linear(TIMER3_10HZ, TRAP_STEP_HOLD_TIME, 
                          4 * TRAP_THROTTLE_STEP, 4 * TRAP_THROTTLE_STEP))
            {
                TRANSITION_TO_NEXT_STATE(TRAP_STATE_STEP_50);
            }
            break;

        case TRAP_STATE_STEP_50:
            if (fun_linear(TIMER3_10HZ, TRAP_STEP_HOLD_TIME, 
                          5 * TRAP_THROTTLE_STEP, 5 * TRAP_THROTTLE_STEP))
            {
                TRANSITION_TO_NEXT_STATE(TRAP_STATE_STEP_60);
            }
            break;

        case TRAP_STATE_STEP_60:
            if (fun_linear(TIMER3_10HZ, TRAP_STEP_HOLD_TIME, 
                          6 * TRAP_THROTTLE_STEP, 6 * TRAP_THROTTLE_STEP))
            {
                TRANSITION_TO_NEXT_STATE(TRAP_STATE_STEP_70);
            }
            break;

        case TRAP_STATE_STEP_70:
            if (fun_linear(TIMER3_10HZ, TRAP_STEP_HOLD_TIME, 
                          7 * TRAP_THROTTLE_STEP, 7 * TRAP_THROTTLE_STEP))
            {
                TRANSITION_TO_NEXT_STATE(TRAP_STATE_STEP_80);
            }
            break;

        case TRAP_STATE_STEP_80:
            if (fun_linear(TIMER3_10HZ, TRAP_STEP_HOLD_TIME, 
                          8 * TRAP_THROTTLE_STEP, 8 * TRAP_THROTTLE_STEP))
            {
                TRANSITION_TO_NEXT_STATE(TRAP_STATE_STEP_90);
            }
            break;

        case TRAP_STATE_STEP_90:
            if (fun_linear(TIMER3_10HZ, TRAP_STEP_HOLD_TIME, 
                          9 * TRAP_THROTTLE_STEP, 9 * TRAP_THROTTLE_STEP))
            {
                TRANSITION_TO_NEXT_STATE(TRAP_STATE_RAMP_DOWN);
            }
            break;

        case TRAP_STATE_RAMP_DOWN:
            if (fun_linear(TIMER3_10HZ, TRAP_RAMP_DOWN_TIME, 
                          TRAP_THROTTLE_MAX, TRAP_THROTTLE_MIN))
            {
                TRANSITION_TO_NEXT_STATE(TRAP_STATE_COMPLETE);
            }
            break;

        case TRAP_STATE_COMPLETE:
            state = TRAP_STATE_INIT;
            returnval = FUNCTION_SUCCESS;
            break;

        default:
            state = TRAP_STATE_INIT;
            break;
    }

    return returnval;
}
