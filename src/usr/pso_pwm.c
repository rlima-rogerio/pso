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
#include "pso_timing.h"
#include <string.h>

/* PWM Profile Configuration */
#define PROFILE_DURATION_MS     30000U  /* 30 seconds total profile duration */
#define TRAPEZOID_RAMP_MS       5000U   /* 5 seconds ramp up/down */
#define TRAPEZOID_HOLD_MS       20000U  /* 20 seconds hold at max */
#define LINEAR_DURATION_MS      30000U  /* 30 seconds linear ramp */
#define STEP_INTERVAL_MS        5000U   /* 5 seconds between steps */
#define STEP_VALUES             {0, 25, 50, 75, 100, 75, 50, 25, 0}

/* Current throttle position */
static uint8_t current_throttle = 0U;
static pwm_profile_t selected_profile = PWM_PROFILE_TRAPEZOID;

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



/**
 * @brief Execute trapezoidal speed profile
 * @param elapsed_ms Time elapsed since profile start
 * @return 1 if profile complete, 0 otherwise
 */
uint8_t execute_trapezoid_profile(uint32_t elapsed_ms)
{
    if (elapsed_ms >= PROFILE_DURATION_MS)
    {
        pwm_set_throttle(0);
        return 1U;  // Profile complete
    }
    
    if (elapsed_ms < TRAPEZOID_RAMP_MS)
    {
        /* Ramp up */
        uint8_t throttle = (uint8_t)((elapsed_ms * 100UL) / TRAPEZOID_RAMP_MS);
        pwm_set_throttle(throttle);
    }
    else if (elapsed_ms < (TRAPEZOID_RAMP_MS + TRAPEZOID_HOLD_MS))
    {
        /* Hold at max */
        pwm_set_throttle(100);
    }
    else
    {
        /* Ramp down */
        uint32_t ramp_down_start = TRAPEZOID_RAMP_MS + TRAPEZOID_HOLD_MS;
        uint32_t ramp_down_elapsed = elapsed_ms - ramp_down_start;
        uint8_t throttle = 100 - (uint8_t)((ramp_down_elapsed * 100UL) / TRAPEZOID_RAMP_MS);
        if (throttle > 100) throttle = 0;
        pwm_set_throttle(throttle);
    }
    
    return 0U;  // Profile still running
}

/**
 * @brief Execute linear ramp profile
 * @param elapsed_ms Time elapsed since profile start
 * @return 1 if profile complete, 0 otherwise
 */
uint8_t execute_linear_profile(uint32_t elapsed_ms)
{
    if (elapsed_ms >= LINEAR_DURATION_MS)
    {
        pwm_set_throttle(0);
        return 1U;
    }
    
    /* Linear ramp from 0 to 100% over profile duration */
    uint8_t throttle = (uint8_t)((elapsed_ms * 100UL) / LINEAR_DURATION_MS);
    if (throttle > 100) throttle = 100;
    
    pwm_set_throttle(throttle);
    return 0U;
}

/**
 * @brief Execute step profile
 * @param elapsed_ms Time elapsed since profile start
 * @return 1 if profile complete, 0 otherwise
 */
uint8_t execute_step_profile(uint32_t elapsed_ms)
{
    static const uint8_t step_values[] = STEP_VALUES;
    static const uint8_t num_steps = sizeof(step_values) / sizeof(step_values[0]);
    
    if (elapsed_ms >= (STEP_INTERVAL_MS * num_steps))
    {
        pwm_set_throttle(0);
        return 1U;
    }
    
    /* Determine current step */
    uint8_t current_step = elapsed_ms / STEP_INTERVAL_MS;
    if (current_step >= num_steps) current_step = num_steps - 1;
    
    pwm_set_throttle(step_values[current_step]);
    return 0U;
}

/**
 * @brief Execute custom profile (extend for custom implementations)
 * @param elapsed_ms Time elapsed since profile start
 * @return 1 if profile complete, 0 otherwise
 */
uint8_t execute_custom_profile(uint32_t elapsed_ms)
{
    /* Custom profile implementation */
    if (elapsed_ms >= PROFILE_DURATION_MS)
    {
        pwm_set_throttle(0);
        return 1U;
    }
    
    /* Example: Sine wave profile */
    // uint8_t throttle = 50 + (uint8_t)(50 * sin(2 * 3.14159 * elapsed_ms / PROFILE_DURATION_MS));
    
    /* For now, use trapezoid as default */
    return execute_trapezoid_profile(elapsed_ms);
}

/**
 * @brief Select PWM profile based on button presses or configuration
 * @return Selected profile type
 */
pwm_profile_t select_pwm_profile(void)
{
    /* Read profile selection from switches or configuration */
    /* For now, default to trapezoid */
    return PWM_PROFILE_TRAPEZOID;
    
    /* Example for switch-based selection:
    if (is_sw1_pressed() && is_sw2_pressed()) return PWM_PROFILE_CUSTOM;
    else if (is_sw1_pressed()) return PWM_PROFILE_LINEAR;
    else if (is_sw2_pressed()) return PWM_PROFILE_STEP;
    else return PWM_PROFILE_TRAPEZOID;
    */
}

/**
 * @brief Get profile name as string
 * @param profile Profile type
 * @return Profile name string
 */
const char* get_profile_name(pwm_profile_t profile)
{
    switch (profile)
    {
        case PWM_PROFILE_TRAPEZOID: return "Trapezoid";
        case PWM_PROFILE_LINEAR:    return "Linear";
        case PWM_PROFILE_STEP:      return "Step";
        case PWM_PROFILE_CUSTOM:    return "Custom";
        default:                    return "Unknown";
    }
}

/**
 * @brief Set throttle value and update PWM hardware
 * @param throttle Throttle percentage (0-100)
 */
void pwm_set_throttle(uint8_t throttle)
{
    if (throttle > 100) throttle = 100;
    current_throttle = throttle;
    
    /* Update PWM hardware */
    set_pwm_position(throttle);
}

/**
 * @brief Get current throttle value
 * @return Current throttle percentage
 */
uint8_t pwm_get_current_throttle(void)
{
    return current_throttle;
}

/**
 * @brief Initialize PWM profile system
 */
void pwm_profile_init(void)
{
    current_throttle = 0U;
    selected_profile = PWM_PROFILE_TRAPEZOID;
}