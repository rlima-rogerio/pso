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

#ifndef PSO_SYSTEM_H_
#define PSO_SYSTEM_H_

#define ERROR_BLINK_COUNT       5U
#define FINISH_BLINK_COUNT      10U


/* LED indication functions */
void indicate_standby(void);
void indicate_streaming(void);
void indicate_pwm_active(void);
void indicate_finish(void);
void indicate_error(void);

#endif /* PSO_SYSTEM_H_ */
