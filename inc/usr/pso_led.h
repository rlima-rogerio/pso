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

#ifndef PSO_LED_H_
#define PSO_LED_H_

void PSO_LEDRedOn    (void);
void PSO_LEDRedOff   (void);
void PSO_LEDGreenOn  (void);
void PSO_LEDGreenOff (void);
void PSO_LEDBlueOn   (void);
void PSO_LEDBlueOff  (void);
void led_blue_toggle (void);
void PSO_LEDWhiteOn  (void);
void PSO_LEDWhiteOff (void);
void PSO_LEDCyanOn   (void);
void PSO_LEDCyanOff  (void);
void PSO_LEDPurpleOn (void);
void PSO_LEDPurpleOff(void);
void PSO_LEDYellowOn (void);
void PSO_LEDYellowOff(void);
void PSO_LEDAllOff   (void);

#endif /* PSO_LED_H_ */
