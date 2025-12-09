#ifndef PSO_DEBUG_H
#define PSO_DEBUG_H

/*******************************************************************************
 * DEBUG GPIO CONFIGURATION
 * 
 * Pin Mapping for timing debug:
 * - PD4 (Pin 70): DEBUG_TIMING_STREAM - Toggles when streaming packet is sent
 * - PD5 (Pin 71): DEBUG_TIMING_LOOP   - Toggles on main loop execution
 * - PD6 (Pin 72): DEBUG_TIMING_ADC    - Toggles on ADC interrupt
 * - PD7 (Pin 73): DEBUG_TIMING_STATE  - Indicates state machine state
 ******************************************************************************/

/* Debug GPIO Pins */
#define DEBUG_TIMING_STREAM    GPIO_PIN_4  /* PD4 - Streaming timing */
#define DEBUG_TIMING_LOOP      GPIO_PIN_5  /* PD5 - Loop timing */
#define DEBUG_TIMING_ADC       GPIO_PIN_6  /* PD6 - ADC ISR timing */
#define DEBUG_TIMING_STATE     GPIO_PIN_7  /* PD7 - State indicator */

/* Debug GPIO Port */
#define DEBUG_GPIO_PORT        GPIO_PORTD_BASE

/* Macros para controle rápido dos pins de debug */
#define DEBUG_STREAM_HIGH()    GPIOPinWrite(DEBUG_GPIO_PORT, DEBUG_TIMING_STREAM, DEBUG_TIMING_STREAM)
#define DEBUG_STREAM_LOW()     GPIOPinWrite(DEBUG_GPIO_PORT, DEBUG_TIMING_STREAM, 0)
#define DEBUG_STREAM_TOGGLE()  GPIOPinWrite(DEBUG_GPIO_PORT, DEBUG_TIMING_STREAM, \
                                           GPIOPinRead(DEBUG_GPIO_PORT, DEBUG_TIMING_STREAM) ^ DEBUG_TIMING_STREAM)

#define DEBUG_LOOP_HIGH()      GPIOPinWrite(DEBUG_GPIO_PORT, DEBUG_TIMING_LOOP, DEBUG_TIMING_LOOP)
#define DEBUG_LOOP_LOW()       GPIOPinWrite(DEBUG_GPIO_PORT, DEBUG_TIMING_LOOP, 0)
#define DEBUG_LOOP_TOGGLE()    GPIOPinWrite(DEBUG_GPIO_PORT, DEBUG_TIMING_LOOP, \
                                           GPIOPinRead(DEBUG_GPIO_PORT, DEBUG_TIMING_LOOP) ^ DEBUG_TIMING_LOOP)

#define DEBUG_ADC_HIGH()       GPIOPinWrite(DEBUG_GPIO_PORT, DEBUG_TIMING_ADC, DEBUG_TIMING_ADC)
#define DEBUG_ADC_LOW()        GPIOPinWrite(DEBUG_GPIO_PORT, DEBUG_TIMING_ADC, 0)
#define DEBUG_ADC_TOGGLE()     GPIOPinWrite(DEBUG_GPIO_PORT, DEBUG_TIMING_ADC, \
                                           GPIOPinRead(DEBUG_GPIO_PORT, DEBUG_TIMING_ADC) ^ DEBUG_TIMING_ADC)

#define DEBUG_STATE_HIGH()     GPIOPinWrite(DEBUG_GPIO_PORT, DEBUG_TIMING_STATE, DEBUG_TIMING_STATE)
#define DEBUG_STATE_LOW()      GPIOPinWrite(DEBUG_GPIO_PORT, DEBUG_TIMING_STATE, 0)
#define DEBUG_STATE_TOGGLE()   GPIOPinWrite(DEBUG_GPIO_PORT, DEBUG_TIMING_STATE, \
                                           GPIOPinRead(DEBUG_GPIO_PORT, DEBUG_TIMING_STATE) ^ DEBUG_TIMING_STATE)

/* State indicator patterns */
#define DEBUG_STATE_IDLE       0x00    /* Off */
#define DEBUG_STATE_INIT       0x01    /* Single pulse */
#define DEBUG_STATE_STREAMING  0x02    /* 10Hz square wave */
#define DEBUG_STATE_PWM_CTRL   0x03    /* 20Hz square wave */
#define DEBUG_STATE_STOPPING   0x04    /* Burst of pulses */
#define DEBUG_STATE_ERROR      0x05    /* Continuous high */

/*******************************************************************************
 * DEBUG TIMING FUNCTIONS
 ************x******************************************************************/
void debug_gpio_init(void);
void debug_set_state_indicator(sys_state_t state);
void debug_timing_pulse(uint8_t debug_pin);
void debug_timing_measure(uint8_t debug_pin, uint32_t *start_time);

#endif /* PSO_DEBUG_H */