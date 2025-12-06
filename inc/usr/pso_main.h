/*******************************************************************************
 * FILENAME:    pso_main.h
 *
 * DESCRIPTION:
 *       Main header file for PSO data acquisition system.
 *       Contains all system-level definitions, structures, and function
 *       prototypes for UART streaming configuration.
 *
 * AUTHOR:      Rogerio Lima
 *              Refactored: 2025
 ******************************************************************************/

#ifndef PSO_MAIN_H_
#define PSO_MAIN_H_

#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * SYSTEM CONFIGURATION
 ******************************************************************************/
#define SYSTEM_CLOCK_HZ         40000000U   /* 40 MHz system clock */
#define UART_BAUD_RATE          115200U     /* UART communication speed */
#define ADC_SAMPLE_RATE_HZ      125000U     /* 125 kHz ADC sampling */

/*******************************************************************************
 * DATA PACKET CONFIGURATION
 ******************************************************************************/
#define PACKET_LENGTH           21U         /* Total packet size in bytes */
#define PACKET_HEADER_SIZE      5U          /* Header size */
#define PACKET_PAYLOAD_SIZE     14U         /* Payload size */
#define PACKET_CHECKSUM_SIZE    2U          /* Checksum size */

/*******************************************************************************
 * FIFO CONFIGURATION
 ******************************************************************************/
#define MAX_FIFO_SIZE           256U        /* Maximum FIFO buffer size */

/*******************************************************************************
 * PWM/MOTOR CONFIGURATION
 ******************************************************************************/
#define PWM_FREQUENCY_HZ        50U         /* PWM frequency (50 Hz for ESC) */
#define PWM_MIN_DUTY_CYCLE      40000U      /* 1.0 ms pulse width */
#define PWM_MID_DUTY_CYCLE      60000U      /* 1.5 ms pulse width */
#define PWM_MAX_DUTY_CYCLE      80000U      /* 2.0 ms pulse width */

/*******************************************************************************
 * TIMER DEFINITIONS
 ******************************************************************************/
#define TIMER3_10HZ             0.1         /* 100ms period for 10Hz timing */
#define TIMER_PERIOD_100MS      100U        /* 100ms in milliseconds */

/*******************************************************************************
 * SYSTEM STATE ENUMERATION
 ******************************************************************************/
typedef enum {
    SYS_STATE_IDLE = 0U,        /* System idle, waiting for command */
    SYS_STATE_INIT = 1U,        /* Initializing streaming */
    SYS_STATE_STREAMING = 2U,   /* Active data streaming */
    SYS_STATE_STOPPING = 3U,    /* Stopping streaming */
    SYS_STATE_ERROR = 4U        /* Error state */
} sys_state_t;

/*******************************************************************************
 * LED COLORS (Port F GPIO Pins)
 ******************************************************************************/
#define LED_RED                 GPIO_PIN_1  /* PF1 - Red LED */
#define LED_BLUE                GPIO_PIN_2  /* PF2 - Blue LED */
#define LED_GREEN               GPIO_PIN_3  /* PF3 - Green LED */
#define LED_WHITE               (LED_RED | LED_BLUE | LED_GREEN)

/*******************************************************************************
 * SWITCH DEFINITIONS (Port F GPIO Pins)
 ******************************************************************************/
#define SW1_PIN                 GPIO_PIN_4  /* PF4 - Switch 1 (Start) */
#define SW2_PIN                 GPIO_PIN_0  /* PF0 - Switch 2 (Stop) */

/*******************************************************************************
 * FUNCTION RETURN VALUES
 ******************************************************************************/
#define FUNC_SUCCESS            1U
#define FUNC_FAILURE            0U
#define FUNC_IN_PROGRESS        0U

/*******************************************************************************
 * DATA STRUCTURES
 ******************************************************************************/

/**
 * @brief UART raw data structure
 */
typedef struct {
    uint8_t rx_buffer[256];     /* Receive buffer */
    uint16_t rx_index;          /* Current receive index */
    uint8_t new_data;           /* New data flag */
} uart_raw_data_t;

/**
 * @brief Circular FIFO structure
 */
typedef struct {
    uint8_t data[MAX_FIFO_SIZE];  /* Data buffer */
    int16_t front;                /* Front index */
    int16_t rear;                 /* Rear index */
} fifo_t;

/**
 * @brief PSO sensor data packet structure
 * 
 * Contains all sensor readings for one sample period
 */
typedef struct {
    uint16_t index;             /* Sample index/timestamp */
    int16_t accel[3];           /* Accelerometer data [x, y, z] */
    uint16_t rpm;               /* Motor RPM */
    int16_t i_motor;            /* Motor current [mA] */
    int16_t v_motor;            /* Motor voltage [mV] */
    int16_t thrust;             /* Thrust force [cN] */
    uint8_t throttle;           /* Throttle position [0-100%] */
} ulink_pso_data_t;

/*******************************************************************************
 * GLOBAL FUNCTION PROTOTYPES
 ******************************************************************************/

/* External global variables (defined in main.c) */
extern uint8_t fix_rpm_start_acq;       /* Flag to fix RPM and start acquisition */
extern uint8_t streaming_active;        /* UART streaming active flag */
extern uint8_t enable_data_capture;     /* Enable data capture flag */

/* Initialization functions */
void PSO_PeripheralEnable(void);
void PSO_GPIOConfig(void);
void PSO_UART0Config(void);
void PSO_Timers(void);
void PSO_ADCConfig(void);
void pso_rpm_config(void);
void pso_pwm_config(void);
void pso_spi0_config(void);

/* LED control functions */
void PSO_LEDRedOn(void);
void PSO_LEDRedOff(void);
void PSO_LEDGreenOn(void);
void PSO_LEDGreenOff(void);
void PSO_LEDBlueOn(void);
void PSO_LEDBlueOff(void);
void PSO_LEDWhiteOn(void);
void PSO_LEDWhiteOff(void);
void led_red_toggle(void);
void led_blue_toggle(void);
void led_green_toggle(void);

/* UART functions */
uint8_t uartBatchWrite(uint32_t ui32Base, uint16_t* txBuffer, 
                       const uint8_t bytesToWrite);

/* FIFO functions */
void fifo_init(fifo_t* fifo);
bool fifo_put(fifo_t* fifo, uint8_t data);
bool fifo_del(fifo_t* fifo);
uint8_t fifo_get(fifo_t* fifo);
bool fifo_is_empty(fifo_t* fifo);
bool fifo_is_full(fifo_t* fifo);
uint16_t fifo_count_elements(fifo_t* fifo);
bool fifo_transfer(fifo_t* src, fifo_t* dst);

/* Data handling functions */
uint8_t packet_data(ulink_pso_data_t* dp);
void copy_data(uint16_t* uart_tx_buf, ulink_pso_data_t* dp);

/* PWM control functions */
uint8_t set_pwm_position(uint8_t pos);
uint8_t fun_linear(double delta_t, uint16_t t_f, uint8_t y_i, uint8_t y_f);
uint8_t fun_trapezoid(void);
void increment(void);
void decrement(void);

/*******************************************************************************
 * INLINE UTILITY FUNCTIONS
 ******************************************************************************/

/**
 * @brief Check if SW1 is pressed
 * @return true if pressed, false otherwise
 */
static inline bool is_sw1_pressed(void)
{
    return !(GPIO_PORTF_DATA_R & SW1_PIN);
}

/**
 * @brief Check if SW2 is pressed
 * @return true if pressed, false otherwise
 */
static inline bool is_sw2_pressed(void)
{
    return !(GPIO_PORTF_DATA_R & SW2_PIN);
}

/**
 * @brief Simple delay function
 * @param count Number of loop iterations
 */
static inline void simple_delay(uint32_t count)
{
    volatile uint32_t i;
    for (i = 0; i < count; i++);
}

#endif /* PSO_MAIN_H_ */