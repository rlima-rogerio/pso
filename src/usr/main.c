/*******************************************************************************
 * FILENAME:    main.c (Refactored)
 *
 * DESCRIPTION:
 *       Main application for PSO data acquisition system with UART streaming.
 *       Instead of saving to SD card, data is streamed continuously via UART.
 *
 * AUTHOR:      Rogerio Lima (Original)
 *              Refactored: 2025
 *
 * CHANGES:
 *       - Removed SD card functionality
 *       - Added UART streaming
 *       - Improved code readability
 *       - Better state machine organization
 *       - Enhanced comments and documentation
 ******************************************************************************/

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
#include "rom.h"
#include "timer.h"
#include "uart.h"
#include "interrupt.h"
#include "pso_init.h"
#include "pso_uart.h"
#include "pso_led.h"
#include "pso_data.h"
#include "adc.h"
#include "pso_pwm.h"
#include "fifo.h"
#include "ulink.h"
#include "ulink_types.h"

/*******************************************************************************
 * SYSTEM STATE MACHINE DEFINITIONS
 ******************************************************************************/
typedef enum {
    SYS_STATE_IDLE = 0U,           /* System idle, waiting for start command */
    SYS_STATE_INIT = 1U,           /* Initialize streaming parameters */
    SYS_STATE_STREAMING = 2U,      /* Active data streaming via UART */
    SYS_STATE_STOPPING = 3U,       /* Gracefully stopping streaming */
    SYS_STATE_ERROR = 4U           /* Error state */
} sys_state_t;

/*******************************************************************************
 * CONFIGURATION DEFINES
 ******************************************************************************/
#define PWM_FREQUENCY           50U
#define STREAMING_RATE_HZ       125000U     /* 125 kHz data rate */
#define FIFO_BUFFER_SIZE        256U        /* FIFO buffer size */
#define ERROR_BLINK_COUNT       5U
#define FINISH_BLINK_COUNT      10U

/*******************************************************************************
 * EXTERNAL VARIABLES
 ******************************************************************************/
extern uart_raw_data_t g_uart0_data;
extern uint8_t g_tx_buffer_uart;
extern uint16_t uart_tx_buffer[ULINK_MAX_PACKET_LEN];
extern uint8_t g_timer_a0_scan_flag;
extern ulink_pso_data_t dp;
extern fifo_t g_fifo_ping;
extern fifo_t g_fifo_pong;
extern uint8_t pwm_throttle;
extern uint8_t fix_rpm_start_acq;

/*******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************/
uint16_t scan_period_actual;
uint8_t streaming_active = 0U;          /* UART streaming active flag */
uint8_t enable_data_capture = 1U;       /* Enable data capture flag */
uint8_t fix_rpm_start_acq = 0U;         /* Flag to fix RPM and start acquisition */

/*******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/
static void system_init(void);
static void stream_data_uart(void);
static void handle_data_capture(void);

/* State machine handlers */
static sys_state_t state_idle(void);
static sys_state_t state_init(void);
static sys_state_t state_streaming(void);
static sys_state_t state_stopping(void);
static sys_state_t state_error(void);

/* LED indication functions */
static void indicate_standby(void);
static void indicate_streaming(void);
static void indicate_finish(void);
static void indicate_error(void);

/*******************************************************************************
 * MAIN FUNCTION
 ******************************************************************************/
int main(void)
{
    sys_state_t sys_state = SYS_STATE_IDLE;

    /* Initialize system peripherals and configurations */
    system_init();

    /***************************************************************************
     * MAIN LOOP
     * 
     * System operation:
     * 1. Wait for start command (SW1 press)
     * 2. Initialize streaming parameters
     * 3. Stream data continuously via UART
     * 4. Execute PWM trapezoid profile during streaming
     * 5. Stop streaming on command (SW2 press or trapezoid complete)
     **************************************************************************/
    while (1)
    {
        /* Capture current scan period for timing calculations */
        scan_period_actual = TIMER3_TAR_R;

        /* Process data when scan timer flag is set */
        if (g_timer_a0_scan_flag)
        {
            g_timer_a0_scan_flag = 0U;

            /* Prepare data packet for transmission */
            packet_data(&dp);
            copy_data(uart_tx_buffer, &dp);

            /* Stream data via UART if streaming is active */
            stream_data_uart();

            /* Handle data capture and buffering */
            handle_data_capture();

            /* Execute system state machine */
            switch (sys_state)
            {
                case SYS_STATE_IDLE:
                    sys_state = state_idle();
                    break;

                case SYS_STATE_INIT:
                    sys_state = state_init();
                    break;

                case SYS_STATE_STREAMING:
                    sys_state = state_streaming();
                    break;

                case SYS_STATE_STOPPING:
                    sys_state = state_stopping();
                    break;

                case SYS_STATE_ERROR:
                    sys_state = state_error();
                    break;

                default:
                    /* Invalid state - return to idle */
                    sys_state = SYS_STATE_IDLE;
                    break;
            }

            /* Update index with actual scan time */
            dp.index = TIMER3_TAR_R - scan_period_actual;
        }
    }
}

/*******************************************************************************
 * SYSTEM INITIALIZATION
 ******************************************************************************/
static void system_init(void)
{
    uint16_t i;

    /* Configure system clock: 16MHz -> PLL -> 400MHz -> /10 = 40MHz */
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | 
                   SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    /* Enable and configure peripherals */
    PSO_PeripheralEnable();
    PSO_GPIOConfig();
    PSO_UART0Config();
    PSO_Timers();
    PSO_ADCConfig();

    /* Stabilization delay */
    for (i = 0U; i < 10000U; i++);

    /* Configure additional peripherals */
    pso_rpm_config();
    pso_pwm_config();
    pso_spi0_config();

    /* Initialize FIFOs */
    fifo_init(&g_fifo_ping);
    fifo_init(&g_fifo_pong);

    /* Enable global interrupts */
    IntMasterEnable();

    /* Turn off all LEDs initially */
    PSO_LEDWhiteOff();
}

/*******************************************************************************
 * UART DATA STREAMING
 * 
 * Transmits data packet via UART0 when streaming is active.
 ******************************************************************************/
static void stream_data_uart(void)
{
    if (streaming_active && enable_data_capture)
    {
        /* Transmit data packet via UART */
        uartBatchWrite(UART0_BASE, uart_tx_buffer, PACKET_LENGTH);
        
        /* Toggle LED to indicate streaming activity */
        led_blue_toggle();
    }
}

/*******************************************************************************
 * DATA CAPTURE HANDLER
 * 
 * Manages data capture and FIFO operations during active streaming.
 ******************************************************************************/
static void handle_data_capture(void)
{
    uint8_t i;
    
    if (fix_rpm_start_acq && enable_data_capture)
    {
        /* Write data to FIFO for buffering */
        for (i = 0U; i < PACKET_LENGTH; i++)
        {
            if (!fifo_put(&g_fifo_ping, (uint8_t)uart_tx_buffer[i]))
            {
                /* FIFO full - data loss prevention */
                break;
            }
        }
    }
}

/*******************************************************************************
 * STATE MACHINE: IDLE STATE
 * 
 * Wait for user input to start streaming.
 * SW1 button press initiates the streaming process.
 ******************************************************************************/
static sys_state_t state_idle(void)
{
    indicate_standby();

    /* Check if SW1 is pressed */
    if (!(GPIO_PORTF_DATA_R & GPIO_PIN_4))
    {
        return SYS_STATE_INIT;
    }

    return SYS_STATE_IDLE;
}

/*******************************************************************************
 * STATE MACHINE: INIT STATE
 * 
 * Initialize streaming parameters and prepare for data transmission.
 ******************************************************************************/
static sys_state_t state_init(void)
{
    /* Reset streaming flags */
    fix_rpm_start_acq = 1U;
    enable_data_capture = 1U;
    streaming_active = 1U;

    /* Clear FIFOs */
    fifo_init(&g_fifo_ping);
    fifo_init(&g_fifo_pong);

    /* Indicate successful initialization */
    PSO_LEDGreenOn();
    
    return SYS_STATE_STREAMING;
}

/*******************************************************************************
 * STATE MACHINE: STREAMING STATE
 * 
 * Active data streaming via UART.
 * Executes PWM trapezoid profile and monitors for stop conditions.
 ******************************************************************************/
static sys_state_t state_streaming(void)
{
    uint8_t trapezoid_complete;

    indicate_streaming();

    /* Execute trapezoid profile (only when data capture is ready) */
    if (!fix_rpm_start_acq)
    {
        trapezoid_complete = fun_trapezoid();
        
        /* Check if trapezoid profile is complete */
        if (trapezoid_complete)
        {
            return SYS_STATE_STOPPING;
        }
    }

    /* Check if SW2 is pressed (manual stop) */
    if (!(GPIO_PORTF_DATA_R & GPIO_PIN_0))
    {
        return SYS_STATE_STOPPING;
    }

    return SYS_STATE_STREAMING;
}

/*******************************************************************************
 * STATE MACHINE: STOPPING STATE
 * 
 * Gracefully stop streaming and cleanup.
 ******************************************************************************/
static sys_state_t state_stopping(void)
{
    /* Disable streaming and data capture */
    streaming_active = 0U;
    enable_data_capture = 0U;
    fix_rpm_start_acq = 0U;

    /* Flush any remaining data in FIFOs */
    while (!fifo_is_empty(&g_fifo_ping))
    {
        uint8_t data = fifo_get(&g_fifo_ping);
        /* Transmit remaining data */
        UARTCharPut(UART0_BASE, data);
    }

    indicate_finish();

    return SYS_STATE_IDLE;
}

/*******************************************************************************
 * STATE MACHINE: ERROR STATE
 * 
 * Handle error conditions and provide visual feedback.
 ******************************************************************************/
static sys_state_t state_error(void)
{
    /* Disable all operations */
    streaming_active = 0U;
    enable_data_capture = 0U;

    indicate_error();

    return SYS_STATE_IDLE;
}

/*******************************************************************************
 * LED INDICATION FUNCTIONS
 ******************************************************************************/

/* Indicate system in standby mode */
static void indicate_standby(void)
{
    static uint16_t blink_counter = 0U;
    
    /* Slow white LED blink */
    if (++blink_counter > 5000U)
    {
        GPIO_PORTF_DATA_R ^= (GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
        blink_counter = 0U;
    }
}

/* Indicate active streaming */
static void indicate_streaming(void)
{
    /* Green LED solid on during streaming */
    PSO_LEDGreenOn();
    PSO_LEDRedOff();
}

/* Indicate streaming finished successfully */
static void indicate_finish(void)
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
static void indicate_error(void)
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