/******************************************************************************
 * FILENAME:    main.c
 *
 * DESCRIPTION:
 *       Data acquisition system for PSO (Propeller Speed Optimizer).
 *       Samples sensor data continuously and:
 *       - WITH USE_SD_CARD: Records to SD card + broadcasts via UART
 *       - WITHOUT USE_SD_CARD: Only broadcasts via UART
 *
 * HARDWARE:
 *       - TM4C123GH6PM (Tiva C LaunchPad) @ 40MHz
 *       - SD Card via SPI0 (optional)
 *       - PWM ESC Control (PC7 - WT1CCP1)
 *       - RPM Sensor (PC6 - WT1CCP0)
 *       - ADC Sensors:
 *           * ADC0: Accel-X (PD1/AIN6), Thrust (PD0/AIN7), V_motor (PE1/AIN2)
 *           * ADC1: Accel-Y (PD2/AIN5), Accel-Z (PD3/AIN4), I_motor (PE2/AIN1)
 *       - UART0 @ 115200 bps for telemetry
 *       - Buttons: SW1 (PF4), SW2 (PF0)
 *       - LEDs: Red (PF1), Blue (PF2), Green (PF3)
 *
 * AUTHOR:      Rogerio Lima
 * VERSION:     4.0 - Dec 2025 - Adaptive FSM with SD/UART modes
 *
 ******************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* TivaWare Drivers */
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

/* Application Modules */
#include "pso_init.h"
#include "pso_uart.h"
#include "pso_led.h"
#include "pso_data.h"
#include "adc.h"
#include "pso_pwm.h"
#include "fifo.h"
#include "ulink.h"
#include "ulink_types.h"
#include "ulink_pso.h"

/* FatFs (SD Card) */
#if (FAT_FS09B || FAT_FS11A)
    #include "ff.h"
#endif

/*******************************************************************************
 * SYSTEM CONFIGURATION
 ******************************************************************************/
#define SYSTEM_CLOCK_HZ         40000000U
#define PWM_FREQUENCY_HZ        50U
#define ADC_SAMPLE_RATE_HZ      5000U

/* Buffer Configuration */
#define BUFFER_SIZE_BYTES       16384U
#define FILENAME_MAX_LENGTH     32U

/* Visual Feedback Configuration */
#define SD_SUCCESS_BLINKS       10U
#define SD_ERROR_BLINKS         5U
#define DEBOUNCE_DELAY_MS       10U

/* Feature Flags */
#define USE_SD_CARD                     /* Comment to disable SD recording */
#define USE_DEBUG_LEDS                  /* Comment to disable LED indicators */
#define USE_STATISTICS                  /* Comment to disable stats tracking */
#define USE_VERBOSE_LOGGING             /* Comment to reduce UART output */

/*******************************************************************************
 * DERIVED CONFIGURATION
 ******************************************************************************/
#ifdef USE_SD_CARD
    #if !(FAT_FS09B || FAT_FS11A)
        #error "USE_SD_CARD requires FAT_FS09B or FAT_FS11A to be enabled"
    #endif
    #define SD_RECORDING_ENABLED    1
#else
    #define SD_RECORDING_ENABLED    0
#endif

/*******************************************************************************
 * SYSTEM STATE MACHINE
 *
 * State flow depends on SD card configuration:
 *
 * WITH SD_CARD:
 *   INIT -> WAIT_SW1 -> SD_INIT -> RECORDING -> CLOSE_FILE -> FINISH -> WAIT_SW1
 *                                      |             |
 *                                   SD_ERROR <-------+
 *
 * WITHOUT SD_CARD:
 *   INIT -> WAIT_SW1 -> ACQUIRE_DATA (continuous) -> WAIT_SW1
 *                            |
 *                       UART_STREAM
 ******************************************************************************/
typedef enum {
    /* Common States */
    SYS_STATE_INIT = 0U,              /* System initialization */
    SYS_STATE_WAIT_SW1,               /* Wait for start trigger */
    SYS_STATE_ACQUIRE_DATA,           /* Acquire sensor data */
    SYS_STATE_UART_STREAM,            /* Stream data via UART */

#if SD_RECORDING_ENABLED
    /* SD Card Recording States */
    SYS_STATE_SD_INIT,                /* Initialize SD filesystem */
    SYS_STATE_SD_RECORDING,           /* Active recording + UART */
    SYS_STATE_SD_CLOSE_FILE,          /* Finalize SD operations */
    SYS_STATE_SD_FINISH,              /* Success indication */
    SYS_STATE_SD_ERROR,               /* Error handling */
#endif

    SYS_STATE_ERROR,                  /* General error state */
    SYS_STATE_MAX                     /* Total number of states */
} sys_state_t;

/*******************************************************************************
 * FUNCTION RETURN CODES
 ******************************************************************************/
typedef enum {
    STATUS_OK = 0U,
    STATUS_IN_PROGRESS,
    STATUS_ERROR,
    STATUS_BUFFER_FULL,
    STATUS_TIMEOUT
} status_t;

/*******************************************************************************
 * ERROR CODES
 ******************************************************************************/
typedef enum {
    ERR_NONE = 0x00,
    ERR_BUFFER_OVERFLOW = 0x01,
    ERR_ADC_FAULT = 0x10,
    ERR_SD_MOUNT = 0x20,
    ERR_SD_OPEN = 0x21,
    ERR_SD_WRITE = 0x22,
    ERR_SD_CLOSE = 0x23,
    ERR_INVALID_STATE = 0xFE,
    ERR_UNKNOWN = 0xFF
} error_code_t;

/*******************************************************************************
 * EXTERNAL VARIABLES
 ******************************************************************************/
extern uart_raw_data_t g_uart0_data;
extern uint16_t uart_tx_buffer[ULINK_MAX_PACKET_LEN];
extern uint8_t g_timer_a0_scan_flag;
extern ulink_pso_data_t dp;
extern fifo_t g_fifo_ping, g_fifo_pong;
extern uint8_t pwm_throttle;
extern volatile uint32_t adc0_buffer[3];
extern volatile uint32_t adc1_buffer[3];
extern uint32_t delta;

/*******************************************************************************
 * FATFS VARIABLES (SD Card)
 ******************************************************************************/
#if SD_RECORDING_ENABLED
    #if FAT_FS11A
        FATFS FatFs;
        FIL Fil;
    #endif

    #if FAT_FS09B
        FATFS fs[1];
        FIL ftxt;
        BYTE buffer[BUFFER_SIZE_BYTES] __attribute__((aligned(4)));
        FRESULT res;
        UINT bw;
        CHAR filename[FILENAME_MAX_LENGTH];
    #endif
#endif

/*******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************/
uint16_t scan_period_actual = 0U;
uint8_t fix_rpm_start_acq = 0U;
uint8_t enable_buffer_write = 1U;
bool system_initialized = false;

#if SD_RECORDING_ENABLED
    uint8_t record_sd_trigger = 0U;
    uint16_t buffer_index = 0U;
    uint8_t file_counter = 0U;
#endif

/*******************************************************************************
 * STATISTICS (Optional)
 ******************************************************************************/
#ifdef USE_STATISTICS
typedef struct {
    uint32_t total_samples;
    uint32_t packets_sent;
    uint32_t buffer_writes;
    uint32_t sd_errors;
    uint32_t uptime_seconds;
} system_stats_t;

static system_stats_t stats = {0};
#endif

/*******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/
/* Initialization */
static void system_init(void);
static void system_self_test(void);
static void print_banner(void);

/* State Machine */
static sys_state_t fsm_state_init(void);
static sys_state_t fsm_state_wait_sw1(void);
static sys_state_t fsm_state_acquire_data(void);
static sys_state_t fsm_state_uart_stream(void);

#if SD_RECORDING_ENABLED
static sys_state_t fsm_state_sd_init(void);
static sys_state_t fsm_state_sd_recording(void);
static sys_state_t fsm_state_sd_close_file(void);
static sys_state_t fsm_state_sd_finish(void);
static sys_state_t fsm_state_sd_error(void);
#endif

static sys_state_t fsm_state_error(error_code_t code);

/* Data Processing */
static status_t process_sensor_data(void);
static status_t transmit_uart_packet(void);

#if SD_RECORDING_ENABLED
static status_t fill_buffer_with_packet(void);
static status_t write_buffer_to_sd(void);
#endif

/* Utilities */
static void indicate_error(error_code_t code);
static bool button_pressed(uint8_t pin, uint32_t debounce_ms);
static void update_statistics(void);
static void print_statistics(void);

/*******************************************************************************
 * MAIN FUNCTION
 ******************************************************************************/
int main(void)
{
    sys_state_t current_state = SYS_STATE_INIT;

    /* System initialization - runs once */
    system_init();
    system_self_test();
    print_banner();

    /**************************************************************************
     * MAIN LOOP
     *
     * The FSM adapts based on USE_SD_CARD configuration:
     * - With SD: Full recording workflow with buffer management
     * - Without SD: Continuous UART streaming only
     **************************************************************************/
    while(1)
    {
        /* Performance monitoring */
        scan_period_actual = TIMER3_TAR_R;

        /* Process on ADC sampling complete (timer interrupt) */
        if (g_timer_a0_scan_flag)
        {
            g_timer_a0_scan_flag = 0U;

#ifdef USE_STATISTICS
            stats.total_samples++;
#endif

            /* Acquire and format sensor data */
            if (process_sensor_data() != STATUS_OK)
            {
                indicate_error(ERR_ADC_FAULT);
            }

            /* Execute Finite State Machine */
            switch (current_state)
            {
                case SYS_STATE_INIT:
                    current_state = fsm_state_init();
                    break;

                case SYS_STATE_WAIT_SW1:
                    current_state = fsm_state_wait_sw1();
                    break;

                case SYS_STATE_ACQUIRE_DATA:
                    current_state = fsm_state_acquire_data();
                    break;

                case SYS_STATE_UART_STREAM:
                    current_state = fsm_state_uart_stream();
                    break;

#if SD_RECORDING_ENABLED
                case SYS_STATE_SD_INIT:
                    current_state = fsm_state_sd_init();
                    break;

                case SYS_STATE_SD_RECORDING:
                    current_state = fsm_state_sd_recording();
                    break;

                case SYS_STATE_SD_CLOSE_FILE:
                    current_state = fsm_state_sd_close_file();
                    break;

                case SYS_STATE_SD_FINISH:
                    current_state = fsm_state_sd_finish();
                    break;

                case SYS_STATE_SD_ERROR:
                    current_state = fsm_state_sd_error();
                    break;
#endif

                case SYS_STATE_ERROR:
                    current_state = fsm_state_error(ERR_INVALID_STATE);
                    break;

                default:
                    indicate_error(ERR_INVALID_STATE);
                    current_state = SYS_STATE_WAIT_SW1;
                    break;
            }

            /* Calculate scan period for diagnostics */
            dp.index = TIMER3_TAR_R - scan_period_actual;
        }

        /* Optional: Background tasks */
        update_statistics();

    } /* End infinite loop */
}

/*******************************************************************************
 * INITIALIZATION FUNCTIONS
 ******************************************************************************/

/**
 * @brief Initialize all system peripherals
 */
static void system_init(void)
{
    uint16_t i;

    /* Configure 40MHz system clock */
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL |
                   SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    /* Initialize peripherals */
    PSO_PeripheralEnable();
    PSO_GPIOConfig();
    PSO_UART0Config();
    PSO_Timers();
    PSO_ADCConfig();

    /* Stabilization delay */
    for (i = 0U; i < 10000U; i++);

    /* Configure subsystems */
    pso_rpm_config();
    pso_pwm_config();

#if SD_RECORDING_ENABLED
    pso_spi0_config();
#endif

    /* Enable interrupts */
    IntMasterEnable();

    /* Initialize state */
    PSO_LEDAllOff();
    system_initialized = true;
}

/**
 * @brief System self-test sequence
 */
static void system_self_test(void)
{
#ifdef USE_DEBUG_LEDS
    /* LED test */
    PSO_LEDRedOn();
    SysCtlDelay(SYSTEM_CLOCK_HZ / 30);
    PSO_LEDRedOff();

    PSO_LEDGreenOn();
    SysCtlDelay(SYSTEM_CLOCK_HZ / 30);
    PSO_LEDGreenOff();

    PSO_LEDBlueOn();
    SysCtlDelay(SYSTEM_CLOCK_HZ / 30);
    PSO_LEDBlueOff();
#endif

    /* ADC baseline check */
    if (adc0_buffer[0] == 0xFFFFFFFF || adc1_buffer[0] == 0xFFFFFFFF)
    {
        indicate_error(ERR_ADC_FAULT);
    }
}

/**
 * @brief Print system banner and configuration
 */
static void print_banner(void)
{
#ifdef USE_VERBOSE_LOGGING
    printf("\n\r");
    printf("=============================================\n\r");
    printf("  PSO Data Acquisition System v4.0\n\r");
    printf("=============================================\n\r");
    printf("Clock:        %lu MHz\n\r", SYSTEM_CLOCK_HZ / 1000000);
    printf("ADC Rate:     %u Hz\n\r", ADC_SAMPLE_RATE_HZ);
    printf("Packet Size:  %u bytes\n\r", PACKET_LENGTH);

#if SD_RECORDING_ENABLED
    printf("Mode:         SD Card + UART\n\r");
    printf("Buffer:       %u bytes\n\r", BUFFER_SIZE_BYTES);
#else
    printf("Mode:         UART Streaming Only\n\r");
#endif

    printf("=============================================\n\r");
    printf("Press SW1 to start...\n\r\n\r");
#endif
}

/*******************************************************************************
 * STATE MACHINE FUNCTIONS
 ******************************************************************************/

/**
 * @brief Initial state - system ready
 */
static sys_state_t fsm_state_init(void)
{
    PSO_LEDGreenOn();
    SysCtlDelay(SYSTEM_CLOCK_HZ / 10);
    PSO_LEDGreenOff();

    return SYS_STATE_WAIT_SW1;
}

/**
 * @brief Wait for SW1 press to start operation
 */
static sys_state_t fsm_state_wait_sw1(void)
{
#ifdef USE_DEBUG_LEDS
    static uint32_t blink_counter = 0;
    if (++blink_counter > 50000)
    {
        led_green_toggle();
        blink_counter = 0;
    }
#endif

    if (button_pressed(GPIO_PIN_4, DEBOUNCE_DELAY_MS))
    {
#ifdef USE_VERBOSE_LOGGING
        printf("\n\rSW1 pressed - Starting acquisition...\n\r");
#endif

#if SD_RECORDING_ENABLED
        return SYS_STATE_SD_INIT;
#else
        return SYS_STATE_ACQUIRE_DATA;
#endif
    }

    return SYS_STATE_WAIT_SW1;
}

/**
 * @brief Acquire sensor data (continuous mode without SD)
 */
static sys_state_t fsm_state_acquire_data(void)
{
    /* Data already acquired in process_sensor_data() */

    /* Check for stop condition (SW2) */
    if (button_pressed(GPIO_PIN_0, DEBOUNCE_DELAY_MS))
    {
#ifdef USE_VERBOSE_LOGGING
        printf("SW2 pressed - Stopping...\n\r");
#endif
        print_statistics();
        return SYS_STATE_WAIT_SW1;
    }

    return SYS_STATE_UART_STREAM;
}

/**
 * @brief Stream data via UART
 */
static sys_state_t fsm_state_uart_stream(void)
{
    if (transmit_uart_packet() == STATUS_OK)
    {
#ifdef USE_STATISTICS
        stats.packets_sent++;
#endif
    }

#if SD_RECORDING_ENABLED
    /* This state not used when SD is enabled */
    return SYS_STATE_SD_RECORDING;
#else
    return SYS_STATE_ACQUIRE_DATA;
#endif
}

/*******************************************************************************
 * SD CARD STATE FUNCTIONS (Only compiled if SD enabled)
 ******************************************************************************/

#if SD_RECORDING_ENABLED

/**
 * @brief Initialize SD card and create data file
 */
static sys_state_t fsm_state_sd_init(void)
{
#if FAT_FS09B
    /* Generate filename */
    snprintf(filename, FILENAME_MAX_LENGTH, "pso_%03d.dat", file_counter);
    file_counter++;

#ifdef USE_VERBOSE_LOGGING
    printf("Creating: %s\n\r", filename);
#endif

    /* Clear filesystem structure */
    memset(&fs, 0, sizeof(FATFS));

    /* Mount filesystem - FatFs v0.09b API: f_mount(vol, fs) */
    res = f_mount(0, &fs[0]);

    if (res != FR_OK)
    {
#ifdef USE_VERBOSE_LOGGING
        printf("ERROR: Mount failed (code %d)\n\r", res);
#endif
        indicate_error(ERR_SD_MOUNT);
        return SYS_STATE_SD_ERROR;
    }

    /* Create file */
    res = f_open(&ftxt, filename, FA_CREATE_ALWAYS | FA_WRITE);

    if (res != FR_OK)
    {
#ifdef USE_VERBOSE_LOGGING
        printf("ERROR: Open failed (code %d)\n\r", res);
#endif
        indicate_error(ERR_SD_OPEN);
        return SYS_STATE_SD_ERROR;
    }

    /* Ready to record */
    fix_rpm_start_acq = 1U;
    enable_buffer_write = 1U;
    buffer_index = 0U;

    PSO_LEDGreenOn();

#ifdef USE_VERBOSE_LOGGING
    printf("Recording started\n\r");
#endif

    return SYS_STATE_SD_RECORDING;
#else
    /* FatFs not enabled */
    return SYS_STATE_SD_ERROR;
#endif
}

/**
 * @brief Active recording mode (SD + UART)
 */
static sys_state_t fsm_state_sd_recording(void)
{
#ifdef USE_DEBUG_LEDS
    static uint32_t led_toggle = 0;
    if (++led_toggle > 2500)
    {
        led_blue_toggle();
        led_toggle = 0;
    }
#endif

    /* Fill buffer with data */
    status_t buffer_status = fill_buffer_with_packet();

    if (buffer_status == STATUS_BUFFER_FULL)
    {
        /* Write full buffer to SD */
        if (write_buffer_to_sd() != STATUS_OK)
        {
            indicate_error(ERR_SD_WRITE);
#ifdef USE_STATISTICS
            stats.sd_errors++;
#endif
        }
        else
        {
#ifdef USE_STATISTICS
            stats.buffer_writes++;
#endif
        }
    }

    /* Also stream via UART */
    transmit_uart_packet();

    /* Execute PWM trapezoid (if not at fixed RPM) */
    static uint8_t trapezoid_complete = 0U;
    if (!fix_rpm_start_acq)
    {
        trapezoid_complete = fun_trapezoid();
    }

    /* Check stop conditions */
    if (button_pressed(GPIO_PIN_0, DEBOUNCE_DELAY_MS) || trapezoid_complete)
    {
#ifdef USE_VERBOSE_LOGGING
        printf("\n\rStopping recording...\n\r");
#endif
        return SYS_STATE_SD_CLOSE_FILE;
    }

    return SYS_STATE_SD_RECORDING;
}

/**
 * @brief Close SD file and unmount
 */
static sys_state_t fsm_state_sd_close_file(void)
{
#if FAT_FS09B
#ifdef USE_VERBOSE_LOGGING
    printf("Closing file...\n\r");
#endif

    PSO_LEDBlueOff();

    /* Write any remaining data */
    if (buffer_index > 0)
    {
        UINT bytes_written;
        res = f_write(&ftxt, buffer, buffer_index, &bytes_written);
    }

    /* Close and unmount - FatFs v0.09b API: f_mount(vol, NULL) */
    res = f_close(&ftxt);
    if (res != FR_OK)
    {
#ifdef USE_VERBOSE_LOGGING
        printf("WARNING: Close returned %d\n\r", res);
#endif
    }

    f_mount(0, NULL);

    enable_buffer_write = 0U;
    PSO_LEDGreenOff();

    print_statistics();

    return SYS_STATE_SD_FINISH;
#else
    /* FatFs not enabled */
    return SYS_STATE_SD_ERROR;
#endif
}

/**
 * @brief Success indication
 */
static sys_state_t fsm_state_sd_finish(void)
{
    uint16_t i;

#ifdef USE_VERBOSE_LOGGING
    printf("Recording complete!\n\r\n\r");
#endif

    for (i = 0U; i < SD_SUCCESS_BLINKS; i++)
    {
        PSO_LEDWhiteOn();
        SysCtlDelay(SYSTEM_CLOCK_HZ / 10);
        PSO_LEDWhiteOff();
        SysCtlDelay(SYSTEM_CLOCK_HZ / 10);
    }

    return SYS_STATE_WAIT_SW1;
}

/**
 * @brief SD error handling
 */
static sys_state_t fsm_state_sd_error(void)
{
    uint16_t i;

#ifdef USE_VERBOSE_LOGGING
    printf("ERROR: SD card operation failed\n\r");
#endif

    for (i = 0U; i < SD_ERROR_BLINKS; i++)
    {
        PSO_LEDRedOn();
        SysCtlDelay(SYSTEM_CLOCK_HZ / 5);
        PSO_LEDRedOff();
        SysCtlDelay(SYSTEM_CLOCK_HZ / 5);
    }

    return SYS_STATE_WAIT_SW1;
}

#endif /* SD_RECORDING_ENABLED */

/*******************************************************************************
 * DATA PROCESSING FUNCTIONS
 ******************************************************************************/

/**
 * @brief Process sensor data from ADC
 */
static status_t process_sensor_data(void)
{
    packet_data(&dp);
    copy_data(uart_tx_buffer, &dp);

    return STATUS_OK;
}

/**
 * @brief Transmit packet via UART
 */
static status_t transmit_uart_packet(void)
{
    uartBatchWrite(UART0_BASE, uart_tx_buffer, PACKET_LENGTH);
    return STATUS_OK;
}

#if SD_RECORDING_ENABLED

/**
 * @brief Fill buffer with current data packet
 */
static status_t fill_buffer_with_packet(void)
{
    uint16_t i;

    if (!fix_rpm_start_acq)
        return STATUS_IN_PROGRESS;

    /* Check if space available */
    if (buffer_index <= (BUFFER_SIZE_BYTES - PACKET_LENGTH))
    {
        for (i = 0U; i < PACKET_LENGTH; i++)
        {
            buffer[buffer_index++] = (uint8_t)uart_tx_buffer[i];
        }
        return STATUS_OK;
    }

    /* Buffer full */
    return STATUS_BUFFER_FULL;
}

/**
 * @brief Write buffer to SD card
 */
static status_t write_buffer_to_sd(void)
{
#if FAT_FS09B
    UINT bytes_written;

#ifdef USE_DEBUG_LEDS
    PSO_LEDRedOn();
#endif

    res = f_write(&ftxt, buffer, buffer_index, &bytes_written);

#ifdef USE_DEBUG_LEDS
    PSO_LEDRedOff();
#endif

    if (res != FR_OK || bytes_written != buffer_index)
    {
        return STATUS_ERROR;
    }

    /* Sync to ensure write */
    f_sync(&ftxt);

    /* Reset buffer */
    buffer_index = 0U;
    fix_rpm_start_acq = 1U;

    return STATUS_OK;
#endif

    return STATUS_ERROR;
}

#endif /* SD_RECORDING_ENABLED */

/*******************************************************************************
 * UTILITY FUNCTIONS
 ******************************************************************************/

/**
 * @brief General error state handler
 */
static sys_state_t fsm_state_error(error_code_t code)
{
    indicate_error(code);
    return SYS_STATE_WAIT_SW1;
}

/**
 * @brief Visual error indication
 */
static void indicate_error(error_code_t code)
{
#ifdef USE_DEBUG_LEDS
    uint8_t i;
    for (i = 0; i < 3; i++)
    {
        PSO_LEDRedOn();
        SysCtlDelay(SYSTEM_CLOCK_HZ / 20);
        PSO_LEDRedOff();
        SysCtlDelay(SYSTEM_CLOCK_HZ / 20);
    }
#endif

#ifdef USE_VERBOSE_LOGGING
    printf("ERROR: 0x%02X\n\r", code);
#endif
}

/**
 * @brief Check if button pressed with debouncing
 */
static bool button_pressed(uint8_t pin, uint32_t debounce_ms)
{
    if (!(GPIO_PORTF_DATA_R & pin))
    {
        SysCtlDelay((SYSTEM_CLOCK_HZ / 1000) * debounce_ms / 3);
        if (!(GPIO_PORTF_DATA_R & pin))
        {
            return true;
        }
    }
    return false;
}

/**
 * @brief Update system statistics
 */
static void update_statistics(void)
{
#ifdef USE_STATISTICS
    static uint32_t counter = 0;
    if (++counter >= ADC_SAMPLE_RATE_HZ)
    {
        stats.uptime_seconds++;
        counter = 0;
    }
#endif
}

/**
 * @brief Print session statistics
 */
static void print_statistics(void)
{
#if defined(USE_STATISTICS) && defined(USE_VERBOSE_LOGGING)
    printf("\n\r========== SESSION STATS ==========\n\r");
    printf("Samples:      %lu\n\r", stats.total_samples);
    printf("UART packets: %lu\n\r", stats.packets_sent);

#if SD_RECORDING_ENABLED
    printf("Buffer writes:%lu\n\r", stats.buffer_writes);
    printf("SD errors:    %lu\n\r", stats.sd_errors);
#endif

    printf("Uptime:       %lu s\n\r", stats.uptime_seconds);
    printf("===================================\n\r\n\r");
#endif
}
