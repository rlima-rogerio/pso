#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"   /* Interrupt and register assignments on the Tiva C LauchPad board */
#include "hw_memmap.h"      /* Macros defining the memory map of the device. */
#include "hw_types.h"       /* Macros for hardware access, direct and via the bit-band region. */
#include "sysctl.h"         /* Prototypes for the system control driver. */
#include "gpio.h"           /* Defines and Macros for GPIO API */
#include "debug.h"          /* Macros for assisting debug of the driver library. */
#include "pwm.h"            /* API function protoypes for Pulse Width Modulation (PWM) ports */
#include "pin_map.h"        /* Mapping of peripherals to pins for all parts. */
#include "hw_gpio.h"        /* Defines and Macros for GPIO hardware. */
#include "rom.h"            /* Macros to facilitate calling functions in the ROM. */
#include "timer.h"          /* Prototypes for the timer module */
#include "uart.h"           /* Defines and Macros for the UART. */
#include "interrupt.h"      /* Prototypes for the NVIC Interrupt Controller Driver */
#include "pso_init.h"
#include "pso_uart.h"
#include "pso_led.h"
#include "pso_data.h"
#include "adc.h"
#include "pso_pwm.h"
#include "ff.h"     /* Declarations of FatFs API */
#include "fifo.h"
#include "ulink.h"
#include "ulink_types.h"

/*******************************************************************************
 * ENUMERATIONS
 ******************************************************************************/
typedef enum {
    SYS_STATE_WAIT_SW1 = 0U,           /* Waits for SW1 be pressed */
    SYS_STATE_SD_INIT = 1U,            /* Starts SD card and register FatFs work area */
    SYS_STATE_RECORDING = 2U,          /* Starts to fill fifos and write data to the memory */
    SYS_STATE_CLOSE_FILE = 3U,         /* Close file and unmount SD card */
    SYS_STATE_FINISH = 4U,             /* Finish recording indication */
    SYS_STATE_SD_ERROR = 5U            /* SD error state */
} sys_state_t;

/*******************************************************************************
 * DEFINES
 ******************************************************************************/
#define PART_TM4C123GH6PM    /* Used for Port/Pin Mapping Definitions defined in "pin_map.h" */
#define PWM_FREQUENCY  50
#define BUFFERSIZE16KB 16384
#define SD_FINISH_ITERATIONS 10U
#define SD_ERROR_ITERATIONS  5U

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

/*******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************/
/* ff11a */
#if (FAT_FS11A)
FATFS FatFs;        /* FatFs work area needed for each volume */
FIL Fil;            /* File object needed for each open file */
#endif

/* ff09b */
#if (FAT_FS09B)
FATFS fs[1];              /* Work area (file system object) for logical drives */
FIL ftxt;                 /* File objects */
BYTE buffer[BUFFERSIZE16KB];  /* File copy buffer */
FRESULT res;              /* FatFs function common result code */
WORD bw;                  /* File write count */
CHAR filename[12];        /* Filename buffer */
#endif

uint16_t scan_period_actual;
uint8_t record_sd = 0U;           /* SD record flag */
uint8_t state = 0U;               /* FIFO FSM */
uint8_t fix_rpm_start_acq = 0U;   /* Flag to fix RPM and start of acq. */
bool b_aux;

/*******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/
static void system_init(void);
static void handle_sd_recording(void);
static void handle_buffer_filling(uint16_t *k, sys_state_t sys_state);
static sys_state_t fsm_state_wait_sw1(void);
static sys_state_t fsm_state_sd_init(uint8_t *percent, uint8_t *enable_fifo_write);
static sys_state_t fsm_state_recording(uint8_t *ret_func);
static sys_state_t fsm_state_close_file(uint8_t *enable_fifo_write);
static sys_state_t fsm_state_finish(void);
static sys_state_t fsm_state_sd_error(void);

/*******************************************************************************
 * MAIN FUNCTION
 ******************************************************************************/
int main(void)
{
    static uint16_t k = 0U;
    uint8_t percent = 0U;
    uint8_t enable_fifo_write = 1U;
    uint8_t ret_func = 0U;
    sys_state_t sys_state = SYS_STATE_WAIT_SW1;

    /* System initialization */
    system_init();

    /***************************************************************************
     * MAIN LOOP
     * 1) Ajustar % do sinal PWM
     * 2) Aguardar tempo para estabilizacao da RPM
     * 3) Esvaziar FIFO
     * 4) Fazer aquisicao
     * 5) Salvar dados no SD cujo nome contenha os parametros do teste.
     **************************************************************************/
    while(1)
    {
        scan_period_actual = TIMER3_TAR_R;

        if (g_timer_a0_scan_flag)
        {
            g_timer_a0_scan_flag = 0U;

            packet_data(&dp);
            copy_data(uart_tx_buffer, &dp);

            /* Recording data to the SD card */
            handle_sd_recording();

            /* System Finite State Machine (FSM) */
            switch (sys_state)
            {
                case SYS_STATE_WAIT_SW1:
                    sys_state = fsm_state_wait_sw1();
                    break;

                case SYS_STATE_SD_INIT:
                    sys_state = fsm_state_sd_init(&percent, &enable_fifo_write);
                    break;

                case SYS_STATE_RECORDING:
                    sys_state = fsm_state_recording(&ret_func);
                    break;

                case SYS_STATE_CLOSE_FILE:
                    sys_state = fsm_state_close_file(&enable_fifo_write);
                    break;

                case SYS_STATE_FINISH:
                    sys_state = fsm_state_finish();
                    break;

                case SYS_STATE_SD_ERROR:
                    sys_state = fsm_state_sd_error();
                    break;

                default:
                    sys_state = SYS_STATE_WAIT_SW1;
                    break;
            }

            /* Fills the buffer only if RPM is fixed */
            handle_buffer_filling(&k, sys_state);

            /* Update index */
            dp.index = TIMER3_TAR_R - scan_period_actual;

        } /* END 'if' */
    } /* END 'while' */
}

/*******************************************************************************
 * FUNCTION IMPLEMENTATIONS
 ******************************************************************************/

/**
 * @brief Initialize system peripherals and configurations
 */
static void system_init(void)
{
    uint16_t i;

    /* 16MHz -> PLL -> 400MHz -> (1/2 * 1/5 = 1/10) = 40MHz  */
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    PSO_PeripheralEnable();
    PSO_GPIOConfig();
    PSO_UART0Config();
    PSO_Timers();
    PSO_ADCConfig();

    /* Small delay for stabilization */
    for (i = 0U; i < 10000U; i++);

    pso_rpm_config();
    pso_pwm_config();
    pso_spi0_config();

    IntMasterEnable();

    PSO_LEDWhiteOff();
}

/**
 * @brief Handle SD card recording process
 */
static void handle_sd_recording(void)
{
    UINT bw_local;

    if (record_sd && enable_fifo_write)
    {
        /* DEBUG */
        GPIO_PORTF_DATA_R |= GPIO_PIN_1; /* Red LED on PF1 */

        res = f_write(&ftxt, &buffer, sizeof(buffer), &bw_local);
        f_close(&ftxt);
        f_mount(0, NULL);

        /* DEBUG */
        GPIO_PORTF_DATA_R &= ~GPIO_PIN_1; /* Red LED off PF1 */

        record_sd = 0U;
    }
}

/**
 * @brief Handle buffer filling logic
 * @param k Pointer to buffer index
 * @param sys_state Current system state
 */
static void handle_buffer_filling(uint16_t *k, sys_state_t sys_state)
{
    uint16_t i;

    if ((*k <= (BUFFERSIZE16KB - PACKET_LENGTH)) && fix_rpm_start_acq)
    {
        for (i = 0U; i < PACKET_LENGTH; i++)
        {
            buffer[*k] = (uint8_t)uart_tx_buffer[i];
            (*k)++;
        }
    }
    else if (sys_state && fix_rpm_start_acq)
    {
        fix_rpm_start_acq = 0U;
        enable_fifo_write = 0U;
        sys_state = SYS_STATE_SD_INIT;
        record_sd = 1U;
        *k = 0U;
    }
}

/**
 * @brief FSM State 0: Wait for SW1 button press
 * @return Next state
 */
static sys_state_t fsm_state_wait_sw1(void)
{
    sd_stand_by();

    /* Switch #1 pressed */
    if (!(GPIO_PORTF_DATA_R & GPIO_PIN_4))
    {
        return SYS_STATE_SD_INIT;
    }

    return SYS_STATE_WAIT_SW1;
}

/**
 * @brief FSM State 1: Initialize SD card and FatFs
 * @param percent Pointer to file counter
 * @param enable_fifo_write Pointer to FIFO write enable flag
 * @return Next state
 */
static sys_state_t fsm_state_sd_init(uint8_t *percent, uint8_t *enable_fifo_write)
{
#if (FAT_FS09B)
    sprintf(filename, "daq_%d.txt", *percent);
    puts("FsFAT Testing");
    (*percent)++;

    memset(&fs, 0, sizeof(FATFS));

    /* Register work area for each volume */
    res = f_mount(0, &fs[0]);

    if (res != FR_OK)
    {
        printf("res = %d f_mount\n", res);
    }

    /* Create destination file on drive 0 */
    res = f_open(&ftxt, filename, FA_CREATE_ALWAYS | FA_WRITE);

    if (res == FR_OK)
    {
        fix_rpm_start_acq = 1U;
        *enable_fifo_write = 1U;
        sd_ok();
        return SYS_STATE_RECORDING;
    }
    else
    {
        return SYS_STATE_SD_ERROR;
    }
#endif

    return SYS_STATE_WAIT_SW1;
}

/**
 * @brief FSM State 2: Recording data
 * @param ret_func Pointer to return function flag
 * @return Next state
 */
static sys_state_t fsm_state_recording(uint8_t *ret_func)
{
    /* DEBUG */
    led_blue_toggle();

    /* Don't change PWM/RPM until have acquired data */
    if (!fix_rpm_start_acq)
    {
        *ret_func = fun_trapezoid();
    }

    /* Switch #2 - Finish the recording */
    if (!(GPIO_PORTF_DATA_R & GPIO_PIN_0) || (*ret_func))
    {
        return SYS_STATE_CLOSE_FILE; /* SW2 pressed: stops recording */
    }

    return SYS_STATE_RECORDING; /* SW2 not pressed */
}

/**
 * @brief FSM State 3: Close file and unmount SD card
 * @param enable_fifo_write Pointer to FIFO write enable flag
 * @return Next state
 */
static sys_state_t fsm_state_close_file(uint8_t *enable_fifo_write)
{
    f_close(&ftxt);
    f_mount(0, NULL);
    *enable_fifo_write = 0U; /* Disables write to the fifo */

    return SYS_STATE_FINISH;
}

/**
 * @brief FSM State 4: Finish recording indication
 * @return Next state
 */
static sys_state_t fsm_state_finish(void)
{
    uint16_t i;

    for (i = 0U; i < SD_FINISH_ITERATIONS; i++)
    {
        sd_finish_record(); /* Turn on white LED */
    }

    return SYS_STATE_WAIT_SW1;
}

/**
 * @brief FSM State 5: SD error handling
 * @return Next state
 */
static sys_state_t fsm_state_sd_error(void)
{
    uint16_t i;

    for (i = 0U; i < SD_ERROR_ITERATIONS; i++)
    {
        sd_error();
    }

    return SYS_STATE_WAIT_SW1;
}
