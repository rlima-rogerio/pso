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
#include "pso_timing.h"
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
//uint8_t streaming_active = 0U;          /* UART streaming active flag */
//uint8_t enable_data_capture = 1U;       /* Enable data capture flag */
//uint8_t fix_rpm_start_acq = 0U;         /* Flag to fix RPM and start acquisition */
//uint32_t sample_counter = 0U;  // ADICIONE ESTA LINHA

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

// int main(void)
// {
//     sys_state_t sys_state = SYS_STATE_IDLE;

//     /* Initialize system peripherals and configurations */
//     system_init();

//     /***************************************************************************
//      * MAIN LOOP
//      * 
//      * System operation:
//      * 1. Wait for start command (SW1 press)
//      * 2. Initialize streaming parameters
//      * 3. Stream data continuously via UART
//      * 4. Execute PWM trapezoid profile during streaming
//      * 5. Stop streaming on command (SW2 press or trapezoid complete)
//      **************************************************************************/
//     while (1)
//     {
//         /* Capture current scan period for timing calculations */
//         scan_period_actual = TIMER3_TAR_R;

//         /* Process data when scan timer flag is set */
//         if (g_timer_a0_scan_flag)
//         {
//             g_timer_a0_scan_flag = 0U;

//             /* Prepare data packet for transmission */
//             packet_data(&dp);
//             copy_data(uart_tx_buffer, &dp);

//             /* Stream data via UART if streaming is active */
//             stream_data_uart();

//             /* Handle data capture and buffering */
//             handle_data_capture();

//             /* Execute system state machine */
//             switch (sys_state)
//             {
//                 case SYS_STATE_IDLE:
//                     sys_state = state_idle();
//                     break;

//                 case SYS_STATE_INIT:
//                     sys_state = state_init();
//                     break;

//                 case SYS_STATE_STREAMING:
//                     sys_state = state_streaming();
//                     break;

//                 case SYS_STATE_STOPPING:
//                     sys_state = state_stopping();
//                     break;

//                 case SYS_STATE_ERROR:
//                     sys_state = state_error();
//                     break;

//                 default:
//                     /* Invalid state - return to idle */
//                     sys_state = SYS_STATE_IDLE;
//                     break;
//             }

//             /* Update index with actual scan time */
//             dp.index = TIMER3_TAR_R - scan_period_actual;
//         }
//     }
// }

/*******************************************************************************
 * MAIN FUNCTION (MODIFICADO)
 ******************************************************************************/
int main(void)
{
    sys_state_t sys_state = SYS_STATE_IDLE;
    static uint32_t last_stream_time = 0;
    static uint32_t last_sample_time = 0;
    static uint32_t sample_counter = 0;
    
    /* Initialize system peripherals and configurations */
    system_init();
    
    /* Inicializa temporização */
    //timing_init(SysCtlClockGet());
    //timing_configure(RATE_1000_HZ, 0);  // Loop principal a 1 kHz
    
    /***************************************************************************
     * MAIN LOOP OTIMIZADO
     * 
     * Estrutura:
     * 1. Processamento em alta prioridade (ADC samples @ 125 kHz)
     * 2. Loop de controle principal @ 1 kHz
     * 3. Streaming de dados @ taxa configurável (ex: 100 Hz)
     * 4. Tarefas de baixa prioridade
     **************************************************************************/
    while (1)
    {
        /* 1. PROCESSAMENTO DE AMOSTRAS ADC (alta prioridade) */
        if (g_timer_a0_scan_flag)
        {
            g_timer_a0_scan_flag = 0U;
            
            /* Coleta dados do ADC (já feito na ISR) */
            /* Apenas atualiza buffer local se necessário */
            sample_counter++;
            
            /* Buffer simples para média (opcional) */
            static uint16_t sample_buffer[10];
            static uint8_t buffer_index = 0;
            
            packet_data(&dp);
            copy_data(uart_tx_buffer, &dp);
            
            /* Armazena para média/processamento posterior */
            // sample_buffer[buffer_index] = adc_value;
            buffer_index = (buffer_index + 1) % 10;
        }
        
        /* 2. LOOP DE CONTROLE PRINCIPAL (1 kHz) */
        if (check_interval_ms(1))
        {
            uint32_t current_time = get_system_ticks();
            
            /* Máquina de estados principal */
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
                    sys_state = SYS_STATE_IDLE;
                    break;
            }
            
            /* 3. STREAMING DE DADOS (executa a cada 10ms = 100 Hz) */
            //if ((current_time - last_stream_time) >= 10)  // 100 Hz
            if (check_interval_ms(10))  // 100 Hz
            {
                last_stream_time = current_time;
                
                if (streaming_active && enable_data_capture)
                {
                    /* Envia pacote via UART */
                    uartBatchWrite(UART0_BASE, uart_tx_buffer, PACKET_LENGTH);
                    
                    /* Toggle LED para indicar atividade */
                    led_green_toggle();
                }
            }
            
            /* 4. ATUALIZAÇÃO DE DADOS (executa a cada 2ms = 500 Hz) */
            if ((current_time - last_sample_time) >= 2)  // 500 Hz
            {
                last_sample_time = current_time;
                
                /* Processamento de dados filtrados/médias */
                if (fix_rpm_start_acq && enable_data_capture)
                {
                    /* Buffer FIFO para não perder dados */
                    handle_data_capture();
                }
            }
            
            /* Atualiza índice com tempo real (para debugging) */
            dp.index = timing_get_execution_count();
        }
        
        /* 5. TAREFAS DE BAIXA PRIORIDADE (executa quando possível) */
        /* Ex: Verificação de botões, monitoramento, etc. */
        static uint32_t last_low_priority = 0;
        if ((get_system_ticks() - last_low_priority) > 50)  // 20 Hz
        {
            last_low_priority = get_system_ticks();
            
            /* Verificações não críticas aqui */
            // check_system_health();
            // update_status_leds();
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
        //led_green_toggle();
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
        PSO_LEDWhiteOff();
        return SYS_STATE_INIT;
    }

    return SYS_STATE_IDLE;
}

/*******************************************************************************
 * STATE MACHINE: INIT STATE
 * 
 * Initialize streaming parameters and prepare for data transmission.
 ******************************************************************************/
//static sys_state_t state_init(void)
//{
//    /* Reset streaming flags */
//    fix_rpm_start_acq = 1U;
//    enable_data_capture = 1U;
//    streaming_active = 1U;
//
//    /* Clear FIFOs */
//    fifo_init(&g_fifo_ping);
//    fifo_init(&g_fifo_pong);
//
//    /* Indicate successful initialization */
//    //PSO_LEDGreenOn();
//
//    return SYS_STATE_STREAMING;
//}

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
    
    led_blue_toggle ();
    /* Slow blue LED blink */
    if (++blink_counter > 5000U)
    {
        //GPIO_PORTF_DATA_R ^= (GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
        led_blue_toggle ();
        blink_counter = 0U;
    }
}

/* Indicate active streaming */
static void indicate_streaming(void)
{
    /* Green LED solid on during streaming */
    PSO_LEDGreenOn();
    // PSO_LEDRedOff();
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

/*******************************************************************************
 * NOVAS FUNÇÕES AUXILIARES
 ******************************************************************************/

/* Estado INIT modificado */
static sys_state_t state_init(void)
{
    /* Reset streaming flags */
    fix_rpm_start_acq = 1U;
    enable_data_capture = 1U;
    streaming_active = 1U;
    
    /* Configura taxas específicas */
    timing_configure(RATE_1000_HZ, 0);  // Loop principal a 1 kHz
    
    /* Reseta contadores */
    timing_reset();
    sample_counter = 0;
    
    /* Clear FIFOs */
    fifo_init(&g_fifo_ping);
    fifo_init(&g_fifo_pong);
    
    /* Indicate successful initialization */
    PSO_LEDGreenOn();
    
    return SYS_STATE_STREAMING;
}

/* Função para monitoramento do sistema */
static void monitor_system_performance(void)
{
    static uint32_t last_monitor_time = 0;
    uint32_t current_time = get_system_ticks();
    
    if ((current_time - last_monitor_time) >= 1000)  // A cada 1 segundo
    {
        last_monitor_time = current_time;
        
        /* Log de performance */
        float actual_rate = timing_get_actual_rate_hz();
        uint32_t missed = timing_get_missed_deadlines();
        uint32_t total = timing_get_execution_count();
        
        /* Pode enviar via UART para debugging */
        // printf("Rate: %.1f Hz, Missed: %lu, Total: %lu\n", 
        //        actual_rate, missed, total);
        
        /* Reset se muitos deadlines perdidos */
        if (missed > 10)
        {
            // System might be overloaded
            PSO_LEDRedOn();
        }
    }
}

