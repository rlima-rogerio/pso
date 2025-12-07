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
#include "systick.h"      // Para funções SysTick
#include "hw_nvic.h"            // Para registros NVIC

/*******************************************************************************
 * SYSTEM STATE MACHINE DEFINITIONS
 ******************************************************************************/
typedef enum {
    SYS_STATE_IDLE = 0U,           /* System idle, waiting for start command */
    SYS_STATE_INIT = 1U,           /* Initialize streaming parameters */
    SYS_STATE_STREAMING = 2U,      /* Active data streaming via UART */
    SYS_STATE_PWM_CONTROL = 3U,    /* PWM control state (trapezoid/linear/step) */
    SYS_STATE_STOPPING = 4U,       /* Gracefully stopping streaming */
    SYS_STATE_ERROR = 5U           /* Error state */
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
pwm_profile_t current_profile = PWM_PROFILE_TRAPEZOID;
uint8_t profile_complete = 0U;     /* Flag indicating profile completion */
uint32_t profile_start_time = 0U;  /* Start time of current profile */

/*******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/
static void system_init(void);
static void stream_data_uart(void);
static void handle_data_capture(void);
static void test_uart_output(void);

/*******************************************************************************
 * SYSYICK FUNCTION PROTOTYPES
 ******************************************************************************/
void SysTick_Handler(void);                    // Handler do SysTick
uint32_t get_systick_ms(void);                 // Obtém ms desde início
void test_systick(void);                       // Função de teste
static void configure_systick(void);           // Configura SysTick

/* State machine handlers */
static sys_state_t state_idle(void);
static sys_state_t state_init(void);
static sys_state_t state_streaming(void);
static sys_state_t state_pwm_control(void);
static sys_state_t state_stopping(void);
static sys_state_t state_error(void);

/* LED indication functions */
static void indicate_standby(void);
static void indicate_streaming(void);
static void indicate_pwm_active(void);
static void indicate_finish(void);
static void indicate_error(void);

int main(void)
{
    sys_state_t sys_state = SYS_STATE_IDLE;
    
    /* Initialize system peripherals and configurations */
    system_init();
    
    /* Initialize PWM profile system */
    pwm_profile_init();
    
    /* TESTE UART */
    test_uart_output();

    /* Test systick */
    test_systick();
    
    /***************************************************************************
     * MAIN LOOP OTIMIZADO
     **************************************************************************/
    while (1)
    {
        /* 1. PROCESSAMENTO DE AMOSTRAS ADC (alta prioridade) */
        if (g_timer_a0_scan_flag)
        {
            g_timer_a0_scan_flag = 0U;
            
            /* Coleta e processa dados */
            packet_data(&dp);
            copy_data(uart_tx_buffer, &dp);
        }
        
        /* 2. LOOP DE CONTROLE PRINCIPAL (1 kHz) */
        if (check_interval_ms(1))
        {
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
                    sys_state = state_streaming();  // Streaming contínuo
                    break;
                    
                case SYS_STATE_PWM_CONTROL:         // NOVO ESTADO
                    sys_state = state_pwm_control(); // Controle PWM + streaming
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

    /* INICIALIZA TEMPORIZAÇÃO - IMPORTANTE! */
    // timing_init(SysCtlClockGet());  // Passa o clock do sistema
    
    /* CONFIGURA SysTick para 1ms */
    SysTickPeriodSet(SysCtlClockGet() / 1000);  // 1ms
    SysTickIntRegister(&SysTick_Handler);       // Registra handler
    SysTickEnable();
    SysTickIntEnable();  

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
static sys_state_t state_init(void)
{
    /* Reset streaming flags */
    fix_rpm_start_acq = 1U;
    enable_data_capture = 1U;
    streaming_active = 1U;
    
    /* Configura perfil PWM (pode ser selecionado por SW1/SW2) */
    current_profile = select_pwm_profile();
    
    /* Configura taxas específicas */
    timing_configure(RATE_1000_HZ, 0);  // Loop principal a 1 kHz
    
    /* Reseta contadores */
    timing_reset();
    sample_counter = 0;
    
    /* Clear FIFOs */
    fifo_init(&g_fifo_ping);
    fifo_init(&g_fifo_pong);
    
    /* Envia mensagem de inicialização */
    const char *init_msg = "System INIT - Starting streaming\r\n";
    while (*init_msg) UARTCharPut(UART0_BASE, *init_msg++);
    
    /* Envia configuração do perfil */
    char profile_msg[50];
    sprintf(profile_msg, "PWM Profile: %s\r\n", get_profile_name(current_profile));
    int i;
    for (i = 0; profile_msg[i] != '\0'; i++)
    {
        UARTCharPut(UART0_BASE, profile_msg[i]);
    }
    
    /* Indicate successful initialization */
    PSO_LEDGreenOn();
    
    return SYS_STATE_STREAMING;
}

/*******************************************************************************
 * STATE MACHINE: STREAMING STATE
 * 
 * Active data streaming via UART.
 * Manages UART data transmission and transitions to PWM control.
 ******************************************************************************/
static sys_state_t state_streaming(void)
{
    static uint32_t last_stream_time = 0;
    static uint32_t stream_packet_count = 0;
    uint32_t current_time = get_systick_ms(); //g_system_ms_counter;//

    indicate_streaming();

    /* STREAMING DE DADOS (executa a cada 10ms = 100 Hz) */
    if ((current_time - last_stream_time) >= 10)  // 100 Hz
    {
        last_stream_time = current_time;
        
        if (enable_data_capture)
        {
            /* EXPLICITAMENTE chama uartBatchWrite para envio de dados */
            uartBatchWrite(UART0_BASE, uart_tx_buffer, PACKET_LENGTH);
            
            stream_packet_count++;
            
            // /* DEBUG: Log a cada 100 pacotes */
            // if (stream_packet_count % 100 == 0)
            // {
            //     // Envia status via UART para debug
            //     char debug_msg[50];
            //     sprintf(debug_msg, "Packets sent: %lu, RPM: %d\r\n", 
            //             stream_packet_count, dp.rpm);
            //     int i;
            //     for (i = 0; debug_msg[i] != '\0'; i++)
            //     {
            //         UARTCharPut(UART0_BASE, debug_msg[i]);
            //     }
            // }
            
            /* Toggle LED para indicar atividade de streaming */
            led_green_toggle();
        }
    }

    /* Transição para controle PWM após streaming estabilizado */
    if (stream_packet_count >= 50)  // Após 50 pacotes (0.5s)
    {
        profile_start_time = current_time;
        profile_complete = 0U;
        return SYS_STATE_PWM_CONTROL;
    }

    /* Check if SW2 is pressed (manual stop) */
    if (!(GPIO_PORTF_DATA_R & GPIO_PIN_0))
    {
        return SYS_STATE_STOPPING;
    }

    return SYS_STATE_STREAMING;
}


/*******************************************************************************
 * STATE MACHINE: PWM CONTROL STATE
 * 
 * Executes PWM control profiles (trapezoid, linear, step).
 * Calculates throttle command based on selected profile.
 ******************************************************************************/
static sys_state_t state_pwm_control(void)
{
    static uint32_t last_pwm_update = 0;
    static uint32_t last_stream_time = 0;
    uint32_t current_time = get_systick_ms();
    uint8_t profile_done = 0U;

    indicate_pwm_active();

    /* ATUALIZAÇÃO PWM (executa a cada 20ms = 50 Hz) */
    if ((current_time - last_pwm_update) >= 20)
    {
        last_pwm_update = current_time;
        
        /* Executa perfil selecionado */
        switch (current_profile)
        {
            case PWM_PROFILE_TRAPEZOID:
                profile_done = execute_trapezoid_profile(current_time - profile_start_time);
                break;
                
            case PWM_PROFILE_LINEAR:
                profile_done = execute_linear_profile(current_time - profile_start_time);
                break;
                
            case PWM_PROFILE_STEP:
                profile_done = execute_step_profile(current_time - profile_start_time);
                break;
                
            case PWM_PROFILE_CUSTOM:
                profile_done = execute_custom_profile(current_time - profile_start_time);
                break;
                
            default:
                /* Fallback para trapezoidal */
                profile_done = execute_trapezoid_profile(current_time - profile_start_time);
                break;
        }
        
        /* Atualiza throttle no pacote de dados */
        dp.throttle = pwm_throttle;
        
        /* DEBUG: Log do throttle */
        if ((current_time - profile_start_time) % 1000 == 0)  // A cada 1s
        {
            char throttle_msg[30];
            sprintf(throttle_msg, "Throttle: %d%%\r\n", pwm_throttle);
            int i;
            for (i = 0; throttle_msg[i] != '\0'; i++)
            {
                UARTCharPut(UART0_BASE, throttle_msg[i]);
            }
        }
    }

    /* STREAMING CONTÍNUO (executa a cada 10ms = 100 Hz) */
    if ((current_time - last_stream_time) >= 10)
    {
        last_stream_time = current_time;
        
        /* Continua streaming dados durante controle PWM */
        if (enable_data_capture)
        {
            uartBatchWrite(UART0_BASE, uart_tx_buffer, PACKET_LENGTH);
            
            /* LED indicador alternado mais rápido durante PWM control */
            GPIO_PORTF_DATA_R ^= GPIO_PIN_2;  // LED azul
        }
    }

    /* Verifica conclusão do perfil */
    if (profile_done)
    {
        profile_complete = 1U;
        
        /* Envia mensagem de conclusão */
        const char *complete_msg = "PWM profile complete!\r\n";
        while (*complete_msg) UARTCharPut(UART0_BASE, *complete_msg++);
        
        return SYS_STATE_STOPPING;
    }

    /* Check if SW2 is pressed (manual stop) */
    if (!(GPIO_PORTF_DATA_R & GPIO_PIN_0))
    {
        return SYS_STATE_STOPPING;
    }

    return SYS_STATE_PWM_CONTROL;
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

/* Indicate PWM control active */
static void indicate_pwm_active(void)
{
    static uint32_t last_blink = 0;
    uint32_t current_time = get_system_ticks();
    
    /* Fast alternating green/blue LEDs during PWM control */
    if ((current_time - last_blink) >= 200)  // 5 Hz blink
    {
        last_blink = current_time;
        static uint8_t blink_state = 0;
        
        if (blink_state == 0)
        {
            PSO_LEDGreenOn();
            PSO_LEDBlueOff();
        }
        else
        {
            PSO_LEDGreenOff();
            PSO_LEDBlueOn();
        }
        blink_state ^= 1U;
    }
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

static void test_uart_output(void)
{
    /* Envia uma string de teste via UART */
    const char *test_msg = "PSO UART TEST - Starting...\r\n";
    
    while (*test_msg)
    {
        UARTCharPut(UART0_BASE, *test_msg++);
    }
    
    /* Envia um pacote de teste */
    uint8_t test_packet[21] = {
        0xFE, 0x0E,                 // STX + Length
        0x00, 0x01,                 // Index
        0x00, 0x64, 0x00, 0x00, 0x00, 0x00, // Accel XYZ
        0x13, 0x88,                 // RPM = 5000
        0x07, 0xD0,                 // Current = 2000 mA
        0x0B, 0xB8,                 // Voltage = 3000 mV
        0x00, 0x64,                 // Thrust = 100 cN
        0x32,                       // Throttle = 50%
        0x00, 0x00                  // Checksum (placeholder)
    };
    
    int i;
    for (i = 0; i < 21; i++)
    {
        UARTCharPut(UART0_BASE, test_packet[i]);
    }
}

static void test_systick(void)
{
    /* Envia mensagem de teste */
    const char *msg = "Testing SysTick...\r\n";
    while (*msg) UARTCharPut(UART0_BASE, *msg++);
    
    uint32_t start_time = get_systick_ms();
    
    /* Testa delay de 1 segundo */
    while ((get_systick_ms() - start_time) < 1000)
    {
        // Aguarda 1 segundo
    }
    
    msg = "SysTick working! 1000ms elapsed.\r\n";
    while (*msg) UARTCharPut(UART0_BASE, *msg++);
}
