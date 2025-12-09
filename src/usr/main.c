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
#include "pso_debug.h"
#include "pso_rpm.h"
#include "pso_system.h"

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
extern uint32_t g_pulse_diff;               /* Pulse difference */
extern uint8_t g_pwm_value;                 /* Global PWM value */

/*******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************/
uint16_t scan_period_actual;
pwm_profile_t current_profile = PWM_PROFILE_NONE;
uint8_t profile_complete = 0U;     /* Flag indicating profile completion */
uint32_t profile_start_time = 0U;  /* Start time of current profile */
uint32_t g_scaled_rpm;

/* PWM Profile Configuration */
static trapezoid_config_t active_trapezoid_config;

/*******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/
static void system_init(void);
static void stream_data_uart(void);
static void handle_data_capture(void);

/*******************************************************************************
 * SYSYICK FUNCTION PROTOTYPES
 ******************************************************************************/
void SysTick_Handler(void);                    // Handler do SysTick
uint32_t get_systick_ms(void);                 // Obtém ms desde início
static void configure_systick(void);           // Configura SysTick

/* State machine handlers */
static sys_state_t state_idle(void);
static sys_state_t state_init(void);
static sys_state_t state_timing(void);
static sys_state_t state_processing(void);
static sys_state_t state_streaming(void);
static sys_state_t state_pwm_control(void);
static sys_state_t state_stopping(void);
static sys_state_t state_error(void);

int main(void)
{
    sys_state_t sys_state = SYS_STATE_INIT;
    
    /* Initialize system peripherals and configurations */
    system_init();

    /* Initialize PWM profile system */
    pwm_profile_init();
    
    /* Configurar perfil trapezoidal personalizado */
    active_trapezoid_config = (trapezoid_config_t){
        .duration_ms = 30000,      /* 30 segundos total */
        .ramp_up_ms = 5000,        /* 5 segundos subida */
        .hold_ms = 20000,          /* 20 segundos em máximo */
        .ramp_down_ms = 5000,      /* 5 segundos descida */
        .min_value = 0,           /* Mínimo 0% */
        .max_value = 100,           /* Máximo 100% */
        .cycles = 2,               /* Repetir 2 vezes */
        .auto_repeat = false
    };
    
    // /* Usar template pré-configurado (opcional) */
    // const trapezoid_config_t* soft_start = pwm_get_trapezoid_template_soft_start();
    // active_trapezoid_config = *soft_start;

    /***************************************************************************
     * MAIN LOOP OTIMIZADO
     **************************************************************************/
    while (1)
    {
        /* DEBUG: Pulse ADC timing pin */
//        DEBUG_STATE_TOGGLE();

        /* 1. PROCESSAMENTO DE AMOSTRAS ADC (alta prioridade) */
        if (g_timer_a0_scan_flag)
        {
            g_timer_a0_scan_flag = 0U;
        }
        
        /* 2. LOOP DE CONTROLE PRINCIPAL */
        /* Máquina de estados principal */
        switch (sys_state)
        {
            case SYS_STATE_INIT:
                sys_state = state_init();
                break;

            case SYS_STATE_IDLE:
                sys_state = state_idle();
                break;

            case SYS_STATE_TIMING:
                sys_state = state_timing();
                break;
                
            case SYS_STATE_DATA_PROCESSING:
                sys_state = state_processing();  // Scaling + filtering
                break;

            case SYS_STATE_STREAMING:
                sys_state = state_streaming();  // Streaming contínuo
                break;
                
            case SYS_STATE_PWM_CONTROL:         
                sys_state = state_pwm_control(); // Controle PWM
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

/*******************************************************************************
 * SYSTEM INITIALIZATION
 ******************************************************************************/
static void system_init(void)
{
    uint16_t i;

    /* Configure system clock: 16MHz -> PLL -> 400MHz -> /10 = 40MHz */
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | 
                   SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    
    /* CONFIGURA SysTick para 1ms */
    SysTickPeriodSet(SysCtlClockGet() / 1000);  // 1ms
    SysTickIntRegister(&SysTick_Handler);       // Registra handler
    SysTickEnable();
    SysTickIntEnable();  

    /* INITIALIZE DEBUG GPIO */
    debug_gpio_init();

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
    
    /* Configura perfil PWM */
    current_profile = PWM_PROFILE_TRAPEZOID;  // Perfil padrão
    
    /* Configura taxas específicas */
    timing_configure(RATE_1000_HZ, 0);  // Loop principal a 1 kHz
    
    /* Reseta contadores */
    timing_reset();
    sample_counter = 0;
    
    /* Clear FIFOs */
    fifo_init(&g_fifo_ping);
    fifo_init(&g_fifo_pong);
    
    /* Configura o perfil trapezoidal ativo */
    pwm_set_trapezoid_config(&active_trapezoid_config);
    
    /* Indicate successful initialization */
    PSO_LEDGreenOn();
    
    return SYS_STATE_IDLE;
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
        return SYS_STATE_TIMING;
    }

    return SYS_STATE_IDLE;
}

/*******************************************************************************
 * STATE MACHINE: TIMING 
 * 
 * Controls the loop rate.
 ******************************************************************************/
static sys_state_t state_timing(void)
{
    /* Controls the loop rate (1 kHz) */
    if (check_interval_ms(1))
    {
        DEBUG_STATE_TOGGLE(); /* PD7 - Yellow channel */

        return SYS_STATE_DATA_PROCESSING;
    }

    return SYS_STATE_TIMING;
}

/*******************************************************************************
 * STATE MACHINE: DATA PROCESSING
 * 
 * Scaling and filtering of acquired data before streaming.
 ******************************************************************************/
static sys_state_t state_processing(void)
{
    uint32_t raw_rpm;

    raw_rpm = rpm_get_raw_count();
    g_scaled_rpm = rpm_calculate(g_pulse_diff, RPM_CALC_PERIOD_MS, BLADE_NUMBER);

    /* Coleta e processa dados */
    packet_data(&dp);
    copy_data(uart_tx_buffer, &dp);
    
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
    uint32_t current_time = get_systick_ms();

    indicate_streaming();

    /* STREAMING DE DADOS (executa a cada 2ms = 500 Hz) */
    if ((current_time - last_stream_time) >= 2)  // 500 Hz
    {
        last_stream_time = current_time;
        
        if (enable_data_capture)
        {
            /* EXPLICITAMENTE chama uartBatchWrite para envio de dados */
            uartBatchWrite(UART0_BASE, uart_tx_buffer, PACKET_LENGTH);
            
            stream_packet_count++;
            
            /* Toggle LED para indicar atividade de streaming */
            led_green_toggle();
            DEBUG_ADC_TOGGLE(); /* PD6 -  */
        }
    }

    /* Check if SW2 is pressed (manual stop) */
    if (!(GPIO_PORTF_DATA_R & GPIO_PIN_0))
    {
        return SYS_STATE_STOPPING;
    }

    return SYS_STATE_PWM_CONTROL;
}

/*******************************************************************************
 * STATE MACHINE: PWM CONTROL STATE
 * 
 * Executes PWM control profiles (trapezoid, linear, step).
 ******************************************************************************/
static sys_state_t state_pwm_control(void)
{
    static uint32_t last_pwm_update = 0;
    static uint32_t last_profile_check = 0;
    uint32_t current_time = get_systick_ms();
    uint8_t profile_result = FUNCTION_RUNNING;

    indicate_pwm_active();

    last_pwm_update = current_time;
    
    /* Calcula tempo decorrido desde início do perfil */
    uint32_t elapsed_ms = current_time - profile_start_time;
    
    /* Executa perfil selecionado */
    switch (current_profile)
    {
        case PWM_PROFILE_TRAPEZOID:
            profile_result = execute_trapezoid_profile(elapsed_ms, &active_trapezoid_config);
            break;
            
        case PWM_PROFILE_LINEAR:
            {
                linear_config_t linear_config = {
                    .duration_ms = 30000,
                    .start_value = 0,
                    .end_value = 100,
                    .cycles = 1,
                    .bidirectional = false,
                    .slew_rate = 0.0f
                };
                profile_result = execute_linear_profile(elapsed_ms, &linear_config);
            }
            break;
            
        case PWM_PROFILE_STEP:
            {
                step_config_t step_config = {
                    .step_interval_ms = 5000,
                    .num_steps = 9,
                    .steps = {0, 25, 50, 75, 100, 75, 50, 25, 0},
                    .cycles = 1,
                    .ping_pong = false
                };
                profile_result = execute_step_profile(elapsed_ms, &step_config);
            }
            break;
            
        case PWM_PROFILE_CUSTOM:
            profile_result = execute_custom_profile(elapsed_ms);
            break;
            
        default:
            /* Fallback para trapezoidal */
            profile_result = execute_trapezoid_profile(elapsed_ms, &active_trapezoid_config);
            break;
    }
    
    /* Atualiza throttle no pacote de dados */
    dp.throttle = g_pwm_value;
        


    /* Verifica conclusão do perfil */
    if (profile_result == FUNCTION_COMPLETE)
    {
        profile_complete = 1U;
        pwm_profile_stop();
        
        return SYS_STATE_STOPPING;
    }

    /* Check if SW2 is pressed (manual stop) */
    if (!(GPIO_PORTF_DATA_R & GPIO_PIN_0))
    {
        pwm_profile_stop();
        return SYS_STATE_STOPPING;
    }

    return SYS_STATE_TIMING;
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

    /* Ensure PWM is stopped */
    pwm_profile_stop();
    pwm_set_throttle(0);  /* Garante PWM em 0% */

    /* Flush any remaining data in FIFOs */
    while (!fifo_is_empty(&g_fifo_ping))
    {
        uint8_t data = fifo_get(&g_fifo_ping);
        /* Transmit remaining data */
        UARTCharPut(UART0_BASE, data);
    }

    indicate_finish();
    
    /* Mensagem de finalização */
    char stop_msg[] = "System Stopped. Press SW1 to restart.\r\n";
    int i;
    for (i = 0; stop_msg[i] != '\0'; i++)
    {
        UARTCharPut(UART0_BASE, stop_msg[i]);
    }

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
    
    /* Stop PWM */
    pwm_profile_stop();

    indicate_error();
    
    /* Mensagem de erro */
    char error_msg[] = "System Error. Please reset.\r\n";
    int i;
    for (i = 0; error_msg[i] != '\0'; i++)
    {
        UARTCharPut(UART0_BASE, error_msg[i]);
    }

    return SYS_STATE_IDLE;
}

