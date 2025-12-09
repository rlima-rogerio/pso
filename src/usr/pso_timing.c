/**
 * @file pso_timing.c
 * @brief Implementação do sistema de temporização
 */

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "pso_timing.h"

/* Variáveis estáticas */
static timing_control_t g_timing_controller;
volatile uint32_t g_system_tick_counter = 0;
static uint32_t g_ticks_per_ms = 40;  // Padrão para 40 MHz

volatile uint32_t g_system_ms_counter = 0;
uint8_t streaming_active = 0U;
uint8_t enable_data_capture = 1U;
uint8_t fix_rpm_start_acq = 0U;
uint32_t sample_counter = 0U;

/**
 * @brief Handler do SysTick (1 ms)
 */
void SysTick_Handler(void)
{
    g_system_tick_counter++;
}


/* Obtém ticks atual em ms */
uint32_t get_systick_ms(void)
{
    return g_system_tick_counter;
}


/**
 * @brief Inicializa o sistema de temporização
 */
void timing_init(uint32_t system_clock_hz)
{
    /* Calcula ticks por milissegundo */
    g_ticks_per_ms = system_clock_hz / 1000;
    
    /* Configura SysTick para 1 ms */
    NVIC_ST_CTRL_R = 0;                    // Desabilita temporariamente
    NVIC_ST_RELOAD_R = g_ticks_per_ms - 1; // Recarga para 1 ms
    NVIC_ST_CURRENT_R = 0;                 // Limpa contador atual
    NVIC_ST_CTRL_R = NVIC_ST_CTRL_CLK_SRC | 
                     NVIC_ST_CTRL_INTEN | 
                     NVIC_ST_CTRL_ENABLE;  // Habilita
    
    /* Inicializa estrutura de controle */
    g_timing_controller.last_tick = 0;
    g_timing_controller.interval_ticks = 10 * g_ticks_per_ms; // Padrão: 100 Hz
    g_timing_controller.actual_interval = 0;
    g_timing_controller.execution_count = 0;
    g_timing_controller.enabled = true;
    g_timing_controller.ready_flag = false;
    g_timing_controller.rate = RATE_100_HZ;
    g_timing_controller.missed_deadlines = 0;
}

/**
 * @brief Configura taxa de amostragem/processamento
 */
void timing_configure(sample_rate_t rate, uint32_t custom_interval_ms)
{
    uint32_t interval_ms;
    
    if (rate == RATE_CUSTOM && custom_interval_ms > 0)
    {
        interval_ms = custom_interval_ms;
        g_timing_controller.rate = RATE_CUSTOM;
    }
    else
    {
        /* Converte enum para ms */
        switch (rate)
        {
            case RATE_1_HZ:    interval_ms = 1000; break;
            case RATE_10_HZ:   interval_ms = 100; break;
            case RATE_50_HZ:   interval_ms = 20; break;
            case RATE_100_HZ:  interval_ms = 10; break;
            case RATE_200_HZ:  interval_ms = 5; break;
            case RATE_500_HZ:  interval_ms = 2; break;
            case RATE_1000_HZ: interval_ms = 1; break;
            case RATE_2000_HZ: interval_ms = 0; break; // Tratamento especial
            default:           interval_ms = 10; break; // Padrão 100 Hz
        }
        g_timing_controller.rate = rate;
    }
    
    /* Configura intervalo em ticks */
    if (interval_ms == 0)
    {
        /* 2000 Hz (0.5 ms) - tratamento especial */
        g_timing_controller.interval_ticks = g_ticks_per_ms / 2;
    }
    else
    {
        g_timing_controller.interval_ticks = interval_ms * g_ticks_per_ms;
    }
    
    g_timing_controller.last_tick = g_system_tick_counter;
    g_timing_controller.ready_flag = false;
    g_timing_controller.missed_deadlines = 0;
}

/**
 * @brief Habilita/desabilita temporização
 */
void timing_enable(bool enable)
{
    g_timing_controller.enabled = enable;
    if (enable)
    {
        g_timing_controller.last_tick = g_system_tick_counter;
    }
}

/**
 * @brief Verifica se o intervalo configurado foi atingido
 */
bool timing_check_interval(void)
{
    if (!g_timing_controller.enabled)
    {
        return false;
    }
    
    uint32_t current_tick = g_system_tick_counter;
    uint32_t elapsed_ticks;
    
    /* Cálculo seguro de diferença (considera overflow) */
    if (current_tick >= g_timing_controller.last_tick)
    {
        elapsed_ticks = current_tick - g_timing_controller.last_tick;
    }
    else
    {
        /* Tratamento de overflow (ocorre a cada ~49.7 dias) */
        elapsed_ticks = (0xFFFFFFFF - g_timing_controller.last_tick) + current_tick + 1;
    }
    
    /* Verifica se intervalo foi atingido */
    if (elapsed_ticks >= g_timing_controller.interval_ticks)
    {
        /* Atualiza estatísticas */
        g_timing_controller.actual_interval = elapsed_ticks / g_ticks_per_ms;
        g_timing_controller.execution_count++;
        g_timing_controller.last_tick = current_tick;
        
        /* Verifica se perdeu deadlines */
        if (elapsed_ticks > (g_timing_controller.interval_ticks * 2))
        {
            g_timing_controller.missed_deadlines++;
        }
        
        return true;
    }
    
    return false;
}

/**
 * @brief Obtém tempo decorrido desde última execução (ms)
 */
uint32_t timing_get_elapsed_ms(void)
{
    return g_timing_controller.actual_interval;
}

/**
 * @brief Obtém contador de execuções
 */
uint32_t timing_get_execution_count(void)
{
    return g_timing_controller.execution_count;
}

/**
 * @brief Obtém taxa real de execução (Hz)
 */
float timing_get_actual_rate_hz(void)
{
    if (g_timing_controller.actual_interval > 0)
    {
        return 1000.0f / g_timing_controller.actual_interval;
    }
    return 0.0f;
}

/**
 * @brief Obtém contador de deadlines perdidos
 */
uint32_t timing_get_missed_deadlines(void)
{
    return g_timing_controller.missed_deadlines;
}

/**
 * @brief Reseta contadores
 */
void timing_reset(void)
{
    g_timing_controller.execution_count = 0;
    g_timing_controller.missed_deadlines = 0;
    g_timing_controller.last_tick = g_system_tick_counter;
    g_timing_controller.ready_flag = false;
}

/**
 * @brief Delay em milissegundos (bloqueante)
 */
void delay_ms(uint32_t milliseconds)
{
    uint32_t start_tick = g_system_tick_counter;
    uint32_t target_ticks = milliseconds * g_ticks_per_ms;
    uint32_t current_tick;
    
    while (1)
    {
        current_tick = g_system_tick_counter;
        
        if (current_tick >= start_tick)
        {
            if ((current_tick - start_tick) >= target_ticks)
                break;
        }
        else
        {
            /* Tratamento de overflow */
            if (((0xFFFFFFFF - start_tick) + current_tick + 1) >= target_ticks)
                break;
        }
    }
}

/**
 * @brief Delay em microsegundos (aproximado)
 */
void delay_us(uint32_t microseconds)
{
    uint32_t cycles = (microseconds * (g_ticks_per_ms / 1000));
    uint32_t i;
    
    for (i = 0; i < cycles; i++)
    {
        __asm(" NOP");
    }
}

/**
 * @brief Obtém contador de ticks do sistema
 */
uint32_t get_system_ticks(void)
{
    return g_system_tick_counter;
}

/* Função para obter tempo em ms */
uint32_t get_system_ms(void)
{
    //return g_system_ms_counter;
    return get_system_ticks();
}

/* Função de timing simplificada */
bool check_interval_ms(uint32_t interval_ms)
{
    static uint32_t last_time = 0;
    uint32_t current_time = get_system_ticks();  //get_system_ms();
    
    if ((current_time - last_time) >= interval_ms)
    {
        last_time = current_time;
        return true;
    }
    
    return false;
}
