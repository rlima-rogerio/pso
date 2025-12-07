/**
 * @file pso_timing.h
 * @brief Sistema de temporização para controle de taxa de amostragem e streaming
 */

#ifndef PSO_TIMING_H
#define PSO_TIMING_H

#include <stdint.h>
#include <stdbool.h>

/* Adicione estas declarações externas */
extern volatile uint32_t g_system_ms_counter;
extern uint8_t streaming_active;
extern uint8_t enable_data_capture;
extern uint8_t fix_rpm_start_acq;
extern uint32_t sample_counter;

/* Taxas de amostragem predefinidas */
typedef enum {
    RATE_1_HZ      = 1000,    // 1 Hz
    RATE_10_HZ     = 100,     // 10 Hz
    RATE_50_HZ     = 20,      // 50 Hz
    RATE_100_HZ    = 10,      // 100 Hz
    RATE_200_HZ    = 5,       // 200 Hz
    RATE_500_HZ    = 2,       // 500 Hz
    RATE_1000_HZ   = 1,       // 1000 Hz
    RATE_2000_HZ   = 0,       // 2000 Hz (especial - 0.5 ms)
    RATE_CUSTOM    = 255      // Taxa personalizada
} sample_rate_t;

/* Estrutura de controle de temporização */
typedef struct {
    uint32_t last_tick;        // Último tick do sistema
    uint32_t interval_ticks;   // Intervalo em ticks
    uint32_t actual_interval;  // Intervalo real medido
    uint32_t execution_count;  // Contador de execuções
    bool enabled;              // Habilitado/desabilitado
    bool ready_flag;           // Flag de pronto para executar
    sample_rate_t rate;        // Taxa configurada
    uint32_t missed_deadlines; // Contador de deadlines perdidos
} timing_control_t;

/* Inicialização e configuração */
void timing_init(uint32_t system_clock_hz);
void timing_configure(sample_rate_t rate, uint32_t custom_interval_ms);
void timing_enable(bool enable);
void timing_reset(void);

/* Controle e monitoramento */
bool timing_check_interval(void);
uint32_t timing_get_elapsed_ms(void);
uint32_t timing_get_execution_count(void);
float timing_get_actual_rate_hz(void);
uint32_t timing_get_missed_deadlines(void);

/* Utilitários */
void delay_ms(uint32_t milliseconds);
void delay_us(uint32_t microseconds);
uint32_t get_system_ticks(void);

extern volatile uint32_t g_system_tick_counter;

bool check_interval_ms(uint32_t interval_ms);

#endif /* PSO_TIMING_H */
