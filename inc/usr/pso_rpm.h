/*******************************************************************************
 * FILENAME:    pso_rpm.h
 *
 * DESCRIPTION:
 *       Header file for RPM measurement module.
 *       Supports two measurement methods selectable via compile-time directive.
 *
 * USAGE:
 *       Define ONE of the following in pso_config.h or project settings:
 *         #define RPM_EDGE_COUNT_METHOD    // Edge counting (default)
 *         #define RPM_EDGE_PERIOD_METHOD   // Edge period measurement
 *
 * AUTHOR:      Rogerio Lima
 * CONSOLIDATED: December 2025
 ******************************************************************************/

#ifndef PSO_RPM_H_
#define PSO_RPM_H_

#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * COMPILE-TIME METHOD SELECTION
 ******************************************************************************/

/* Select measurement method (define ONE in pso_config.h) */
#define RPM_EDGE_COUNT_METHOD  /* Edge counting (default) */
//#define RPM_EDGE_PERIOD_METHOD /* Period measurement */



/* If neither is defined, default to edge count */
#if !defined(RPM_EDGE_COUNT_METHOD) && !defined(RPM_EDGE_PERIOD_METHOD)
    #define RPM_EDGE_COUNT_METHOD
#endif

/* Ensure only one method is selected */
#if defined(RPM_EDGE_COUNT_METHOD) && defined(RPM_EDGE_PERIOD_METHOD)
    #error "Only one RPM measurement method can be defined!"
#endif

/*******************************************************************************
 * COMMON CONFIGURATION (Both Methods)
 ******************************************************************************/

/* Blade/pulse configuration */
#define BLADE_NUMBER            2U      /* Pulses per revolution */

/* Timer configuration */
#define RPM_CALC_PERIOD_MS      100U    /* Timer3 interrupt period (ms) */

/* RPM validation range */
#define RPM_MIN_VALID           1U      /* Minimum valid RPM */
#define RPM_MAX_VALID           100000U /* Maximum valid RPM */

/*******************************************************************************
 * METHOD-SPECIFIC CONFIGURATION
 ******************************************************************************/

#ifdef RPM_EDGE_PERIOD_METHOD
/*-----------------------------------------------------------------------------
 * EDGE PERIOD METHOD - CONFIGURATION
 *---------------------------------------------------------------------------*/

/* Period validation */
#define MIN_EDGE_INTERVAL_US    100U     /* Min valid period (100 μs) */
#define MAX_EDGE_INTERVAL_MS    60000U   /* Max valid period (60 s) */

/* Timeout for stopped motor detection */
#define RPM_STOP_TIMEOUT_MS     2000U    /* Motor stopped if no edge for 2s */

/* Moving average filter */
#define RPM_FILTER_SAMPLES      4U       /* Filter size (2-10 samples) */

#else /* RPM_EDGE_COUNT_METHOD */
/*-----------------------------------------------------------------------------
 * EDGE COUNT METHOD - CONFIGURATION
 *---------------------------------------------------------------------------*/

/* Timeout for stopped motor */
#define RPM_STOP_TIMEOUT_MS     1000U    /* Motor stopped if count=0 for 1s */

#endif /* RPM_EDGE_PERIOD_METHOD / RPM_EDGE_COUNT_METHOD */

/*******************************************************************************
 * GLOBAL VARIABLE DECLARATIONS
 ******************************************************************************/

/* Common variables (both methods) */
extern volatile uint32_t g_rpm_value;       /* Current RPM value */
extern volatile uint32_t g_rpm_ready_flag;  /* New data flag */

/* Timer3A flag (declared in pso_isr.c) */
extern volatile uint32_t g_timer_a3_scan_flag;

/* Compatibility variable (may be used in other modules) */
extern uint32_t wt1cpp0_tav_buffer;

#ifdef RPM_EDGE_COUNT_METHOD
/*-----------------------------------------------------------------------------
 * EDGE COUNT METHOD - VARIABLES
 *---------------------------------------------------------------------------*/
extern volatile uint32_t g_rpm_raw_count;   /* Raw pulse count */
extern uint32_t g_pulse_diff;               /* Pulse difference */

#else /* RPM_EDGE_PERIOD_METHOD */
/*-----------------------------------------------------------------------------
 * EDGE PERIOD METHOD - VARIABLES
 *---------------------------------------------------------------------------*/
extern uint32_t g_edge_interval_us;         /* Period between edges (μs) */
extern uint32_t g_last_edge_time_us;        /* Time of last edge (μs) */
extern uint32_t g_last_capture_value;       /* Last timer capture value */
extern uint32_t g_edge_valid_count;         /* Valid edges counter */
extern uint32_t g_edge_timeout_counter;     /* Timeout counter (ms) */

#endif /* RPM_EDGE_COUNT_METHOD / RPM_EDGE_PERIOD_METHOD */

/*******************************************************************************
 * COMMON API FUNCTIONS (Both Methods)
 ******************************************************************************/

/**
 * @brief Get current RPM value
 * @return Current RPM (0 if stopped)
 */
uint32_t rpm_get_value(void);

/**
 * @brief Check if new RPM data is available
 * @return true if new data available
 */
bool rpm_is_ready(void);

/**
 * @brief Clear RPM ready flag
 */
void rpm_clear_ready_flag(void);

/**
 * @brief Get raw timer count value
 * @return Current timer count
 * 
 * @note Maintained for compatibility with both methods
 */
uint32_t rpm_get_raw_count(void);

/**
 * @brief Calculate RPM from pulse difference
 * @param pulse_diff Number of pulses
 * @param period_ms Measurement period (ms)
 * @param pulses_per_rev Pulses per revolution
 * @return Calculated RPM
 * 
 * @note In edge count method: used for RPM calculation
 *       In edge period method: maintained for compatibility
 */
uint32_t rpm_calculate(uint32_t pulse_diff, 
                       uint32_t period_ms, 
                       uint32_t pulses_per_rev);

/**
 * @brief Validate RPM reading
 * @param rpm RPM value to validate
 * @return true if valid
 */
bool rpm_is_valid(uint32_t rpm);

/**
 * @brief Reset RPM measurement system
 */
void rpm_reset(void);

/**
 * @brief Convert frequency to RPM
 * @param frequency_hz Pulse frequency in Hz
 * @param pulses_per_rev Pulses per revolution
 * @return Equivalent RPM
 */
uint32_t rpm_from_frequency(uint32_t frequency_hz, uint32_t pulses_per_rev);

/**
 * @brief Convert RPM to frequency
 * @param rpm RPM value
 * @param pulses_per_rev Pulses per revolution
 * @return Equivalent frequency in Hz
 */
uint32_t rpm_to_frequency(uint32_t rpm, uint32_t pulses_per_rev);

/*******************************************************************************
 * METHOD-SPECIFIC API FUNCTIONS
 ******************************************************************************/

#ifdef RPM_EDGE_COUNT_METHOD
/*-----------------------------------------------------------------------------
 * EDGE COUNT METHOD - API FUNCTIONS
 *---------------------------------------------------------------------------*/

/**
 * @brief Get pulse difference from last measurement
 * @return Number of pulses in last period
 */
uint32_t rpm_get_delta(void);

#else /* RPM_EDGE_PERIOD_METHOD */
/*-----------------------------------------------------------------------------
 * EDGE PERIOD METHOD - API FUNCTIONS
 *---------------------------------------------------------------------------*/

/**
 * @brief Calculate RPM from edge period
 * @param period_us Period between edges (μs)
 * @param pulses_per_rev Pulses per revolution
 * @return Calculated RPM
 */
uint32_t rpm_from_period_us(uint32_t period_us, uint32_t pulses_per_rev);

/**
 * @brief Get edge interval in microseconds
 * @return Period between last two edges (μs)
 */
uint32_t rpm_get_edge_interval_us(void);

/**
 * @brief Check if motor has stopped
 * @return true if no edges for timeout period
 */
bool rpm_is_stopped(void);

/**
 * @brief Get filtered RPM value
 * @return Filtered RPM (moving average)
 */
uint32_t rpm_get_filtered(void);

/**
 * @brief Update RPM filter with new value
 * @param new_rpm New RPM value to add
 */
void rpm_update_filter(uint32_t new_rpm);

/**
 * @brief Reset moving average filter
 */
void rpm_reset_filter(void);

#endif /* RPM_EDGE_COUNT_METHOD / RPM_EDGE_PERIOD_METHOD */

/*******************************************************************************
 * USAGE EXAMPLES
 ******************************************************************************/

/**
 * EDGE COUNT METHOD:
 * ==================
 * 
 * Initialization (in pso_init.c):
 *   - Configure WTimer1A as edge counter (no interrupts)
 *   - Configure Timer3A for 100ms periodic interrupt
 * 
 * In main loop:
 *   if (rpm_is_ready()) {
 *       uint32_t rpm = rpm_get_value();
 *       uint32_t delta = rpm_get_delta();
 *       printf("RPM: %lu (delta: %lu)\n", rpm, delta);
 *       rpm_clear_ready_flag();
 *   }
 * 
 * In Timer3A ISR (pso_isr.c):
 *   - Reads pulse count from WTimer1A
 *   - Calculates pulse difference
 *   - Stores in g_pulse_diff
 *   - Sets g_rpm_ready_flag
 * 
 * In main loop (when flag set):
 *   - Calculate: RPM = (g_pulse_diff * 600) / BLADE_NUMBER
 *   - Store in g_rpm_value
 * 
 * 
 * EDGE PERIOD METHOD:
 * ===================
 * 
 * Initialization (in pso_init.c):
 *   - Configure WTimer1A as edge capture WITH interrupts
 *   - Configure Timer3A for 100ms timeout detection
 * 
 * In main loop:
 *   if (rpm_is_ready()) {
 *       uint32_t rpm = rpm_get_value();
 *       uint32_t rpm_filtered = rpm_get_filtered();
 *       uint32_t period = rpm_get_edge_interval_us();
 *       printf("RPM: %lu (filtered: %lu, period: %lu us)\n", 
 *              rpm, rpm_filtered, period);
 *       rpm_clear_ready_flag();
 *   }
 * 
 * In WTimer1A ISR (pso_isr.c):
 *   - Captures timer value on each edge
 *   - Calculates period since last edge
 *   - Calculates RPM: 60,000,000 / (period_μs * BLADE_NUMBER)
 *   - Updates filter
 *   - Sets g_rpm_ready_flag
 * 
 * In Timer3A ISR (pso_isr.c):
 *   - Increments timeout counter
 *   - Sets RPM=0 if timeout reached
 */

/*******************************************************************************
 * PERFORMANCE COMPARISON
 ******************************************************************************/

/**
 * EDGE COUNT METHOD:
 * ==================
 * Update Rate:     10 Hz (100ms fixed)
 * CPU Load:        ~0.01% (12 μs every 100ms)
 * Min RPM:         ~60 RPM (limited by 100ms window)
 * Max RPM:         ~100,000 RPM (sensor limited)
 * Resolution:      Poor at low RPM, good at high RPM
 * Latency:         100ms fixed
 * Complexity:      Simple, proven
 * ISRs:            Timer3A only (10 Hz)
 * 
 * Advantages:
 *   + Low CPU usage (no edge interrupts)
 *   + Predictable update rate
 *   + Simple algorithm
 * 
 * Disadvantages:
 *   - 100ms latency
 *   - Poor low-RPM resolution
 *   - Limited minimum RPM
 * 
 * Best for:
 *   - High RPM applications (> 1000 RPM)
 *   - CPU-limited systems
 *   - Fixed sampling rate needed
 * 
 * 
 * EDGE PERIOD METHOD:
 * ===================
 * Update Rate:     Per edge (instant)
 * CPU Load:        ~0.5% @ 1000 RPM (variable)
 * Min RPM:         ~15 RPM (2s timeout)
 * Max RPM:         ~300,000 RPM (100μs min period)
 * Resolution:      Excellent at all RPM
 * Latency:         Instant (5-8 μs)
 * Complexity:      Moderate
 * ISRs:            WTimer1A (per edge) + Timer3A (10 Hz)
 * 
 * Advantages:
 *   + Instant updates (no latency)
 *   + Excellent resolution at low RPM
 *   + Wide measurement range
 *   + High precision (25ns timer)
 * 
 * Disadvantages:
 *   - Higher CPU usage (ISR per edge)
 *   - Variable update rate
 *   - More complex
 * 
 * Best for:
 *   - Low RPM applications (< 1000 RPM)
 *   - Instant response needed
 *   - High precision required
 * 
 * 
 * QUICK COMPARISON:
 * =================
 * 
 * Feature          | Edge Count   | Edge Period
 * -----------------|--------------|-------------
 * Update Rate      | 10 Hz        | Per edge
 * CPU Load         | ~0.01%       | ~0.5% @ 1kRPM
 * Min RPM          | ~60          | ~15
 * Max RPM          | ~100k        | ~300k
 * Latency          | 100ms        | 5-8 μs
 * Resolution @100  | ±10 RPM      | ±0.1 RPM
 * Resolution @1k   | ±1 RPM       | ±0.001 RPM
 */

/*******************************************************************************
 * CONFIGURATION GUIDE
 ******************************************************************************/

/**
 * BLADE_NUMBER CONFIGURATION:
 * ===========================
 * 
 * BLADE_NUMBER = Number of pulses per complete motor revolution
 * 
 * Common configurations:
 * 
 * 1. Brushless Motor with Hall Sensor:
 *    - 2-pole motor (1 pole pair) → BLADE_NUMBER = 2
 *    - 4-pole motor (2 pole pairs) → BLADE_NUMBER = 4
 *    - 6-pole motor (3 pole pairs) → BLADE_NUMBER = 6
 *    - 8-pole motor (4 pole pairs) → BLADE_NUMBER = 8
 * 
 * 2. Optical Encoder:
 *    - 360 PPR (pulses per revolution) → BLADE_NUMBER = 360
 *    - 1024 PPR encoder → BLADE_NUMBER = 1024
 * 
 * 3. Magnetic Encoder:
 *    - 12 magnets → BLADE_NUMBER = 12
 *    - 24 magnets → BLADE_NUMBER = 24
 * 
 * How to determine BLADE_NUMBER:
 * 1. Rotate motor slowly by hand (1 complete revolution)
 * 2. Count pulses on PC6 using oscilloscope
 * 3. BLADE_NUMBER = total pulses counted
 * 
 * 
 * METHOD SELECTION GUIDE:
 * =======================
 * 
 * Choose EDGE COUNT METHOD if:
 *   - Application RPM > 1000 RPM
 *   - CPU resources limited
 *   - Fixed update rate acceptable
 *   - Simple algorithm preferred
 * 
 * Choose EDGE PERIOD METHOD if:
 *   - Application RPM < 1000 RPM
 *   - Instant updates required
 *   - High precision needed
 *   - CPU resources available
 * 
 * 
 * TIMEOUT CONFIGURATION:
 * ======================
 * 
 * RPM_STOP_TIMEOUT_MS should be set based on minimum expected RPM:
 * 
 * For 1 RPM minimum with BLADE_NUMBER=2:
 *   Period = 60,000,000 μs / (1 × 2) = 30,000,000 μs = 30 seconds
 *   Set: RPM_STOP_TIMEOUT_MS = 35000 (with 20% margin)
 * 
 * For 15 RPM minimum with BLADE_NUMBER=2:
 *   Period = 60,000,000 μs / (15 × 2) = 2,000,000 μs = 2 seconds
 *   Set: RPM_STOP_TIMEOUT_MS = 2000 (current default) ✓
 * 
 * For 60 RPM minimum with BLADE_NUMBER=2:
 *   Period = 60,000,000 μs / (60 × 2) = 500,000 μs = 500 ms
 *   Set: RPM_STOP_TIMEOUT_MS = 1000 (with 2× margin)
 */

#endif /* PSO_RPM_H_ */
