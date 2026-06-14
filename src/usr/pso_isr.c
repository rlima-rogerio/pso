/*******************************************************************************
 * FILE:        pso_isr.c
 *
 * DESCRIPTION:
 *     PSO Interrupt Service Routines (ISRs) Module.
 *     Contains interrupt handlers for UART, timers, and ADCs used in the
 *     Propeller Speed Optimizer (PSO) system. Handles real-time data
 *     acquisition, RPM calculation, and system timing.
 *
 *     RPM is measured with the edge-period method only. WTimer1A captures
 *     both sensor edges on PC6/WT1CCP0, and Timer3A handles stopped-motor
 *     timeout detection every 100 ms.
 *
 * AUTHOR:      Rogerio Lima
 * UPDATED:     June 2026
 *
 *******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
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
#include "interrupt.h"
#include "timer.h"
#include "uart.h"
#include "hw_uart.h"
#include "pso_init.h"
#include "pso_uart.h"
#include "pso_pwm.h"
#include "pso_timing.h"
#include "fifo.h"
#include "pso_data.h"
#include "ulink.h"
#include "ulink_pso.h"
#include "ulink_types.h"
#include "pso_debug.h"
#include "pso_rpm.h"

/*******************************************************************************
 * GLOBAL VARIABLES (External Declarations)
 *******************************************************************************/
extern volatile uint8_t g_led_toggle_flag;   /* LED event flag */
extern uart_raw_data_t g_uart0_data;         /* UART0 receive buffer */

/*******************************************************************************
 * MODULE-SPECIFIC GLOBAL VARIABLES
 *******************************************************************************/
uint8_t g_timer_a0_scan_flag = 0U;           /* Timer0A scan completion flag */
volatile uint32_t g_timer_a3_scan_flag = 0U; /* Timer3A scan flag (RPM ready) */
bool g_rpm_reset = false;                     // RPM reset flag

/*******************************************************************************
 * ADC DATA BUFFERS
 *     These buffers store raw ADC values from both ADC modules.
 *     Buffer indices correspond to specific sensor channels:
 *       0: Ax (Acceleration X-axis) / Ay (Acceleration Y-axis)
 *       1: Strain Gauge / Az (Acceleration Z-axis)
 *       2: Motor Voltage / Motor Current
 *******************************************************************************/
volatile uint32_t adc0_buffer[3];            /* ADC0 channel data buffer */
volatile uint32_t adc1_buffer[3];            /* ADC1 channel data buffer */

/*******************************************************************************
 * TIMING AND MEASUREMENT VARIABLES
 *******************************************************************************/
uint32_t wt1cpp0_tav_buffer;                 /* Wide Timer 1 capture buffer */
extern uint32_t g_edge_interval_us;          /* Period between edges in μs */
extern uint32_t g_last_edge_time_us;         /* Time of last edge in μs */
extern uint32_t g_last_capture_value;        /* Last timer capture value */
extern uint32_t g_edge_valid_count;          /* Valid edges counter */
extern uint32_t g_edge_timeout_counter;      /* Timeout for stopped motor */
uint32_t g_period_us; // Debugging variable
uint32_t g_period_ticks; // Debugging variable

/*******************************************************************************
 * FUNCTION: UART0IntHandler
 *
 * DESCRIPTION:
 *     Interrupt handler for UART0 receive operations.
 *     Reads available characters from UART0 receive FIFO and stores them
 *     in the global UART data buffer. Sets new_data flag when data is received.
 *
 * PARAMETERS:
 *     None
 *
 * RETURNS:
 *     void
 *
 * OPERATION:
 *     1. Reads and clears UART interrupt status
 *     2. Reads all available characters from receive FIFO
 *     3. Stores characters in circular buffer
 *     4. Sets new_data flag for main loop processing
 *
 * NOTES:
 *     - Non-blocking operation
 *     - Buffer overflow protection needed in main loop
 *     - Interrupt-driven for minimal CPU usage
 *******************************************************************************/
void UART0IntHandler(void)
{
    uint32_t ui32Status;

    /* 1. Get interrupt status and clear interrupts */
    ui32Status = UARTIntStatus(UART0_BASE, true);
    UARTIntClear(UART0_BASE, ui32Status);

    /* 2. Read all available characters from receive FIFO */
    while (UARTCharsAvail(UART0_BASE))
    {
        /* Store character in a circular buffer. */
        g_uart0_data.rx_buffer[g_uart0_data.rx_index] =
            (uint8_t)(HWREG(UART0_BASE + UART_O_DR) & 0xFFU);
        g_uart0_data.rx_index = (uint8_t)((g_uart0_data.rx_index + 1U) % UART_MAX_BUFFER);
    }

    /* 3. Set flag to indicate new data is available */
    g_uart0_data.new_data = 1;
}

/*******************************************************************************
 * FUNCTION: Timer0AIntHandler
 *
 * DESCRIPTION:
 *     Timer0A interrupt handler (currently minimal implementation).
 *     Clears the timer interrupt flag. Can be expanded for periodic tasks.
 *
 * PARAMETERS:
 *     None
 *
 * RETURNS:
 *     void
 *
 * NOTES:
 *     - Basic implementation - can be extended for system timing
 *     - Ensure Timer0 is configured with appropriate period
 *******************************************************************************/
void Timer0AIntHandler(void)
{
    /* Clear the timer interrupt flag */
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

/*******************************************************************************
 * RPM MEASUREMENT ISRs - EDGE-PERIOD IMPLEMENTATION
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION: WTimer1AIntHandler (EDGE PERIOD METHOD)
 *
 * DESCRIPTION:
 *     Wide Timer 1A edge capture interrupt handler for RPM measurement.
 *     Triggered on both edges at PC6 (WT1CCP0). The handler computes each
 *     blade-pulse midpoint and calculates RPM from consecutive midpoints.
 *
 * PARAMETERS:
 *     None
 *
 * RETURNS:
 *     void
 *
 * OPERATION:
 *     1. Read current timer capture value
 *     2. Calculate period since last edge (handles overflow)
 *     3. Convert timer ticks to microseconds (40 ticks = 1 μs)
 *     4. Validate period (noise filter + range check)
 *     5. Calculate RPM: 60,000,000 / (period_μs × BLADE_NUMBER)
 *     6. Update filter and set ready flag
 *     7. Reset timeout counter
 *     8. Clear interrupt flag
 *
 * RPM FORMULA:
 *     RPM = 60,000,000 / (period_μs × BLADE_NUMBER)
 *     
 *     Example (1000 RPM, BLADE_NUMBER=2):
 *       Period = 60,000,000 / (1000 × 2) = 30,000 μs = 30 ms
 *       Verify: 60,000,000 / (30,000 × 2) = 1000 RPM ✓
 *
 * NOTES:
 *     - ISR execution time: 5-8 μs
 *     - Instant RPM update (no 100ms delay)
 *     - Handles 32-bit timer overflow
 *     - Filters noise with MIN_EDGE_INTERVAL_US
 *******************************************************************************/

 #ifdef BLADE_DETECTION_LOGIC_DIRECT
 void WTimer1AIntHandler(void)
{
    /*
     * EDGE PERIOD METHOD - DIRECT LOGIC (Blade = HIGH pulse)
     * Timer: CAPTURE / EDGE-TIME / BOTH EDGES
     * Inference: read GPIO level after capture
     *   level_high == 1 -> RISE (start HIGH)
     *   level_high == 0 -> FALL (end HIGH) -> compute midpoint & RPM
     */

    /* ---- local constants (keep behavior consistent) ---- */
    enum { TICKS_PER_US = 40U };              /* 40 MHz => 40 ticks/us */
    enum { US_ROUND = (TICKS_PER_US / 2U) };  /* for rounding */

    /* ---- persistent state ---- */
    static bool     have_prev = false;
    static uint32_t t_prev = 0;

    static bool     have_mid = false;
    static uint32_t t_mid_prev = 0;

    static uint32_t t_rise = 0;
    static uint32_t t_fall = 0;

    static uint32_t last_rpm = 0;

    /* ---- capture timestamp ---- */
    uint32_t t_now = WTIMER1_TAR_R;

    /* Clear interrupt flag early */
    WTIMER1_ICR_R = TIMER_ICR_CAECINT;

    if (!have_prev)
    {
        t_prev = t_now;
        have_prev = true;
        return;
    }

    /* Time between consecutive edges (ticks/us) */
    uint32_t dt_ticks = (uint32_t)(t_now - t_prev);                  /* wrap-safe */
    uint32_t dt_us    = (dt_ticks + US_ROUND) / TICKS_PER_US;

    t_prev = t_now; /* update for next ISR */

    /* Gate BEFORE reading GPIO level: reject very short intervals (debounce/glitch) */
    if (dt_us < RPM_HIGH_MIN_US)
    {
        return;
    }

    /* Read current pin level (PC6). */
    const bool level_high = ((GPIO_PORTC_DATA_R & GPIO_PIN_6) != 0U);

    if (level_high)
    {
        /* RISE edge: start of HIGH pulse */
        t_rise = t_now;
        return;
    }

    /* FALL edge: end of HIGH pulse */
    t_fall = t_now;

    /* Midpoint (wrap-safe): compute width then midpoint, avoids (t_rise+t_fall) overflow */
    const uint32_t w_high_ticks = (uint32_t)(t_fall - t_rise);        /* wrap-safe */
    const uint32_t t_mid        = (uint32_t)(t_rise + (w_high_ticks / 2U));

    if (!have_mid)
    {
        t_mid_prev = t_mid;
        have_mid = true;
        return;
    }

    /* Interval between consecutive midpoints */
    const uint32_t dp_ticks = (uint32_t)(t_mid - t_mid_prev);
    const uint32_t dp_us    = (dp_ticks + US_ROUND) / TICKS_PER_US;

    /* Debug: keep variables consistent */
    g_period_ticks = w_high_ticks;  /* HIGH pulse width (ticks) */
    g_period_us    = (w_high_ticks + US_ROUND) / TICKS_PER_US; /* HIGH pulse width (us) */
    g_edge_interval_us = dp_us;     /* midpoint-to-midpoint interval (us) */

    /* Optional: enable/disable range check while debugging */
#if 0
    if ((dp_us < RPM_PERIOD_MIN_US) || (dp_us > RPM_PERIOD_MAX_US) || (dp_us == 0U))
    {
        return;
    }
#else
    if (dp_us == 0U)
    {
        return; /* avoid divide-by-zero */
    }
#endif

    g_edge_valid_count++;
    g_edge_timeout_counter = 0U;

    /* RPM candidate: RPM = 60,000,000 / (period_us * pulses_per_rev) */
    uint32_t rpm = 60000000UL / (dp_us * BLADE_NUMBER);

    /* Clamp to operating range; if out-of-range, keep last valid */
    if ((rpm >= RPM_MIN_OPERATING) && (rpm <= RPM_MAX_OPERATING))
    {
        if ((last_rpm != 0U) &&
            ((rpm >= (uint32_t)(18U * last_rpm / 10U)) ||  /* +80% */
             (rpm <= (uint32_t)( 8U * last_rpm / 10U))))   /* -20% */
        {
            /* Probably spurious measurement, but within valid window */
            rpm = last_rpm;
        }
        last_rpm = rpm;
    }
    else
    {
        /* Outside valid measurement: use last valid measurement */
        rpm = last_rpm;
    }

    g_rpm_value = rpm;

    /* Update moving average filter (ISR owns the filter update) */
    rpm_update_filter(g_rpm_value);

    /* Signal new RPM available */
    g_rpm_ready_flag = 1U;

    /* Update reference only on processed event */
    t_mid_prev = t_mid;

    DEBUG_ADC_TOGGLE(); /* PD6 - ADC debug pin */
}
#else /* BLADE_DETECTION_LOGIC_INVERTED */
/*
    * EDGE PERIOD METHOD - INVERTED LOGIC (Blade = LOW pulse)
    * Timer: CAPTURE / EDGE-TIME / BOTH EDGES
    * Inference: read GPIO level after capture
    *   level_high == 0 -> FALL (start LOW)
    *   level_high == 1 -> RISE (end LOW)  -> compute midpoint & RPM
    */

/* ---- local constants (keep behavior consistent) ---- */
void WTimer1AIntHandler(void)
{
    /* ======================================================================
     * CONSTANTS
     * ====================================================================== */
    enum { TICKS_PER_US = 40U };     /* 40 MHz => 40 ticks/microsecond */
    enum { US_ROUND = 20U };         /* Rounding factor: TICKS_PER_US/2 */
    
    /* ======================================================================
     * PERSISTENT STATE VARIABLES
     * 
     * These static variables preserve their values between ISR calls,
     * implementing a state machine that tracks edge detection and midpoint
     * calculation across multiple interrupts.
     * ====================================================================== */
    
    /* Edge Detection State Machine */
    static bool have_rising_edge = false;   /* True when rising edge detected */
    static bool have_falling_edge = false;  /* True when falling edge detected */
    static bool toggle = false;             /* XOR result (unused, kept for compatibility) */
    static bool last_level = false;         /* Previous pin level for XOR calculation */
    
    /* Edge Timestamps (in timer ticks) */
    static uint32_t t_rise = 0;      /* Timestamp of last rising edge */
    static uint32_t t_fall = 0;      /* Timestamp of last falling edge */
    
    /* Midpoint Calculation */
    static uint32_t t_mid = 0;            /* Current midpoint timestamp */
    static uint32_t t_mid_prev = 0;       /* Previous midpoint timestamp */
    static uint32_t t_mid_counter = 0;    /* Count of valid midpoints processed */
    
    /* RPM Calculation State */
    static uint32_t rpm_prev = 0;         /* Previous accepted RPM value */
    static bool compute_rpm = false;      /* Flag to trigger RPM calculation */
    
    
    /* ======================================================================
     * CAPTURE CURRENT TIMESTAMP
     * 
     * Read timer value immediately to minimize jitter. The timer counts UP
     * continuously at 40 MHz (25 ns resolution).
     * ====================================================================== */
    uint32_t t_now = WTIMER1_TAR_R;
    
    /* Clear interrupt flag early to avoid missing next edge */
    WTIMER1_ICR_R = TIMER_ICR_CAECINT;
    

    if (g_rpm_reset)
    {
        g_rpm_reset = false;

        /* Reset RPM values */
        rpm_prev = 0U;

        /* Reset edge detection flags */
        have_rising_edge = false;
        have_falling_edge = false;

        /* Reset edge timestamps */
        t_rise = 0U;
        t_fall = 0U;

        /* Reset midpoint calculation */
        t_mid = 0U;
        t_mid_prev = 0U;
        t_mid_counter = 0U;

        /* Reset RPM calculation flag */
        compute_rpm = false;

        /* Reset XOR state (compatibility) */
        toggle = false;
        last_level = false;
    }

    /* ======================================================================
     * TIMEOUT COUNTER RESET
     * 
     * Reset the edge timeout counter to indicate motor is running.
     * This prevents false "stopped" detection during normal operation.
     * This is done on every valid edge interrupt. The incrementing of the
     * timeout counter occurs in a separate timer ISR (Timer3AIntHandler).
     * ====================================================================== */
    g_edge_timeout_counter = 0U; 

    /* ======================================================================
     * READ CURRENT GPIO LEVEL
     * 
     * Read the digital level of PC6 pin. This determines whether current
     * interrupt was triggered by rising edge (LOW→HIGH) or falling edge
     * (HIGH→LOW).
     * ====================================================================== */
    const bool level_high = ((GPIO_PORTC_DATA_R & GPIO_PIN_6) != 0U);
    
    /* ======================================================================
     * EDGE DETECTION USING XOR
     * 
     * MATLAB equivalent: toggle(n) = xor(level(n-1), level(n))
     * 
     * The XOR operation detects transitions:
     *   - If level changed: toggle = true (edge occurred)
     *   - If level unchanged: toggle = false (no edge, spurious interrupt)
     *  
     * This filters out noise and ensures we only process real edges.
     * ====================================================================== */
    toggle = (last_level != level_high);
    last_level = level_high;
    
    /* ──────────────────────────────────────────────────────────────────
     * RISING EDGE DETECTION
     * 
     * Condition: toggle=true AND level_high=true
     * Meaning: Pin transitioned from LOW to HIGH (start of HIGH pulse)
     * 
     * Action: Record timestamp and set flag
     * ────────────────────────────────────────────────────────────────── */
    if (toggle && level_high)
    {
        t_rise = t_now;
        have_rising_edge = true;
    }
    
    /* ──────────────────────────────────────────────────────────────────
     * FALLING EDGE DETECTION
     * 
     * Condition: toggle=true AND level_high=false
     * Meaning: Pin transitioned from HIGH to LOW (end of HIGH pulse)
     * 
     * Action: Record timestamp and set flag
     * ────────────────────────────────────────────────────────────────── */
    if (toggle && !level_high)
    {
        t_fall = t_now;
        have_falling_edge = true;
    }
    
    /* ======================================================================
     * MIDPOINT CALCULATION
     * 
     * After detecting both rising and falling edges, we have a complete
     * HIGH pulse. Calculate the midpoint timestamp, which represents the
     * center of the blade passing through the sensor.
     * 
     * MATLAB equivalent:
     *   if (have_rising_edge && have_falling_edge)
     *       tsec_mid_prev = tsec_mid;
     *       if (tsec_low - tsec_high > 1e-5)  % 10us minimum
     *           tsec_mid = (tsec_low + tsec_high) / 2
     *           have_rising_edge = false;
     *           have_falling_edge = false;
     *           compute_rpm = true;
     *           t_mid_counter = t_mid_counter + 1;
     *       end
     *   end
     * ====================================================================== */
    
    if (have_rising_edge && have_falling_edge)
    {
        /* ──────────────────────────────────────────────────────────────
         * CALCULATE HIGH PULSE WIDTH
         * 
         * Subtract timestamps (automatically handles timer wraparound due
         * to unsigned arithmetic properties).
         * Convert from ticks to microseconds with rounding.
         * ────────────────────────────────────────────────────────────── */
        const uint32_t w_high_ticks = (uint32_t)(t_fall - t_rise);  /* Wrap-safe */
        const uint32_t w_high_us = (w_high_ticks + US_ROUND) / TICKS_PER_US;
        
        /* Save previous midpoint before updating (matches MATLAB order) */
        t_mid_prev = t_mid;
        
        /* ──────────────────────────────────────────────────────────────
         * VALIDATE PULSE WIDTH
         * 
         * Reject pulses shorter than RPM_HIGH_MIN_US (typically 10us).
         * This filters out electrical noise and contact bounce.
         * 
         * MATLAB: if (tsec_low - tsec_high > 1e-5)
         * ────────────────────────────────────────────────────────────── */
        if (w_high_us > RPM_HIGH_MIN_US)
        {
            /* ──────────────────────────────────────────────────────────
             * CALCULATE MIDPOINT TIMESTAMP
             * 
             * MATLAB: tsec_mid = (tsec_low + tsec_high) / 2
             * C: t_mid = t_rise + (w_high_ticks / 2)
             * 
             * Both formulas are equivalent. The C version avoids potential
             * overflow from adding two large timestamps.
             * ────────────────────────────────────────────────────────── */
            t_mid = (uint32_t)(t_rise + (w_high_ticks / 2U));

            /* Reset edge detection flags for next pulse */
            have_rising_edge = false;
            have_falling_edge = false;
            
            /* Enable RPM calculation flag */
            compute_rpm = true;
            
            /* Increment valid midpoint counter */
            t_mid_counter++;
        }
        
        /* ══════════════════════════════════════════════════════════════
         * RPM CALCULATION
         * 
         * After accumulating at least 2 midpoints, calculate RPM from the
         * time interval between consecutive midpoints.
         * 
         * MATLAB equivalent:
         *   if ((t_mid_counter > 1) && (compute_rpm))
         *       compute_rpm = false;
         *       RPM = 60 / (2 * (tsec_mid - tsec_mid_prev));
         *   end
         * 
         * WHY DIVIDE BY 2?
         * ─────────────────
         * Each midpoint represents one blade passing. With 2 blades per
         * revolution, the interval between midpoints is HALF the period:
         * 
         *   Midpoint 1 → Blade A passes
         *   Midpoint 2 → Blade B passes  } Half revolution
         *   Midpoint 3 → Blade A passes  } Complete revolution
         * 
         * Therefore:
         *   Full period = 2 * (t_mid - t_mid_prev)
         *   Frequency = 1 / period = 1 / (2 * interval)
         *   RPM = Frequency * 60 = 60 / (2 * interval)
         * 
         * FORMULA CONVERSION:
         * ───────────────────
         * MATLAB: RPM = 60 / (2 * tsec_interval)
         *   where tsec_interval is in seconds
         * 
         * C: RPM = 60,000,000 / (2 * dp_us)
         *   where dp_us is in microseconds
         *   60,000,000 = 60 seconds * 1,000,000 us/second
         * 
         * Simplified: RPM = 30,000,000 / dp_us
         * 
         * Generic form: RPM = 60,000,000 / (dp_us * BLADE_NUMBER)
         *   where BLADE_NUMBER = 2
         * ══════════════════════════════════════════════════════════════ */
        
        if (compute_rpm && (t_mid_counter > 1U))
        {
            /* Clear compute flag (single-shot calculation) */
            compute_rpm = false;

            /* ──────────────────────────────────────────────────────────
             * CALCULATE MIDPOINT INTERVAL
             * 
             * Subtract timestamps (wrap-safe unsigned arithmetic).
             * Convert from ticks to microseconds with rounding.
             * ────────────────────────────────────────────────────────── */
            const uint32_t dp_ticks = (uint32_t)(t_mid - t_mid_prev);
            const uint32_t dp_us = (dp_ticks + US_ROUND) / TICKS_PER_US;

            g_period_ticks = w_high_ticks;
            g_period_us = w_high_us;
            g_edge_interval_us = dp_us;

            if (dp_us == 0U)
            {
                return;
            }

            /* ──────────────────────────────────────────────────────────
             * CALCULATE RPM
             * 
             * Formula: RPM = 60,000,000 / (dp_us * BLADE_NUMBER)
             * 
             * Example (30,000 RPM, 2 blades):
             *   Frequency = 30000/60 = 500 rps
             *   Period = 1/500 = 2 ms = 2000 us per revolution
             *   Midpoint interval = 2000/2 = 1000 us
             *   RPM = 60,000,000 / (1000 * 2) = 30,000 ✓
             * ────────────────────────────────────────────────────────── */
            uint32_t rpm = 60000000UL / (dp_us * BLADE_NUMBER);

            // if ((rpm >= RPM_MIN_OPERATING) && (rpm <= RPM_MAX_OPERATING) && (t_mid_counter > 3U))
            // {
            //     if ((rpm_prev != 0U) &&
            //         ((rpm >= (uint32_t)(18U * rpm_prev / 10U)) ||  /* +80% */
            //          (rpm <= (uint32_t)( 8U * rpm_prev / 10U))))   /* -80% */
            //     {
            //         /* Probably spurious measurement, but within valid window */
            //         rpm = rpm_prev;
            //     }
            //     rpm_prev = rpm;
            // }
            // else
            // {
            //     /* Outside valid measurement: use last valid measurement */
            //     rpm = rpm_prev;
            // }

            if ((rpm >= RPM_MIN_OPERATING) && (rpm <= RPM_MAX_OPERATING) && (t_mid_counter > 3U))
            {
                /* Valid RPM range and enough samples collected */
                
                if (rpm_prev == 0U)
                {
                    /* FIRST VALID MEASUREMENT after motor start or reset */
                    /* Accept unconditionally - no previous reference to compare */
                    rpm_prev = rpm;
                }
                else
                {
                    /* SUBSEQUENT MEASUREMENTS - Apply variation check */
                    
                    /* Calculate upper and lower bounds (±80%) */
                    uint32_t upper_bound = (uint32_t)(18U * rpm_prev / 10U);  /* +80% */
                    uint32_t lower_bound = (uint32_t)( 8U * rpm_prev / 10U);  /* -20% */
                    
                    if ((rpm > upper_bound) || (rpm < lower_bound))
                    {
                        /* Variation too large - probably spurious measurement */
                        /* Use last valid RPM instead of rejecting completely */
                        rpm = rpm_prev;
                    }
                    else
                    {
                        /* Variation is acceptable - update reference */
                        rpm_prev = rpm;
                    }
                }
            }
            else
            {
                /* Outside valid measurement range or not enough samples */
                /* Use last valid measurement (or 0 if no valid measurement yet) */
                rpm = rpm_prev;
            }

            /* Update global RPM value (read by main loop) */
            g_rpm_value = rpm;

            /* Update moving average filter for noise reduction */
            rpm_update_filter(g_rpm_value);
            g_edge_valid_count++;
            g_rpm_ready_flag = 1U;
        }
    }
}

#endif


/*******************************************************************************
 * FUNCTION: Timer3AIntHandler (EDGE PERIOD METHOD)
 *
 * DESCRIPTION:
 *     Timer3A interrupt handler - called every 100ms (10 Hz).
 *     In edge-period mode, this handler only performs timeout detection
 *     and periodic system tasks. RPM calculation is done in WTimer1A ISR.
 *
 * PARAMETERS:
 *     None
 *
 * RETURNS:
 *     void
 *
 * OPERATION:
 *     1. Clear timer interrupt flag
 *     2. Increment timeout counter
 *     3. Check for motor stopped condition (no edges detected)
 *     4. Execute periodic PWM control
 *     5. Toggle LED for visual feedback
 *
 * NOTES:
 *     - Handles timeout and periodic housekeeping only
 *     - RPM calculation moved to WTimer1A ISR
 *******************************************************************************/
void Timer3AIntHandler(void)
{
    /* 1. Clear the timer interrupt flag FIRST */
    TIMER3_ICR_R |= TIMER_ICR_TATOCINT;
    
    /* 2. Increment timeout counter (100ms per interrupt) */
    g_edge_timeout_counter += RPM_CALC_PERIOD_MS;
    
    /* 3. Check for motor stopped condition */
    if (g_edge_timeout_counter >= RPM_STOP_TIMEOUT_MS)
    {
        /* No edges detected for timeout period - motor stopped */
        g_rpm_value = 0;
        g_edge_interval_us = 0;
        g_rpm_ready_flag = 1U;  /* Signal RPM update (to 0) */
    }
    
    /* 4. Execute periodic PWM control */
    increment();
    
    /* 5. Toggle LED for visual feedback */
    g_led_toggle_flag = 1U;
    
    /* 6. Optional debug pin toggling */
    /* DEBUG_STATE_TOGGLE(); */
}

/*******************************************************************************
 * COMMON ISR HANDLERS (Independent of RPM Method)
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION: WTimer1BIntHandler
 *
 * DESCRIPTION:
 *     Wide Timer 1B interrupt handler.
 *     Clears the capture event interrupt flag. Used for pulse timing/capture.
 *
 * PARAMETERS:
 *     None
 *
 * RETURNS:
 *     void
 *
 * NOTES:
 *     - Ensure WTimer1B is properly configured for capture mode
 *     - Consider adding pulse measurement logic here
 *******************************************************************************/
void WTimer1BIntHandler(void)
{
    /* Clear the timer capture interrupt */
    TimerIntClear(WTIMER1_BASE, TIMER_CAPB_EVENT);
}

/*******************************************************************************
 * FUNCTION: WTimer5AIntHandler
 *
 * DESCRIPTION:
 *     Wide Timer 5A interrupt handler.
 *     Clears the capture event interrupt flag.
 *
 * PARAMETERS:
 *     None
 *
 * RETURNS:
 *     void
 *
 * NOTES:
 *     - Basic implementation
 *     - Can be extended for system timing/capture functions
 *******************************************************************************/
void WTimer5AIntHandler(void)
{
    /* Clear the timer capture interrupt */
    TimerIntClear(WTIMER5_BASE, TIMER_CAPA_EVENT);
}

/*******************************************************************************
 * FUNCTION: WTimer5BIntHandler
 *
 * DESCRIPTION:
 *     Wide Timer 5B interrupt handler.
 *     Clears the capture event interrupt flag.
 *
 * PARAMETERS:
 *     None
 *
 * RETURNS:
 *     void
 *
 * NOTES:
 *     - Basic implementation
 *     - Can be extended for system timing/capture functions
 *******************************************************************************/
void WTimer5BIntHandler(void)
{
    /* Clear the timer capture interrupt */
    TimerIntClear(WTIMER5_BASE, TIMER_CAPB_EVENT);
}

/*******************************************************************************
 * FUNCTION: ADC0SS1IntHandler
 *
 * DESCRIPTION:
 *     ADC0 Sample Sequencer 1 interrupt handler.
 *     Reads 3-channel ADC data from FIFO into buffer.
 *     Channels: PD1 (AIN6 - Ax), PD0 (AIN7 - Strain Gauge), PE1 (AIN2 - Vmotor)
 *
 * PARAMETERS:
 *     None
 *
 * RETURNS:
 *     void
 *
 * CHANNEL MAPPING:
 *     adc0_buffer[0]: PD1 (AIN6) - Ax acceleration
 *     adc0_buffer[1]: PD0 (AIN7) - Strain Gauge
 *     adc0_buffer[2]: PE1 (AIN2) - Motor Voltage
 *
 * OPERATION:
 *     1. Read 3 ADC samples from FIFO
 *     2. Simultaneously read from ADC1 for synchronization
 *     3. Clear ADC interrupt flag
 *     4. Set scan completion flag for main loop
 *
 * NOTES:
 *     - Hardware averaging may be enabled in ADC configuration
 *     - Data is raw ADC values (0-4095 for 12-bit)
 *     - Channel order matches ADC sequence configuration
 *******************************************************************************/
void ADC0SS1IntHandler(void)
{
    uint8_t k = 0U;

    /* 1. Read 3-channel data from both ADC FIFOs */
    for (k = 0U; k < 3U; k++)
    {
        /* ADC0 channels */
        adc0_buffer[k] = ADC0_SSFIFO1_R;  /* Channel order: Ax, Strain, Vmotor */

        /* ADC1 channels (read simultaneously for time alignment) */
        adc1_buffer[k] = ADC1_SSFIFO1_R;  /* Channel order: Ay, Az, Imotor */
    }

    /* 2. Acknowledge Sample Sequencer 1 interrupt */
    ADC0_ISC_R = ADC_ISC_IN1;

    /* 3. Set flag to indicate ADC scan is complete */
    g_timer_a0_scan_flag = 1U;

    /* 4. Optional debug pin toggle for timing measurement */
    /* DEBUG_STATE_TOGGLE(); */  /* PD7 - State indicator */

    /* 5. Optional LED indicator */
    /* GPIO_PORTF_DATA_R ^= GPIO_PIN_2; */  /* Blue LED on PF2 */
}

/*******************************************************************************
 * FUNCTION: ADC1SS1IntHandler
 *
 * DESCRIPTION:
 *     ADC1 Sample Sequencer 1 interrupt handler.
 *     Reads 3-channel ADC data from FIFO into buffer.
 *     Channels: PD2 (AIN5 - Ay), PD3 (AIN4 - Az), PE2 (AIN1 - Imotor)
 *
 * PARAMETERS:
 *     None
 *
 * RETURNS:
 *     void
 *
 * CHANNEL MAPPING:
 *     adc1_buffer[0]: PD2 (AIN5) - Ay acceleration
 *     adc1_buffer[1]: PD3 (AIN4) - Az acceleration
 *     adc1_buffer[2]: PE2 (AIN1) - Motor Current
 *
 * NOTES:
 *     - This handler is called but data may also be read in ADC0 handler
 *     - Ensure proper interrupt clearing to prevent stuck interrupts
 *     - Consider merging with ADC0 handler if simultaneous sampling needed
 *******************************************************************************/
void ADC1SS1IntHandler(void)
{
    /* ADC1 SS1 interrupts are not enabled in the current acquisition design.
     * ADC0SS1IntHandler drains ADC1's FIFO after the shared timer trigger so
     * both ADC modules remain time-aligned. If this handler ever runs because
     * of a stale/pending flag, acknowledge it and leave the FIFO untouched.
     */
    ADC1_ISC_R = ADC_ISC_IN1;
}

/*******************************************************************************
 * ISR SUMMARY AND NOTES
 ******************************************************************************/

/**
 * INTERRUPT PRIORITY (typical NVIC configuration):
 * 
 * Priority 0 (Highest):
 *   - SysTick (1ms system tick)
 * 
 * Priority 1 (High):
 *   - ADC0 SS1 (time-critical data acquisition)
 *   - ADC1 SS1 (not enabled; ADC1 FIFO is drained by ADC0 SS1)
 * 
 * Priority 2 (Medium):
 *   - WTimer1A (RPM edge capture - PERIOD METHOD ONLY)
 *   - Timer3A (RPM timeout detection)
 * 
 * Priority 3 (Medium):
 *   - UART0 (command processing)
 * 
 * Priority 4+ (Low):
 *   - Other timers (system timing)
 * 
 * 
 * RPM ISR BEHAVIOR:
 *
 * Edge-period method:
 *   - WTimer1A: Interrupt per edge, calculates RPM from pulse midpoints
 *   - Timer3A: Timeout detection only
 *   - CPU load: Variable with blade-pulse frequency
 *   - Latency: Per accepted blade pulse
 * 
 * 
 * SAFETY CONSIDERATIONS:
 *   - Keep ISRs short and efficient
 *   - Clear interrupt flags early in handler
 *   - Use volatile for shared variables
 *   - Consider interrupt nesting and priority
 *   - Test with worst-case interrupt rates
 * 
 * 
 * DEBUG FEATURES:
 *   - Debug pins PD4-PD7 can be toggled in ISRs
 *   - LED indicators provide visual system status
 *   - Consider adding ISR execution time measurement
 *   - Use oscilloscope for timing verification
 */
