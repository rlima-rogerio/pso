/******************************************************************************
 * FILENAME:    main_config.h
 *
 * DESCRIPTION:
 *       Central configuration file for PSO Data Acquisition System.
 *       All system-wide settings and feature flags are defined here.
 *
 * USAGE:
 *       Modify this file to enable/disable features and adjust parameters.
 *       Changes require recompilation.
 *
 * AUTHOR:      Rogerio Lima
 * VERSION:     4.0 - Dec 2025
 *
 ******************************************************************************/

#ifndef MAIN_CONFIG_H_
#define MAIN_CONFIG_H_

#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * FEATURE CONFIGURATION
 *
 * Enable or disable major system features by commenting/uncommenting below
 ******************************************************************************/

/**
 * @brief Enable SD card recording
 *
 * When defined:
 *   - System will initialize SD card via SPI0
 *   - Data buffered and written to files
 *   - FSM includes SD-specific states
 *   - Requires FAT_FS09B or FAT_FS11A to be enabled
 *
 * When undefined:
 *   - SD card code completely removed
 *   - Continuous UART streaming only
 *   - Lower memory footprint
 *   - No buffer management overhead
 */
#define USE_SD_CARD

/**
 * @brief Enable debug LED indicators
 *
 * When defined:
 *   - LEDs show system state visually
 *   - Useful for debugging without UART
 *
 * When undefined:
 *   - LED code removed (saves CPU cycles)
 */
#define USE_DEBUG_LEDS

/**
 * @brief Enable statistics tracking
 *
 * When defined:
 *   - Counts samples, packets, errors
 *   - Prints statistics at end of session
 *
 * When undefined:
 *   - No stat tracking (saves RAM)
 */
#define USE_STATISTICS

/**
 * @brief Enable verbose logging
 *
 * When defined:
 *   - Detailed messages via UART
 *   - System status updates
 *
 * When undefined:
 *   - Minimal output (saves bandwidth)
 */
#define USE_VERBOSE_LOGGING

/**
 * @brief Enable PWM trapezoid profile
 *
 * When defined:
 *   - Automatic throttle sweep 0-100%
 *   - Used for motor characterization
 *
 * When undefined:
 *   - Manual PWM control only
 */
#define USE_PWM_TRAPEZOID

/*******************************************************************************
 * SYSTEM CLOCK CONFIGURATION
 ******************************************************************************/
#define SYSTEM_CLOCK_HZ         40000000U   /* 40 MHz system clock */
#define SYSTEM_CLOCK_MHZ        40U

/*******************************************************************************
 * TIMING CONFIGURATION
 ******************************************************************************/
#define ADC_SAMPLE_RATE_HZ      5000U       /* ADC sampling frequency */
#define PWM_FREQUENCY_HZ        50U         /* ESC PWM frequency */
#define RPM_UPDATE_RATE_HZ      10U         /* RPM measurement rate */
#define UART_BAUD_RATE          115200U     /* UART0 baud rate */

/*******************************************************************************
 * BUFFER CONFIGURATION
 ******************************************************************************/

#ifdef USE_SD_CARD
    /**
     * @brief SD card buffer size
     *
     * Larger buffer:
     *   - Fewer SD writes (better performance)
     *   - More RAM usage
     *   - Longer delay between writes
     *
     * Recommended: 4KB - 32KB
     */
    #define BUFFER_SIZE_BYTES       16384U  /* 16 KB */

    /**
     * @brief Maximum filename length
     */
    #define FILENAME_MAX_LENGTH     32U

    /**
     * @brief File naming pattern
     *
     * Files created as: pso_000.dat, pso_001.dat, etc.
     */
    #define FILE_PREFIX             "pso_"
    #define FILE_EXTENSION          ".dat"
#endif

/*******************************************************************************
 * USER INTERFACE CONFIGURATION
 ******************************************************************************/

/**
 * @brief Button debounce delay (milliseconds)
 */
#define DEBOUNCE_DELAY_MS       10U

/**
 * @brief LED blink patterns
 */
#define LED_BLINK_SUCCESS       10U         /* Success indication */
#define LED_BLINK_ERROR         5U          /* Error indication */
#define LED_BLINK_PERIOD_MS     100U        /* Blink period */

/*******************************************************************************
 * HARDWARE PIN DEFINITIONS
 ******************************************************************************/

/* Buttons */
#define SW1_PIN                 GPIO_PIN_4  /* Start button (PF4) */
#define SW2_PIN                 GPIO_PIN_0  /* Stop button (PF0) */

/* LEDs */
#define LED_RED_PIN             GPIO_PIN_1  /* Red LED (PF1) */
#define LED_BLUE_PIN            GPIO_PIN_2  /* Blue LED (PF2) */
#define LED_GREEN_PIN           GPIO_PIN_3  /* Green LED (PF3) */

/* ADC Channels */
#define ADC0_ACCEL_X            6U          /* AIN6 - PD1 */
#define ADC0_THRUST             7U          /* AIN7 - PD0 */
#define ADC0_V_MOTOR            2U          /* AIN2 - PE1 */

#define ADC1_ACCEL_Y            5U          /* AIN5 - PD2 */
#define ADC1_ACCEL_Z            4U          /* AIN4 - PD3 */
#define ADC1_I_MOTOR            1U          /* AIN1 - PE2 */

/*******************************************************************************
 * PWM CONFIGURATION
 ******************************************************************************/

#ifdef USE_PWM_TRAPEZOID
    /**
     * @brief Trapezoid profile timing (in 100ms units)
     */
    #define TRAP_RAMP_START_TIME    3U      /* Initial delay: 300ms */
    #define TRAP_STEP_HOLD_TIME     2U      /* Hold each step: 200ms */
    #define TRAP_RAMP_DOWN_TIME     3U      /* Ramp down: 300ms */

    /**
     * @brief Throttle steps
     */
    #define TRAP_THROTTLE_STEP      10U     /* 10% increments */
    #define TRAP_THROTTLE_MIN       0U      /* 0% minimum */
    #define TRAP_THROTTLE_MAX       100U    /* 100% maximum */
#endif

/*******************************************************************************
 * ERROR CODES
 ******************************************************************************/
typedef enum {
    ERR_NONE = 0x00,

    /* Buffer Errors */
    ERR_BUFFER_OVERFLOW = 0x01,
    ERR_BUFFER_UNDERFLOW = 0x02,

    /* Hardware Errors */
    ERR_ADC_FAULT = 0x10,
    ERR_UART_FAULT = 0x11,
    ERR_PWM_FAULT = 0x12,
    ERR_TIMER_FAULT = 0x13,

    /* SD Card Errors */
    ERR_SD_MOUNT = 0x20,
    ERR_SD_OPEN = 0x21,
    ERR_SD_WRITE = 0x22,
    ERR_SD_READ = 0x23,
    ERR_SD_CLOSE = 0x24,
    ERR_SD_SYNC = 0x25,

    /* System Errors */
    ERR_INVALID_STATE = 0xFE,
    ERR_UNKNOWN = 0xFF
} error_code_t;

/*******************************************************************************
 * STATUS CODES
 ******************************************************************************/
typedef enum {
    STATUS_OK = 0U,
    STATUS_IN_PROGRESS,
    STATUS_ERROR,
    STATUS_TIMEOUT,
    STATUS_BUFFER_FULL,
    STATUS_BUFFER_EMPTY
} status_t;

/*******************************************************************************
 * VALIDATION MACROS
 ******************************************************************************/

/* Ensure required dependencies are met */
#ifdef USE_SD_CARD
    #if !defined(FAT_FS09B) && !defined(FAT_FS11A)
        #error "USE_SD_CARD requires FAT_FS09B or FAT_FS11A"
    #endif

    #if BUFFER_SIZE_BYTES < 1024
        #error "BUFFER_SIZE_BYTES must be at least 1024"
    #endif

    #if BUFFER_SIZE_BYTES > 65536
        #error "BUFFER_SIZE_BYTES exceeds maximum (64KB)"
    #endif
#endif

/* Ensure PACKET_LENGTH is defined */
#ifndef PACKET_LENGTH
    #error "PACKET_LENGTH must be defined in pso_data.h"
#endif

/*******************************************************************************
 * UTILITY MACROS
 ******************************************************************************/

/**
 * @brief Convert milliseconds to clock cycles
 */
#define MS_TO_CYCLES(ms)    ((SYSTEM_CLOCK_HZ / 1000U) * (ms))

/**
 * @brief Convert microseconds to clock cycles
 */
#define US_TO_CYCLES(us)    ((SYSTEM_CLOCK_HZ / 1000000U) * (us))

/**
 * @brief Create delay from milliseconds
 */
#define DELAY_MS(ms)        SysCtlDelay(MS_TO_CYCLES(ms) / 3U)

/**
 * @brief Check if value is in range
 */
#define IN_RANGE(val, min, max)     (((val) >= (min)) && ((val) <= (max)))

/**
 * @brief Clamp value to range
 */
#define CLAMP(val, min, max)        \
    (((val) < (min)) ? (min) : (((val) > (max)) ? (max) : (val)))

/*******************************************************************************
 * DEBUG MACROS
 ******************************************************************************/

#ifdef USE_VERBOSE_LOGGING
    #define DEBUG_PRINT(fmt, ...)   printf(fmt, ##__VA_ARGS__)
#else
    #define DEBUG_PRINT(fmt, ...)   do {} while(0)
#endif

#ifdef USE_DEBUG_LEDS
    #define DEBUG_LED_ON(led)       PSO_LED##led##On()
    #define DEBUG_LED_OFF(led)      PSO_LED##led##Off()
    #define DEBUG_LED_TOGGLE(led)   led_##led##_toggle()
#else
    #define DEBUG_LED_ON(led)       do {} while(0)
    #define DEBUG_LED_OFF(led)      do {} while(0)
    #define DEBUG_LED_TOGGLE(led)   do {} while(0)
#endif

/*******************************************************************************
 * STATISTICS STRUCTURE
 ******************************************************************************/

#ifdef USE_STATISTICS
typedef struct {
    uint32_t total_samples;         /* Total ADC samples acquired */
    uint32_t packets_sent;          /* UART packets transmitted */
    uint32_t buffer_writes;         /* SD buffer writes */
    uint32_t sd_errors;             /* SD card errors */
    uint32_t adc_overruns;          /* ADC overrun count */
    uint32_t uptime_seconds;        /* System uptime */
    uint16_t max_scan_period;       /* Maximum scan period (cycles) */
    uint16_t min_scan_period;       /* Minimum scan period (cycles) */
} system_stats_t;
#endif

/*******************************************************************************
 * VERSION INFORMATION
 ******************************************************************************/
#define FIRMWARE_VERSION_MAJOR  4
#define FIRMWARE_VERSION_MINOR  0
#define FIRMWARE_VERSION_PATCH  0

#define FIRMWARE_VERSION_STRING "4.0.0"
#define FIRMWARE_DATE           "2025-12-05"
#define FIRMWARE_AUTHOR         "Rogerio Lima"

/*******************************************************************************
 * COMPILE-TIME CONFIGURATION SUMMARY
 ******************************************************************************/

#define CONFIG_SUMMARY() \
    DEBUG_PRINT("\n\rConfiguration Summary:\n\r"); \
    DEBUG_PRINT("  Version:     %s\n\r", FIRMWARE_VERSION_STRING); \
    DEBUG_PRINT("  Date:        %s\n\r", FIRMWARE_DATE); \
    DEBUG_PRINT("  Clock:       %lu MHz\n\r", SYSTEM_CLOCK_MHZ); \
    DEBUG_PRINT("  ADC Rate:    %u Hz\n\r", ADC_SAMPLE_RATE_HZ); \
    DEBUG_PRINT("  Packet Size: %u bytes\n\r", PACKET_LENGTH); \
    DEBUG_PRINT("  Features:\n\r"); \
    DEBUG_PRINT("    SD Card:   %s\n\r", FEATURE_STATUS(USE_SD_CARD)); \
    DEBUG_PRINT("    Statistics:%s\n\r", FEATURE_STATUS(USE_STATISTICS)); \
    DEBUG_PRINT("    Debug LEDs:%s\n\r", FEATURE_STATUS(USE_DEBUG_LEDS)); \
    DEBUG_PRINT("    Verbose:   %s\n\r", FEATURE_STATUS(USE_VERBOSE_LOGGING));

#define FEATURE_STATUS(feature) \
    (defined(feature) ? "Enabled" : "Disabled")

#endif /* MAIN_CONFIG_H_ */
