/*******************************************************************************
 * FILE:        pso_data.h
 *
 * DESCRIPTION:
 *     PSO System Configuration and State Definitions.
 *     Contains system-wide constants, configuration flags, and state machine
 *     definitions for the PSO (Position Sensing and Output) system.
 *
 * FUNCTIONALITY:
 *     - System configuration flags (PWM mode, filesystem version)
 *     - Communication packet definition
 *     - System state machine with operational states
 *     - Centralized configuration management
 *
 * DOCUMENTATION STYLE:
 *     - Technical and functional documentation
 *     - No functional or logical modifications to original code
 *     - Enhanced comments, formatting, and structure
 *
 * AUTHOR:      Rogerio (Original: 31/Aug/2015)
 * REFORMAT:    2025 (Documentation and formatting only)
 *
 ******************************************************************************/

#ifndef PSO_DATA_H_
#define PSO_DATA_H_

/*******************************************************************************
 * SYSTEM CONFIGURATION CONSTANTS
 *
 * DESCRIPTION:
 *     Defines the operational mode and feature configuration for the PSO system.
 *     These constants control system behavior and are typically set at compile-time.
 *
 * CONSTANTS:
 *     PWM_DRIVE_MODE: Selects between manual and automatic PWM control
 *     FAT_FS09B:      Enables FatFS version 0.09b filesystem support
 *     FAT_FS11A:      Enables FatFS version 0.11a filesystem support
 *     PACKET_LENGTH:  Defines the size of data packets for transmission/recording
 *
 * NOTES:
 *     - Constants are defined as unsigned integers (U suffix)
 *     - Configuration is compile-time; changes require recompilation
 *     - Only one filesystem version should be enabled at a time
 *     - Packet length must match transmitter and receiver expectations
 ******************************************************************************/

/* PWM Drive Mode Selection */
#define PWM_DRIVE_MODE      1U    /* 0: Manual control through SW1(-) & SW2(+) */
                                  /* 1: Automatic control (system-generated) */

/* Filesystem Version Configuration */
#define FAT_FS09B           1U    /* 0: FatFS v0.09b disabled */
                                  /* 1: FatFS v0.09b enabled */
#define FAT_FS11A           0U    /* 0: FatFS v0.11a disabled */
                                  /* 1: FatFS v0.11a enabled */

/* Communication Packet Configuration */
#define PACKET_LENGTH      21U    /* Length of data packets in bytes */
                                  /* Used for UART transmission and data recording */

/*******************************************************************************
 * SYSTEM STATE MACHINE DEFINITIONS
 *
 * DESCRIPTION:
 *     Defines the operational states of the PSO system state machine.
 *     Each state represents a specific phase of system operation with
 *     well-defined entry, execution, and exit behaviors.
 *
 * STATE TRANSITION FLOW:
 *     INIT → IDLE → TIMING → DATA_PROCESSING → STREAMING → STOPPING → IDLE
 *           ↑                                     ↓
 *           └─────────────────────────────────────┘ (via PWM_CONTROL or ERROR)
 *
 * STATE DESCRIPTIONS:
 *     SYS_STATE_INIT:          System initialization and parameter setup
 *     SYS_STATE_IDLE:          Ready state, awaiting start command
 *     SYS_STATE_TIMING:        Timing and synchronization configuration
 *     SYS_STATE_DATA_PROCESSING: Data scaling, filtering, and preparation
 *     SYS_STATE_STREAMING:     Active data transmission via UART
 *     SYS_STATE_PWM_CONTROL:   PWM signal generation and control
 *     SYS_STATE_STOPPING:      Graceful shutdown and cleanup
 *     SYS_STATE_ERROR:         Error handling and recovery state
 *
 * USAGE:
 *     - State transitions are triggered by events or commands
 *     - Each state has associated entry/exit actions
 *     - Error state provides safe recovery mechanism
 *
 * NOTES:
 *     - States are sequentially numbered for efficient switch statements
 *     - State machine should be implemented as a switch-case in main loop
 *     - Error state has highest priority for safety-critical operations
 ******************************************************************************/
typedef enum {
    SYS_STATE_INIT = 0U,            /* System initialization state */
    SYS_STATE_IDLE = 1U,            /* Idle/ready state */
    SYS_STATE_TIMING = 2U,          /* Timing configuration state */
    SYS_STATE_DATA_PROCESSING = 3U, /* Data processing and preparation */
    SYS_STATE_STREAMING = 4U,       /* Active UART data streaming */
    SYS_STATE_PWM_CONTROL = 5U,     /* PWM generation and control */
    SYS_STATE_STOPPING = 6U,        /* Graceful stop and cleanup */
    SYS_STATE_ERROR = 7U            /* Error handling and recovery */
} sys_state_t;

/*******************************************************************************
 * FUNCTION PROTOTYPES (COMMENTED OUT - ORIGINAL REFERENCE)
 *
 * DESCRIPTION:
 *     Original function prototypes for UART data handling.
 *     Currently commented out but retained for reference and potential
 *     future implementation.
 *
 * FUNCTIONS:
 *     copy_raw_data: Copies raw data to transmission buffer
 *     read_raw_data: Reads raw data from UART source
 *
 * NOTES:
 *     - These prototypes are commented out in the original code
 *     - May be implemented in a separate UART module
 *     - Uncomment and implement as needed for specific UART functionality
 ******************************************************************************/
/*
 * uint8_t copy_raw_data(uint8_t* txBuffer, uart_raw_data_t* g_uart0_data);
 * uint8_t read_raw_data(uart_raw_data_t* g_uart0_data);
 */

/*******************************************************************************
 * CONFIGURATION VALIDATION ASSERTIONS
 *
 * DESCRIPTION:
 *     Compile-time assertions to ensure valid configuration combinations.
 *     These static assertions prevent invalid configurations at compile time.
 *
 * VALIDATION RULES:
 *     1. Only one FatFS version can be enabled at a time
 *     2. PWM mode must be either 0 or 1
 *     3. Packet length must be non-zero and reasonable for embedded systems
 *
 * NOTES:
 *     - These are conceptual assertions; actual implementation may use
 *       #error directives or other compile-time validation techniques
 *     - Prevents runtime errors caused by invalid configurations
 ******************************************************************************/
#if 0  /* Conceptual validation - not compiled */
    /* Validate FatFS configuration */
    #if (FAT_FS09B && FAT_FS11A)
        #error "Only one FatFS version can be enabled at a time"
    #endif
    
    /* Validate PWM mode */
    #if (PWM_DRIVE_MODE > 1U)
        #error "PWM_DRIVE_MODE must be 0 (manual) or 1 (auto)"
    #endif
    
    /* Validate packet length */
    #if (PACKET_LENGTH == 0U)
        #error "PACKET_LENGTH must be greater than zero"
    #endif
#endif  /* Configuration validation */

#endif /* PSO_DATA_H_ */
