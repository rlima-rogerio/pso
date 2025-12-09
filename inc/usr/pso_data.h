/******************************************************************************
 * pos_uart.h
 *
 *  Created on: 31/Aug/2015
 *      Author: Rogerio
 *****************************************************************************/
#ifndef PSO_DATA_H_
#define PSO_DATA_H_

#define PWM_DRIVE_MODE      1U    /* 0: Manual through SW1(-) & SW2(+) */
                                  /* 1: Auto */
#define FAT_FS09B           1U    /* 0: without FatFS, 1: with FatFS v0.09b */
#define FAT_FS11A           0U    /* 0: without FatFS, 1: with FatFS v0.11b */
#define PACKET_LENGTH      21U    /* Length of the data xmitted/recorded    */

/*******************************************************************************
 * SYSTEM STATE MACHINE DEFINITIONS
 ******************************************************************************/
typedef enum {
    SYS_STATE_INIT = 0U,            /* Initialize streaming parameters */
    SYS_STATE_IDLE = 1U,            /* System idle, waiting for start command */
    SYS_STATE_TIMING = 2U,          /* Timing configuration state */
    SYS_STATE_DATA_PROCESSING = 3U, /* Data scaling and filtering */
    SYS_STATE_STREAMING = 4U,       /* Active data streaming via UART */
    SYS_STATE_PWM_CONTROL = 5U,     /* PWM control state (trapezoid/linear/step) */
    SYS_STATE_STOPPING = 6U,        /* Gracefully stopping streaming */
    SYS_STATE_ERROR = 7U            /* Error state */
} sys_state_t;

// /******************************************************************************
// *
// *  The following are prototypes for the UART functions.
// *
// ******************************************************************************/
// uint8_t copy_raw_data (uint8_t* txBuffer, uart_raw_data_t* g_uart0_data);
// uint8_t read_raw_data (uart_raw_data_t* g_uart0_data);

#endif /* PSO_DATA_H_ */
