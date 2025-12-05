/******************************************************************************
 * FILENAME:    pso_data.h
 *
 * DESCRIPTION:
 *       Data structure definitions and configuration for PSO system
 *
 * AUTHOR:      Rogerio
 * VERSION:     1.2 - Dec 2025 - Removed sys_state_t (moved to main.c)
 *
 *****************************************************************************/
#ifndef PSO_DATA_H_
#define PSO_DATA_H_

#include <stdint.h>

/******************************************************************************
 * SYSTEM CONFIGURATION
 *****************************************************************************/
#define PWM_DRIVE_MODE      1U    /* 0: Manual through SW1(-) & SW2(+) */
                                  /* 1: Auto */
#define FAT_FS09B           1U    /* 0: without FatFS, 1: with FatFS v0.09b */
#define FAT_FS11A           0U    /* 0: without FatFS, 1: with FatFS v0.11b */
#define PACKET_LENGTH      21U    /* Length of the data xmitted/recorded    */

/******************************************************************************
 * FORWARD DECLARATIONS
 *
 * These allow us to reference types from other headers without including them.
 * This prevents circular dependencies.
 *****************************************************************************/
struct uart_raw_data_t;  /* Defined in pso_uart.h - use struct tag only */

/******************************************************************************
 * FUNCTION PROTOTYPES
 *
 * Note: Using "struct uart_raw_data_t*" instead of "uart_raw_data_t*"
 * to avoid typedef conflicts
 *****************************************************************************/
uint8_t copy_raw_data(uint8_t* txBuffer, struct uart_raw_data_t* g_uart0_data);
uint8_t read_raw_data(struct uart_raw_data_t* g_uart0_data);

#endif /* PSO_DATA_H_ */
