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

/******************************************************************************
*
*  The following are prototypes for the UART functions.
*
******************************************************************************/
uint8_t copy_raw_data (uint8_t* txBuffer, uart_raw_data_t* g_uart0_data);
uint8_t read_raw_data (uart_raw_data_t* g_uart0_data);


//typedef enum {
//    SYS_STATE_WAIT_SW1 = 0U,           /* Waits for SW1 be pressed */
//    SYS_STATE_SD_INIT = 1U,            /* Starts SD card and register FatFs work area */
//    SYS_STATE_RECORDING = 2U,          /* Starts to fill fifos and write data to the memory */
//    SYS_STATE_CLOSE_FILE = 3U,         /* Close file and unmount SD card */
//    SYS_STATE_FINISH = 4U,             /* Finish recording indication */
//    SYS_STATE_SD_ERROR = 5U            /* SD error state */
//} sys_state_t;
//
//#endif /* PSO_DATA_H_ */
