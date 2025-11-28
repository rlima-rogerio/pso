/******************************************************************************
 * pos_uart.h
 *
 *  Created on: 04/04/2015
 *      Author: Rogerio
 *****************************************************************************/
#ifndef PSO_UART_H_
#define PSO_UART_H_

/******************************************************************************
*
*  The following are defines for the UART register offsets.
*
******************************************************************************/
#define UART_MAX_BUFFER                  100 // UART0 max buffer length


/******************************************************************************
*
*  The following are prototypes for the UART functions.
*
******************************************************************************/
uint8_t uartBatchWrite (uint32_t ui32Base, uint16_t* txBuffer, const uint8_t bytesToWrite);
uint8_t i2cBatchRead (uint32_t ui32Base, uint8_t* rxBuffer, uint8_t bytesToRead);


/******************************************************************************
*
*  Structure for holding data from UART port.
*
******************************************************************************/
typedef struct uart_raw_data_t
{
	uint8_t rx_buffer[UART_MAX_BUFFER];     /* Rx buffer       */
	uint8_t rx_index;                       /* Rx buffer index */
    uint8_t new_data;                       /* New data flag   */

} uart_raw_data_t;

#endif /* PSO_UART_H_ */
