/*
 * myinit.c
 *
 *  Created on: 31/Aug/2015
 *      Author: Rogerio
 */

#include <stdint.h>
#include "pso_uart.h"
#include "pso_data.h"


/*******************************************************************************
* Function Name  : copy_raw_data
* Input          : uint32_t
* Output         : uint8_t*
* Return         : 1 if success, 0 if fail
* Description    : Writes a pre-defined number of bytes (bytesToWrite) stored
*                  in (txBuffer) into the UART addressed by (ui32Base) address.
*                  Copy data to txBuffer just prior send it over UART.
*******************************************************************************/
uint8_t copy_raw_data (uint8_t* txBuffer, uart_raw_data_t* g_uart0_data)
{
    uint8_t returnval;

    txBuffer[0] = g_uart0_data->rx_buffer[0];
    txBuffer[1] = g_uart0_data->rx_buffer[1];
    txBuffer[2] = g_uart0_data->rx_buffer[2];
    txBuffer[3] = g_uart0_data->rx_buffer[3];
    txBuffer[4] = g_uart0_data->rx_buffer[4];
    txBuffer[5] = g_uart0_data->rx_buffer[5];
    txBuffer[6] = g_uart0_data->rx_buffer[6];
    txBuffer[7] = g_uart0_data->rx_buffer[7];
    txBuffer[0] = g_uart0_data->rx_index++;


    return returnval;
}


/*******************************************************************************
* Function Name  : read_raw_data
* Input          : None
* Output         : uint8_t*
* Return         : 1 if success, 0 if fail
* Description    : Writes a pre-defined number of bytes (bytesToWrite) stored
*                  in (txBuffer) into the UART addressed by (ui32Base) address.
*                  Copy data to txBuffer just prior send it over UART.
*******************************************************************************/
uint8_t read_raw_data (uart_raw_data_t* g_uart0_data)
{
    uint8_t returnval;

    g_uart0_data->rx_buffer[0] = 'P';
    g_uart0_data->rx_buffer[1] = 'S';
    g_uart0_data->rx_buffer[2] = 'O';
    g_uart0_data->rx_buffer[3] = '-';
    g_uart0_data->rx_buffer[4] = 'v';
    g_uart0_data->rx_buffer[5] = '1';
    g_uart0_data->rx_buffer[6] = '\r';
    g_uart0_data->rx_buffer[7] = '\n';
    g_uart0_data->rx_index++;


    return returnval;
}

