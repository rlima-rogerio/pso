/*******************************************************************************
 * FILE:        pso_data.c
 *
 * DESCRIPTION:
 *     PSO Data Processing Module.
 *     Contains functions for handling raw data transfer and packet preparation
 *     for the UART communication system.
 *
 * DOCUMENTATION STYLE:
 *     - Technical and functional
 *     - No functional or logical modifications
 *     - Improved comments and formatting only
 *
 * AUTHOR:      Rogerio Lima
 * DATE:        31/Aug/2015
 * REFORMAT:    2025 (Documentation and formatting only)
 *
 *******************************************************************************/

#include <stdint.h>
#include "pso_uart.h"
#include "pso_data.h"

/*******************************************************************************
 * FUNCTION: copy_raw_data
 * 
 * DESCRIPTION:
 *     Copies raw data from the UART receive buffer to a transmit buffer.
 *     This function is typically called just before sending data over UART.
 *     The function copies a fixed number of bytes from the source buffer
 *     to the destination buffer.
 * 
 * PARAMETERS:
 *     txBuffer      - Pointer to destination transmit buffer
 *     g_uart0_data  - Pointer to UART data structure containing source data
 * 
 * RETURNS:
 *     uint8_t - Status indicator (currently always returns 0)
 *               1 = success, 0 = fail (implementation pending)
 * 
 * NOTES:
 *     - Currently copies 8 bytes of fixed test data
 *     - The function always returns 0 (success/fail logic not implemented)
 *     - txBuffer[0] is overwritten twice in current implementation
 *******************************************************************************/
uint8_t copy_raw_data(uint8_t* txBuffer, uart_raw_data_t* g_uart0_data)
{
    uint8_t returnval = 0U;  // Return value (currently always 0)

    /* Copy data from UART receive buffer to transmit buffer */
    txBuffer[0] = g_uart0_data->rx_buffer[0];  // Byte 0
    txBuffer[1] = g_uart0_data->rx_buffer[1];  // Byte 1
    txBuffer[2] = g_uart0_data->rx_buffer[2];  // Byte 2
    txBuffer[3] = g_uart0_data->rx_buffer[3];  // Byte 3
    txBuffer[4] = g_uart0_data->rx_buffer[4];  // Byte 4
    txBuffer[5] = g_uart0_data->rx_buffer[5];  // Byte 5
    txBuffer[6] = g_uart0_data->rx_buffer[6];  // Byte 6
    txBuffer[7] = g_uart0_data->rx_buffer[7];  // Byte 7
    txBuffer[8] = g_uart0_data->rx_index++;    // Store and increment index

    return returnval;  // Currently always returns 0
}

/*******************************************************************************
 * FUNCTION: read_raw_data
 * 
 * DESCRIPTION:
 *     Populates the UART receive buffer with predefined test data.
 *     This function is used for testing and demonstration purposes,
 *     loading a fixed ASCII message into the buffer.
 * 
 * PARAMETERS:
 *     g_uart0_data - Pointer to UART data structure to populate
 * 
 * RETURNS:
 *     uint8_t - Status indicator (currently always returns 0)
 *               1 = success, 0 = fail (implementation pending)
 * 
 * NOTES:
 *     - Loads the ASCII string "PSO-v1" followed by CR/LF
 *     - The function always returns 0 (success/fail logic not implemented)
 *     - Increments the receive index after loading data
 *******************************************************************************/
uint8_t read_raw_data(uart_raw_data_t* g_uart0_data)
{
    uint8_t returnval = 0U;  // Return value (currently always 0)

    /* Load predefined test data into UART receive buffer */
    g_uart0_data->rx_buffer[0] = 'P';  // ASCII 'P'
    g_uart0_data->rx_buffer[1] = 'S';  // ASCII 'S'
    g_uart0_data->rx_buffer[2] = 'O';  // ASCII 'O'
    g_uart0_data->rx_buffer[3] = '-';  // ASCII '-'
    g_uart0_data->rx_buffer[4] = 'v';  // ASCII 'v'
    g_uart0_data->rx_buffer[5] = '1';  // ASCII '1'
    g_uart0_data->rx_buffer[6] = '\r'; // Carriage Return
    g_uart0_data->rx_buffer[7] = '\n'; // Line Feed
    
    /* Increment receive index (tracking variable) */
    g_uart0_data->rx_index++;

    return returnval;  // Currently always returns 0
}
