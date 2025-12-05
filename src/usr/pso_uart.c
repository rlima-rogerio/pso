/******************************************************************************
* FILENAME:    pso_uart.c
*
* DESCRIPTION:
*       High-level functions for UART modules.
*
* FUNCTIONS:
*    uint8_t uartBatchWrite ( uint8_t, uint8_t*, uint8_t)
*    uint8_t uartBatchRead ( uint8_t, uint8_t, uint8_t*, uint8_t)
*
* NOTES:
*       Functions to implement high-level procedures of the I2C peripheral.
*
* REFERENCES:
*       Adapted from CH Robotics.
*
* START DATE:    04 Apr 2015
*
* CHANGES :
*
* VERSION DATE        WHO                    DETAIL
* 1.0     04 Apr 2015 Rogerio Lima         Start-up coding.
******************************************************************************/

/*
 * SECTION ORDER
 *     1. Comment block
 *     2. Include statements
 *     3. Definitions: data type, constants, macros
 *     4. Static data declarations
 *     5. Private functions prototypes
 *     6. Public function bodies
 *     7. Private function bodies
 *
 */

#include <stdint.h>
#include "hw_uart.h"
#include "hw_types.h"
#include "pso_uart.h"

uart_raw_data_t* g_uart0_data;
uint8_t* g_tx_buffer_uart;

/*******************************************************************************
* Function Name  : uartBatchWrite
* Input          : uint32_t ui32Base, uint8_t* txBuffer, uint8_t bytesToWrite
* Output         : uint8_t* status_flag
* Return         : 1 if success, 0 if fail
* Description    : Writes a pre-defined number of bytes (bytesToWrite) stored
*                  in (txBuffer) into the UART addressed by (ui32Base) address.
*******************************************************************************/
uint8_t uartBatchWrite (uint32_t ui32Base, uint16_t* txBuffer, const uint8_t bytesToWrite)
{
    static uint16_t i;
    static uint8_t returnval;


    for (i = 0U; i < bytesToWrite; i++)
    {
         // Wait until space is available.
         while(HWREG(ui32Base + UART_O_FR) & UART_FR_TXFF)
         {
         }
         // Send the char.
         HWREG(ui32Base + UART_O_DR) =  txBuffer[i];
    }

    return returnval;
}


/*******************************************************************************
* Function Name  : i2cBatchRead
* Input          : uint8_t bytesToRead,
* Output         : uint8_t* rxBuffer
* Return         : 1 if success, 0 if fail
* Description    : Reads a pre-defined number of bytes (bytesToRead) stored
*                  in (rxBuffer).
*******************************************************************************/
uint8_t i2cBatchRead (uint32_t ui32Base, uint8_t* rxBuffer, uint8_t bytesToRead)
{

}
