/******************************************************************************
* FILENAME:    pso_uart.c
*
* DESCRIPTION:
*       High-level functions for UART modules.
*
* FUNCTIONS:
*    uint8_t uartBatchWrite ( uint8_t, uint8_t*, uint8_t)
*    uint8_t i2cBatchRead ( uint8_t, uint8_t, uint8_t*, uint8_t)
*
* NOTES:
*       Functions to implement high-level procedures of the UART peripheral.
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
* 1.1     05 Dec 2025 Rogerio Lima         Fixed warnings
******************************************************************************/

#include <stdint.h>
#include <stddef.h>     /* For NULL definition */
#include "hw_uart.h"
#include "hw_types.h"
#include "pso_uart.h"

uart_raw_data_t* g_uart0_data;
uint8_t* g_tx_buffer_uart;

/*******************************************************************************
* Function Name  : uartBatchWrite
* Input          : uint32_t ui32Base, uint16_t* txBuffer, uint8_t bytesToWrite
* Output         : None
* Return         : 1 if success, 0 if fail
* Description    : Writes a pre-defined number of bytes (bytesToWrite) stored
*                  in (txBuffer) into the UART addressed by (ui32Base) address.
*******************************************************************************/
uint8_t uartBatchWrite(uint32_t ui32Base, uint16_t* txBuffer, const uint8_t bytesToWrite)
{
    uint16_t i;
    uint8_t returnval = 1U;  /* Assume success */

    /* Validate parameters */
    if (txBuffer == NULL || bytesToWrite == 0U)
    {
        return 0U;  /* Error: invalid parameters */
    }

    for (i = 0U; i < bytesToWrite; i++)
    {
        /* Wait until space is available */
        while(HWREG(ui32Base + UART_O_FR) & UART_FR_TXFF)
        {
            /* Busy wait - could add timeout here */
        }

        /* Send the char */
        HWREG(ui32Base + UART_O_DR) = txBuffer[i];
    }

    return returnval;
}


/*******************************************************************************
* Function Name  : i2cBatchRead
* Input          : uint32_t ui32Base, uint8_t* rxBuffer, uint8_t bytesToRead
* Output         : uint8_t* rxBuffer
* Return         : 1 if success, 0 if fail
* Description    : Reads a pre-defined number of bytes (bytesToRead) stored
*                  in (rxBuffer).
*
* NOTE:          : This function is a placeholder and needs implementation.
*******************************************************************************/
uint8_t i2cBatchRead(uint32_t ui32Base, uint8_t* rxBuffer, uint8_t bytesToRead)
{
    /* TODO: Implement I2C batch read functionality */
    /* This is a placeholder implementation */

    (void)ui32Base;      /* Unused parameter */
    (void)rxBuffer;      /* Unused parameter */
    (void)bytesToRead;   /* Unused parameter */

    return 0U;  /* Not implemented - return failure */
}
