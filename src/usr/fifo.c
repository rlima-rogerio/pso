/******************************************************************************
* FILENAME:    fifo.c
*
* DESCRIPTION:
*       Functions to implement a circular FIFO queue.
*
* FUNCTIONS:
*    void fifo_init()
*    bool fifo_put()
*    bool fifo_del()
*    uint8_t fifo_get()
*    bool fifo_is_empty()
*    bool fifo_is_full()
*    void fifo_print()
*    uint16_t fifo_count_elements()
*    bool fifo_transfer()
*    bool fifo_ping_pong()
*
* NOTES:
*       None.
*
* REFERENCES:
*       None.
*
* START DATE:    28 Sep 2015
*
* CHANGES :
*
* VERSION DATE        WHO                    DETAIL
* 1.0     28 Sep 2015 Rogerio Lima         Start-up coding.
* 1.1     05 Dec 2025 Rogerio Lima         Fixed warnings, implemented ping_pong
*
* -----------------------------------------------------------------------------
* 2.0
******************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>     /* For NULL definition */
#include <string.h>
#include "fifo.h"

/* Global variables */
fifo_t g_fifo_ping;
fifo_t g_fifo_pong;

/******************************************************************************
 * Function Name  : fifo_init
 * Input          : fifo_t* fifo
 * Output         : None
 * Return         : None
 * Description    : Initializes the queue assuming an empty queue has both
 *                  front and rear indices as -1.
 *****************************************************************************/
void fifo_init(fifo_t* fifo)
{
    if (fifo == NULL)
    {
        return;
    }

    fifo->rear = -1;
    fifo->front = -1;
}

/******************************************************************************
 * Function Name  : fifo_put
 * Input          : fifo_t* fifo, uint8_t data
 * Output         : None
 * Return         : true if success, false if fail
 * Description    : Inserts an element in queue at rear end
 *****************************************************************************/
bool fifo_put(fifo_t* fifo, uint8_t data)
{
    bool returnval;

    if (fifo == NULL)
    {
        return false;
    }

    /* FIFO is full and cannot insert new data */
    if (fifo_is_full(fifo))
    {
        returnval = false;
    }
    else if (fifo_is_empty(fifo))
    {
        fifo->front = 0;
        fifo->rear = 0;
        fifo->data[fifo->rear] = data;
        returnval = true;
    }
    else
    {
        fifo->rear = (fifo->rear + 1) % MAX_FIFO_SIZE;
        fifo->data[fifo->rear] = data;
        returnval = true;
    }

    return returnval;
}

/******************************************************************************
 * Function Name  : fifo_del
 * Input          : fifo_t* fifo
 * Output         : None
 * Return         : true if success, false if fail
 * Description    : Deletes an element in queue from front end
 *****************************************************************************/
bool fifo_del(fifo_t* fifo)
{
    bool returnval;

    if (fifo == NULL)
    {
        return false;
    }

    /* FIFO is empty and cannot delete data */
    if (fifo_is_empty(fifo))
    {
        returnval = false;
    }
    else if(fifo->front == fifo->rear)
    {
        fifo->front = -1;
        fifo->rear = -1;
        returnval = true;
    }
    else
    {
        fifo->front = (fifo->front + 1) % MAX_FIFO_SIZE;
        returnval = true;
    }

    return returnval;
}

/******************************************************************************
 * Function Name  : fifo_get
 * Input          : fifo_t* fifo
 * Output         : None
 * Return         : Data byte when available, 0xFF when FIFO is empty
 * Description    : Returns an element at front of queue. Reading an element
 *                  automatically deletes it.
 *****************************************************************************/
uint8_t fifo_get(fifo_t* fifo)
{
    uint8_t returnval;

    if (fifo == NULL)
    {
        return 0xFF;
    }

    /* FIFO is empty and cannot return data */
    if (fifo->front == -1)
    {
        returnval = 0xFF;  /* Changed from -1 to 0xFF for uint8_t */
    }
    else
    {
        returnval = fifo->data[fifo->front];
        fifo_del(fifo);
    }

    return returnval;
}

/******************************************************************************
 * Function Name  : fifo_is_empty
 * Input          : fifo_t* fifo
 * Output         : None
 * Return         : true if it's empty, false otherwise
 * Description    : Checks if FIFO is empty or not
 *****************************************************************************/
bool fifo_is_empty(fifo_t* fifo)
{
    if (fifo == NULL)
    {
        return true;
    }

    return ((fifo->front == -1) && (fifo->rear == -1));
}

/******************************************************************************
 * Function Name  : fifo_is_full
 * Input          : fifo_t* fifo
 * Output         : None
 * Return         : true if it's full, false otherwise
 * Description    : Checks if FIFO is full or not
 *****************************************************************************/
bool fifo_is_full(fifo_t* fifo)
{
    if (fifo == NULL)
    {
        return false;
    }

    return ((fifo->rear + 1) % MAX_FIFO_SIZE == fifo->front);
}

/******************************************************************************
 * Function Name  : fifo_print
 * Input          : fifo_t* fifo
 * Output         : None
 * Return         : None
 * Description    : Prints the elements in queue from front to rear. Should be
 *                  used only to test the code.
 *****************************************************************************/
void fifo_print(fifo_t* fifo)
{
    int16_t count;
    int16_t index, i;

    if (fifo == NULL || fifo_is_empty(fifo))
    {
        printf("FIFO is empty\n\r");
        return;
    }

    /* Calculate the number of elements in queue */
    count = (int16_t)((fifo->rear + MAX_FIFO_SIZE - fifo->front) % MAX_FIFO_SIZE + 1);

    printf("FIFO contents (%d elements):\n\r", count);
    for (i = 0; i < count; i++)
    {
        index = (fifo->front + i) % MAX_FIFO_SIZE;
        printf("%d \n\r", fifo->data[index]);
    }
}

/******************************************************************************
 * Function Name  : fifo_count_elements
 * Input          : fifo_t* fifo
 * Output         : None
 * Return         : Number of elements in queue, 0 if empty or NULL
 * Description    : Calculates the number of elements in queue.
 *****************************************************************************/
uint16_t fifo_count_elements(fifo_t* fifo)
{
    uint16_t returnval;

    if (fifo == NULL || fifo_is_empty(fifo))
    {
        return 0U;
    }

    returnval = (uint16_t)((fifo->rear + MAX_FIFO_SIZE - fifo->front) % MAX_FIFO_SIZE + 1);

    return returnval;
}

/******************************************************************************
 * Function Name  : fifo_transfer
 * Input          : fifo_t* src
 * Output         : fifo_t* dst
 * Return         : true if all transferred, false if some remain or error
 * Description    : Transfers all elements from 'src' queue to 'dst' queue.
 *****************************************************************************/
bool fifo_transfer(fifo_t* src, fifo_t* dst)
{
    bool returnval;

    /* Validate parameters */
    if (src == NULL || dst == NULL)
    {
        return false;
    }

    /* If source is empty or destination is full at start, return false */
    returnval = !(fifo_is_empty(src) || fifo_is_full(dst));

    /* Transfer all elements */
    while (!fifo_is_empty(src) && !fifo_is_full(dst))
    {
        fifo_put(dst, fifo_get(src));
    }

    return returnval;
}

/******************************************************************************
 * Function Name  : fifo_ping_pong
 * Input          : uint8_t* data_in  - Input data buffer
 *                : uint16_t size     - Number of bytes to process
 *                : uint8_t flag      - Control flag (0=ping, 1=pong)
 * Output         : uint8_t* data_out - Output data buffer
 * Return         : true if success, false if fail
 * Description    : Implements ping-pong buffering mechanism using two FIFOs.
 *                  When flag=0, writes to ping FIFO and reads from pong FIFO.
 *                  When flag=1, writes to pong FIFO and reads from ping FIFO.
 *****************************************************************************/
bool fifo_ping_pong(uint8_t* data_in, uint8_t* data_out, uint16_t size, uint8_t flag)
{
    uint16_t i;
    fifo_t* write_fifo;
    fifo_t* read_fifo;

    /* Validate parameters */
    if (data_in == NULL || data_out == NULL || size == 0U)
    {
        return false;
    }

    /* Select which FIFO to write to and read from based on flag */
    if (flag == 0U)
    {
        write_fifo = &g_fifo_ping;
        read_fifo = &g_fifo_pong;
    }
    else
    {
        write_fifo = &g_fifo_pong;
        read_fifo = &g_fifo_ping;
    }

    /* Write data to the active FIFO */
    for (i = 0U; i < size; i++)
    {
        if (!fifo_put(write_fifo, data_in[i]))
        {
            /* FIFO full - cannot write all data */
            return false;
        }
    }

    /* Read data from the inactive FIFO */
    for (i = 0U; i < size; i++)
    {
        if (fifo_is_empty(read_fifo))
        {
            /* No more data to read */
            data_out[i] = 0x00;  /* Fill with zeros */
        }
        else
        {
            data_out[i] = fifo_get(read_fifo);
        }
    }

    return true;
}
