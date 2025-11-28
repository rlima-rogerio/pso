/******************************************************************************
* FILENAME:    fifo.c
*
* DESCRIPTION:
*       Functions to implement a circular FIFO queue.
*
* FUNCTIONS:
*    void LEDRedOn    (void);

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
*
* -----------------------------------------------------------------------------
* 2.0
******************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
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
void fifo_init (fifo_t* fifo)
{
    fifo->rear =  -1;
    fifo->front = -1;
}

/******************************************************************************
 * Function Name  : fifo_put
 * Input          : fifo_t* fifo
 * Output         : None
 * Return         : 1 if success, 0 if fail
 * Description    : Inserts an element in queue at rear end
 *****************************************************************************/
bool fifo_put (fifo_t* fifo, uint8_t data)
{
    bool returnval;


    /* FIFO is full and cannot inserts new data */
    if (fifo_is_full(fifo))
    {
        returnval = false;
    }
    else if (fifo_is_empty(fifo))
    {
        fifo->front = 0;
        fifo->rear  = 0;
        fifo->data[fifo->rear] = data;
        returnval = true;
    }
    else
    {
        fifo->rear = (fifo->rear + 1)%MAX_FIFO_SIZE;
        fifo->data[fifo->rear] = data;
        returnval = true;
    }

    return returnval;

}

/******************************************************************************
 * Function Name  : fifo_del
 * Input          : fifo_t* fifo
 * Output         : None
 * Return         : 1 if success, 0 if fail
 * Description    : Deletes an element in queue from front end
 *****************************************************************************/
bool fifo_del (fifo_t* fifo)
{
    bool returnval;


    /* FIFO is empty and cannot deletes an old data */
    if (fifo_is_empty(fifo))
    {
        returnval = false;
    }
    else if( fifo->front == fifo->rear)
    {
        fifo->front = -1;
        fifo->rear  = -1;
        returnval = true;
    }
    else
    {
        fifo->front = (fifo->front + 1)%MAX_FIFO_SIZE;
        returnval = true;
    }

    return returnval;

}

/******************************************************************************
 * Function Name  : fifo_get
 * Input          : fifo_t* fifo
 * Output         : None
 * Return         : -1 when FIFO is empty, (uint8_t) returnval otherwise.
 * Description    : Returns an element at front of queue. Reading an element
 *                  automatically deletes it.
 *****************************************************************************/
uint8_t fifo_get (fifo_t* fifo)
{
    uint8_t returnval;


    /* FIFO is empty and cannot returns data */
    if (fifo->front == -1)
    {
        returnval = -1;
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
 * Return         : 1 if it's empty, 0 otherwise
 * Description    : Checks if either FIFO is empty or not
 *****************************************************************************/
bool fifo_is_empty (fifo_t* fifo)
{

    return ( (fifo->front == -1) && (fifo->rear == -1));

}

/******************************************************************************
 * Function Name  : fifo_is_full
 * Input          : fifo_t* fifo
 * Output         : None
 * Return         : 1 if it's full, 0 otherwise
 * Description    : Checks if either FIFO is full or not
 *****************************************************************************/
bool fifo_is_full (fifo_t* fifo)
{

    return ( (fifo->rear + 1)%MAX_FIFO_SIZE == fifo->front ? true : false );

}

/******************************************************************************
 * Function Name  : fifo_print
 * Input          : fifo_t* fifo
 * Output         : None
 * Return         : None
 * Description    : Prints the elements in queue from front to rear. Should be
 *                  used only to test the code.
 *****************************************************************************/
void fifo_print (fifo_t* fifo)
{
    int16_t count;
    int16_t index, i;

    /* Calculates the number of elements in queue */
    count = (fifo->rear + MAX_FIFO_SIZE - fifo->front)%MAX_FIFO_SIZE + 1;

    for (i = 0; i < count ; i++)
    {
        index = (fifo->front + i)%MAX_FIFO_SIZE;
        printf ("%d \n\r", fifo->data[index]);
    }

}


/******************************************************************************
 * Function Name  : fifo_count_elements
 * Input          : fifo_t* fifo
 * Output         : None
 * Return         : -1 if FIFO is empty, # of elements in queue.
 * Description    : Calculates the number of elements in queue.
 *****************************************************************************/
uint16_t fifo_count_elements (fifo_t* fifo)
{
    uint16_t returnval;

    returnval = (fifo->rear + MAX_FIFO_SIZE - fifo->front)%MAX_FIFO_SIZE + 1;

    return returnval;

}


/******************************************************************************
 * Function Name  : fifo_transfer
 * Input          : fifo_t* src
 * Output         : fifo_t* dst
 * Return         : 1 if success, 0 if fail
 * Description    : Transfers all new elements of 'src' queue and puts them in
 *                  'dst' queue.
 *****************************************************************************/
bool fifo_transfer (fifo_t* src, fifo_t* dst)
{
    bool returnval;

    /* If in the early transfer 'src' is empty or 'dst' is full, it means
     * that no one element in queue  was not transfered                      */
    returnval = (fifo_is_empty(src)) || (fifo_is_full(dst));

    while ( !(fifo_is_empty(src)) && !(fifo_is_full(dst)) )
    {
    	fifo_put (dst, fifo_get (src) );
    }

    return returnval;

}


/******************************************************************************
 * Function Name  : fifo_ping_pong
 * Input          : uint8_t* data_in
 *                : uint8_t flag
 * Output         : uint8_t* data_out
 *                : uint16_t size
 * Return         : 1 if success, 0 if fail
 * Description    : Transfers all new elements of 'src' queue and puts them in
 *                  'dst' queue.
 *****************************************************************************/
bool fifo_ping_pong (uint8_t* data_in, uint8_t* data_out, uint16_t size, uint8_t flag)
{


}
