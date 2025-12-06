/*******************************************************************************
 * FILENAME:    fifo.c (Refactored)
 *
 * DESCRIPTION:
 *       Circular FIFO (First-In-First-Out) queue implementation for buffering
 *       data packets during streaming operations.
 *
 * FEATURES:
 *       - Circular buffer implementation
 *       - Thread-safe design for ISR context
 *       - Overflow/underflow protection
 *       - Transfer between multiple FIFOs
 *
 * AUTHOR:      Rogerio Lima (Original)
 *              Refactored: 2025
 ******************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "fifo.h"

/*******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************/
fifo_t g_fifo_ping;     /* Primary FIFO buffer */
fifo_t g_fifo_pong;     /* Secondary FIFO buffer (for ping-pong buffering) */

/*******************************************************************************
 * FUNCTION IMPLEMENTATIONS
 ******************************************************************************/

/**
 * @brief Initialize FIFO to empty state
 * 
 * Sets both front and rear indices to -1, indicating an empty queue.
 * Must be called before using any other FIFO functions.
 * 
 * @param fifo Pointer to FIFO structure to initialize
 */
void fifo_init(fifo_t* fifo)
{
    if (fifo == NULL)
    {
        return;
    }
    
    fifo->rear = -1;
    fifo->front = -1;
}

/**
 * @brief Insert element at rear of FIFO
 * 
 * Adds a new byte to the FIFO queue. If FIFO is full, insertion fails.
 * Handles both empty FIFO and normal insertion cases.
 * 
 * @param fifo Pointer to FIFO structure
 * @param data Byte to insert
 * @return true if insertion successful, false if FIFO is full
 */
bool fifo_put(fifo_t* fifo, uint8_t data)
{
    if (fifo == NULL)
    {
        return false;
    }

    /* Check if FIFO is full */
    if (fifo_is_full(fifo))
    {
        return false;  /* Cannot insert, FIFO is full */
    }
    
    /* Handle empty FIFO case */
    if (fifo_is_empty(fifo))
    {
        fifo->front = 0;
        fifo->rear = 0;
        fifo->data[fifo->rear] = data;
        return true;
    }
    
    /* Normal insertion - circular increment */
    fifo->rear = (fifo->rear + 1) % MAX_FIFO_SIZE;
    fifo->data[fifo->rear] = data;
    
    return true;
}

/**
 * @brief Remove element from front of FIFO
 * 
 * Removes the oldest element from the FIFO without returning it.
 * Updates indices to maintain circular buffer integrity.
 * 
 * @param fifo Pointer to FIFO structure
 * @return true if deletion successful, false if FIFO is empty
 */
bool fifo_del(fifo_t* fifo)
{
    if (fifo == NULL)
    {
        return false;
    }

    /* Check if FIFO is empty */
    if (fifo_is_empty(fifo))
    {
        return false;  /* Cannot delete from empty FIFO */
    }
    
    /* Check if this is the last element */
    if (fifo->front == fifo->rear)
    {
        /* Reset to empty state */
        fifo->front = -1;
        fifo->rear = -1;
        return true;
    }
    
    /* Normal deletion - circular increment */
    fifo->front = (fifo->front + 1) % MAX_FIFO_SIZE;
    
    return true;
}

/**
 * @brief Get and remove element from front of FIFO
 * 
 * Retrieves the oldest element and automatically removes it from the queue.
 * This is the typical way to consume data from the FIFO.
 * 
 * @param fifo Pointer to FIFO structure
 * @return Data byte if successful, 0xFF if FIFO is empty
 * 
 * @note Returns 0xFF on error, which could be a valid data value.
 *       Check fifo_is_empty() before calling if this is a concern.
 */
uint8_t fifo_get(fifo_t* fifo)
{
    uint8_t data;
    
    if (fifo == NULL)
    {
        return 0xFF;  /* Error value */
    }

    /* Check if FIFO is empty */
    if (fifo_is_empty(fifo))
    {
        return 0xFF;  /* Error value */
    }
    
    /* Get data from front */
    data = fifo->data[fifo->front];
    
    /* Remove element from FIFO */
    fifo_del(fifo);
    
    return data;
}

/**
 * @brief Check if FIFO is empty
 * 
 * @param fifo Pointer to FIFO structure
 * @return true if FIFO is empty, false otherwise
 */
bool fifo_is_empty(fifo_t* fifo)
{
    if (fifo == NULL)
    {
        return true;
    }
    
    return ((fifo->front == -1) && (fifo->rear == -1));
}

/**
 * @brief Check if FIFO is full
 * 
 * Uses circular buffer logic to determine if the next insertion
 * would overlap with the front index.
 * 
 * @param fifo Pointer to FIFO structure
 * @return true if FIFO is full, false otherwise
 */
bool fifo_is_full(fifo_t* fifo)
{
    if (fifo == NULL)
    {
        return true;
    }
    
    return ((fifo->rear + 1) % MAX_FIFO_SIZE == fifo->front);
}

/**
 * @brief Count number of elements in FIFO
 * 
 * Calculates the current number of elements using circular buffer math.
 * Useful for monitoring buffer usage and flow control.
 * 
 * @param fifo Pointer to FIFO structure
 * @return Number of elements currently in FIFO (0 if empty or NULL)
 */
uint16_t fifo_count_elements(fifo_t* fifo)
{
    if (fifo == NULL || fifo_is_empty(fifo))
    {
        return 0;
    }
    
    return ((fifo->rear + MAX_FIFO_SIZE - fifo->front) % MAX_FIFO_SIZE + 1);
}

/**
 * @brief Transfer all elements from source to destination FIFO
 * 
 * Moves all elements from src FIFO to dst FIFO in FIFO order.
 * Stops if destination becomes full.
 * 
 * @param src Source FIFO (will be emptied)
 * @param dst Destination FIFO (will receive elements)
 * @return true if any elements were NOT transferred (src empty or dst full),
 *         false if all elements transferred successfully
 * 
 * @note Return value indicates incomplete transfer, not success/failure
 */
bool fifo_transfer(fifo_t* src, fifo_t* dst)
{
    if (src == NULL || dst == NULL)
    {
        return true;  /* Error - indicate incomplete */
    }

    /* Check initial conditions */
    if (fifo_is_empty(src) || fifo_is_full(dst))
    {
        return true;  /* Incomplete transfer */
    }

    /* Transfer elements until src is empty or dst is full */
    while (!fifo_is_empty(src) && !fifo_is_full(dst))
    {
        uint8_t data = fifo_get(src);
        fifo_put(dst, data);
    }
    
    /* Return true if transfer was incomplete */
    return !fifo_is_empty(src);
}

/**
 * @brief Print FIFO contents (Debug function)
 * 
 * Prints all elements in FIFO from front to rear.
 * Should only be used for debugging, not in production.
 * 
 * @param fifo Pointer to FIFO structure
 * 
 * @warning This function uses printf and should not be called from ISR context
 */
void fifo_print(fifo_t* fifo)
{
    int16_t count;
    int16_t index;
    int16_t i;
    
    if (fifo == NULL || fifo_is_empty(fifo))
    {
        printf("FIFO is empty\n\r");
        return;
    }

    /* Calculate number of elements */
    count = fifo_count_elements(fifo);
    
    printf("FIFO contents (%d elements):\n\r", count);

    /* Print each element */
    for (i = 0; i < count; i++)
    {
        index = (fifo->front + i) % MAX_FIFO_SIZE;
        printf("[%d]: 0x%02X (%d)\n\r", i, fifo->data[index], fifo->data[index]);
    }
}

/**
 * @brief Peek at element without removing it
 * 
 * Returns the front element without removing it from the FIFO.
 * Useful for examining data before committing to removing it.
 * 
 * @param fifo Pointer to FIFO structure
 * @return Data byte at front if successful, 0xFF if FIFO is empty
 * 
 * @note This function is not in the original implementation but is
 *       commonly useful for FIFO operations
 */
uint8_t fifo_peek(fifo_t* fifo)
{
    if (fifo == NULL || fifo_is_empty(fifo))
    {
        return 0xFF;  /* Error value */
    }
    
    return fifo->data[fifo->front];
}

/**
 * @brief Clear all elements from FIFO
 * 
 * Resets FIFO to empty state without actually clearing data array.
 * This is faster than removing elements one by one.
 * 
 * @param fifo Pointer to FIFO structure
 */
void fifo_clear(fifo_t* fifo)
{
    if (fifo == NULL)
    {
        return;
    }
    
    fifo->front = -1;
    fifo->rear = -1;
}

/**
 * @brief Get available space in FIFO
 * 
 * Calculates how many more elements can be added before FIFO is full.
 * 
 * @param fifo Pointer to FIFO structure
 * @return Number of free slots in FIFO
 */
uint16_t fifo_available_space(fifo_t* fifo)
{
    if (fifo == NULL)
    {
        return 0;
    }
    
    if (fifo_is_empty(fifo))
    {
        return MAX_FIFO_SIZE;
    }
    
    if (fifo_is_full(fifo))
    {
        return 0;
    }
    
    return (MAX_FIFO_SIZE - fifo_count_elements(fifo));
}