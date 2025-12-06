/*******************************************************************************
 * FILENAME:    fifo.h (Refactored)
 *
 * DESCRIPTION:
 *       Header file for circular FIFO implementation.
 *       Provides interface for thread-safe buffering operations.
 *
 * AUTHOR:      Rogerio Lima (Original)
 *              Refactored: 2025
 ******************************************************************************/

#ifndef FIFO_H_
#define FIFO_H_

#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * CONFIGURATION
 ******************************************************************************/
#define MAX_FIFO_SIZE   256U    /* Maximum FIFO buffer size (must be power of 2) */

/*******************************************************************************
 * DATA STRUCTURES
 ******************************************************************************/

/**
 * @brief Circular FIFO structure
 * 
 * Implements a circular buffer with the following characteristics:
 * - front: Index of oldest element (-1 if empty)
 * - rear: Index of newest element (-1 if empty)
 * - data: Fixed-size byte array for storage
 * 
 * Empty condition: front == -1 && rear == -1
 * Full condition: (rear + 1) % MAX_FIFO_SIZE == front
 */
typedef struct {
    uint8_t data[MAX_FIFO_SIZE];  /* Data storage array */
    int16_t front;                /* Front index (oldest element) */
    int16_t rear;                 /* Rear index (newest element) */
} fifo_t;

/*******************************************************************************
 * GLOBAL VARIABLES (External)
 ******************************************************************************/
extern fifo_t g_fifo_ping;    /* Primary FIFO buffer */
extern fifo_t g_fifo_pong;    /* Secondary FIFO buffer */

/*******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/

/**
 * @brief Initialize FIFO to empty state
 * @param fifo Pointer to FIFO structure
 */
void fifo_init(fifo_t* fifo);

/**
 * @brief Insert element at rear of FIFO
 * @param fifo Pointer to FIFO structure
 * @param data Byte to insert
 * @return true if successful, false if FIFO is full
 */
bool fifo_put(fifo_t* fifo, uint8_t data);

/**
 * @brief Remove element from front of FIFO (without returning it)
 * @param fifo Pointer to FIFO structure
 * @return true if successful, false if FIFO is empty
 */
bool fifo_del(fifo_t* fifo);

/**
 * @brief Get and remove element from front of FIFO
 * @param fifo Pointer to FIFO structure
 * @return Data byte if successful, 0xFF if FIFO is empty
 */
uint8_t fifo_get(fifo_t* fifo);

/**
 * @brief Check if FIFO is empty
 * @param fifo Pointer to FIFO structure
 * @return true if empty, false otherwise
 */
bool fifo_is_empty(fifo_t* fifo);

/**
 * @brief Check if FIFO is full
 * @param fifo Pointer to FIFO structure
 * @return true if full, false otherwise
 */
bool fifo_is_full(fifo_t* fifo);

/**
 * @brief Count number of elements in FIFO
 * @param fifo Pointer to FIFO structure
 * @return Number of elements currently in FIFO
 */
uint16_t fifo_count_elements(fifo_t* fifo);

/**
 * @brief Transfer all elements from source to destination FIFO
 * @param src Source FIFO (will be emptied)
 * @param dst Destination FIFO (will receive elements)
 * @return true if incomplete transfer, false if all transferred
 */
bool fifo_transfer(fifo_t* src, fifo_t* dst);

/**
 * @brief Print FIFO contents (Debug only)
 * @param fifo Pointer to FIFO structure
 * @warning Do not use in ISR context or production code
 */
void fifo_print(fifo_t* fifo);

/**
 * @brief Peek at front element without removing it
 * @param fifo Pointer to FIFO structure
 * @return Data byte at front if successful, 0xFF if empty
 */
uint8_t fifo_peek(fifo_t* fifo);

/**
 * @brief Clear all elements from FIFO
 * @param fifo Pointer to FIFO structure
 */
void fifo_clear(fifo_t* fifo);

/**
 * @brief Get available space in FIFO
 * @param fifo Pointer to FIFO structure
 * @return Number of free slots
 */
uint16_t fifo_available_space(fifo_t* fifo);

/*******************************************************************************
 * USAGE EXAMPLES
 ******************************************************************************/
#if 0  /* Example code - not compiled */

/* Example 1: Basic FIFO usage */
void example_basic_usage(void)
{
    fifo_t my_fifo;
    uint8_t data;
    
    /* Initialize FIFO */
    fifo_init(&my_fifo);
    
    /* Add data */
    fifo_put(&my_fifo, 0x42);
    fifo_put(&my_fifo, 0x43);
    
    /* Get data */
    data = fifo_get(&my_fifo);  /* Returns 0x42 */
    data = fifo_get(&my_fifo);  /* Returns 0x43 */
}

/* Example 2: Ping-pong buffering */
void example_pingpong_buffering(void)
{
    fifo_t buffer_a, buffer_b;
    
    fifo_init(&buffer_a);
    fifo_init(&buffer_b);
    
    /* Fill buffer A in ISR */
    fifo_put(&buffer_a, sensor_data);
    
    /* When buffer A is ready, transfer to buffer B for processing */
    if (!fifo_is_empty(&buffer_a))
    {
        fifo_transfer(&buffer_a, &buffer_b);
        
        /* Process buffer B in main loop */
        while (!fifo_is_empty(&buffer_b))
        {
            uint8_t data = fifo_get(&buffer_b);
            process_data(data);
        }
    }
}

/* Example 3: Flow control */
void example_flow_control(void)
{
    fifo_t tx_buffer;
    uint16_t free_space;
    
    fifo_init(&tx_buffer);
    
    /* Check available space before adding data */
    free_space = fifo_available_space(&tx_buffer);
    
    if (free_space >= packet_size)
    {
        /* Safe to add entire packet */
        for (int i = 0; i < packet_size; i++)
        {
            fifo_put(&tx_buffer, packet[i]);
        }
    }
    else
    {
        /* Not enough space - handle overflow */
        handle_buffer_overflow();
    }
}

#endif  /* Example code */

#endif /* FIFO_H_ */
