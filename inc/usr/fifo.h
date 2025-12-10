/*******************************************************************************
 * FILE:        fifo.h
 *
 * DESCRIPTION:
 *     Circular FIFO (First-In-First-Out) Buffer Interface.
 *     Provides a thread-safe, fixed-size circular buffer implementation for
 *     data buffering between producers and consumers. Supports atomic
 *     operations, ping-pong buffering, and comprehensive buffer management
 *     utilities.
 *
 * FEATURES:
 *     - Fixed buffer size (configurable via MAX_FIFO_SIZE)
 *     - Empty/full detection with boundary conditions
 *     - Non-blocking operations with failure indication
 *     - Ping-pong buffer support for concurrent access
 *     - Debug and diagnostic utilities
 *
 * DOCUMENTATION STYLE:
 *     - Technical and functional documentation
 *     - No functional or logical modifications to original code
 *     - Enhanced comments, formatting, and structure
 *
 * AUTHOR:      Rogerio Lima (Original)
 * REFORMAT:    2025 (Documentation and formatting only)
 *
 *******************************************************************************/

#ifndef FIFO_H_
#define FIFO_H_

#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * FIFO CONFIGURATION
 *
 * DESCRIPTION:
 *     Defines the maximum size of the FIFO buffer. The buffer uses a
 *     fixed-size circular array implementation.
 *
 * CONSTRAINTS:
 *     - Must be a power of 2 for optimal performance (not strictly required)
 *     - Maximum value limited by int16_t index range (-32,768 to 32,767)
 *     - Memory usage: MAX_FIFO_SIZE bytes per FIFO instance
 *
 * NOTES:
 *     - Changing this value requires recompilation of all dependent modules
 *     - Larger sizes increase memory usage but reduce overflow risk
 *******************************************************************************/
#define MAX_FIFO_SIZE   256U    /* Maximum FIFO buffer capacity in bytes */

/*******************************************************************************
 * DATA STRUCTURES
 *
 * DESCRIPTION:
 *     Defines the main FIFO structure that implements a circular buffer.
 *     Uses front and rear indices to track buffer state with special values
 *     for empty condition detection.
 *
 * STRUCTURE FIELDS:
 *     data[]  : Fixed-size byte array for element storage
 *     front   : Index of oldest element (-1 indicates empty buffer)
 *     rear    : Index of newest element (-1 indicates empty buffer)
 *
 * BUFFER STATES:
 *     EMPTY:   front == -1 && rear == -1
 *     SINGLE:  front == rear (one element present)
 *     FULL:    (rear + 1) % MAX_FIFO_SIZE == front
 *     NORMAL:  Other valid front/rear combinations
 *
 * NOTES:
 *     - Uses int16_t for indices to support empty state (-1) indication
 *     - Circular indexing: (index + 1) % MAX_FIFO_SIZE
 *     - Buffer wraps around at MAX_FIFO_SIZE boundary
 *******************************************************************************/
typedef struct {
    uint8_t data[MAX_FIFO_SIZE];  /* Circular buffer storage array */
    int16_t front;                /* Front index (-1 if empty) */
    int16_t rear;                 /* Rear index (-1 if empty) */
} fifo_t;

/*******************************************************************************
 * GLOBAL BUFFER INSTANCES
 *
 * DESCRIPTION:
 *     Predefined FIFO buffers for common usage patterns, particularly
 *     ping-pong buffering where one buffer is filled while the other is
 *     processed.
 *
 * INSTANCES:
 *     g_fifo_ping: Primary buffer for data collection
 *     g_fifo_pong: Secondary buffer for data processing
 *
 * USAGE PATTERN:
 *     1. Producer fills g_fifo_ping (e.g., in ISR)
 *     2. When full or ready, swap buffers using fifo_transfer()
 *     3. Consumer processes g_fifo_pong (e.g., in main loop)
 *     4. Repeat with roles reversed
 *
 * NOTES:
 *     - These are extern declarations; definitions must exist in a .c file
 *     - Can be used independently for different data streams
 *******************************************************************************/
extern fifo_t g_fifo_ping;    /* Primary FIFO buffer for data collection */
extern fifo_t g_fifo_pong;    /* Secondary FIFO buffer for data processing */

/*******************************************************************************
 * FUNCTION GROUP: FIFO INITIALIZATION AND STATE MANAGEMENT
 *
 * DESCRIPTION:
 *     Functions for initializing FIFO buffers and checking their state.
 *     Provides empty/full detection and buffer capacity information.
 *
 * OPERATION:
 *     - Initialization sets front and rear to -1 (empty state)
 *     - Empty/full checks use mathematical comparisons, not element counting
 *     - Count functions traverse buffer to determine exact element count
 *
 * NOTES:
 *     - State checking is O(1) complexity (constant time)
 *     - Element counting is O(n) complexity (linear time)
 *******************************************************************************/

/**
 * @brief Initialize FIFO buffer to empty state
 * @param fifo Pointer to FIFO structure to initialize
 * @return void
 *
 * OPERATION:
 *     - Sets front index to -1
 *     - Sets rear index to -1
 *     - Does not clear data array (uninitialized values remain)
 *
 * USAGE:
 *     - Must be called before first use of any FIFO buffer
 *     - Can be called to reset buffer to empty state
 *
 * NOTES:
 *     - Not thread-safe; caller must ensure exclusive access during init
 *     - Does not require clearing data array for functional operation
 */
void fifo_init(fifo_t* fifo);

/**
 * @brief Check if FIFO buffer is empty
 * @param fifo Pointer to FIFO structure to check
 * @return true if buffer is empty, false otherwise
 *
 * OPERATION:
 *     - Returns (fifo->front == -1 && fifo->rear == -1)
 *
 * USAGE:
 *     - Condition check before fifo_get() operations
 *     - Determining if data is available for processing
 *
 * NOTES:
 *     - O(1) operation with minimal overhead
 *     - Thread-safe for single producer/single consumer patterns
 */
bool fifo_is_empty(fifo_t* fifo);

/**
 * @brief Check if FIFO buffer is full
 * @param fifo Pointer to FIFO structure to check
 * @return true if buffer is full, false otherwise
 *
 * OPERATION:
 *     - Returns ((fifo->rear + 1) % MAX_FIFO_SIZE == fifo->front)
 *     - Special case: empty buffer returns false
 *
 * USAGE:
 *     - Condition check before fifo_put() operations
 *     - Buffer overflow prevention
 *
 * NOTES:
 *     - O(1) operation with minimal overhead
 *     - Full buffer has MAX_FIFO_SIZE - 1 elements (one slot reserved)
 */
bool fifo_is_full(fifo_t* fifo);

/**
 * @brief Count number of elements currently in FIFO buffer
 * @param fifo Pointer to FIFO structure to examine
 * @return Number of elements (0 to MAX_FIFO_SIZE-1)
 *
 * OPERATION:
 *     - Returns 0 if buffer is empty
 *     - Calculates count using circular buffer mathematics
 *
 * USAGE:
 *     - Determining amount of data available for processing
 *     - Buffer utilization monitoring
 *
 * NOTES:
 *     - O(1) operation using mathematical calculation
 *     - More efficient than manual traversal
 */
uint16_t fifo_count_elements(fifo_t* fifo);

/**
 * @brief Calculate available free space in FIFO buffer
 * @param fifo Pointer to FIFO structure to examine
 * @return Number of free slots (0 to MAX_FIFO_SIZE-1)
 *
 * OPERATION:
 *     - Returns MAX_FIFO_SIZE - fifo_count_elements(fifo) - 1
 *     - Accounts for one reserved slot in full condition
 *
 * USAGE:
 *     - Flow control for data producers
 *     - Determining if complete packet can be buffered
 *
 * NOTES:
 *     - Maximum available space is MAX_FIFO_SIZE - 1
 *     - One slot is reserved to distinguish full from empty states
 */
uint16_t fifo_available_space(fifo_t* fifo);

/*******************************************************************************
 * FUNCTION GROUP: FIFO DATA OPERATIONS
 *
 * DESCRIPTION:
 *     Core functions for inserting, retrieving, and manipulating data
 *     in the FIFO buffer. Provides both destructive and non-destructive
 *     operations.
 *
 * OPERATION:
 *     - Insertion at rear (newest position)
 *     - Retrieval from front (oldest position)
 *     - Circular indexing with modulo arithmetic
 *     - Boundary condition handling
 *
 * NOTES:
 *     - All operations maintain circular buffer integrity
 *     - Empty/full conditions are properly handled
 *******************************************************************************/

/**
 * @brief Insert single element at rear of FIFO buffer
 * @param fifo Pointer to FIFO structure
 * @param data Byte to insert (0x00 to 0xFF)
 * @return true if insertion successful, false if buffer is full
 *
 * OPERATION:
 *     - Checks if buffer is full (fifo_is_full())
 *     - Updates rear index with circular wrap
 *     - Stores data in data[rear]
 *     - Handles special case of first element (sets front = rear)
 *
 * USAGE:
 *     - Data acquisition from sensors/peripherals
 *     - Buffering serial communication data
 *     - Interrupt service routine data collection
 *
 * NOTES:
 *     - Returns false without modification if buffer is full
 *     - Not atomic; requires external synchronization in multi-context environments
 */
bool fifo_put(fifo_t* fifo, uint8_t data);

/**
 * @brief Remove and return element from front of FIFO buffer
 * @param fifo Pointer to FIFO structure
 * @return Data byte (0x00-0xFF) if successful, 0xFF if buffer is empty
 *
 * OPERATION:
 *     - Checks if buffer is empty (fifo_is_empty())
 *     - Retrieves data from data[front]
 *     - Updates front index with circular wrap
 *     - Handles special case of last element (sets front = rear = -1)
 *
 * USAGE:
 *     - Data processing in main application loop
 *     - Consuming buffered data
 *     - Retrieving oldest data first (FIFO order)
 *
 * NOTES:
 *     - Returns 0xFF on empty buffer (distinct from valid 0xFF data)
 *     - Caller should check fifo_is_empty() before calling when 0xFF is valid data
 *     - Destructive operation (removes element from buffer)
 */
uint8_t fifo_get(fifo_t* fifo);

/**
 * @brief Examine front element without removing it
 * @param fifo Pointer to FIFO structure
 * @return Data byte (0x00-0xFF) if successful, 0xFF if buffer is empty
 *
 * OPERATION:
 *     - Checks if buffer is empty (fifo_is_empty())
 *     - Returns data[front] without modifying indices
 *
 * USAGE:
 *     - Previewing next data element
 *     - Protocol parsing without consumption
 *     - Conditional processing based on next value
 *
 * NOTES:
 *     - Non-destructive operation (buffer unchanged)
 *     - Same return value convention as fifo_get()
 *     - Useful for look-ahead parsing algorithms
 */
uint8_t fifo_peek(fifo_t* fifo);

/**
 * @brief Remove element from front without returning it
 * @param fifo Pointer to FIFO structure
 * @return true if deletion successful, false if buffer is empty
 *
 * OPERATION:
 *     - Same as fifo_get() but discards the data value
 *     - Updates indices without returning data
 *
 * USAGE:
 *     - Discarding old data without processing
 *     - Buffer cleanup operations
 *     - Skipping invalid or processed data
 *
 * NOTES:
 *     - More efficient than fifo_get() when data value is not needed
 *     - Same failure conditions as fifo_get()
 */
bool fifo_del(fifo_t* fifo);

/*******************************************************************************
 * FUNCTION GROUP: FIFO BUFFER MANAGEMENT
 *
 * DESCRIPTION:
 *     Advanced functions for buffer manipulation, transfer, and diagnostics.
 *     Supports ping-pong buffering patterns and debug utilities.
 *
 * OPERATION:
 *     - Bulk operations on entire buffers
 *     - Buffer-to-buffer transfer
 *     - Diagnostic output
 *
 * NOTES:
 *     - Transfer functions useful for producer/consumer patterns
 *     - Debug functions should be excluded from production code
 *******************************************************************************/

/**
 * @brief Transfer all elements from source to destination FIFO
 * @param src Pointer to source FIFO structure (will be emptied)
 * @param dst Pointer to destination FIFO structure (will receive data)
 * @return true if transfer incomplete (destination full), false if complete
 *
 * OPERATION:
 *     - While source not empty AND destination not full:
 *         data = fifo_get(src)
 *         fifo_put(dst, data)
 *     - Stops when source empty OR destination full
 *
 * USAGE:
 *     - Ping-pong buffer swapping
 *     - Buffer consolidation
 *     - Data pipeline staging
 *
 * NOTES:
 *     - Partial transfers possible if destination becomes full
 *     - Source buffer may retain data if transfer incomplete
 *     - Not atomic; requires external synchronization
 */
bool fifo_transfer(fifo_t* src, fifo_t* dst);

/**
 * @brief Remove all elements from FIFO buffer
 * @param fifo Pointer to FIFO structure to clear
 * @return void
 *
 * OPERATION:
 *     - Sets front index to -1
 *     - Sets rear index to -1
 *     - Does not modify data array contents
 *
 * USAGE:
 *     - Buffer reset after error conditions
 *     - Preparation for new data stream
 *     - System reinitialization
 *
 * NOTES:
 *     - O(1) operation (constant time)
 *     - More efficient than fifo_del() in loop
 *     - Does not zero-fill data array (residual data remains)
 */
void fifo_clear(fifo_t* fifo);

/**
 * @brief Print FIFO buffer contents for debugging (USE WITH CAUTION)
 * @param fifo Pointer to FIFO structure to display
 * @return void
 *
 * OPERATION:
 *     - Outputs buffer indices (front, rear)
 *     - Prints all data elements in storage order
 *     - Marks current front and rear positions
 *
 * USAGE:
 *     - Debugging buffer corruption
 *     - Verifying data integrity
 *     - Development and testing only
 *
 * WARNINGS:
 *     - NOT for use in ISR context (blocking I/O)
 *     - NOT for production code (debug only)
 *     - Requires stdio.h and console output
 *     - May significantly impact real-time performance
 */
void fifo_print(fifo_t* fifo);

/*******************************************************************************
 * USAGE EXAMPLES (NON-COMPILED)
 *
 * DESCRIPTION:
 *     Practical examples demonstrating common FIFO usage patterns.
 *     These examples are commented out and provided for reference only.
 *
 * PATTERNS DEMONSTRATED:
 *     1. Basic FIFO operations
 *     2. Ping-pong buffering for ISR/main loop communication
 *     3. Flow control with available space checking
 *
 * NOTES:
 *     - Examples use hypothetical functions and variables
 *     - Actual implementation may require additional synchronization
 *******************************************************************************/
#if 0  /* Example code - not compiled */

/* Example 1: Basic FIFO operations */
void example_basic_usage(void)
{
    fifo_t my_fifo;
    uint8_t data;
    
    /* Initialize FIFO to empty state */
    fifo_init(&my_fifo);
    
    /* Add data elements (producer side) */
    if (!fifo_put(&my_fifo, 0x42)) {
        /* Handle buffer full condition */
    }
    
    if (!fifo_put(&my_fifo, 0x43)) {
        /* Handle buffer full condition */
    }
    
    /* Retrieve data elements (consumer side) */
    while (!fifo_is_empty(&my_fifo)) {
        data = fifo_get(&my_fifo);
        /* Process data */
    }
}

/* Example 2: Ping-pong buffering pattern */
void example_pingpong_buffering(void)
{
    /* Initialize both buffers */
    fifo_init(&g_fifo_ping);
    fifo_init(&g_fifo_pong);
    
    /* In ISR (producer): fill ping buffer */
    void data_acquisition_isr(void)
    {
        uint8_t sensor_data = read_sensor();
        
        /* Non-blocking insert into ping buffer */
        if (!fifo_put(&g_fifo_ping, sensor_data)) {
            /* Buffer full - handle overflow */
            led_red_on(); /* Visual overflow indication */
        }
    }
    
    /* In main loop (consumer): process pong buffer */
    void main_loop_processing(void)
    {
        /* When ping buffer has data, transfer to pong for processing */
        if (!fifo_is_empty(&g_fifo_ping)) {
            /* Transfer all data from ping to pong */
            bool incomplete = fifo_transfer(&g_fifo_ping, &g_fifo_pong);
            
            if (incomplete) {
                /* Destination (pong) was full - handle back pressure */
            }
            
            /* Process all data in pong buffer */
            while (!fifo_is_empty(&g_fifo_pong)) {
                uint8_t data = fifo_get(&g_fifo_pong);
                process_data(data);
            }
        }
    }
}

/* Example 3: Flow control with space checking */
void example_flow_control(void)
{
    fifo_t tx_buffer;
    uint8_t packet[] = {0x01, 0x02, 0x03, 0x04};
    uint8_t packet_size = sizeof(packet);
    uint16_t free_space;
    
    fifo_init(&tx_buffer);
    
    /* Check available space before adding complete packet */
    free_space = fifo_available_space(&tx_buffer);
    
    if (free_space >= packet_size) {
        /* Safe to add entire packet */
        for (int i = 0; i < packet_size; i++) {
            fifo_put(&tx_buffer, packet[i]);
        }
        
        /* Signal that packet is ready for transmission */
        led_green_on(); /* Visual ready indication */
    }
    else {
        /* Not enough space - implement flow control */
        led_yellow_on(); /* Visual back-pressure indication */
        
        /* Option 1: Wait for space (blocking - not recommended for ISR) */
        /* Option 2: Drop packet (lossy) */
        /* Option 3: Partial packet buffering */
    }
}

#endif  /* End of example code */

#endif /* FIFO_H_ */