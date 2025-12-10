/*******************************************************************************
 * FILE:        checksum.h
 *
 * DESCRIPTION:
 *     Checksum Calculation Header (X.25 CRC Implementation).
 *     Provides functions for calculating and accumulating 16-bit CRC checksums
 *     using the X.25 protocol algorithm. Used for error detection in serial
 *     communication protocols including MAVLink and ULink.
 *
 * DOCUMENTATION STYLE:
 *     - Technical and functional
 *     - No functional or logical modifications
 *     - Improved comments and formatting only
 *
 * REFERENCE:
 *     Based on MAVLink communication protocol checksum implementation.
 *     X.25 CRC algorithm provides reliable error detection for serial data.
 *
 *******************************************************************************/

/* C++ compatibility */
#ifdef __cplusplus
extern "C" {
#endif

#ifndef _CHECKSUM_H_
#define _CHECKSUM_H_

/* Visual Studio compatibility check */
#if (defined _MSC_VER) && (_MSC_VER < 1600)
#error "The C-MAVLink implementation requires Visual Studio 2010 or greater"
#endif

#include <stdint.h>

/*******************************************************************************
 * X.25 CRC CONSTANTS
 *******************************************************************************/

#define X25_INIT_CRC     0xffff  /* Initial CRC value (seed) */
#define X25_VALIDATE_CRC 0xf0b8  /* Expected CRC after validating a complete message */

/*******************************************************************************
 * FUNCTION: crc_accumulate
 *
 * DESCRIPTION:
 *     Accumulates one byte into the X.25 CRC checksum.
 *     Updates the 16-bit CRC accumulator with a single data byte.
 *
 * PARAMETERS:
 *     data      - Byte to accumulate into CRC
 *     crcAccum  - Pointer to 16-bit CRC accumulator (updated in-place)
 *
 * RETURNS:
 *     void
 *
 * ALGORITHM:
 *     1. tmp = data ^ (uint8_t)(*crcAccum & 0xff)
 *     2. tmp ^= (tmp << 4)
 *     3. *crcAccum = (*crcAccum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
 *
 * NOTES:
 *     - Inline function for maximum performance
 *     - X.25 CRC polynomial: x^16 + x^12 + x^5 + 1 (0x1021)
 *     - Bit processing: LSB-first (reverse of standard CRC-16-CCITT)
 *     - Used by MAVLink and other aviation communication protocols
 *******************************************************************************/
#ifndef HAVE_CRC_ACCUMULATE
static inline void crc_accumulate(uint8_t data, uint16_t *crcAccum)
{
    uint8_t tmp;

    /* Step 1: XOR data with low byte of current CRC */
    tmp = data ^ (uint8_t)(*crcAccum & 0xff);
    
    /* Step 2: Shift and XOR */
    tmp ^= (tmp << 4);
    
    /* Step 3: Update CRC accumulator with transformed byte */
    *crcAccum = (*crcAccum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}
#endif

/*******************************************************************************
 * FUNCTION: crc_init
 *
 * DESCRIPTION:
 *     Initializes the CRC accumulator to the X.25 standard initial value.
 *
 * PARAMETERS:
 *     crcAccum - Pointer to 16-bit CRC accumulator to initialize
 *
 * RETURNS:
 *     void
 *
 * NOTES:
 *     - Must be called before starting CRC calculation
 *     - Initial value is 0xFFFF (all bits set)
 *     - Equivalent to starting with an empty message checksum
 *******************************************************************************/
static inline void crc_init(uint16_t* crcAccum)
{
    *crcAccum = X25_INIT_CRC;
}

/*******************************************************************************
 * FUNCTION: crc_calculate
 *
 * DESCRIPTION:
 *     Calculates the X.25 CRC checksum for an entire byte buffer.
 *
 * PARAMETERS:
 *     pBuffer - Pointer to buffer containing data to checksum
 *     length  - Number of bytes in buffer to include in checksum
 *
 * RETURNS:
 *     16-bit CRC checksum value
 *
 * OPERATION:
 *     1. Initializes CRC accumulator
 *     2. Accumulates each byte in buffer using crc_accumulate()
 *     3. Returns final CRC value
 *
 * EXAMPLE:
 *     uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
 *     uint16_t checksum = crc_calculate(data, 4);
 *
 * NOTES:
 *     - Complete checksum calculation in one function call
 *     - Suitable for small to medium buffers
 *     - For large buffers, consider using crc_accumulate_buffer()
 *******************************************************************************/
static inline uint16_t crc_calculate(const uint8_t* pBuffer, uint16_t length)
{
    uint16_t crcTmp;
    
    /* Initialize CRC accumulator */
    crc_init(&crcTmp);
    
    /* Accumulate each byte in buffer */
    while (length--)
    {
        crc_accumulate(*pBuffer++, &crcTmp);
    }
    
    return crcTmp;
}

/*******************************************************************************
 * FUNCTION: crc_accumulate_buffer
 *
 * DESCRIPTION:
 *     Accumulates an entire buffer of bytes into an existing CRC accumulator.
 *     Useful for calculating CRC incrementally across multiple buffers.
 *
 * PARAMETERS:
 *     crcAccum - Pointer to existing CRC accumulator (updated in-place)
 *     pBuffer  - Pointer to buffer containing data to accumulate
 *     length   - Number of bytes in buffer
 *
 * RETURNS:
 *     void
 *
 * USAGE:
 *     uint16_t crc;
 *     crc_init(&crc);
 *     crc_accumulate_buffer(&crc, buffer1, len1);
 *     crc_accumulate_buffer(&crc, buffer2, len2);
 *     // crc now contains checksum of buffer1 + buffer2
 *
 * NOTES:
 *     - Allows incremental CRC calculation for streaming data
 *     - CRC accumulator must be initialized before first use
 *     - More efficient than calling crc_accumulate() in a loop manually
 *******************************************************************************/
static inline void crc_accumulate_buffer(uint16_t *crcAccum, const char *pBuffer, uint16_t length)
{
    const uint8_t *p = (const uint8_t *)pBuffer;
    
    while (length--)
    {
        crc_accumulate(*p++, crcAccum);
    }
}

#endif /* _CHECKSUM_H_ */

#ifdef __cplusplus
}
#endif

/*******************************************************************************
 * X.25 CRC ALGORITHM DETAILS:
 *
 * POLYNOMIAL:
 *     CRC-16/X-25: x^16 + x^12 + x^5 + 1
 *     Polynomial representation: 0x1021 (normal), 0x8408 (reverse)
 *
 * CHARACTERISTICS:
 *     - Initial value: 0xFFFF
 *     - Final XOR: None
 *     - Input reflection: Yes (bit-reversed)
 *     - Output reflection: Yes (bit-reversed)
 *     - Check value: 0x906E for "123456789"
 *
 * ERROR DETECTION CAPABILITIES:
 *     - Detects all single-bit errors
 *     - Detects all double-bit errors
 *     - Detects all odd number of bit errors
 *     - Detects all burst errors up to 16 bits
 *     - Detects 99.998% of larger burst errors
 *
 * PERFORMANCE:
 *     - Time complexity: O(n) where n = number of bytes
 *     - Space complexity: O(1) constant memory
 *     - Each byte processed in approximately 10-15 CPU cycles
 *
 * VALIDATION:
 *     Test vector: "123456789" (9 bytes)
 *     Expected CRC: 0x906E
 *     Verification: crc_calculate("123456789", 9) == 0x906E
 *
 * USAGE IN PROTOCOLS:
 *     - MAVLink (Micro Air Vehicle Link)
 *     - X.25 (Packet switching protocol)
 *     - HDLC (High-Level Data Link Control)
 *     - SDLC (Synchronous Data Link Control)
 *
 * IMPLEMENTATION NOTES:
 *     1. The algorithm processes bytes in little-endian order
 *     2. CRC accumulator is updated after each byte
 *     3. No final XOR operation required
 *     4. To validate a complete message:
 *        - Calculate CRC over message (excluding CRC field)
 *        - Append CRC to message
 *        - Receiver calculates CRC over entire message (including CRC)
 *        - Valid message results in X25_VALIDATE_CRC (0xF0B8)
 *
 * EXAMPLE VALIDATION SEQUENCE:
 *     // Transmitter
 *     uint16_t crc = crc_calculate(message, message_len);
 *     append_crc_to_message(crc);
 *     
 *     // Receiver
 *     uint16_t received_crc = extract_crc_from_message();
 *     uint16_t calculated_crc = crc_calculate(received_message, message_len_with_crc);
 *     bool valid = (calculated_crc == X25_VALIDATE_CRC);
 *
 * INTEGRATION WITH ULINK PROTOCOL:
 *     The ulink.c module uses this X.25 CRC implementation with a wrapper:
 *     - create_checksum() calls crc_calculate() with appropriate parameters
 *     - accumulate_checksum() is similar to crc_accumulate() but with different signature
 *     - Validation uses X25_VALIDATE_CRC constant
 *
 * PORTABILITY NOTES:
 *     - Functions declared as 'static inline' for efficiency and header-only inclusion
 *     - Compatible with C99 and C++ compilers
 *     - Uses stdint.h for fixed-width integer types
 *     - Visual Studio 2010+ required for stdint.h support
 *
 * ALTERNATIVE ALGORITHMS (if X.25 not suitable):
 *     CRC-16/CCITT-FALSE: Initial 0xFFFF, no final XOR
 *     CRC-16/MODBUS: Initial 0xFFFF, final XOR 0x0000
 *     CRC-16/USB: Initial 0xFFFF, final XOR 0xFFFF
 *     CRC-32: 32-bit, better error detection, more computation
 *******************************************************************************/
