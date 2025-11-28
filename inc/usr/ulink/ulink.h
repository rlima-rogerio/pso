#ifndef ULINK_H
#define ULINK_H

/** @file
 * ulink adapter header.
 * Include this file instead of "ulink.h".
 */

#include <stdint.h>
#include "ulink_types.h"
#include "ulink_pso.h"

/** @file
 *	@brief ULINK comm protocol built from pso.xml
 *	@see http://ULINK.org
 */

#ifndef ULINK_STX
#define ULINK_STX 254
#endif

#ifndef ULINK_ENDIAN
#define ULINK_ENDIAN ULINK_LITTLE_ENDIAN
#endif

#ifndef ULINK_ALIGNED_FIELDS
#define ULINK_ALIGNED_FIELDS 1
#endif

#ifndef ULINK_CRC_EXTRA
#define ULINK_CRC_EXTRA 1
#endif

extern ulink_system_t ulink_system;

extern char uart_msg_ind_in;			// Variable to index Gumstix_Message_in
extern char uart_msg_ind_out;		// Variable to index Gumstix_Message_out
extern uint16_t uart_rx_buffer[ULINK_MAX_PACKET_LEN];
extern uint16_t uart_tx_buffer[ULINK_MAX_PACKET_LEN];


void create_message(uint8_t system_id, uint8_t component_id, uint16_t* msg, const ulink_pso_data_t* pso_data);
uint16_t create_checksum(uint16_t* msg, uint16_t length);
void accumulate_checksum(uint8_t data, uint16_t *crcAccum);
uint8_t parse_message(uint16_t *msg, ulink_pso_command_t *command);

// Rx/Tx UART functions
void uart_write ( );
int uart_read();

uint8_t packet_data (ulink_pso_data_t* dp);
void copy_data (uint16_t* uart_tx_buf, ulink_pso_data_t* dp);

#endif  // ULINK_H
