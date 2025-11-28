#ifndef MAVLINK_UART_H
#define MAVLINK_UART_H


#include "mavlink_bridge.h"
#include "mavlink/v1.0/mavlink_types.h"

// Gumstix variables
extern char uart_msg_ind_in;			// Variable to index Gumstix_Message_in
extern char uart_msg_ind_out;		// Variable to index Gumstix_Message_out
extern uint16_t uart_rx_buffer[MAVLINK_MAX_PACKET_LEN];
extern uint16_t uart_tx_buffer[MAVLINK_MAX_PACKET_LEN];



// Rx/Tx UART functions
void uart_write ();
int uart_read();

#endif  // MAVLINK_UART_H
