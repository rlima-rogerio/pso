#ifndef PDVA__MAVLINK_BRIDGE_H
#define PDVA__MAVLINK_BRIDGE_H

/** @file
 * MAVLink adapter header.
 * Include this file instead of "mavlink.h".
 */



/*** Fix some weird bugs ***/
#define HAVE_ENUM_MAV_SYS_STATUS_SENSOR
float atan2f(float x, float y);
/***************************/

#define MAVLINK_COMM_NUM_BUFFERS 1
//#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#define GUMSTIX_COMM_CHANNEL MAVLINK_COMM_1

#include "mavlink_uart.h"
#include "mavlink/v1.0/mavlink_types.h"

extern mavlink_system_t mavlink_system;

#include "mavlink/v1.0/pso/mavlink.h"


//#include "mavlink/v1.0/mavlink_helpers.h"


void create_message(uint8_t system_id, uint8_t component_id, uint8_t chan, uint16_t* msg, const mavlink_pso_data_t* pso_data);
uint16_t create_checksum(uint16_t* msg, uint16_t length);
void accumulate_checksum(uint8_t data, uint16_t *crcAccum);
uint8_t parse_message(uint16_t *msg, mavlink_pso_command_t *command);

#endif // not PDVA__MAVLINK_BRIDGE_H
