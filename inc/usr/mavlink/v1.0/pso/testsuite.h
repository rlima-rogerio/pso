/** @file
 *	@brief MAVLink comm protocol testsuite generated from pso.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef PSO_TESTSUITE_H
#define PSO_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_pso(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_common(system_id, component_id, last_msg);
	mavlink_test_pso(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_pso_data(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_pso_data_t packet_in = {
		17235,17339,17443,17547,{ 17651, 17652, 17653 }
    };
	mavlink_pso_data_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.rpm = packet_in.rpm;
        	packet1.v_motor = packet_in.v_motor;
        	packet1.i_motor = packet_in.i_motor;
        	packet1.thrust = packet_in.thrust;
        
        	mav_array_memcpy(packet1.accel, packet_in.accel, sizeof(int16_t)*3);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pso_data_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_pso_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pso_data_pack(system_id, component_id, &msg , packet1.rpm , packet1.v_motor , packet1.i_motor , packet1.thrust , packet1.accel );
	mavlink_msg_pso_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pso_data_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.rpm , packet1.v_motor , packet1.i_motor , packet1.thrust , packet1.accel );
	mavlink_msg_pso_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_pso_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pso_data_send(MAVLINK_COMM_1 , packet1.rpm , packet1.v_motor , packet1.i_motor , packet1.thrust , packet1.accel );
	mavlink_msg_pso_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_pso_command(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_pso_command_t packet_in = {
		17235,17339,17443
    };
	mavlink_pso_command_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.state = packet_in.state;
        	packet1.throttle = packet_in.throttle;
        	packet1.debug = packet_in.debug;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pso_command_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_pso_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pso_command_pack(system_id, component_id, &msg , packet1.state , packet1.throttle , packet1.debug );
	mavlink_msg_pso_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pso_command_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.state , packet1.throttle , packet1.debug );
	mavlink_msg_pso_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_pso_command_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pso_command_send(MAVLINK_COMM_1 , packet1.state , packet1.throttle , packet1.debug );
	mavlink_msg_pso_command_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_pso(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_pso_data(system_id, component_id, last_msg);
	mavlink_test_pso_command(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // PSO_TESTSUITE_H
