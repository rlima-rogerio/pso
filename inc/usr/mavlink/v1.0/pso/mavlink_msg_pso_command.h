// MESSAGE PSO_COMMAND PACKING

#define MAVLINK_MSG_ID_PSO_COMMAND 151

typedef struct __mavlink_pso_command_t
{
 uint16_t state; /*< Aileron command, 0 to 65535*/
 uint16_t throttle; /*< Throttle command, 0 to 65535*/
 uint16_t debug; /*< Debug command, 0 to 65535*/
} mavlink_pso_command_t;

#define MAVLINK_MSG_ID_PSO_COMMAND_LEN 6
#define MAVLINK_MSG_ID_151_LEN 6

#define MAVLINK_MSG_ID_PSO_COMMAND_CRC 141
#define MAVLINK_MSG_ID_151_CRC 141



#define MAVLINK_MESSAGE_INFO_PSO_COMMAND { \
	"PSO_COMMAND", \
	3, \
	{  { "state", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_pso_command_t, state) }, \
         { "throttle", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_pso_command_t, throttle) }, \
         { "debug", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_pso_command_t, debug) }, \
         } \
}


/**
 * @brief Pack a pso_command message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param state Aileron command, 0 to 65535
 * @param throttle Throttle command, 0 to 65535
 * @param debug Debug command, 0 to 65535
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pso_command_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t state, uint16_t throttle, uint16_t debug)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PSO_COMMAND_LEN];
	_mav_put_uint16_t(buf, 0, state);
	_mav_put_uint16_t(buf, 2, throttle);
	_mav_put_uint16_t(buf, 4, debug);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PSO_COMMAND_LEN);
#else
	mavlink_pso_command_t packet;
	packet.state = state;
	packet.throttle = throttle;
	packet.debug = debug;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PSO_COMMAND_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PSO_COMMAND;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PSO_COMMAND_LEN, MAVLINK_MSG_ID_PSO_COMMAND_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PSO_COMMAND_LEN);
#endif
}

/**
 * @brief Pack a pso_command message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param state Aileron command, 0 to 65535
 * @param throttle Throttle command, 0 to 65535
 * @param debug Debug command, 0 to 65535
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pso_command_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t state,uint16_t throttle,uint16_t debug)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PSO_COMMAND_LEN];
	_mav_put_uint16_t(buf, 0, state);
	_mav_put_uint16_t(buf, 2, throttle);
	_mav_put_uint16_t(buf, 4, debug);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PSO_COMMAND_LEN);
#else
	mavlink_pso_command_t packet;
	packet.state = state;
	packet.throttle = throttle;
	packet.debug = debug;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PSO_COMMAND_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PSO_COMMAND;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PSO_COMMAND_LEN, MAVLINK_MSG_ID_PSO_COMMAND_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PSO_COMMAND_LEN);
#endif
}

/**
 * @brief Encode a pso_command struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param pso_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pso_command_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_pso_command_t* pso_command)
{
	return mavlink_msg_pso_command_pack(system_id, component_id, msg, pso_command->state, pso_command->throttle, pso_command->debug);
}

/**
 * @brief Encode a pso_command struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pso_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pso_command_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_pso_command_t* pso_command)
{
	return mavlink_msg_pso_command_pack_chan(system_id, component_id, chan, msg, pso_command->state, pso_command->throttle, pso_command->debug);
}

/**
 * @brief Send a pso_command message
 * @param chan MAVLink channel to send the message
 *
 * @param state Aileron command, 0 to 65535
 * @param throttle Throttle command, 0 to 65535
 * @param debug Debug command, 0 to 65535
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_pso_command_send(mavlink_channel_t chan, uint16_t state, uint16_t throttle, uint16_t debug)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PSO_COMMAND_LEN];
	_mav_put_uint16_t(buf, 0, state);
	_mav_put_uint16_t(buf, 2, throttle);
	_mav_put_uint16_t(buf, 4, debug);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PSO_COMMAND, buf, MAVLINK_MSG_ID_PSO_COMMAND_LEN, MAVLINK_MSG_ID_PSO_COMMAND_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PSO_COMMAND, buf, MAVLINK_MSG_ID_PSO_COMMAND_LEN);
#endif
#else
	mavlink_pso_command_t packet;
	packet.state = state;
	packet.throttle = throttle;
	packet.debug = debug;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PSO_COMMAND, (const char *)&packet, MAVLINK_MSG_ID_PSO_COMMAND_LEN, MAVLINK_MSG_ID_PSO_COMMAND_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PSO_COMMAND, (const char *)&packet, MAVLINK_MSG_ID_PSO_COMMAND_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_PSO_COMMAND_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_pso_command_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t state, uint16_t throttle, uint16_t debug)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint16_t(buf, 0, state);
	_mav_put_uint16_t(buf, 2, throttle);
	_mav_put_uint16_t(buf, 4, debug);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PSO_COMMAND, buf, MAVLINK_MSG_ID_PSO_COMMAND_LEN, MAVLINK_MSG_ID_PSO_COMMAND_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PSO_COMMAND, buf, MAVLINK_MSG_ID_PSO_COMMAND_LEN);
#endif
#else
	mavlink_pso_command_t *packet = (mavlink_pso_command_t *)msgbuf;
	packet->state = state;
	packet->throttle = throttle;
	packet->debug = debug;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PSO_COMMAND, (const char *)packet, MAVLINK_MSG_ID_PSO_COMMAND_LEN, MAVLINK_MSG_ID_PSO_COMMAND_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PSO_COMMAND, (const char *)packet, MAVLINK_MSG_ID_PSO_COMMAND_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE PSO_COMMAND UNPACKING


/**
 * @brief Get field state from pso_command message
 *
 * @return Aileron command, 0 to 65535
 */
static inline uint16_t mavlink_msg_pso_command_get_state(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field throttle from pso_command message
 *
 * @return Throttle command, 0 to 65535
 */
static inline uint16_t mavlink_msg_pso_command_get_throttle(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field debug from pso_command message
 *
 * @return Debug command, 0 to 65535
 */
static inline uint16_t mavlink_msg_pso_command_get_debug(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Decode a pso_command message into a struct
 *
 * @param msg The message to decode
 * @param pso_command C-struct to decode the message contents into
 */
static inline void mavlink_msg_pso_command_decode(const mavlink_message_t* msg, mavlink_pso_command_t* pso_command)
{
#if MAVLINK_NEED_BYTE_SWAP
	pso_command->state = mavlink_msg_pso_command_get_state(msg);
	pso_command->throttle = mavlink_msg_pso_command_get_throttle(msg);
	pso_command->debug = mavlink_msg_pso_command_get_debug(msg);
#else
	memcpy(pso_command, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_PSO_COMMAND_LEN);
#endif
}
