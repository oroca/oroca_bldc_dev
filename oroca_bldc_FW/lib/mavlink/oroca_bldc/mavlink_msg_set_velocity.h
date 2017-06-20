// MESSAGE SET_VELOCITY PACKING

#define MAVLINK_MSG_ID_SET_VELOCITY 222

typedef struct MAVLINK_PACKED __mavlink_set_velocity_t
{
 uint16_t ang_vel; /*< arg 1*/
 uint8_t set_Velocity; /*< set cmd 2*/
} mavlink_set_velocity_t;

#define MAVLINK_MSG_ID_SET_VELOCITY_LEN 3
#define MAVLINK_MSG_ID_SET_VELOCITY_MIN_LEN 3
#define MAVLINK_MSG_ID_222_LEN 3
#define MAVLINK_MSG_ID_222_MIN_LEN 3

#define MAVLINK_MSG_ID_SET_VELOCITY_CRC 148
#define MAVLINK_MSG_ID_222_CRC 148



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SET_VELOCITY { \
	222, \
	"SET_VELOCITY", \
	2, \
	{  { "ang_vel", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_set_velocity_t, ang_vel) }, \
         { "set_Velocity", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_set_velocity_t, set_Velocity) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SET_VELOCITY { \
	"SET_VELOCITY", \
	2, \
	{  { "ang_vel", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_set_velocity_t, ang_vel) }, \
         { "set_Velocity", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_set_velocity_t, set_Velocity) }, \
         } \
}
#endif

/**
 * @brief Pack a set_velocity message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param set_Velocity set cmd 2
 * @param ang_vel arg 1
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_velocity_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t set_Velocity, uint16_t ang_vel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_VELOCITY_LEN];
	_mav_put_uint16_t(buf, 0, ang_vel);
	_mav_put_uint8_t(buf, 2, set_Velocity);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_VELOCITY_LEN);
#else
	mavlink_set_velocity_t packet;
	packet.ang_vel = ang_vel;
	packet.set_Velocity = set_Velocity;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_VELOCITY_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_VELOCITY;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_VELOCITY_MIN_LEN, MAVLINK_MSG_ID_SET_VELOCITY_LEN, MAVLINK_MSG_ID_SET_VELOCITY_CRC);
}

/**
 * @brief Pack a set_velocity message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_Velocity set cmd 2
 * @param ang_vel arg 1
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_velocity_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t set_Velocity,uint16_t ang_vel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_VELOCITY_LEN];
	_mav_put_uint16_t(buf, 0, ang_vel);
	_mav_put_uint8_t(buf, 2, set_Velocity);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_VELOCITY_LEN);
#else
	mavlink_set_velocity_t packet;
	packet.ang_vel = ang_vel;
	packet.set_Velocity = set_Velocity;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_VELOCITY_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_VELOCITY;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_VELOCITY_MIN_LEN, MAVLINK_MSG_ID_SET_VELOCITY_LEN, MAVLINK_MSG_ID_SET_VELOCITY_CRC);
}

/**
 * @brief Encode a set_velocity struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_velocity C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_velocity_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_velocity_t* set_velocity)
{
	return mavlink_msg_set_velocity_pack(system_id, component_id, msg, set_velocity->set_Velocity, set_velocity->ang_vel);
}

/**
 * @brief Encode a set_velocity struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_velocity C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_velocity_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_velocity_t* set_velocity)
{
	return mavlink_msg_set_velocity_pack_chan(system_id, component_id, chan, msg, set_velocity->set_Velocity, set_velocity->ang_vel);
}

/**
 * @brief Send a set_velocity message
 * @param chan MAVLink channel to send the message
 *
 * @param set_Velocity set cmd 2
 * @param ang_vel arg 1
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_velocity_send(mavlink_channel_t chan, uint8_t set_Velocity, uint16_t ang_vel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_VELOCITY_LEN];
	_mav_put_uint16_t(buf, 0, ang_vel);
	_mav_put_uint8_t(buf, 2, set_Velocity);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_VELOCITY, buf, MAVLINK_MSG_ID_SET_VELOCITY_MIN_LEN, MAVLINK_MSG_ID_SET_VELOCITY_LEN, MAVLINK_MSG_ID_SET_VELOCITY_CRC);
#else
	mavlink_set_velocity_t packet;
	packet.ang_vel = ang_vel;
	packet.set_Velocity = set_Velocity;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_VELOCITY, (const char *)&packet, MAVLINK_MSG_ID_SET_VELOCITY_MIN_LEN, MAVLINK_MSG_ID_SET_VELOCITY_LEN, MAVLINK_MSG_ID_SET_VELOCITY_CRC);
#endif
}

/**
 * @brief Send a set_velocity message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_set_velocity_send_struct(mavlink_channel_t chan, const mavlink_set_velocity_t* set_velocity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_set_velocity_send(chan, set_velocity->set_Velocity, set_velocity->ang_vel);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_VELOCITY, (const char *)set_velocity, MAVLINK_MSG_ID_SET_VELOCITY_MIN_LEN, MAVLINK_MSG_ID_SET_VELOCITY_LEN, MAVLINK_MSG_ID_SET_VELOCITY_CRC);
#endif
}

#if MAVLINK_MSG_ID_SET_VELOCITY_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_velocity_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t set_Velocity, uint16_t ang_vel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint16_t(buf, 0, ang_vel);
	_mav_put_uint8_t(buf, 2, set_Velocity);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_VELOCITY, buf, MAVLINK_MSG_ID_SET_VELOCITY_MIN_LEN, MAVLINK_MSG_ID_SET_VELOCITY_LEN, MAVLINK_MSG_ID_SET_VELOCITY_CRC);
#else
	mavlink_set_velocity_t *packet = (mavlink_set_velocity_t *)msgbuf;
	packet->ang_vel = ang_vel;
	packet->set_Velocity = set_Velocity;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_VELOCITY, (const char *)packet, MAVLINK_MSG_ID_SET_VELOCITY_MIN_LEN, MAVLINK_MSG_ID_SET_VELOCITY_LEN, MAVLINK_MSG_ID_SET_VELOCITY_CRC);
#endif
}
#endif

#endif

// MESSAGE SET_VELOCITY UNPACKING


/**
 * @brief Get field set_Velocity from set_velocity message
 *
 * @return set cmd 2
 */
static inline uint8_t mavlink_msg_set_velocity_get_set_Velocity(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field ang_vel from set_velocity message
 *
 * @return arg 1
 */
static inline uint16_t mavlink_msg_set_velocity_get_ang_vel(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Decode a set_velocity message into a struct
 *
 * @param msg The message to decode
 * @param set_velocity C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_velocity_decode(const mavlink_message_t* msg, mavlink_set_velocity_t* set_velocity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	set_velocity->ang_vel = mavlink_msg_set_velocity_get_ang_vel(msg);
	set_velocity->set_Velocity = mavlink_msg_set_velocity_get_set_Velocity(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SET_VELOCITY_LEN? msg->len : MAVLINK_MSG_ID_SET_VELOCITY_LEN;
        memset(set_velocity, 0, MAVLINK_MSG_ID_SET_VELOCITY_LEN);
	memcpy(set_velocity, _MAV_PAYLOAD(msg), len);
#endif
}
