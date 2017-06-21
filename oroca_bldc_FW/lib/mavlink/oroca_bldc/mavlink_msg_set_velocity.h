#pragma once
// MESSAGE SET_VELOCITY PACKING

#define MAVLINK_MSG_ID_SET_VELOCITY 220

MAVPACKED(
typedef struct __mavlink_set_velocity_t {
 uint16_t ref_angular_velocity; /*< velocity value*/
}) mavlink_set_velocity_t;

#define MAVLINK_MSG_ID_SET_VELOCITY_LEN 2
#define MAVLINK_MSG_ID_SET_VELOCITY_MIN_LEN 2
#define MAVLINK_MSG_ID_220_LEN 2
#define MAVLINK_MSG_ID_220_MIN_LEN 2

#define MAVLINK_MSG_ID_SET_VELOCITY_CRC 100
#define MAVLINK_MSG_ID_220_CRC 100



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SET_VELOCITY { \
    220, \
    "SET_VELOCITY", \
    1, \
    {  { "ref_angular_velocity", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_set_velocity_t, ref_angular_velocity) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SET_VELOCITY { \
    "SET_VELOCITY", \
    1, \
    {  { "ref_angular_velocity", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_set_velocity_t, ref_angular_velocity) }, \
         } \
}
#endif

/**
 * @brief Pack a set_velocity message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param ref_angular_velocity velocity value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_velocity_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t ref_angular_velocity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_VELOCITY_LEN];
    _mav_put_uint16_t(buf, 0, ref_angular_velocity);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_VELOCITY_LEN);
#else
    mavlink_set_velocity_t packet;
    packet.ref_angular_velocity = ref_angular_velocity;

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
 * @param ref_angular_velocity velocity value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_velocity_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t ref_angular_velocity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_VELOCITY_LEN];
    _mav_put_uint16_t(buf, 0, ref_angular_velocity);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_VELOCITY_LEN);
#else
    mavlink_set_velocity_t packet;
    packet.ref_angular_velocity = ref_angular_velocity;

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
    return mavlink_msg_set_velocity_pack(system_id, component_id, msg, set_velocity->ref_angular_velocity);
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
    return mavlink_msg_set_velocity_pack_chan(system_id, component_id, chan, msg, set_velocity->ref_angular_velocity);
}

/**
 * @brief Send a set_velocity message
 * @param chan MAVLink channel to send the message
 *
 * @param ref_angular_velocity velocity value
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_velocity_send(mavlink_channel_t chan, uint16_t ref_angular_velocity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_VELOCITY_LEN];
    _mav_put_uint16_t(buf, 0, ref_angular_velocity);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_VELOCITY, buf, MAVLINK_MSG_ID_SET_VELOCITY_MIN_LEN, MAVLINK_MSG_ID_SET_VELOCITY_LEN, MAVLINK_MSG_ID_SET_VELOCITY_CRC);
#else
    mavlink_set_velocity_t packet;
    packet.ref_angular_velocity = ref_angular_velocity;

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
    mavlink_msg_set_velocity_send(chan, set_velocity->ref_angular_velocity);
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
static inline void mavlink_msg_set_velocity_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t ref_angular_velocity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, ref_angular_velocity);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_VELOCITY, buf, MAVLINK_MSG_ID_SET_VELOCITY_MIN_LEN, MAVLINK_MSG_ID_SET_VELOCITY_LEN, MAVLINK_MSG_ID_SET_VELOCITY_CRC);
#else
    mavlink_set_velocity_t *packet = (mavlink_set_velocity_t *)msgbuf;
    packet->ref_angular_velocity = ref_angular_velocity;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_VELOCITY, (const char *)packet, MAVLINK_MSG_ID_SET_VELOCITY_MIN_LEN, MAVLINK_MSG_ID_SET_VELOCITY_LEN, MAVLINK_MSG_ID_SET_VELOCITY_CRC);
#endif
}
#endif

#endif

// MESSAGE SET_VELOCITY UNPACKING


/**
 * @brief Get field ref_angular_velocity from set_velocity message
 *
 * @return velocity value
 */
static inline uint16_t mavlink_msg_set_velocity_get_ref_angular_velocity(const mavlink_message_t* msg)
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
    set_velocity->ref_angular_velocity = mavlink_msg_set_velocity_get_ref_angular_velocity(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SET_VELOCITY_LEN? msg->len : MAVLINK_MSG_ID_SET_VELOCITY_LEN;
        memset(set_velocity, 0, MAVLINK_MSG_ID_SET_VELOCITY_LEN);
    memcpy(set_velocity, _MAV_PAYLOAD(msg), len);
#endif
}
