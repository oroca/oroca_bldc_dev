#pragma once
// MESSAGE SET_ENCODERMODE PACKING

#define MAVLINK_MSG_ID_SET_ENCODERMODE 222

MAVPACKED(
typedef struct __mavlink_set_encodermode_t {
 uint8_t resp; /*< 0:No Resp, 1:Resp*/
 uint8_t encoderMode; /*< 0:none 1:ABI 2:AS50XX 3:AJALL */
}) mavlink_set_encodermode_t;

#define MAVLINK_MSG_ID_SET_ENCODERMODE_LEN 2
#define MAVLINK_MSG_ID_SET_ENCODERMODE_MIN_LEN 2
#define MAVLINK_MSG_ID_222_LEN 2
#define MAVLINK_MSG_ID_222_MIN_LEN 2

#define MAVLINK_MSG_ID_SET_ENCODERMODE_CRC 21
#define MAVLINK_MSG_ID_222_CRC 21



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SET_ENCODERMODE { \
    222, \
    "SET_ENCODERMODE", \
    2, \
    {  { "resp", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_set_encodermode_t, resp) }, \
         { "encoderMode", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_set_encodermode_t, encoderMode) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SET_ENCODERMODE { \
    "SET_ENCODERMODE", \
    2, \
    {  { "resp", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_set_encodermode_t, resp) }, \
         { "encoderMode", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_set_encodermode_t, encoderMode) }, \
         } \
}
#endif

/**
 * @brief Pack a set_encodermode message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param resp 0:No Resp, 1:Resp
 * @param encoderMode 0:none 1:ABI 2:AS50XX 3:AJALL 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_encodermode_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t resp, uint8_t encoderMode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_ENCODERMODE_LEN];
    _mav_put_uint8_t(buf, 0, resp);
    _mav_put_uint8_t(buf, 1, encoderMode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_ENCODERMODE_LEN);
#else
    mavlink_set_encodermode_t packet;
    packet.resp = resp;
    packet.encoderMode = encoderMode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_ENCODERMODE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_ENCODERMODE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_ENCODERMODE_MIN_LEN, MAVLINK_MSG_ID_SET_ENCODERMODE_LEN, MAVLINK_MSG_ID_SET_ENCODERMODE_CRC);
}

/**
 * @brief Pack a set_encodermode message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param resp 0:No Resp, 1:Resp
 * @param encoderMode 0:none 1:ABI 2:AS50XX 3:AJALL 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_encodermode_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t resp,uint8_t encoderMode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_ENCODERMODE_LEN];
    _mav_put_uint8_t(buf, 0, resp);
    _mav_put_uint8_t(buf, 1, encoderMode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_ENCODERMODE_LEN);
#else
    mavlink_set_encodermode_t packet;
    packet.resp = resp;
    packet.encoderMode = encoderMode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_ENCODERMODE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_ENCODERMODE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_ENCODERMODE_MIN_LEN, MAVLINK_MSG_ID_SET_ENCODERMODE_LEN, MAVLINK_MSG_ID_SET_ENCODERMODE_CRC);
}

/**
 * @brief Encode a set_encodermode struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_encodermode C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_encodermode_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_encodermode_t* set_encodermode)
{
    return mavlink_msg_set_encodermode_pack(system_id, component_id, msg, set_encodermode->resp, set_encodermode->encoderMode);
}

/**
 * @brief Encode a set_encodermode struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_encodermode C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_encodermode_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_encodermode_t* set_encodermode)
{
    return mavlink_msg_set_encodermode_pack_chan(system_id, component_id, chan, msg, set_encodermode->resp, set_encodermode->encoderMode);
}

/**
 * @brief Send a set_encodermode message
 * @param chan MAVLink channel to send the message
 *
 * @param resp 0:No Resp, 1:Resp
 * @param encoderMode 0:none 1:ABI 2:AS50XX 3:AJALL 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_encodermode_send(mavlink_channel_t chan, uint8_t resp, uint8_t encoderMode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_ENCODERMODE_LEN];
    _mav_put_uint8_t(buf, 0, resp);
    _mav_put_uint8_t(buf, 1, encoderMode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ENCODERMODE, buf, MAVLINK_MSG_ID_SET_ENCODERMODE_MIN_LEN, MAVLINK_MSG_ID_SET_ENCODERMODE_LEN, MAVLINK_MSG_ID_SET_ENCODERMODE_CRC);
#else
    mavlink_set_encodermode_t packet;
    packet.resp = resp;
    packet.encoderMode = encoderMode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ENCODERMODE, (const char *)&packet, MAVLINK_MSG_ID_SET_ENCODERMODE_MIN_LEN, MAVLINK_MSG_ID_SET_ENCODERMODE_LEN, MAVLINK_MSG_ID_SET_ENCODERMODE_CRC);
#endif
}

/**
 * @brief Send a set_encodermode message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_set_encodermode_send_struct(mavlink_channel_t chan, const mavlink_set_encodermode_t* set_encodermode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_set_encodermode_send(chan, set_encodermode->resp, set_encodermode->encoderMode);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ENCODERMODE, (const char *)set_encodermode, MAVLINK_MSG_ID_SET_ENCODERMODE_MIN_LEN, MAVLINK_MSG_ID_SET_ENCODERMODE_LEN, MAVLINK_MSG_ID_SET_ENCODERMODE_CRC);
#endif
}

#if MAVLINK_MSG_ID_SET_ENCODERMODE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_encodermode_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t resp, uint8_t encoderMode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, resp);
    _mav_put_uint8_t(buf, 1, encoderMode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ENCODERMODE, buf, MAVLINK_MSG_ID_SET_ENCODERMODE_MIN_LEN, MAVLINK_MSG_ID_SET_ENCODERMODE_LEN, MAVLINK_MSG_ID_SET_ENCODERMODE_CRC);
#else
    mavlink_set_encodermode_t *packet = (mavlink_set_encodermode_t *)msgbuf;
    packet->resp = resp;
    packet->encoderMode = encoderMode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ENCODERMODE, (const char *)packet, MAVLINK_MSG_ID_SET_ENCODERMODE_MIN_LEN, MAVLINK_MSG_ID_SET_ENCODERMODE_LEN, MAVLINK_MSG_ID_SET_ENCODERMODE_CRC);
#endif
}
#endif

#endif

// MESSAGE SET_ENCODERMODE UNPACKING


/**
 * @brief Get field resp from set_encodermode message
 *
 * @return 0:No Resp, 1:Resp
 */
static inline uint8_t mavlink_msg_set_encodermode_get_resp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field encoderMode from set_encodermode message
 *
 * @return 0:none 1:ABI 2:AS50XX 3:AJALL 
 */
static inline uint8_t mavlink_msg_set_encodermode_get_encoderMode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a set_encodermode message into a struct
 *
 * @param msg The message to decode
 * @param set_encodermode C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_encodermode_decode(const mavlink_message_t* msg, mavlink_set_encodermode_t* set_encodermode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    set_encodermode->resp = mavlink_msg_set_encodermode_get_resp(msg);
    set_encodermode->encoderMode = mavlink_msg_set_encodermode_get_encoderMode(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SET_ENCODERMODE_LEN? msg->len : MAVLINK_MSG_ID_SET_ENCODERMODE_LEN;
        memset(set_encodermode, 0, MAVLINK_MSG_ID_SET_ENCODERMODE_LEN);
    memcpy(set_encodermode, _MAV_PAYLOAD(msg), len);
#endif
}
