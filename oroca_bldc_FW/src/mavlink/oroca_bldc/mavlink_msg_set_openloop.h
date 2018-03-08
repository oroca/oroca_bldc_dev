#pragma once
// MESSAGE SET_OPENLOOP PACKING

#define MAVLINK_MSG_ID_SET_OPENLOOP 221

MAVPACKED(
typedef struct __mavlink_set_openloop_t {
 uint8_t resp; /*< 0:No Resp, 1:Resp*/
 uint8_t openLoopMode; /*< 0:closedloop 1:openloop */
}) mavlink_set_openloop_t;

#define MAVLINK_MSG_ID_SET_OPENLOOP_LEN 2
#define MAVLINK_MSG_ID_SET_OPENLOOP_MIN_LEN 2
#define MAVLINK_MSG_ID_221_LEN 2
#define MAVLINK_MSG_ID_221_MIN_LEN 2

#define MAVLINK_MSG_ID_SET_OPENLOOP_CRC 84
#define MAVLINK_MSG_ID_221_CRC 84



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SET_OPENLOOP { \
    221, \
    "SET_OPENLOOP", \
    2, \
    {  { "resp", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_set_openloop_t, resp) }, \
         { "openLoopMode", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_set_openloop_t, openLoopMode) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SET_OPENLOOP { \
    "SET_OPENLOOP", \
    2, \
    {  { "resp", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_set_openloop_t, resp) }, \
         { "openLoopMode", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_set_openloop_t, openLoopMode) }, \
         } \
}
#endif

/**
 * @brief Pack a set_openloop message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param resp 0:No Resp, 1:Resp
 * @param openLoopMode 0:closedloop 1:openloop 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_openloop_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t resp, uint8_t openLoopMode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_OPENLOOP_LEN];
    _mav_put_uint8_t(buf, 0, resp);
    _mav_put_uint8_t(buf, 1, openLoopMode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_OPENLOOP_LEN);
#else
    mavlink_set_openloop_t packet;
    packet.resp = resp;
    packet.openLoopMode = openLoopMode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_OPENLOOP_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_OPENLOOP;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_OPENLOOP_MIN_LEN, MAVLINK_MSG_ID_SET_OPENLOOP_LEN, MAVLINK_MSG_ID_SET_OPENLOOP_CRC);
}

/**
 * @brief Pack a set_openloop message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param resp 0:No Resp, 1:Resp
 * @param openLoopMode 0:closedloop 1:openloop 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_openloop_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t resp,uint8_t openLoopMode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_OPENLOOP_LEN];
    _mav_put_uint8_t(buf, 0, resp);
    _mav_put_uint8_t(buf, 1, openLoopMode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_OPENLOOP_LEN);
#else
    mavlink_set_openloop_t packet;
    packet.resp = resp;
    packet.openLoopMode = openLoopMode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_OPENLOOP_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_OPENLOOP;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_OPENLOOP_MIN_LEN, MAVLINK_MSG_ID_SET_OPENLOOP_LEN, MAVLINK_MSG_ID_SET_OPENLOOP_CRC);
}

/**
 * @brief Encode a set_openloop struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_openloop C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_openloop_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_openloop_t* set_openloop)
{
    return mavlink_msg_set_openloop_pack(system_id, component_id, msg, set_openloop->resp, set_openloop->openLoopMode);
}

/**
 * @brief Encode a set_openloop struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_openloop C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_openloop_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_openloop_t* set_openloop)
{
    return mavlink_msg_set_openloop_pack_chan(system_id, component_id, chan, msg, set_openloop->resp, set_openloop->openLoopMode);
}

/**
 * @brief Send a set_openloop message
 * @param chan MAVLink channel to send the message
 *
 * @param resp 0:No Resp, 1:Resp
 * @param openLoopMode 0:closedloop 1:openloop 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_openloop_send(mavlink_channel_t chan, uint8_t resp, uint8_t openLoopMode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_OPENLOOP_LEN];
    _mav_put_uint8_t(buf, 0, resp);
    _mav_put_uint8_t(buf, 1, openLoopMode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_OPENLOOP, buf, MAVLINK_MSG_ID_SET_OPENLOOP_MIN_LEN, MAVLINK_MSG_ID_SET_OPENLOOP_LEN, MAVLINK_MSG_ID_SET_OPENLOOP_CRC);
#else
    mavlink_set_openloop_t packet;
    packet.resp = resp;
    packet.openLoopMode = openLoopMode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_OPENLOOP, (const char *)&packet, MAVLINK_MSG_ID_SET_OPENLOOP_MIN_LEN, MAVLINK_MSG_ID_SET_OPENLOOP_LEN, MAVLINK_MSG_ID_SET_OPENLOOP_CRC);
#endif
}

/**
 * @brief Send a set_openloop message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_set_openloop_send_struct(mavlink_channel_t chan, const mavlink_set_openloop_t* set_openloop)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_set_openloop_send(chan, set_openloop->resp, set_openloop->openLoopMode);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_OPENLOOP, (const char *)set_openloop, MAVLINK_MSG_ID_SET_OPENLOOP_MIN_LEN, MAVLINK_MSG_ID_SET_OPENLOOP_LEN, MAVLINK_MSG_ID_SET_OPENLOOP_CRC);
#endif
}

#if MAVLINK_MSG_ID_SET_OPENLOOP_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_openloop_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t resp, uint8_t openLoopMode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, resp);
    _mav_put_uint8_t(buf, 1, openLoopMode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_OPENLOOP, buf, MAVLINK_MSG_ID_SET_OPENLOOP_MIN_LEN, MAVLINK_MSG_ID_SET_OPENLOOP_LEN, MAVLINK_MSG_ID_SET_OPENLOOP_CRC);
#else
    mavlink_set_openloop_t *packet = (mavlink_set_openloop_t *)msgbuf;
    packet->resp = resp;
    packet->openLoopMode = openLoopMode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_OPENLOOP, (const char *)packet, MAVLINK_MSG_ID_SET_OPENLOOP_MIN_LEN, MAVLINK_MSG_ID_SET_OPENLOOP_LEN, MAVLINK_MSG_ID_SET_OPENLOOP_CRC);
#endif
}
#endif

#endif

// MESSAGE SET_OPENLOOP UNPACKING


/**
 * @brief Get field resp from set_openloop message
 *
 * @return 0:No Resp, 1:Resp
 */
static inline uint8_t mavlink_msg_set_openloop_get_resp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field openLoopMode from set_openloop message
 *
 * @return 0:closedloop 1:openloop 
 */
static inline uint8_t mavlink_msg_set_openloop_get_openLoopMode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a set_openloop message into a struct
 *
 * @param msg The message to decode
 * @param set_openloop C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_openloop_decode(const mavlink_message_t* msg, mavlink_set_openloop_t* set_openloop)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    set_openloop->resp = mavlink_msg_set_openloop_get_resp(msg);
    set_openloop->openLoopMode = mavlink_msg_set_openloop_get_openLoopMode(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SET_OPENLOOP_LEN? msg->len : MAVLINK_MSG_ID_SET_OPENLOOP_LEN;
        memset(set_openloop, 0, MAVLINK_MSG_ID_SET_OPENLOOP_LEN);
    memcpy(set_openloop, _MAV_PAYLOAD(msg), len);
#endif
}
