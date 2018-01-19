#pragma once
// MESSAGE DEBUG_STRING PACKING

#define MAVLINK_MSG_ID_DEBUG_STRING 0

MAVPACKED(
typedef struct __mavlink_debug_string_t {
 uint8_t resp; /*< 0:No Resp, 1:Resp*/
 char dbg_str[250]; /*< string*/
}) mavlink_debug_string_t;

#define MAVLINK_MSG_ID_DEBUG_STRING_LEN 251
#define MAVLINK_MSG_ID_DEBUG_STRING_MIN_LEN 251
#define MAVLINK_MSG_ID_0_LEN 251
#define MAVLINK_MSG_ID_0_MIN_LEN 251

#define MAVLINK_MSG_ID_DEBUG_STRING_CRC 163
#define MAVLINK_MSG_ID_0_CRC 163

#define MAVLINK_MSG_DEBUG_STRING_FIELD_DBG_STR_LEN 250

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DEBUG_STRING { \
    0, \
    "DEBUG_STRING", \
    2, \
    {  { "resp", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_debug_string_t, resp) }, \
         { "dbg_str", NULL, MAVLINK_TYPE_CHAR, 250, 1, offsetof(mavlink_debug_string_t, dbg_str) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DEBUG_STRING { \
    "DEBUG_STRING", \
    2, \
    {  { "resp", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_debug_string_t, resp) }, \
         { "dbg_str", NULL, MAVLINK_TYPE_CHAR, 250, 1, offsetof(mavlink_debug_string_t, dbg_str) }, \
         } \
}
#endif

/**
 * @brief Pack a debug_string message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param resp 0:No Resp, 1:Resp
 * @param dbg_str string
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_debug_string_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t resp, const char *dbg_str)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DEBUG_STRING_LEN];
    _mav_put_uint8_t(buf, 0, resp);
    _mav_put_char_array(buf, 1, dbg_str, 250);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DEBUG_STRING_LEN);
#else
    mavlink_debug_string_t packet;
    packet.resp = resp;
    mav_array_memcpy(packet.dbg_str, dbg_str, sizeof(char)*250);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DEBUG_STRING_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DEBUG_STRING;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DEBUG_STRING_MIN_LEN, MAVLINK_MSG_ID_DEBUG_STRING_LEN, MAVLINK_MSG_ID_DEBUG_STRING_CRC);
}

/**
 * @brief Pack a debug_string message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param resp 0:No Resp, 1:Resp
 * @param dbg_str string
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_debug_string_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t resp,const char *dbg_str)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DEBUG_STRING_LEN];
    _mav_put_uint8_t(buf, 0, resp);
    _mav_put_char_array(buf, 1, dbg_str, 250);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DEBUG_STRING_LEN);
#else
    mavlink_debug_string_t packet;
    packet.resp = resp;
    mav_array_memcpy(packet.dbg_str, dbg_str, sizeof(char)*250);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DEBUG_STRING_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DEBUG_STRING;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DEBUG_STRING_MIN_LEN, MAVLINK_MSG_ID_DEBUG_STRING_LEN, MAVLINK_MSG_ID_DEBUG_STRING_CRC);
}

/**
 * @brief Encode a debug_string struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param debug_string C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_debug_string_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_debug_string_t* debug_string)
{
    return mavlink_msg_debug_string_pack(system_id, component_id, msg, debug_string->resp, debug_string->dbg_str);
}

/**
 * @brief Encode a debug_string struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param debug_string C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_debug_string_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_debug_string_t* debug_string)
{
    return mavlink_msg_debug_string_pack_chan(system_id, component_id, chan, msg, debug_string->resp, debug_string->dbg_str);
}

/**
 * @brief Send a debug_string message
 * @param chan MAVLink channel to send the message
 *
 * @param resp 0:No Resp, 1:Resp
 * @param dbg_str string
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_debug_string_send(mavlink_channel_t chan, uint8_t resp, const char *dbg_str)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DEBUG_STRING_LEN];
    _mav_put_uint8_t(buf, 0, resp);
    _mav_put_char_array(buf, 1, dbg_str, 250);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG_STRING, buf, MAVLINK_MSG_ID_DEBUG_STRING_MIN_LEN, MAVLINK_MSG_ID_DEBUG_STRING_LEN, MAVLINK_MSG_ID_DEBUG_STRING_CRC);
#else
    mavlink_debug_string_t packet;
    packet.resp = resp;
    mav_array_memcpy(packet.dbg_str, dbg_str, sizeof(char)*250);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG_STRING, (const char *)&packet, MAVLINK_MSG_ID_DEBUG_STRING_MIN_LEN, MAVLINK_MSG_ID_DEBUG_STRING_LEN, MAVLINK_MSG_ID_DEBUG_STRING_CRC);
#endif
}

/**
 * @brief Send a debug_string message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_debug_string_send_struct(mavlink_channel_t chan, const mavlink_debug_string_t* debug_string)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_debug_string_send(chan, debug_string->resp, debug_string->dbg_str);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG_STRING, (const char *)debug_string, MAVLINK_MSG_ID_DEBUG_STRING_MIN_LEN, MAVLINK_MSG_ID_DEBUG_STRING_LEN, MAVLINK_MSG_ID_DEBUG_STRING_CRC);
#endif
}

#if MAVLINK_MSG_ID_DEBUG_STRING_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_debug_string_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t resp, const char *dbg_str)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, resp);
    _mav_put_char_array(buf, 1, dbg_str, 250);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG_STRING, buf, MAVLINK_MSG_ID_DEBUG_STRING_MIN_LEN, MAVLINK_MSG_ID_DEBUG_STRING_LEN, MAVLINK_MSG_ID_DEBUG_STRING_CRC);
#else
    mavlink_debug_string_t *packet = (mavlink_debug_string_t *)msgbuf;
    packet->resp = resp;
    mav_array_memcpy(packet->dbg_str, dbg_str, sizeof(char)*250);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG_STRING, (const char *)packet, MAVLINK_MSG_ID_DEBUG_STRING_MIN_LEN, MAVLINK_MSG_ID_DEBUG_STRING_LEN, MAVLINK_MSG_ID_DEBUG_STRING_CRC);
#endif
}
#endif

#endif

// MESSAGE DEBUG_STRING UNPACKING


/**
 * @brief Get field resp from debug_string message
 *
 * @return 0:No Resp, 1:Resp
 */
static inline uint8_t mavlink_msg_debug_string_get_resp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field dbg_str from debug_string message
 *
 * @return string
 */
static inline uint16_t mavlink_msg_debug_string_get_dbg_str(const mavlink_message_t* msg, char *dbg_str)
{
    return _MAV_RETURN_char_array(msg, dbg_str, 250,  1);
}

/**
 * @brief Decode a debug_string message into a struct
 *
 * @param msg The message to decode
 * @param debug_string C-struct to decode the message contents into
 */
static inline void mavlink_msg_debug_string_decode(const mavlink_message_t* msg, mavlink_debug_string_t* debug_string)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    debug_string->resp = mavlink_msg_debug_string_get_resp(msg);
    mavlink_msg_debug_string_get_dbg_str(msg, debug_string->dbg_str);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DEBUG_STRING_LEN? msg->len : MAVLINK_MSG_ID_DEBUG_STRING_LEN;
        memset(debug_string, 0, MAVLINK_MSG_ID_DEBUG_STRING_LEN);
    memcpy(debug_string, _MAV_PAYLOAD(msg), len);
#endif
}
