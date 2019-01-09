#pragma once
// MESSAGE CMD_SET_APPCONF PACKING

#define MAVLINK_MSG_ID_CMD_SET_APPCONF 222

MAVPACKED(
typedef struct __mavlink_cmd_set_appconf_t {
 uint16_t data[128]; /*< */
 uint8_t resp; /*< 0:No Resp, 1:Resp*/
}) mavlink_cmd_set_appconf_t;

#define MAVLINK_MSG_ID_CMD_SET_APPCONF_LEN 257
#define MAVLINK_MSG_ID_CMD_SET_APPCONF_MIN_LEN 257
#define MAVLINK_MSG_ID_222_LEN 257
#define MAVLINK_MSG_ID_222_MIN_LEN 257

#define MAVLINK_MSG_ID_CMD_SET_APPCONF_CRC 236
#define MAVLINK_MSG_ID_222_CRC 236

#define MAVLINK_MSG_CMD_SET_APPCONF_FIELD_DATA_LEN 128

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CMD_SET_APPCONF { \
    222, \
    "CMD_SET_APPCONF", \
    2, \
    {  { "resp", NULL, MAVLINK_TYPE_UINT8_T, 0, 256, offsetof(mavlink_cmd_set_appconf_t, resp) }, \
         { "data", NULL, MAVLINK_TYPE_UINT16_T, 128, 0, offsetof(mavlink_cmd_set_appconf_t, data) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CMD_SET_APPCONF { \
    "CMD_SET_APPCONF", \
    2, \
    {  { "resp", NULL, MAVLINK_TYPE_UINT8_T, 0, 256, offsetof(mavlink_cmd_set_appconf_t, resp) }, \
         { "data", NULL, MAVLINK_TYPE_UINT16_T, 128, 0, offsetof(mavlink_cmd_set_appconf_t, data) }, \
         } \
}
#endif

/**
 * @brief Pack a cmd_set_appconf message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param resp 0:No Resp, 1:Resp
 * @param data 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cmd_set_appconf_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t resp, const uint16_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CMD_SET_APPCONF_LEN];
    _mav_put_uint8_t(buf, 256, resp);
    _mav_put_uint16_t_array(buf, 0, data, 128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CMD_SET_APPCONF_LEN);
#else
    mavlink_cmd_set_appconf_t packet;
    packet.resp = resp;
    mav_array_memcpy(packet.data, data, sizeof(uint16_t)*128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CMD_SET_APPCONF_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CMD_SET_APPCONF;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CMD_SET_APPCONF_MIN_LEN, MAVLINK_MSG_ID_CMD_SET_APPCONF_LEN, MAVLINK_MSG_ID_CMD_SET_APPCONF_CRC);
}

/**
 * @brief Pack a cmd_set_appconf message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param resp 0:No Resp, 1:Resp
 * @param data 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cmd_set_appconf_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t resp,const uint16_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CMD_SET_APPCONF_LEN];
    _mav_put_uint8_t(buf, 256, resp);
    _mav_put_uint16_t_array(buf, 0, data, 128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CMD_SET_APPCONF_LEN);
#else
    mavlink_cmd_set_appconf_t packet;
    packet.resp = resp;
    mav_array_memcpy(packet.data, data, sizeof(uint16_t)*128);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CMD_SET_APPCONF_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CMD_SET_APPCONF;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CMD_SET_APPCONF_MIN_LEN, MAVLINK_MSG_ID_CMD_SET_APPCONF_LEN, MAVLINK_MSG_ID_CMD_SET_APPCONF_CRC);
}

/**
 * @brief Encode a cmd_set_appconf struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param cmd_set_appconf C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cmd_set_appconf_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_cmd_set_appconf_t* cmd_set_appconf)
{
    return mavlink_msg_cmd_set_appconf_pack(system_id, component_id, msg, cmd_set_appconf->resp, cmd_set_appconf->data);
}

/**
 * @brief Encode a cmd_set_appconf struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param cmd_set_appconf C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cmd_set_appconf_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_cmd_set_appconf_t* cmd_set_appconf)
{
    return mavlink_msg_cmd_set_appconf_pack_chan(system_id, component_id, chan, msg, cmd_set_appconf->resp, cmd_set_appconf->data);
}

/**
 * @brief Send a cmd_set_appconf message
 * @param chan MAVLink channel to send the message
 *
 * @param resp 0:No Resp, 1:Resp
 * @param data 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_cmd_set_appconf_send(mavlink_channel_t chan, uint8_t resp, const uint16_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CMD_SET_APPCONF_LEN];
    _mav_put_uint8_t(buf, 256, resp);
    _mav_put_uint16_t_array(buf, 0, data, 128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CMD_SET_APPCONF, buf, MAVLINK_MSG_ID_CMD_SET_APPCONF_MIN_LEN, MAVLINK_MSG_ID_CMD_SET_APPCONF_LEN, MAVLINK_MSG_ID_CMD_SET_APPCONF_CRC);
#else
    mavlink_cmd_set_appconf_t packet;
    packet.resp = resp;
    mav_array_memcpy(packet.data, data, sizeof(uint16_t)*128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CMD_SET_APPCONF, (const char *)&packet, MAVLINK_MSG_ID_CMD_SET_APPCONF_MIN_LEN, MAVLINK_MSG_ID_CMD_SET_APPCONF_LEN, MAVLINK_MSG_ID_CMD_SET_APPCONF_CRC);
#endif
}

/**
 * @brief Send a cmd_set_appconf message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_cmd_set_appconf_send_struct(mavlink_channel_t chan, const mavlink_cmd_set_appconf_t* cmd_set_appconf)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_cmd_set_appconf_send(chan, cmd_set_appconf->resp, cmd_set_appconf->data);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CMD_SET_APPCONF, (const char *)cmd_set_appconf, MAVLINK_MSG_ID_CMD_SET_APPCONF_MIN_LEN, MAVLINK_MSG_ID_CMD_SET_APPCONF_LEN, MAVLINK_MSG_ID_CMD_SET_APPCONF_CRC);
#endif
}

#if MAVLINK_MSG_ID_CMD_SET_APPCONF_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_cmd_set_appconf_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t resp, const uint16_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 256, resp);
    _mav_put_uint16_t_array(buf, 0, data, 128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CMD_SET_APPCONF, buf, MAVLINK_MSG_ID_CMD_SET_APPCONF_MIN_LEN, MAVLINK_MSG_ID_CMD_SET_APPCONF_LEN, MAVLINK_MSG_ID_CMD_SET_APPCONF_CRC);
#else
    mavlink_cmd_set_appconf_t *packet = (mavlink_cmd_set_appconf_t *)msgbuf;
    packet->resp = resp;
    mav_array_memcpy(packet->data, data, sizeof(uint16_t)*128);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CMD_SET_APPCONF, (const char *)packet, MAVLINK_MSG_ID_CMD_SET_APPCONF_MIN_LEN, MAVLINK_MSG_ID_CMD_SET_APPCONF_LEN, MAVLINK_MSG_ID_CMD_SET_APPCONF_CRC);
#endif
}
#endif

#endif

// MESSAGE CMD_SET_APPCONF UNPACKING


/**
 * @brief Get field resp from cmd_set_appconf message
 *
 * @return 0:No Resp, 1:Resp
 */
static inline uint8_t mavlink_msg_cmd_set_appconf_get_resp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  256);
}

/**
 * @brief Get field data from cmd_set_appconf message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_cmd_set_appconf_get_data(const mavlink_message_t* msg, uint16_t *data)
{
    return _MAV_RETURN_uint16_t_array(msg, data, 128,  0);
}

/**
 * @brief Decode a cmd_set_appconf message into a struct
 *
 * @param msg The message to decode
 * @param cmd_set_appconf C-struct to decode the message contents into
 */
static inline void mavlink_msg_cmd_set_appconf_decode(const mavlink_message_t* msg, mavlink_cmd_set_appconf_t* cmd_set_appconf)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_cmd_set_appconf_get_data(msg, cmd_set_appconf->data);
    cmd_set_appconf->resp = mavlink_msg_cmd_set_appconf_get_resp(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CMD_SET_APPCONF_LEN? msg->len : MAVLINK_MSG_ID_CMD_SET_APPCONF_LEN;
        memset(cmd_set_appconf, 0, MAVLINK_MSG_ID_CMD_SET_APPCONF_LEN);
    memcpy(cmd_set_appconf, _MAV_PAYLOAD(msg), len);
#endif
}
