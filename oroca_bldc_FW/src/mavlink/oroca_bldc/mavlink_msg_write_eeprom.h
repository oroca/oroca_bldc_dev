#pragma once
// MESSAGE WRITE_EEPROM PACKING

#define MAVLINK_MSG_ID_WRITE_EEPROM 20

MAVPACKED(
typedef struct __mavlink_write_eeprom_t {
 uint8_t resp; /*<  0:No Resp, 1:Resp*/
 uint8_t param; /*<  */
}) mavlink_write_eeprom_t;

#define MAVLINK_MSG_ID_WRITE_EEPROM_LEN 2
#define MAVLINK_MSG_ID_WRITE_EEPROM_MIN_LEN 2
#define MAVLINK_MSG_ID_20_LEN 2
#define MAVLINK_MSG_ID_20_MIN_LEN 2

#define MAVLINK_MSG_ID_WRITE_EEPROM_CRC 235
#define MAVLINK_MSG_ID_20_CRC 235



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_WRITE_EEPROM { \
    20, \
    "WRITE_EEPROM", \
    2, \
    {  { "resp", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_write_eeprom_t, resp) }, \
         { "param", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_write_eeprom_t, param) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_WRITE_EEPROM { \
    "WRITE_EEPROM", \
    2, \
    {  { "resp", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_write_eeprom_t, resp) }, \
         { "param", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_write_eeprom_t, param) }, \
         } \
}
#endif

/**
 * @brief Pack a write_eeprom message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param resp  0:No Resp, 1:Resp
 * @param param  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_write_eeprom_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t resp, uint8_t param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WRITE_EEPROM_LEN];
    _mav_put_uint8_t(buf, 0, resp);
    _mav_put_uint8_t(buf, 1, param);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WRITE_EEPROM_LEN);
#else
    mavlink_write_eeprom_t packet;
    packet.resp = resp;
    packet.param = param;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WRITE_EEPROM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WRITE_EEPROM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_WRITE_EEPROM_MIN_LEN, MAVLINK_MSG_ID_WRITE_EEPROM_LEN, MAVLINK_MSG_ID_WRITE_EEPROM_CRC);
}

/**
 * @brief Pack a write_eeprom message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param resp  0:No Resp, 1:Resp
 * @param param  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_write_eeprom_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t resp,uint8_t param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WRITE_EEPROM_LEN];
    _mav_put_uint8_t(buf, 0, resp);
    _mav_put_uint8_t(buf, 1, param);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WRITE_EEPROM_LEN);
#else
    mavlink_write_eeprom_t packet;
    packet.resp = resp;
    packet.param = param;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WRITE_EEPROM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WRITE_EEPROM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_WRITE_EEPROM_MIN_LEN, MAVLINK_MSG_ID_WRITE_EEPROM_LEN, MAVLINK_MSG_ID_WRITE_EEPROM_CRC);
}

/**
 * @brief Encode a write_eeprom struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param write_eeprom C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_write_eeprom_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_write_eeprom_t* write_eeprom)
{
    return mavlink_msg_write_eeprom_pack(system_id, component_id, msg, write_eeprom->resp, write_eeprom->param);
}

/**
 * @brief Encode a write_eeprom struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param write_eeprom C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_write_eeprom_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_write_eeprom_t* write_eeprom)
{
    return mavlink_msg_write_eeprom_pack_chan(system_id, component_id, chan, msg, write_eeprom->resp, write_eeprom->param);
}

/**
 * @brief Send a write_eeprom message
 * @param chan MAVLink channel to send the message
 *
 * @param resp  0:No Resp, 1:Resp
 * @param param  
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_write_eeprom_send(mavlink_channel_t chan, uint8_t resp, uint8_t param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WRITE_EEPROM_LEN];
    _mav_put_uint8_t(buf, 0, resp);
    _mav_put_uint8_t(buf, 1, param);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WRITE_EEPROM, buf, MAVLINK_MSG_ID_WRITE_EEPROM_MIN_LEN, MAVLINK_MSG_ID_WRITE_EEPROM_LEN, MAVLINK_MSG_ID_WRITE_EEPROM_CRC);
#else
    mavlink_write_eeprom_t packet;
    packet.resp = resp;
    packet.param = param;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WRITE_EEPROM, (const char *)&packet, MAVLINK_MSG_ID_WRITE_EEPROM_MIN_LEN, MAVLINK_MSG_ID_WRITE_EEPROM_LEN, MAVLINK_MSG_ID_WRITE_EEPROM_CRC);
#endif
}

/**
 * @brief Send a write_eeprom message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_write_eeprom_send_struct(mavlink_channel_t chan, const mavlink_write_eeprom_t* write_eeprom)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_write_eeprom_send(chan, write_eeprom->resp, write_eeprom->param);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WRITE_EEPROM, (const char *)write_eeprom, MAVLINK_MSG_ID_WRITE_EEPROM_MIN_LEN, MAVLINK_MSG_ID_WRITE_EEPROM_LEN, MAVLINK_MSG_ID_WRITE_EEPROM_CRC);
#endif
}

#if MAVLINK_MSG_ID_WRITE_EEPROM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_write_eeprom_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t resp, uint8_t param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, resp);
    _mav_put_uint8_t(buf, 1, param);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WRITE_EEPROM, buf, MAVLINK_MSG_ID_WRITE_EEPROM_MIN_LEN, MAVLINK_MSG_ID_WRITE_EEPROM_LEN, MAVLINK_MSG_ID_WRITE_EEPROM_CRC);
#else
    mavlink_write_eeprom_t *packet = (mavlink_write_eeprom_t *)msgbuf;
    packet->resp = resp;
    packet->param = param;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WRITE_EEPROM, (const char *)packet, MAVLINK_MSG_ID_WRITE_EEPROM_MIN_LEN, MAVLINK_MSG_ID_WRITE_EEPROM_LEN, MAVLINK_MSG_ID_WRITE_EEPROM_CRC);
#endif
}
#endif

#endif

// MESSAGE WRITE_EEPROM UNPACKING


/**
 * @brief Get field resp from write_eeprom message
 *
 * @return  0:No Resp, 1:Resp
 */
static inline uint8_t mavlink_msg_write_eeprom_get_resp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field param from write_eeprom message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_write_eeprom_get_param(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a write_eeprom message into a struct
 *
 * @param msg The message to decode
 * @param write_eeprom C-struct to decode the message contents into
 */
static inline void mavlink_msg_write_eeprom_decode(const mavlink_message_t* msg, mavlink_write_eeprom_t* write_eeprom)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    write_eeprom->resp = mavlink_msg_write_eeprom_get_resp(msg);
    write_eeprom->param = mavlink_msg_write_eeprom_get_param(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_WRITE_EEPROM_LEN? msg->len : MAVLINK_MSG_ID_WRITE_EEPROM_LEN;
        memset(write_eeprom, 0, MAVLINK_MSG_ID_WRITE_EEPROM_LEN);
    memcpy(write_eeprom, _MAV_PAYLOAD(msg), len);
#endif
}
