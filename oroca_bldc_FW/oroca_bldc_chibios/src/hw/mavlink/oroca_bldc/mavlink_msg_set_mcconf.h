#pragma once
// MESSAGE SET_MCCONF PACKING

#define MAVLINK_MSG_ID_SET_MCCONF 121

MAVPACKED(
typedef struct __mavlink_set_mcconf_t {
 uint16_t uRSHUNT; /*<  x10^3 */
 uint16_t uPWMFREQUENCY; /*<  x10^3 */
 uint16_t uDKP; /*<  x10^3 */
 uint16_t uDKI; /*<  x10^3 */
 uint16_t uDKC; /*<  x10^3 */
 uint16_t uDOUTMAX; /*<  x10^3 */
 uint16_t uQKP; /*<  x10^3 */
 uint16_t uQKI; /*<  x10^3 */
 uint16_t uQKC; /*<  x10^3 */
 uint16_t uQOUTMAX; /*<  x10^3 */
 uint16_t uWKP; /*<  x10^3 */
 uint16_t uWKI; /*<  x10^3 */
 uint16_t uWKC; /*<  x10^3 */
 uint16_t uWOUTMAX; /*<  x10^3 */
 uint16_t uPLLKP; /*<  x10^3 */
 uint16_t uPLLKI; /*<  x10^3 */
 uint16_t uPLLKC; /*<  x10^3 */
 uint16_t uPLLOUTMAX; /*<  x10^3 */
 uint8_t resp; /*<  0:No Resp, 1:Resp*/
 uint8_t uVDD; /*<  x10^1 */
}) mavlink_set_mcconf_t;

#define MAVLINK_MSG_ID_SET_MCCONF_LEN 38
#define MAVLINK_MSG_ID_SET_MCCONF_MIN_LEN 38
#define MAVLINK_MSG_ID_121_LEN 38
#define MAVLINK_MSG_ID_121_MIN_LEN 38

#define MAVLINK_MSG_ID_SET_MCCONF_CRC 107
#define MAVLINK_MSG_ID_121_CRC 107



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SET_MCCONF { \
    121, \
    "SET_MCCONF", \
    20, \
    {  { "resp", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_set_mcconf_t, resp) }, \
         { "uVDD", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_set_mcconf_t, uVDD) }, \
         { "uRSHUNT", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_set_mcconf_t, uRSHUNT) }, \
         { "uPWMFREQUENCY", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_set_mcconf_t, uPWMFREQUENCY) }, \
         { "uDKP", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_set_mcconf_t, uDKP) }, \
         { "uDKI", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_set_mcconf_t, uDKI) }, \
         { "uDKC", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_set_mcconf_t, uDKC) }, \
         { "uDOUTMAX", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_set_mcconf_t, uDOUTMAX) }, \
         { "uQKP", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_set_mcconf_t, uQKP) }, \
         { "uQKI", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_set_mcconf_t, uQKI) }, \
         { "uQKC", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_set_mcconf_t, uQKC) }, \
         { "uQOUTMAX", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_set_mcconf_t, uQOUTMAX) }, \
         { "uWKP", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_set_mcconf_t, uWKP) }, \
         { "uWKI", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_set_mcconf_t, uWKI) }, \
         { "uWKC", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_set_mcconf_t, uWKC) }, \
         { "uWOUTMAX", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_set_mcconf_t, uWOUTMAX) }, \
         { "uPLLKP", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_set_mcconf_t, uPLLKP) }, \
         { "uPLLKI", NULL, MAVLINK_TYPE_UINT16_T, 0, 30, offsetof(mavlink_set_mcconf_t, uPLLKI) }, \
         { "uPLLKC", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_set_mcconf_t, uPLLKC) }, \
         { "uPLLOUTMAX", NULL, MAVLINK_TYPE_UINT16_T, 0, 34, offsetof(mavlink_set_mcconf_t, uPLLOUTMAX) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SET_MCCONF { \
    "SET_MCCONF", \
    20, \
    {  { "resp", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_set_mcconf_t, resp) }, \
         { "uVDD", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_set_mcconf_t, uVDD) }, \
         { "uRSHUNT", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_set_mcconf_t, uRSHUNT) }, \
         { "uPWMFREQUENCY", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_set_mcconf_t, uPWMFREQUENCY) }, \
         { "uDKP", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_set_mcconf_t, uDKP) }, \
         { "uDKI", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_set_mcconf_t, uDKI) }, \
         { "uDKC", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_set_mcconf_t, uDKC) }, \
         { "uDOUTMAX", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_set_mcconf_t, uDOUTMAX) }, \
         { "uQKP", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_set_mcconf_t, uQKP) }, \
         { "uQKI", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_set_mcconf_t, uQKI) }, \
         { "uQKC", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_set_mcconf_t, uQKC) }, \
         { "uQOUTMAX", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_set_mcconf_t, uQOUTMAX) }, \
         { "uWKP", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_set_mcconf_t, uWKP) }, \
         { "uWKI", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_set_mcconf_t, uWKI) }, \
         { "uWKC", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_set_mcconf_t, uWKC) }, \
         { "uWOUTMAX", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_set_mcconf_t, uWOUTMAX) }, \
         { "uPLLKP", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_set_mcconf_t, uPLLKP) }, \
         { "uPLLKI", NULL, MAVLINK_TYPE_UINT16_T, 0, 30, offsetof(mavlink_set_mcconf_t, uPLLKI) }, \
         { "uPLLKC", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_set_mcconf_t, uPLLKC) }, \
         { "uPLLOUTMAX", NULL, MAVLINK_TYPE_UINT16_T, 0, 34, offsetof(mavlink_set_mcconf_t, uPLLOUTMAX) }, \
         } \
}
#endif

/**
 * @brief Pack a set_mcconf message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param resp  0:No Resp, 1:Resp
 * @param uVDD  x10^1 
 * @param uRSHUNT  x10^3 
 * @param uPWMFREQUENCY  x10^3 
 * @param uDKP  x10^3 
 * @param uDKI  x10^3 
 * @param uDKC  x10^3 
 * @param uDOUTMAX  x10^3 
 * @param uQKP  x10^3 
 * @param uQKI  x10^3 
 * @param uQKC  x10^3 
 * @param uQOUTMAX  x10^3 
 * @param uWKP  x10^3 
 * @param uWKI  x10^3 
 * @param uWKC  x10^3 
 * @param uWOUTMAX  x10^3 
 * @param uPLLKP  x10^3 
 * @param uPLLKI  x10^3 
 * @param uPLLKC  x10^3 
 * @param uPLLOUTMAX  x10^3 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_mcconf_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t resp, uint8_t uVDD, uint16_t uRSHUNT, uint16_t uPWMFREQUENCY, uint16_t uDKP, uint16_t uDKI, uint16_t uDKC, uint16_t uDOUTMAX, uint16_t uQKP, uint16_t uQKI, uint16_t uQKC, uint16_t uQOUTMAX, uint16_t uWKP, uint16_t uWKI, uint16_t uWKC, uint16_t uWOUTMAX, uint16_t uPLLKP, uint16_t uPLLKI, uint16_t uPLLKC, uint16_t uPLLOUTMAX)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_MCCONF_LEN];
    _mav_put_uint16_t(buf, 0, uRSHUNT);
    _mav_put_uint16_t(buf, 2, uPWMFREQUENCY);
    _mav_put_uint16_t(buf, 4, uDKP);
    _mav_put_uint16_t(buf, 6, uDKI);
    _mav_put_uint16_t(buf, 8, uDKC);
    _mav_put_uint16_t(buf, 10, uDOUTMAX);
    _mav_put_uint16_t(buf, 12, uQKP);
    _mav_put_uint16_t(buf, 14, uQKI);
    _mav_put_uint16_t(buf, 16, uQKC);
    _mav_put_uint16_t(buf, 18, uQOUTMAX);
    _mav_put_uint16_t(buf, 20, uWKP);
    _mav_put_uint16_t(buf, 22, uWKI);
    _mav_put_uint16_t(buf, 24, uWKC);
    _mav_put_uint16_t(buf, 26, uWOUTMAX);
    _mav_put_uint16_t(buf, 28, uPLLKP);
    _mav_put_uint16_t(buf, 30, uPLLKI);
    _mav_put_uint16_t(buf, 32, uPLLKC);
    _mav_put_uint16_t(buf, 34, uPLLOUTMAX);
    _mav_put_uint8_t(buf, 36, resp);
    _mav_put_uint8_t(buf, 37, uVDD);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_MCCONF_LEN);
#else
    mavlink_set_mcconf_t packet;
    packet.uRSHUNT = uRSHUNT;
    packet.uPWMFREQUENCY = uPWMFREQUENCY;
    packet.uDKP = uDKP;
    packet.uDKI = uDKI;
    packet.uDKC = uDKC;
    packet.uDOUTMAX = uDOUTMAX;
    packet.uQKP = uQKP;
    packet.uQKI = uQKI;
    packet.uQKC = uQKC;
    packet.uQOUTMAX = uQOUTMAX;
    packet.uWKP = uWKP;
    packet.uWKI = uWKI;
    packet.uWKC = uWKC;
    packet.uWOUTMAX = uWOUTMAX;
    packet.uPLLKP = uPLLKP;
    packet.uPLLKI = uPLLKI;
    packet.uPLLKC = uPLLKC;
    packet.uPLLOUTMAX = uPLLOUTMAX;
    packet.resp = resp;
    packet.uVDD = uVDD;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_MCCONF_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_MCCONF;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_MCCONF_MIN_LEN, MAVLINK_MSG_ID_SET_MCCONF_LEN, MAVLINK_MSG_ID_SET_MCCONF_CRC);
}

/**
 * @brief Pack a set_mcconf message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param resp  0:No Resp, 1:Resp
 * @param uVDD  x10^1 
 * @param uRSHUNT  x10^3 
 * @param uPWMFREQUENCY  x10^3 
 * @param uDKP  x10^3 
 * @param uDKI  x10^3 
 * @param uDKC  x10^3 
 * @param uDOUTMAX  x10^3 
 * @param uQKP  x10^3 
 * @param uQKI  x10^3 
 * @param uQKC  x10^3 
 * @param uQOUTMAX  x10^3 
 * @param uWKP  x10^3 
 * @param uWKI  x10^3 
 * @param uWKC  x10^3 
 * @param uWOUTMAX  x10^3 
 * @param uPLLKP  x10^3 
 * @param uPLLKI  x10^3 
 * @param uPLLKC  x10^3 
 * @param uPLLOUTMAX  x10^3 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_mcconf_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t resp,uint8_t uVDD,uint16_t uRSHUNT,uint16_t uPWMFREQUENCY,uint16_t uDKP,uint16_t uDKI,uint16_t uDKC,uint16_t uDOUTMAX,uint16_t uQKP,uint16_t uQKI,uint16_t uQKC,uint16_t uQOUTMAX,uint16_t uWKP,uint16_t uWKI,uint16_t uWKC,uint16_t uWOUTMAX,uint16_t uPLLKP,uint16_t uPLLKI,uint16_t uPLLKC,uint16_t uPLLOUTMAX)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_MCCONF_LEN];
    _mav_put_uint16_t(buf, 0, uRSHUNT);
    _mav_put_uint16_t(buf, 2, uPWMFREQUENCY);
    _mav_put_uint16_t(buf, 4, uDKP);
    _mav_put_uint16_t(buf, 6, uDKI);
    _mav_put_uint16_t(buf, 8, uDKC);
    _mav_put_uint16_t(buf, 10, uDOUTMAX);
    _mav_put_uint16_t(buf, 12, uQKP);
    _mav_put_uint16_t(buf, 14, uQKI);
    _mav_put_uint16_t(buf, 16, uQKC);
    _mav_put_uint16_t(buf, 18, uQOUTMAX);
    _mav_put_uint16_t(buf, 20, uWKP);
    _mav_put_uint16_t(buf, 22, uWKI);
    _mav_put_uint16_t(buf, 24, uWKC);
    _mav_put_uint16_t(buf, 26, uWOUTMAX);
    _mav_put_uint16_t(buf, 28, uPLLKP);
    _mav_put_uint16_t(buf, 30, uPLLKI);
    _mav_put_uint16_t(buf, 32, uPLLKC);
    _mav_put_uint16_t(buf, 34, uPLLOUTMAX);
    _mav_put_uint8_t(buf, 36, resp);
    _mav_put_uint8_t(buf, 37, uVDD);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_MCCONF_LEN);
#else
    mavlink_set_mcconf_t packet;
    packet.uRSHUNT = uRSHUNT;
    packet.uPWMFREQUENCY = uPWMFREQUENCY;
    packet.uDKP = uDKP;
    packet.uDKI = uDKI;
    packet.uDKC = uDKC;
    packet.uDOUTMAX = uDOUTMAX;
    packet.uQKP = uQKP;
    packet.uQKI = uQKI;
    packet.uQKC = uQKC;
    packet.uQOUTMAX = uQOUTMAX;
    packet.uWKP = uWKP;
    packet.uWKI = uWKI;
    packet.uWKC = uWKC;
    packet.uWOUTMAX = uWOUTMAX;
    packet.uPLLKP = uPLLKP;
    packet.uPLLKI = uPLLKI;
    packet.uPLLKC = uPLLKC;
    packet.uPLLOUTMAX = uPLLOUTMAX;
    packet.resp = resp;
    packet.uVDD = uVDD;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_MCCONF_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_MCCONF;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_MCCONF_MIN_LEN, MAVLINK_MSG_ID_SET_MCCONF_LEN, MAVLINK_MSG_ID_SET_MCCONF_CRC);
}

/**
 * @brief Encode a set_mcconf struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_mcconf C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_mcconf_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_mcconf_t* set_mcconf)
{
    return mavlink_msg_set_mcconf_pack(system_id, component_id, msg, set_mcconf->resp, set_mcconf->uVDD, set_mcconf->uRSHUNT, set_mcconf->uPWMFREQUENCY, set_mcconf->uDKP, set_mcconf->uDKI, set_mcconf->uDKC, set_mcconf->uDOUTMAX, set_mcconf->uQKP, set_mcconf->uQKI, set_mcconf->uQKC, set_mcconf->uQOUTMAX, set_mcconf->uWKP, set_mcconf->uWKI, set_mcconf->uWKC, set_mcconf->uWOUTMAX, set_mcconf->uPLLKP, set_mcconf->uPLLKI, set_mcconf->uPLLKC, set_mcconf->uPLLOUTMAX);
}

/**
 * @brief Encode a set_mcconf struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_mcconf C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_mcconf_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_mcconf_t* set_mcconf)
{
    return mavlink_msg_set_mcconf_pack_chan(system_id, component_id, chan, msg, set_mcconf->resp, set_mcconf->uVDD, set_mcconf->uRSHUNT, set_mcconf->uPWMFREQUENCY, set_mcconf->uDKP, set_mcconf->uDKI, set_mcconf->uDKC, set_mcconf->uDOUTMAX, set_mcconf->uQKP, set_mcconf->uQKI, set_mcconf->uQKC, set_mcconf->uQOUTMAX, set_mcconf->uWKP, set_mcconf->uWKI, set_mcconf->uWKC, set_mcconf->uWOUTMAX, set_mcconf->uPLLKP, set_mcconf->uPLLKI, set_mcconf->uPLLKC, set_mcconf->uPLLOUTMAX);
}

/**
 * @brief Send a set_mcconf message
 * @param chan MAVLink channel to send the message
 *
 * @param resp  0:No Resp, 1:Resp
 * @param uVDD  x10^1 
 * @param uRSHUNT  x10^3 
 * @param uPWMFREQUENCY  x10^3 
 * @param uDKP  x10^3 
 * @param uDKI  x10^3 
 * @param uDKC  x10^3 
 * @param uDOUTMAX  x10^3 
 * @param uQKP  x10^3 
 * @param uQKI  x10^3 
 * @param uQKC  x10^3 
 * @param uQOUTMAX  x10^3 
 * @param uWKP  x10^3 
 * @param uWKI  x10^3 
 * @param uWKC  x10^3 
 * @param uWOUTMAX  x10^3 
 * @param uPLLKP  x10^3 
 * @param uPLLKI  x10^3 
 * @param uPLLKC  x10^3 
 * @param uPLLOUTMAX  x10^3 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_mcconf_send(mavlink_channel_t chan, uint8_t resp, uint8_t uVDD, uint16_t uRSHUNT, uint16_t uPWMFREQUENCY, uint16_t uDKP, uint16_t uDKI, uint16_t uDKC, uint16_t uDOUTMAX, uint16_t uQKP, uint16_t uQKI, uint16_t uQKC, uint16_t uQOUTMAX, uint16_t uWKP, uint16_t uWKI, uint16_t uWKC, uint16_t uWOUTMAX, uint16_t uPLLKP, uint16_t uPLLKI, uint16_t uPLLKC, uint16_t uPLLOUTMAX)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_MCCONF_LEN];
    _mav_put_uint16_t(buf, 0, uRSHUNT);
    _mav_put_uint16_t(buf, 2, uPWMFREQUENCY);
    _mav_put_uint16_t(buf, 4, uDKP);
    _mav_put_uint16_t(buf, 6, uDKI);
    _mav_put_uint16_t(buf, 8, uDKC);
    _mav_put_uint16_t(buf, 10, uDOUTMAX);
    _mav_put_uint16_t(buf, 12, uQKP);
    _mav_put_uint16_t(buf, 14, uQKI);
    _mav_put_uint16_t(buf, 16, uQKC);
    _mav_put_uint16_t(buf, 18, uQOUTMAX);
    _mav_put_uint16_t(buf, 20, uWKP);
    _mav_put_uint16_t(buf, 22, uWKI);
    _mav_put_uint16_t(buf, 24, uWKC);
    _mav_put_uint16_t(buf, 26, uWOUTMAX);
    _mav_put_uint16_t(buf, 28, uPLLKP);
    _mav_put_uint16_t(buf, 30, uPLLKI);
    _mav_put_uint16_t(buf, 32, uPLLKC);
    _mav_put_uint16_t(buf, 34, uPLLOUTMAX);
    _mav_put_uint8_t(buf, 36, resp);
    _mav_put_uint8_t(buf, 37, uVDD);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_MCCONF, buf, MAVLINK_MSG_ID_SET_MCCONF_MIN_LEN, MAVLINK_MSG_ID_SET_MCCONF_LEN, MAVLINK_MSG_ID_SET_MCCONF_CRC);
#else
    mavlink_set_mcconf_t packet;
    packet.uRSHUNT = uRSHUNT;
    packet.uPWMFREQUENCY = uPWMFREQUENCY;
    packet.uDKP = uDKP;
    packet.uDKI = uDKI;
    packet.uDKC = uDKC;
    packet.uDOUTMAX = uDOUTMAX;
    packet.uQKP = uQKP;
    packet.uQKI = uQKI;
    packet.uQKC = uQKC;
    packet.uQOUTMAX = uQOUTMAX;
    packet.uWKP = uWKP;
    packet.uWKI = uWKI;
    packet.uWKC = uWKC;
    packet.uWOUTMAX = uWOUTMAX;
    packet.uPLLKP = uPLLKP;
    packet.uPLLKI = uPLLKI;
    packet.uPLLKC = uPLLKC;
    packet.uPLLOUTMAX = uPLLOUTMAX;
    packet.resp = resp;
    packet.uVDD = uVDD;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_MCCONF, (const char *)&packet, MAVLINK_MSG_ID_SET_MCCONF_MIN_LEN, MAVLINK_MSG_ID_SET_MCCONF_LEN, MAVLINK_MSG_ID_SET_MCCONF_CRC);
#endif
}

/**
 * @brief Send a set_mcconf message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_set_mcconf_send_struct(mavlink_channel_t chan, const mavlink_set_mcconf_t* set_mcconf)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_set_mcconf_send(chan, set_mcconf->resp, set_mcconf->uVDD, set_mcconf->uRSHUNT, set_mcconf->uPWMFREQUENCY, set_mcconf->uDKP, set_mcconf->uDKI, set_mcconf->uDKC, set_mcconf->uDOUTMAX, set_mcconf->uQKP, set_mcconf->uQKI, set_mcconf->uQKC, set_mcconf->uQOUTMAX, set_mcconf->uWKP, set_mcconf->uWKI, set_mcconf->uWKC, set_mcconf->uWOUTMAX, set_mcconf->uPLLKP, set_mcconf->uPLLKI, set_mcconf->uPLLKC, set_mcconf->uPLLOUTMAX);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_MCCONF, (const char *)set_mcconf, MAVLINK_MSG_ID_SET_MCCONF_MIN_LEN, MAVLINK_MSG_ID_SET_MCCONF_LEN, MAVLINK_MSG_ID_SET_MCCONF_CRC);
#endif
}

#if MAVLINK_MSG_ID_SET_MCCONF_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_mcconf_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t resp, uint8_t uVDD, uint16_t uRSHUNT, uint16_t uPWMFREQUENCY, uint16_t uDKP, uint16_t uDKI, uint16_t uDKC, uint16_t uDOUTMAX, uint16_t uQKP, uint16_t uQKI, uint16_t uQKC, uint16_t uQOUTMAX, uint16_t uWKP, uint16_t uWKI, uint16_t uWKC, uint16_t uWOUTMAX, uint16_t uPLLKP, uint16_t uPLLKI, uint16_t uPLLKC, uint16_t uPLLOUTMAX)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, uRSHUNT);
    _mav_put_uint16_t(buf, 2, uPWMFREQUENCY);
    _mav_put_uint16_t(buf, 4, uDKP);
    _mav_put_uint16_t(buf, 6, uDKI);
    _mav_put_uint16_t(buf, 8, uDKC);
    _mav_put_uint16_t(buf, 10, uDOUTMAX);
    _mav_put_uint16_t(buf, 12, uQKP);
    _mav_put_uint16_t(buf, 14, uQKI);
    _mav_put_uint16_t(buf, 16, uQKC);
    _mav_put_uint16_t(buf, 18, uQOUTMAX);
    _mav_put_uint16_t(buf, 20, uWKP);
    _mav_put_uint16_t(buf, 22, uWKI);
    _mav_put_uint16_t(buf, 24, uWKC);
    _mav_put_uint16_t(buf, 26, uWOUTMAX);
    _mav_put_uint16_t(buf, 28, uPLLKP);
    _mav_put_uint16_t(buf, 30, uPLLKI);
    _mav_put_uint16_t(buf, 32, uPLLKC);
    _mav_put_uint16_t(buf, 34, uPLLOUTMAX);
    _mav_put_uint8_t(buf, 36, resp);
    _mav_put_uint8_t(buf, 37, uVDD);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_MCCONF, buf, MAVLINK_MSG_ID_SET_MCCONF_MIN_LEN, MAVLINK_MSG_ID_SET_MCCONF_LEN, MAVLINK_MSG_ID_SET_MCCONF_CRC);
#else
    mavlink_set_mcconf_t *packet = (mavlink_set_mcconf_t *)msgbuf;
    packet->uRSHUNT = uRSHUNT;
    packet->uPWMFREQUENCY = uPWMFREQUENCY;
    packet->uDKP = uDKP;
    packet->uDKI = uDKI;
    packet->uDKC = uDKC;
    packet->uDOUTMAX = uDOUTMAX;
    packet->uQKP = uQKP;
    packet->uQKI = uQKI;
    packet->uQKC = uQKC;
    packet->uQOUTMAX = uQOUTMAX;
    packet->uWKP = uWKP;
    packet->uWKI = uWKI;
    packet->uWKC = uWKC;
    packet->uWOUTMAX = uWOUTMAX;
    packet->uPLLKP = uPLLKP;
    packet->uPLLKI = uPLLKI;
    packet->uPLLKC = uPLLKC;
    packet->uPLLOUTMAX = uPLLOUTMAX;
    packet->resp = resp;
    packet->uVDD = uVDD;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_MCCONF, (const char *)packet, MAVLINK_MSG_ID_SET_MCCONF_MIN_LEN, MAVLINK_MSG_ID_SET_MCCONF_LEN, MAVLINK_MSG_ID_SET_MCCONF_CRC);
#endif
}
#endif

#endif

// MESSAGE SET_MCCONF UNPACKING


/**
 * @brief Get field resp from set_mcconf message
 *
 * @return  0:No Resp, 1:Resp
 */
static inline uint8_t mavlink_msg_set_mcconf_get_resp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  36);
}

/**
 * @brief Get field uVDD from set_mcconf message
 *
 * @return  x10^1 
 */
static inline uint8_t mavlink_msg_set_mcconf_get_uVDD(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  37);
}

/**
 * @brief Get field uRSHUNT from set_mcconf message
 *
 * @return  x10^3 
 */
static inline uint16_t mavlink_msg_set_mcconf_get_uRSHUNT(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field uPWMFREQUENCY from set_mcconf message
 *
 * @return  x10^3 
 */
static inline uint16_t mavlink_msg_set_mcconf_get_uPWMFREQUENCY(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field uDKP from set_mcconf message
 *
 * @return  x10^3 
 */
static inline uint16_t mavlink_msg_set_mcconf_get_uDKP(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field uDKI from set_mcconf message
 *
 * @return  x10^3 
 */
static inline uint16_t mavlink_msg_set_mcconf_get_uDKI(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field uDKC from set_mcconf message
 *
 * @return  x10^3 
 */
static inline uint16_t mavlink_msg_set_mcconf_get_uDKC(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field uDOUTMAX from set_mcconf message
 *
 * @return  x10^3 
 */
static inline uint16_t mavlink_msg_set_mcconf_get_uDOUTMAX(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Get field uQKP from set_mcconf message
 *
 * @return  x10^3 
 */
static inline uint16_t mavlink_msg_set_mcconf_get_uQKP(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field uQKI from set_mcconf message
 *
 * @return  x10^3 
 */
static inline uint16_t mavlink_msg_set_mcconf_get_uQKI(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  14);
}

/**
 * @brief Get field uQKC from set_mcconf message
 *
 * @return  x10^3 
 */
static inline uint16_t mavlink_msg_set_mcconf_get_uQKC(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field uQOUTMAX from set_mcconf message
 *
 * @return  x10^3 
 */
static inline uint16_t mavlink_msg_set_mcconf_get_uQOUTMAX(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  18);
}

/**
 * @brief Get field uWKP from set_mcconf message
 *
 * @return  x10^3 
 */
static inline uint16_t mavlink_msg_set_mcconf_get_uWKP(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Get field uWKI from set_mcconf message
 *
 * @return  x10^3 
 */
static inline uint16_t mavlink_msg_set_mcconf_get_uWKI(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  22);
}

/**
 * @brief Get field uWKC from set_mcconf message
 *
 * @return  x10^3 
 */
static inline uint16_t mavlink_msg_set_mcconf_get_uWKC(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Get field uWOUTMAX from set_mcconf message
 *
 * @return  x10^3 
 */
static inline uint16_t mavlink_msg_set_mcconf_get_uWOUTMAX(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  26);
}

/**
 * @brief Get field uPLLKP from set_mcconf message
 *
 * @return  x10^3 
 */
static inline uint16_t mavlink_msg_set_mcconf_get_uPLLKP(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  28);
}

/**
 * @brief Get field uPLLKI from set_mcconf message
 *
 * @return  x10^3 
 */
static inline uint16_t mavlink_msg_set_mcconf_get_uPLLKI(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  30);
}

/**
 * @brief Get field uPLLKC from set_mcconf message
 *
 * @return  x10^3 
 */
static inline uint16_t mavlink_msg_set_mcconf_get_uPLLKC(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  32);
}

/**
 * @brief Get field uPLLOUTMAX from set_mcconf message
 *
 * @return  x10^3 
 */
static inline uint16_t mavlink_msg_set_mcconf_get_uPLLOUTMAX(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  34);
}

/**
 * @brief Decode a set_mcconf message into a struct
 *
 * @param msg The message to decode
 * @param set_mcconf C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_mcconf_decode(const mavlink_message_t* msg, mavlink_set_mcconf_t* set_mcconf)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    set_mcconf->uRSHUNT = mavlink_msg_set_mcconf_get_uRSHUNT(msg);
    set_mcconf->uPWMFREQUENCY = mavlink_msg_set_mcconf_get_uPWMFREQUENCY(msg);
    set_mcconf->uDKP = mavlink_msg_set_mcconf_get_uDKP(msg);
    set_mcconf->uDKI = mavlink_msg_set_mcconf_get_uDKI(msg);
    set_mcconf->uDKC = mavlink_msg_set_mcconf_get_uDKC(msg);
    set_mcconf->uDOUTMAX = mavlink_msg_set_mcconf_get_uDOUTMAX(msg);
    set_mcconf->uQKP = mavlink_msg_set_mcconf_get_uQKP(msg);
    set_mcconf->uQKI = mavlink_msg_set_mcconf_get_uQKI(msg);
    set_mcconf->uQKC = mavlink_msg_set_mcconf_get_uQKC(msg);
    set_mcconf->uQOUTMAX = mavlink_msg_set_mcconf_get_uQOUTMAX(msg);
    set_mcconf->uWKP = mavlink_msg_set_mcconf_get_uWKP(msg);
    set_mcconf->uWKI = mavlink_msg_set_mcconf_get_uWKI(msg);
    set_mcconf->uWKC = mavlink_msg_set_mcconf_get_uWKC(msg);
    set_mcconf->uWOUTMAX = mavlink_msg_set_mcconf_get_uWOUTMAX(msg);
    set_mcconf->uPLLKP = mavlink_msg_set_mcconf_get_uPLLKP(msg);
    set_mcconf->uPLLKI = mavlink_msg_set_mcconf_get_uPLLKI(msg);
    set_mcconf->uPLLKC = mavlink_msg_set_mcconf_get_uPLLKC(msg);
    set_mcconf->uPLLOUTMAX = mavlink_msg_set_mcconf_get_uPLLOUTMAX(msg);
    set_mcconf->resp = mavlink_msg_set_mcconf_get_resp(msg);
    set_mcconf->uVDD = mavlink_msg_set_mcconf_get_uVDD(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SET_MCCONF_LEN? msg->len : MAVLINK_MSG_ID_SET_MCCONF_LEN;
        memset(set_mcconf, 0, MAVLINK_MSG_ID_SET_MCCONF_LEN);
    memcpy(set_mcconf, _MAV_PAYLOAD(msg), len);
#endif
}
