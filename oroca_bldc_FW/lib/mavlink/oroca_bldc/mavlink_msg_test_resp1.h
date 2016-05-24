// MESSAGE TEST_RESP1 PACKING

#define MAVLINK_MSG_ID_TEST_RESP1 221

typedef struct __mavlink_test_resp1_t
{
 uint8_t status; /*< status 1*/
} mavlink_test_resp1_t;

#define MAVLINK_MSG_ID_TEST_RESP1_LEN 1
#define MAVLINK_MSG_ID_221_LEN 1

#define MAVLINK_MSG_ID_TEST_RESP1_CRC 177
#define MAVLINK_MSG_ID_221_CRC 177



#define MAVLINK_MESSAGE_INFO_TEST_RESP1 { \
	"TEST_RESP1", \
	1, \
	{  { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_test_resp1_t, status) }, \
         } \
}


/**
 * @brief Pack a test_resp1 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param status status 1
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_test_resp1_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TEST_RESP1_LEN];
	_mav_put_uint8_t(buf, 0, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TEST_RESP1_LEN);
#else
	mavlink_test_resp1_t packet;
	packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TEST_RESP1_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_TEST_RESP1;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TEST_RESP1_LEN, MAVLINK_MSG_ID_TEST_RESP1_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TEST_RESP1_LEN);
#endif
}

/**
 * @brief Pack a test_resp1 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param status status 1
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_test_resp1_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TEST_RESP1_LEN];
	_mav_put_uint8_t(buf, 0, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TEST_RESP1_LEN);
#else
	mavlink_test_resp1_t packet;
	packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TEST_RESP1_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_TEST_RESP1;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TEST_RESP1_LEN, MAVLINK_MSG_ID_TEST_RESP1_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TEST_RESP1_LEN);
#endif
}

/**
 * @brief Encode a test_resp1 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param test_resp1 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_test_resp1_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_test_resp1_t* test_resp1)
{
	return mavlink_msg_test_resp1_pack(system_id, component_id, msg, test_resp1->status);
}

/**
 * @brief Encode a test_resp1 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param test_resp1 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_test_resp1_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_test_resp1_t* test_resp1)
{
	return mavlink_msg_test_resp1_pack_chan(system_id, component_id, chan, msg, test_resp1->status);
}

/**
 * @brief Send a test_resp1 message
 * @param chan MAVLink channel to send the message
 *
 * @param status status 1
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_test_resp1_send(mavlink_channel_t chan, uint8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TEST_RESP1_LEN];
	_mav_put_uint8_t(buf, 0, status);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TEST_RESP1, buf, MAVLINK_MSG_ID_TEST_RESP1_LEN, MAVLINK_MSG_ID_TEST_RESP1_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TEST_RESP1, buf, MAVLINK_MSG_ID_TEST_RESP1_LEN);
#endif
#else
	mavlink_test_resp1_t packet;
	packet.status = status;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TEST_RESP1, (const char *)&packet, MAVLINK_MSG_ID_TEST_RESP1_LEN, MAVLINK_MSG_ID_TEST_RESP1_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TEST_RESP1, (const char *)&packet, MAVLINK_MSG_ID_TEST_RESP1_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_TEST_RESP1_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_test_resp1_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, status);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TEST_RESP1, buf, MAVLINK_MSG_ID_TEST_RESP1_LEN, MAVLINK_MSG_ID_TEST_RESP1_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TEST_RESP1, buf, MAVLINK_MSG_ID_TEST_RESP1_LEN);
#endif
#else
	mavlink_test_resp1_t *packet = (mavlink_test_resp1_t *)msgbuf;
	packet->status = status;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TEST_RESP1, (const char *)packet, MAVLINK_MSG_ID_TEST_RESP1_LEN, MAVLINK_MSG_ID_TEST_RESP1_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TEST_RESP1, (const char *)packet, MAVLINK_MSG_ID_TEST_RESP1_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE TEST_RESP1 UNPACKING


/**
 * @brief Get field status from test_resp1 message
 *
 * @return status 1
 */
static inline uint8_t mavlink_msg_test_resp1_get_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a test_resp1 message into a struct
 *
 * @param msg The message to decode
 * @param test_resp1 C-struct to decode the message contents into
 */
static inline void mavlink_msg_test_resp1_decode(const mavlink_message_t* msg, mavlink_test_resp1_t* test_resp1)
{
#if MAVLINK_NEED_BYTE_SWAP
	test_resp1->status = mavlink_msg_test_resp1_get_status(msg);
#else
	memcpy(test_resp1, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_TEST_RESP1_LEN);
#endif
}
