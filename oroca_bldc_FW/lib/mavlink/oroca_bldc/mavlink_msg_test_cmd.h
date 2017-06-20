// MESSAGE TEST_CMD PACKING

#define MAVLINK_MSG_ID_TEST_CMD 220

typedef struct MAVLINK_PACKED __mavlink_test_cmd_t
{
 uint16_t arg2; /*< argument 2*/
 uint8_t cmd_1; /*< test cmd 1*/
 uint8_t arg1; /*< argument 1*/
} mavlink_test_cmd_t;

#define MAVLINK_MSG_ID_TEST_CMD_LEN 4
#define MAVLINK_MSG_ID_TEST_CMD_MIN_LEN 4
#define MAVLINK_MSG_ID_220_LEN 4
#define MAVLINK_MSG_ID_220_MIN_LEN 4

#define MAVLINK_MSG_ID_TEST_CMD_CRC 37
#define MAVLINK_MSG_ID_220_CRC 37



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TEST_CMD { \
	220, \
	"TEST_CMD", \
	3, \
	{  { "arg2", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_test_cmd_t, arg2) }, \
         { "cmd_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_test_cmd_t, cmd_1) }, \
         { "arg1", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_test_cmd_t, arg1) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TEST_CMD { \
	"TEST_CMD", \
	3, \
	{  { "arg2", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_test_cmd_t, arg2) }, \
         { "cmd_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_test_cmd_t, cmd_1) }, \
         { "arg1", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_test_cmd_t, arg1) }, \
         } \
}
#endif

/**
 * @brief Pack a test_cmd message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param cmd_1 test cmd 1
 * @param arg1 argument 1
 * @param arg2 argument 2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_test_cmd_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t cmd_1, uint8_t arg1, uint16_t arg2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TEST_CMD_LEN];
	_mav_put_uint16_t(buf, 0, arg2);
	_mav_put_uint8_t(buf, 2, cmd_1);
	_mav_put_uint8_t(buf, 3, arg1);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TEST_CMD_LEN);
#else
	mavlink_test_cmd_t packet;
	packet.arg2 = arg2;
	packet.cmd_1 = cmd_1;
	packet.arg1 = arg1;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TEST_CMD_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_TEST_CMD;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TEST_CMD_MIN_LEN, MAVLINK_MSG_ID_TEST_CMD_LEN, MAVLINK_MSG_ID_TEST_CMD_CRC);
}

/**
 * @brief Pack a test_cmd message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param cmd_1 test cmd 1
 * @param arg1 argument 1
 * @param arg2 argument 2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_test_cmd_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t cmd_1,uint8_t arg1,uint16_t arg2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TEST_CMD_LEN];
	_mav_put_uint16_t(buf, 0, arg2);
	_mav_put_uint8_t(buf, 2, cmd_1);
	_mav_put_uint8_t(buf, 3, arg1);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TEST_CMD_LEN);
#else
	mavlink_test_cmd_t packet;
	packet.arg2 = arg2;
	packet.cmd_1 = cmd_1;
	packet.arg1 = arg1;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TEST_CMD_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_TEST_CMD;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TEST_CMD_MIN_LEN, MAVLINK_MSG_ID_TEST_CMD_LEN, MAVLINK_MSG_ID_TEST_CMD_CRC);
}

/**
 * @brief Encode a test_cmd struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param test_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_test_cmd_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_test_cmd_t* test_cmd)
{
	return mavlink_msg_test_cmd_pack(system_id, component_id, msg, test_cmd->cmd_1, test_cmd->arg1, test_cmd->arg2);
}

/**
 * @brief Encode a test_cmd struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param test_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_test_cmd_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_test_cmd_t* test_cmd)
{
	return mavlink_msg_test_cmd_pack_chan(system_id, component_id, chan, msg, test_cmd->cmd_1, test_cmd->arg1, test_cmd->arg2);
}

/**
 * @brief Send a test_cmd message
 * @param chan MAVLink channel to send the message
 *
 * @param cmd_1 test cmd 1
 * @param arg1 argument 1
 * @param arg2 argument 2
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_test_cmd_send(mavlink_channel_t chan, uint8_t cmd_1, uint8_t arg1, uint16_t arg2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TEST_CMD_LEN];
	_mav_put_uint16_t(buf, 0, arg2);
	_mav_put_uint8_t(buf, 2, cmd_1);
	_mav_put_uint8_t(buf, 3, arg1);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TEST_CMD, buf, MAVLINK_MSG_ID_TEST_CMD_MIN_LEN, MAVLINK_MSG_ID_TEST_CMD_LEN, MAVLINK_MSG_ID_TEST_CMD_CRC);
#else
	mavlink_test_cmd_t packet;
	packet.arg2 = arg2;
	packet.cmd_1 = cmd_1;
	packet.arg1 = arg1;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TEST_CMD, (const char *)&packet, MAVLINK_MSG_ID_TEST_CMD_MIN_LEN, MAVLINK_MSG_ID_TEST_CMD_LEN, MAVLINK_MSG_ID_TEST_CMD_CRC);
#endif
}

/**
 * @brief Send a test_cmd message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_test_cmd_send_struct(mavlink_channel_t chan, const mavlink_test_cmd_t* test_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_test_cmd_send(chan, test_cmd->cmd_1, test_cmd->arg1, test_cmd->arg2);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TEST_CMD, (const char *)test_cmd, MAVLINK_MSG_ID_TEST_CMD_MIN_LEN, MAVLINK_MSG_ID_TEST_CMD_LEN, MAVLINK_MSG_ID_TEST_CMD_CRC);
#endif
}

#if MAVLINK_MSG_ID_TEST_CMD_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_test_cmd_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t cmd_1, uint8_t arg1, uint16_t arg2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint16_t(buf, 0, arg2);
	_mav_put_uint8_t(buf, 2, cmd_1);
	_mav_put_uint8_t(buf, 3, arg1);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TEST_CMD, buf, MAVLINK_MSG_ID_TEST_CMD_MIN_LEN, MAVLINK_MSG_ID_TEST_CMD_LEN, MAVLINK_MSG_ID_TEST_CMD_CRC);
#else
	mavlink_test_cmd_t *packet = (mavlink_test_cmd_t *)msgbuf;
	packet->arg2 = arg2;
	packet->cmd_1 = cmd_1;
	packet->arg1 = arg1;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TEST_CMD, (const char *)packet, MAVLINK_MSG_ID_TEST_CMD_MIN_LEN, MAVLINK_MSG_ID_TEST_CMD_LEN, MAVLINK_MSG_ID_TEST_CMD_CRC);
#endif
}
#endif

#endif

// MESSAGE TEST_CMD UNPACKING


/**
 * @brief Get field cmd_1 from test_cmd message
 *
 * @return test cmd 1
 */
static inline uint8_t mavlink_msg_test_cmd_get_cmd_1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field arg1 from test_cmd message
 *
 * @return argument 1
 */
static inline uint8_t mavlink_msg_test_cmd_get_arg1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field arg2 from test_cmd message
 *
 * @return argument 2
 */
static inline uint16_t mavlink_msg_test_cmd_get_arg2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Decode a test_cmd message into a struct
 *
 * @param msg The message to decode
 * @param test_cmd C-struct to decode the message contents into
 */
static inline void mavlink_msg_test_cmd_decode(const mavlink_message_t* msg, mavlink_test_cmd_t* test_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	test_cmd->arg2 = mavlink_msg_test_cmd_get_arg2(msg);
	test_cmd->cmd_1 = mavlink_msg_test_cmd_get_cmd_1(msg);
	test_cmd->arg1 = mavlink_msg_test_cmd_get_arg1(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TEST_CMD_LEN? msg->len : MAVLINK_MSG_ID_TEST_CMD_LEN;
        memset(test_cmd, 0, MAVLINK_MSG_ID_TEST_CMD_LEN);
	memcpy(test_cmd, _MAV_PAYLOAD(msg), len);
#endif
}
