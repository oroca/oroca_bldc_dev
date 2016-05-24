/** @file
 *	@brief MAVLink comm protocol testsuite generated from oroca_bldc.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef OROCA_BLDC_TESTSUITE_H
#define OROCA_BLDC_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_oroca_bldc(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_common(system_id, component_id, last_msg);
	mavlink_test_oroca_bldc(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_test_cmd(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_test_cmd_t packet_in = {
		17235,139,206
    };
	mavlink_test_cmd_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.arg2 = packet_in.arg2;
        	packet1.cmd_1 = packet_in.cmd_1;
        	packet1.arg1 = packet_in.arg1;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_test_cmd_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_test_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_test_cmd_pack(system_id, component_id, &msg , packet1.cmd_1 , packet1.arg1 , packet1.arg2 );
	mavlink_msg_test_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_test_cmd_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.cmd_1 , packet1.arg1 , packet1.arg2 );
	mavlink_msg_test_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_test_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_test_cmd_send(MAVLINK_COMM_1 , packet1.cmd_1 , packet1.arg1 , packet1.arg2 );
	mavlink_msg_test_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_test_resp1(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_test_resp1_t packet_in = {
		5
    };
	mavlink_test_resp1_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.status = packet_in.status;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_test_resp1_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_test_resp1_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_test_resp1_pack(system_id, component_id, &msg , packet1.status );
	mavlink_msg_test_resp1_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_test_resp1_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.status );
	mavlink_msg_test_resp1_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_test_resp1_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_test_resp1_send(MAVLINK_COMM_1 , packet1.status );
	mavlink_msg_test_resp1_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_oroca_bldc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_test_cmd(system_id, component_id, last_msg);
	mavlink_test_test_resp1(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // OROCA_BLDC_TESTSUITE_H
