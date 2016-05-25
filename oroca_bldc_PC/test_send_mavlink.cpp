//#include <mavlink/oroca_bldc/oroca_bldc.h>
//#include <mavlink/common/mavlink.h>
#include <mavlink/oroca_bldc/mavlink.h>

#include <QCoreApplication>
#include <QDebug>

#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)


//mavlink_message_t tx_msg;
uint8_t buf[BUFFER_LENGTH];
int send(uint8_t *buf){


    #if MAVLINK_CRC_EXTRA

#endif //
    mavlink_message_t msg;

    /*Send Heartbeat */
    //mavlink_msg_heartbeat_pack(9, 121, &msg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
    mavlink_msg_test_cmd_pack( 9, 121, &msg, 91,92,93);
    //mavlink_msg_message_interval_pack( 9, 121, &msg, 91,92);

    int len = mavlink_msg_to_send_buffer(buf, &msg);

    //qDebug("checksum=%04X \n", msg.checksum);


    // buf, len 으로 전송 하면 됨...
    return len;
}




bool recv( const uint8_t *buf, int recv_len)
{
    bool recvFrame = false;

    //uint8_t buf[BUFFER_LENGTH];

     qDebug("\nRecv len=%d \n", recv_len);

     mavlink_message_t msg;
     mavlink_status_t status;


    // 모든 문자에서 검출 ..
    for( int i=0; i< recv_len ; i++){
        //qDebug("%02X ", buf[i]);

        if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status) == MAVLINK_FRAMING_OK)
        {
            recvFrame = true;
            // Packet received
            qDebug("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
            if( MAVLINK_MSG_ID_TEST_CMD == msg.msgid )
            {
                mavlink_test_cmd_t test_cmd;
                mavlink_msg_test_cmd_decode( &msg, &test_cmd);
                qDebug("seq=%d \n", test_cmd.arg1);
            }
        }

        /*
        mavlink_message_t* rxmsg = mavlink_get_channel_buffer(MAVLINK_COMM_0);


        qDebug("parse_state=%d packet_idx=%d current_rx_seq=%d, packet_rx_success_count=%d, packet_rx_drop_count=%d ",
               status.parse_state,
               status.packet_idx,
               status.current_rx_seq,
               status.packet_rx_success_count,
               status.packet_rx_drop_count);
         qDebug("checksum=%02X =>  %02X", rxmsg->checksum , buf[i] );
        */

    }
    return recvFrame;

}

