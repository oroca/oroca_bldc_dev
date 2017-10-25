/*
	Copyright 2012-2014 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

/*
 * comm_can.h
 *
 *  Created on: 7 dec 2014
 *      Author: benjamin
 */

#ifndef __MAVLINK_CAN_PROC_H__
#define __MAVLINK_CAN_PROC_H__

#include <chtypes.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EVT_CAN EVENT_MASK(2)

// CAN commands
typedef enum {
	CAN_PACKET_SET_VELOCITY = 0,
	CAN_PACKET_SET_CURRENT,
	CAN_PACKET_SET_RPM,
	CAN_PACKET_SET_POS,
	CAN_PACKET_FILL_RX_BUFFER,
	CAN_PACKET_FILL_RX_BUFFER_LONG,
	CAN_PACKET_PROCESS_RX_BUFFER,
	CAN_PACKET_PROCESS_SHORT_BUFFER,
	CAN_PACKET_STATUS
} CAN_PACKET_ID;

#define CAN_PACKET_BROADCASTING	0xFF

typedef struct {
	int id;
	systime_t rx_time;
	float rpm;
	float current;//?????
	float duty;//?????
} can_status_msg;


// Settings
#define CAN_STATUS_MSG_INT_MS		1
#define CAN_STATUS_MSGS_TO_STORE		10

// Functions
void comm_can_init(void);
void comm_can_transmit(uint32_t id, uint8_t *data, uint8_t len);
void comm_can_send_buffer(uint8_t controller_id, uint8_t *data, unsigned int len, bool send);

#ifdef __cplusplus
}
#endif


#endif /* COMM_CAN_H_ */
