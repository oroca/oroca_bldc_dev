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
 * comm_can.c
 *
 *  Created on: 7 dec 2014
 *      Author: benjamin
 */

#include <string.h>
#include "comm_can.h"
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "conf_general.h"
//#include "buffer.h"
#include "mc_interface.h"
#include "timeout.h"
//#include "commands.h"
#include "app.h"
//#include "crc.h"
//#include "packet.h"


// Settings
#define CANDx			CAND1
#define RX_FRAMES_SIZE	100
#define RX_BUFFER_SIZE	PACKET_MAX_PL_LEN

// Threads
static THD_WORKING_AREA(cancom_read_thread_wa, 512);
static THD_WORKING_AREA(cancom_process_thread_wa, 4096);
static THD_WORKING_AREA(cancom_status_thread_wa, 1024);
static THD_FUNCTION(cancom_read_thread, arg);
static THD_FUNCTION(cancom_status_thread, arg);
static THD_FUNCTION(cancom_process_thread, arg);

// Variables
static can_status_msg stat_msgs[CAN_STATUS_MSGS_TO_STORE];
static mutex_t can_mtx;
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static unsigned int rx_buffer_last_id;
static CANRxFrame rx_frames[RX_FRAMES_SIZE];
static int rx_frame_read;
static int rx_frame_write;
static thread_t *process_tp;

/*
 * 500KBaud, automatic wakeup, automatic recover
 * from abort mode.
 * See section 22.7.7 on the STM32 reference manual.
 */
static const CANConfig cancfg = {
		CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
		CAN_BTR_SJW(0) | CAN_BTR_TS2(1) |
		CAN_BTR_TS1(8) | CAN_BTR_BRP(6)
};

// Private functions
static void send_packet_wrapper(unsigned char *data, unsigned int len);

void comm_can_init(void) {
	for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
		stat_msgs[i].id = -1;
	}

	rx_frame_read = 0;
	rx_frame_write = 0;

	chMtxObjectInit(&can_mtx);

	palSetPadMode(GPIOB, 8,
			PAL_MODE_ALTERNATE(GPIO_AF_CAN1) |
			PAL_STM32_OTYPE_PUSHPULL |
			PAL_STM32_OSPEED_MID1);
	palSetPadMode(GPIOB, 9,
			PAL_MODE_ALTERNATE(GPIO_AF_CAN1) |
			PAL_STM32_OTYPE_PUSHPULL |
			PAL_STM32_OSPEED_MID1);

	canStart(&CANDx, &cancfg);

	chThdCreateStatic(cancom_read_thread_wa, sizeof(cancom_read_thread_wa), NORMALPRIO + 1,
			cancom_read_thread, NULL);
	chThdCreateStatic(cancom_status_thread_wa, sizeof(cancom_status_thread_wa), NORMALPRIO,
			cancom_status_thread, NULL);
	chThdCreateStatic(cancom_process_thread_wa, sizeof(cancom_process_thread_wa), NORMALPRIO,
			cancom_process_thread, NULL);
}

static THD_FUNCTION(cancom_read_thread, arg) {
	(void)arg;
	chRegSetThreadName("CAN");

	event_listener_t el;
	CANRxFrame rxmsg;

	chEvtRegister(&CANDx.rxfull_event, &el, 0);

	while(!chThdShouldTerminateX()) {
		if (chEvtWaitAnyTimeout(ALL_EVENTS, MS2ST(10)) == 0) {
			continue;
		}

		msg_t result = canReceive(&CANDx, CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE);

		while (result == MSG_OK) {
			rx_frames[rx_frame_write++] = rxmsg;
			if (rx_frame_write == RX_FRAMES_SIZE) {
				rx_frame_write = 0;
			}

			chEvtSignal(process_tp, (eventmask_t) 1);

			result = canReceive(&CANDx, CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE);
		}
	}

	chEvtUnregister(&CANDx.rxfull_event, &el);
}

static THD_FUNCTION(cancom_process_thread, arg) {
	(void)arg;

	chRegSetThreadName("Cancom process");
	process_tp = chThdGetSelfX();

	int32_t ind = 0;
	unsigned int rxbuf_len;
	unsigned int rxbuf_ind;
	uint8_t crc_low;
	uint8_t crc_high;
	bool commands_send;

	for(;;) {
		chEvtWaitAny((eventmask_t) 1);

		while (rx_frame_read != rx_frame_write) {
			CANRxFrame rxmsg = rx_frames[rx_frame_read++];

			if (rxmsg.IDE == CAN_IDE_EXT) {
				uint8_t id = rxmsg.EID & 0xFF;
				//CAN_PACKET_ID cmd = rxmsg.EID >> 8;
				can_status_msg *stat_tmp;


			}

			if (rx_frame_read == RX_FRAMES_SIZE) {
				rx_frame_read = 0;
			}
		}
	}
}

static THD_FUNCTION(cancom_status_thread, arg) {
	(void)arg;
	chRegSetThreadName("CAN status");

	for(;;) {
		if (app_get_configuration()->send_can_status) {
			// Send status message
		//	int32_t send_index = 0;
		//	uint8_t buffer[8];
		//	buffer_append_int32(buffer, (int32_t)mc_interface_get_rpm(), &send_index);
		//	buffer_append_int16(buffer, (int16_t)(mc_interface_get_tot_current() * 10.0), &send_index);
		//	buffer_append_int16(buffer, (int16_t)(mc_interface_get_duty_cycle_now() * 1000.0), &send_index);
		//	comm_can_transmit(app_get_configuration()->controller_id | ((uint32_t)CAN_PACKET_STATUS << 8), buffer, send_index);
		}

		systime_t sleep_time = CH_CFG_ST_FREQUENCY / app_get_configuration()->send_can_status_rate_hz;
		if (sleep_time == 0) {
			sleep_time = 1;
		}

		chThdSleep(sleep_time);
	}
}

void comm_can_transmit(uint32_t id, uint8_t *data, uint8_t len) {
#if CAN_ENABLE
	CANTxFrame txmsg;
	txmsg.IDE = CAN_IDE_EXT;
	txmsg.EID = id;
	txmsg.RTR = CAN_RTR_DATA;
	txmsg.DLC = len;
	memcpy(txmsg.data8, data, len);

	chMtxLock(&can_mtx);
	canTransmit(&CANDx, CAN_ANY_MAILBOX, &txmsg, MS2ST(20));
	chMtxUnlock(&can_mtx);

#else
	(void)id;
	(void)data;
	(void)len;
#endif
}

/**
 * Send a buffer up to RX_BUFFER_SIZE bytes as fragments. If the buffer is 6 bytes or less
 * it will be sent in a single CAN frame, otherwise it will be split into
 * several frames.
 *
 * @param controller_id
 * The controller id to send to.
 *
 * @param data
 * The payload.
 *
 * @param len
 * The payload length.
 *
 * @param send
 * If true, this packet will be passed to the send function of commands.
 * Otherwise, it will be passed to the process function.
 */
void comm_can_send_buffer(uint8_t controller_id, uint8_t *data, unsigned int len, bool send) {
	uint8_t send_buffer[8];

	if (len <= 6) {
		uint32_t ind = 0;
		send_buffer[ind++] = app_get_configuration()->controller_id;
		send_buffer[ind++] = send;
		memcpy(send_buffer + ind, data, len);
		ind += len;
		//comm_can_transmit(controller_id | ((uint32_t)CAN_PACKET_PROCESS_SHORT_BUFFER << 8), send_buffer, ind);
	} else {
		unsigned int end_a = 0;
		for (unsigned int i = 0;i < len;i += 7) {
			if (i > 255) {
				break;
			}

			end_a = i + 7;

			uint8_t send_len = 7;
			send_buffer[0] = i;

			if ((i + 7) <= len) {
				memcpy(send_buffer + 1, data + i, send_len);
			} else {
				send_len = len - i;
				memcpy(send_buffer + 1, data + i, send_len);
			}

			//comm_can_transmit(controller_id | ((uint32_t)CAN_PACKET_FILL_RX_BUFFER << 8), send_buffer, send_len + 1);
		}

		for (unsigned int i = end_a;i < len;i += 6) {
			uint8_t send_len = 6;
			send_buffer[0] = i >> 8;
			send_buffer[1] = i & 0xFF;

			if ((i + 6) <= len) {
				memcpy(send_buffer + 2, data + i, send_len);
			} else {
				send_len = len - i;
				memcpy(send_buffer + 2, data + i, send_len);
			}

			//comm_can_transmit(controller_id | ((uint32_t)CAN_PACKET_FILL_RX_BUFFER_LONG << 8), send_buffer, send_len + 2);
		}

	//	uint32_t ind = 0;
	//	send_buffer[ind++] = app_get_configuration()->controller_id;
	//	send_buffer[ind++] = send;
	//	send_buffer[ind++] = len >> 8;
	//	send_buffer[ind++] = len & 0xFF;
	//	unsigned short crc = crc16(data, len);
	//	send_buffer[ind++] = (uint8_t)(crc >> 8);
	//	send_buffer[ind++] = (uint8_t)(crc & 0xFF);

		//comm_can_transmit(controller_id | ((uint32_t)CAN_PACKET_PROCESS_RX_BUFFER << 8), send_buffer, ind++);
	}
}

/**
 * Get status message by index.
 *
 * @param index
 * Index in the array
 *
 * @return
 * The message or 0 for an invalid index.
 */
can_status_msg *comm_can_get_status_msg_index(int index) {
	if (index < CAN_STATUS_MSGS_TO_STORE) {
		return &stat_msgs[index];
	} else {
		return 0;
	}
}

/**
 * Get status message by id.
 *
 * @param id
 * Id of the controller that sent the status message.
 *
 * @return
 * The message or 0 for an invalid id.
 */
can_status_msg *comm_can_get_status_msg_id(int id) {
	for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
		if (stat_msgs[i].id == id) {
			return &stat_msgs[i];
		}
	}

	return 0;
}

static void send_packet_wrapper(unsigned char *data, unsigned int len) {
	comm_can_send_buffer(rx_buffer_last_id, data, len, true);
}
