/*
	Copyright 2012-2015 Benjamin Vedder	benjamin@vedder.se

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
 * nrf_driver.h
 *
 *  Created on: 29 mar 2015
 *      Author: benjamin
 */

#ifndef NRF_NRF_DRIVER_H_
#define NRF_NRF_DRIVER_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct {
	uint8_t js_x;
	uint8_t js_y;
	bool bt_c;
	bool bt_z;
	bool bt_push;
	float vbat;
} mote_state;

typedef enum {
	MOTE_PACKET_BATT_LEVEL = 0,
	MOTE_PACKET_BUTTONS,
	MOTE_PACKET_ALIVE,
	MOTE_PACKET_FILL_RX_BUFFER,
	MOTE_PACKET_FILL_RX_BUFFER_LONG,
	MOTE_PACKET_PROCESS_RX_BUFFER,
	MOTE_PACKET_PROCESS_SHORT_BUFFER,
} MOTE_PACKET;

// Functions
void nrf_driver_init(void);
void nrf_driver_send_buffer(unsigned char *data, unsigned int len);

#endif /* NRF_NRF_DRIVER_H_ */
