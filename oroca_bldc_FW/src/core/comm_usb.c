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
 * comm.c
 *
 *  Created on: 22 nov 2012
 *      Author: benjamin
 */

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"

#include "mavlink_proc.h"
#include "comm_usb.h"
#include "comm_usb_serial.h"

#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>


// Settings
#define PACKET_HANDLER				0

// Private variables
#define SERIAL_RX_BUFFER_SIZE		2048
static uint8_t serial_rx_buffer[SERIAL_RX_BUFFER_SIZE];
static int serial_rx_read_pos = 0;
static int serial_rx_write_pos = 0;
static THD_WORKING_AREA(serial_read_thread_wa, 512);
static THD_WORKING_AREA(serial_process_thread_wa, 4096);

//static uint8_t obuf[512];
//static output_queue_t oq;
#define SERIAL_TX_BUFFER_SIZE		2048
static uint8_t serial_tx_buffer[SERIAL_TX_BUFFER_SIZE];
static int serial_tx_read_pos = 0;
static int serial_tx_write_pos = 0;
static THD_WORKING_AREA(serial_write_thread_wa, 512);

static mutex_t send_mutex;
static thread_t *process_rx_tp;
static thread_t *process_tx_tp;


// Private functions
int usb_uart_printf( const char *fmt, ...);

static THD_FUNCTION(serial_read_thread, arg) {
	(void)arg;

	chRegSetThreadName("USB-Serial read");

	uint8_t buffer[128];
	int i;
	int len;
	int had_data = 0;

	for(;;) 
	{
		len = chSequentialStreamRead(&SDU1, (uint8_t*) buffer, 1);

		for (i = 0;i < len;i++) 
		{
			serial_rx_buffer[serial_rx_write_pos++] = buffer[i];

			if (serial_rx_write_pos == SERIAL_RX_BUFFER_SIZE) 
			{
				serial_rx_write_pos = 0;
			}
			
			had_data = 1;
		}

		if (had_data) 
		{
			chEvtSignal(process_rx_tp, (eventmask_t) 1);
			had_data = 0;
		}
	}
}

static THD_FUNCTION(serial_process_thread, arg) {
	(void)arg;

	chRegSetThreadName("USB-Serial process");

	process_rx_tp = chThdGetSelfX();

	for(;;) {
		chEvtWaitAny((eventmask_t) 1);

		while (serial_rx_read_pos != serial_rx_write_pos) {

			if( mavlink_byte_recv( serial_rx_buffer[serial_rx_read_pos++] ) )
			{
				//mavlink_uart_send( 1 ); //hand shake?
			}

			if (serial_rx_read_pos == SERIAL_RX_BUFFER_SIZE) 
			{
				serial_rx_read_pos = 0;
			}
		}
	}
}


static THD_FUNCTION(serial_write_thread, arg) {
	(void)arg;

	chRegSetThreadName("USB-Serial write");

	process_tx_tp = chThdGetSelfX();

	for(;;)
	{
		chEvtWaitAny((eventmask_t) 1);

		//chvprintf(&SDU1, (uint8_t *)".");
	
		while (serial_tx_read_pos != serial_tx_write_pos) 
		{
			chSequentialStreamWrite(&SDU1, (uint8_t)serial_tx_buffer[serial_tx_read_pos++],1);

			if (serial_tx_read_pos == SERIAL_TX_BUFFER_SIZE) 
			{
				serial_tx_read_pos = 0;
			}
		}
	}
}



void usb_serial_send(uint8_t* buffer, uint16_t len)
{
		uint16_t i;
		int had_data = 0;

		for (i = 0;i < len;i++) 
		{
			serial_tx_buffer[serial_tx_write_pos++] = buffer[i];

			if (serial_tx_write_pos == SERIAL_TX_BUFFER_SIZE) 
			{
				serial_tx_write_pos = 0;
			}
			
			had_data = 1;
		}

		if (had_data) 
		{
			chEvtSignal(process_tx_tp, (eventmask_t) 1);
			had_data = 0;
		}
}


void comm_usb_init(void) {
	comm_usb_serial_init();
	//packet_init(send_packet, process_packet, PACKET_HANDLER);

	chMtxObjectInit(&send_mutex);

//	oqObjectInit(&oq, obuf, SERIAL_BUFFERS_SIZE, NULL, NULL);

	// Threads
	chThdCreateStatic(serial_read_thread_wa, sizeof(serial_read_thread_wa), NORMALPRIO, serial_read_thread, NULL);
	chThdCreateStatic(serial_process_thread_wa, sizeof(serial_process_thread_wa), NORMALPRIO, serial_process_thread, NULL);
	chThdCreateStatic(serial_write_thread_wa, sizeof(serial_write_thread_wa), NORMALPRIO, serial_write_thread, NULL);
}




int usb_uart_printf( const char *fmt, ...)
{
	int ret = 0;
	va_list arg;
	va_start (arg, fmt);
	int len;
	static char print_buffer[255];

	len = vsnprintf(print_buffer, 255, fmt, arg);
	va_end (arg);

	ret = chSequentialStreamWrite(&SDU1, print_buffer, len);

	return ret;
}


int usb_uart_write( uint8_t *p_data, uint32_t len )
{
	int ret = 0;

	ret = chSequentialStreamWrite(&SDU1, p_data, len);

	return ret;
}

uint8_t usb_uart_getch( void )
{
	uint8_t buffer[128];
	int len;


	len = chSequentialStreamRead(&SDU1, (uint8_t*) buffer, 1);

	return buffer[0];
}

