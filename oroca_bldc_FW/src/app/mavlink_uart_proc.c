/*
	Copyright 2012-2015 OROCA ESC Project 	www.oroca.org

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
 * mavlink_proc.c
 *
 *  Created on: 18 apr 2014
 *      Author: bakchajang
 */


#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"

#include <math.h>
#include <chthreads.h>
#include <Chvt.h>

#include "timeout.h"
#include "utils.h"
#include "mcpwm.h"

#include "ui_app.h"
#include "../../lib/mavlink/oroca_bldc/mavlink.h"
#include "mavlink_uart_proc.h"
//#include "uart3.h"


// Threads
static THD_FUNCTION(mavlink_uart_thread, arg);
static THD_WORKING_AREA(mavlink_uart_thread_wa, 1024);

thread_t *pMavlinkThread;

int mavlink_uart_send( uint8_t data )
{
    mavlink_message_t msg; 
    uint8_t buf[1024];     

    mavlink_msg_set_velocity_pack( 9, 121, &msg, data );

    int len = mavlink_msg_to_send_buffer(buf, &msg);

    Uart3_write(buf, len);

    return 0;
}


bool mavlink_uart_recv( uint8_t ch )
{
	bool ret = false;

	mavlink_message_t msg; 
	mavlink_status_t status; 

	uint8_t temp = mavlink_parse_char(MAVLINK_COMM_0, ch, &msg, &status);

	//Uart3_printf(&SD3, "%02x ",ch);

	if (temp == MAVLINK_FRAMING_OK)
	{
		if( MAVLINK_MSG_ID_SET_VELOCITY == msg.msgid ) 
		{
			mavlink_set_velocity_t set_velocity;
			mavlink_msg_set_velocity_decode( &msg, &set_velocity);

		//	Uart3_printf(&SD3, "SET_VELOCITY\r\n");
		//	Uart3_printf(&SD3, "value : %d",(uint16_t)set_velocity.ref_angular_velocity );

			//--------------------------------------------------------------------------------
			//test code
			int16_t tmp_value = set_velocity.ref_angular_velocity - 1500;
			float vel = (float)tmp_value / 700.0f;

			CtrlParm.qVelRef = vel / 100.0f;
			//------------------------------------------------------------------------------
			ret = true;
		}
    }


	return ret;
}



static THD_FUNCTION(mavlink_uart_thread, arg)
{
	(void)arg;

	uint8_t Ch;

	chRegSetThreadName("mavlink_uart_rx_process");

	for(;;)
	{
		//chThdSleepMilliseconds(1);
		Ch = Uart3_getch();

		if( mavlink_uart_recv( Ch ) )
		{
			//mavlink_uart_send( 1 ); //hand shake?

			ui_events |= EVT_UART_RX;
		}
	}

	return 0;

}


void mavlink_uart_proc_configure(void)
{
	Uart3_init();
	return;
}

void mavlink_uart_proc_start(void)
{
	chThdCreateStatic(mavlink_uart_thread_wa, sizeof(mavlink_uart_thread_wa), NORMALPRIO, mavlink_uart_thread, NULL);
}


//--------------------------------------------------------------
//function below

