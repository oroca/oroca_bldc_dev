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


#include "timeout.h"
#include "utils.h"

#include <math.h>
#include <chthreads.h>
#include <Chvt.h>

#include "uart3_print.h"

#include "../../lib/mavlink/oroca_bldc/mavlink.h"


#define EVT_UART_RX EVENT_MASK(0)
#define EVT_CAN_RX EVENT_MASK(1)


// Threads
static THD_FUNCTION(mavlink_uart_thread, arg);
static THD_WORKING_AREA(mavlink_uart_thread_wa, 1024);

static THD_FUNCTION(mavlink_thread, arg);
static THD_WORKING_AREA(mavlink_thread_wa, 128);

thread_t *pMavlinkThread;
eventmask_t mavlink_events = 0;




int mavlink_uart_send( uint8_t data )
{
    mavlink_message_t msg; 
    uint8_t buf[1024];     

    mavlink_msg_set_velocity_pack( 9, 121, &msg, data );

    int len = mavlink_msg_to_send_buffer(buf, &msg);

    //usb_uart_write(buf, len);
    //uart3_write(buf, len);
}


bool mavlink_uart_recv( uint8_t ch )
{
	bool ret = false;

	mavlink_message_t msg; 
	mavlink_status_t status; 

	if (mavlink_parse_char(MAVLINK_COMM_0, ch, &msg, &status) == MAVLINK_FRAMING_OK)
	{

		if( MAVLINK_MSG_ID_SET_VELOCITY == msg.msgid ) 
		{
			mavlink_set_velocity_t set_velocity;
			mavlink_msg_set_velocity_decode( &msg, &set_velocity);

			//Serial.print("seq= ");
			//Serial.println(test_cmd.arg1);
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

	//process_tp = chThdSelf();

	for(;;)
	{
		//chThdSleepMilliseconds(1);

		Ch = Uart3_getch();

		//Uart3_printf(&SD3, (uint8_t *)"0x%x", Ch);
		
		if( mavlink_uart_recv( Ch ) )
		{
			mavlink_uart_send( 1 );

			mavlink_events |= EVT_UART_RX;

		}

		if (mavlink_events) 
		{
		    chSysLockFromISR();
		    chEvtSignalI(pMavlinkThread, mavlink_events);
		    chSysUnlockFromISR();
  		}
	}

	return 0;

}




void mavlink_proc_configure(void)
{

	mavlink_events=0;
}

void mavlink_proc_start(void) 
{
	chThdCreateStatic(mavlink_uart_thread_wa, sizeof(mavlink_uart_thread_wa), NORMALPRIO, mavlink_uart_thread, NULL);


	//Uart3_printf(&SD3, (uint8_t *)"mavlink_proc_start.....\r\n");  //170530  
	chThdCreateStatic(mavlink_thread_wa, sizeof(mavlink_thread_wa),NORMALPRIO + 1, mavlink_thread, NULL);
}

 
static THD_FUNCTION(mavlink_thread, arg)
{
 
	/* Thread activity.*/
	while (true)
	{
		/* Waiting for any event.*/
		eventmask_t evt = chEvtWaitAny(ALL_EVENTS);

		/* Serving events.*/
		if (evt & EVT_UART_RX)
		{
		  /* Error event.*/
		  //uart_rx_handler();
		}
		if (evt & EVT_CAN_RX)
		{
		  /* Error event.*/
		  //can_rx_handler();
		}

	}
}



