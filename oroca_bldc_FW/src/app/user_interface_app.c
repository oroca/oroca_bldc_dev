/*
	Copyright 2012-2014 OROCA ESC Project 	www.oroca.org

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
 * app.c
 *
 */
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "hw.h"

#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "mavlink_uart_proc.h"
#include "user_interface_app.h"
#include "uart3.h"

eventmask_t ui_events = 0;

// Threads
static THD_FUNCTION(user_interface_thread, arg);
static THD_WORKING_AREA(user_interface_thread_wa, 512);

void user_interface_configure(void)
{
	//Uart3_printf(&SD3, (uint8_t *)"oroca_bldc\r\n");//170530
	mavlink_uart_proc_configure();
	mavlink_uart_proc_start();

	//Uart3_printf(&SD3, (uint8_t *)"app_init.....\r\n");
	app_ppm_configure();
	app_ppm_start();
}

static THD_FUNCTION(user_interface_thread, arg)
{
	(void)arg;

	//uint8_t Ch;

	chRegSetThreadName("user_interface_process");

	for(;;)
	{
		if (ui_events)
		{
			chSysLockFromISR();
			//chEvtSignalI(pMavlinkThread, ui_events);
			chSysUnlockFromISR();
  		}
	}

}


void user_interface_start(void)
{
	chThdCreateStatic(user_interface_thread_wa, sizeof(user_interface_thread_wa), NORMALPRIO, user_interface_thread, NULL);
}
