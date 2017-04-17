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
 * app.c
 *
 */
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "hw.h"
#include "app.h"

#include "uart3_print.h"

#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>


// Private variables
app_use app_to_use = APP_PPM;

void app_init(void)
{
	//Uart3_printf(&SD3, (uint8_t *)"app_init.....\r\n");
	switch (app_to_use)
	{
		case APP_PPM:
			app_ppm_configure();
			app_ppm_start();
			break;

		case APP_UART:
			hw_stop_i2c();
			//app_uartcomm_start();
			break;

		default:
			break;
	}
}

