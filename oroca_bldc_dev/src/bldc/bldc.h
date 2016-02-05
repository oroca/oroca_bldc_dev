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
 * main.h
 *
 *  Created on: 10 jul 2012
 *      Author: BenjaminVe
 */

#ifndef MAIN_BLDC_H_
#define MAIN_BLDC_H_

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>


#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"

#include "conf_general.h"


#include "mcpwm.h"
#include "ledpwm.h"
#include "comm_usb.h"
#include "ledpwm.h"
#include "terminal.h"
#include "hw.h"
#include "app.h"
#include "packet.h"
#include "commands.h"
#include "timeout.h"
#include "comm_can.h"
#include "ws2811.h"
#include "led_external.h"
#include "encoder.h"

//jsyoon
#include<stdarg.h>
#include "chprintf.h"
#include "memstreams.h"


int bldc_init(void);
int bldc_start(void);


/*
//jsyoon 2015.12.14
void spi_dac_hw_init(void);
void spi_dac_write_A( short data);
void spi_dac_write_B( short data);
void spi_dac_write_AB( short data);


// Function prototypes
void main_dma_adc_handler(void);
float main_get_last_adc_isr_duration(void);
void main_sample_print_data(bool at_start, uint16_t len, uint8_t decimation);
*/
#endif /* MAIN_H_ */
