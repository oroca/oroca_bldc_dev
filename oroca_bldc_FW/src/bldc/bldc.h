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
#include <stdarg.h>

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"

//#include "conf_general.h"

#define MCCONF_OUTRUNNER2



#include "hw.h"
#include "mcpwm.h"
#include "comm_usb.h"

//jsyoon
//#include "chprintf.h"
//#include "memstreams.h"


#ifdef __cplusplus
extern "C" {
#endif

int bldc_init(void);
int bldc_start(void);

#ifdef __cplusplus
}
#endif


#endif /* MAIN_H_ */
