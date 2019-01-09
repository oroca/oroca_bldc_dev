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
 * hw.h
 *
 *  Created on: 12 apr 2014
 *      Author: bakchajang
 */

#ifndef HW_H_
#define HW_H_

#include "stm32f4xx_conf.h"
#include "conf_general.h"

/*
 * Select only one hardware version
 */
//#if !defined(HW_VERSION_OROCA)
//#define HW_VERSION_40
//#define HW_VERSION_45
//#define HW_VERSION_46 // Also for 4.7
//#define HW_VERSION_48
//#define HW_VERSION_49
//#define HW_VERSION_410 // Also for 4.11 and 4.12
//#define HW_VERSION_R2
//#define HW_VERSION_VICTOR_R1A
#define HW_VERSION_OROCA
//#endif


#ifdef HW_VERSION_40
#include "hw_40.h"
#elif defined HW_VERSION_45
#include "hw_45.h"
#elif defined HW_VERSION_46
#include "hw_46.h"
#elif defined HW_VERSION_48
#include "hw_48.h"
#elif defined HW_VERSION_49
#include "hw_49.h"
#elif defined HW_VERSION_410
#include "hw_410.h"
#elif defined HW_VERSION_R2
#include "hw_r2.h"
#elif defined HW_VERSION_VICTOR_R1A
#include "hw_victor_r1a.h"
#elif defined HW_VERSION_OROCA
#include "hw_oroca.h"
#else
#error "No hardware version defined"
//#include "hw_oroca.h"
#endif

// Functions
void hw_init_gpio(void);
void hw_setup_adc_channels(void);
void hw_setup_servo_outputs(void);
void hw_start_i2c(void);
void hw_stop_i2c(void);
void hw_try_restore_i2c(void);

#endif /* HW_H_ */
