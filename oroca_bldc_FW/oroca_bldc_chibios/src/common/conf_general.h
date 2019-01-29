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
 * conf_general.h
 *
 *  Created on: 14 apr 2014
 *      Author: benjamin
 */

#ifndef CONF_GENERAL_H_
#define CONF_GENERAL_H_

// Firmware version
#define FW_VERSION_MAJOR	2
#define FW_VERSION_MINOR	18

//#include "datatypes.h"

#include <stdint.h>
#include <stdbool.h>

#include "ch.h"
//#include "app.h"

#include "mc_interface.h"


#define PACKET_MAX_PL_LEN		1024

/*
 * Settings
 */

// Settings and parameters to override
//#define VIN_R1				33000.0
//#define VIN_R1				39200.0
//#define VIN_R2				2200.0
//#define CURRENT_AMP_GAIN	10.0
//#define CURRENT_SHUNT_RES	0.0005
//#define WS2811_ENABLE			1
//#define CURR1_DOUBLE_SAMPLE		0
//#define CURR2_DOUBLE_SAMPLE		0

/*
 * Select default user motor configuration
 */
//#define MCCONF_DEFAULT_USER		"mcconf_sten.h"
//#define MCCONF_DEFAULT_USER		"mcconf_sp_540kv.h"
//#define MCCONF_DEFAULT_USER		"mcconf_castle_2028.h"

/*
 * Select default user app configuration
 */
//#define APPCONF_DEFAULT_USER		"appconf_example_ppm.h"
//#define APPCONF_DEFAULT_USER		"appconf_custom.h"

/*
 * Select which custom application to use. To configure the default applications and
 * their settings, go to conf_general_read_app_configuration and enter the default init
 * values.
 */
//#define USE_APP_STEN

/*
 * Enable usb-serial
 */
#define USB_SERIAL_ENABLE				0



/*
 * Enable CAN-bus
 */
#define CAN_ENABLE				0

/*
 * Settings for the external LEDs (hardcoded for now)
 */
#define LED_EXT_BATT_LOW		28.0
#define LED_EXT_BATT_HIGH		33.0

/*
 * Output WS2811 signal on the HALL1 pin. Notice that hall sensors can't be used
 * at the same time.
 */
#ifndef WS2811_ENABLE
#define WS2811_ENABLE			0
#endif
#define WS2811_CLK_HZ			800000
#define WS2811_LED_NUM			28
#define WS2811_USE_CH2			1		// 0: CH1 (PB6) 1: CH2 (PB7)

/*
 * Servo output driver
 */
#ifndef SERVO_OUT_ENABLE
#define SERVO_OUT_ENABLE		0		// Enable servo output
#endif
#define SERVO_OUT_SIMPLE		1		// Use simple HW-based driver (recommended)
#define SERVO_OUT_PULSE_MIN_US	1000	// Minimum pulse length in microseconds
#define SERVO_OUT_PULSE_MAX_US	2000	// Maximum pulse length in microseconds
#define SERVO_OUT_RATE_HZ		50		// Update rate in Hz

// Functions
void conf_general_init(void);
//void conf_general_get_default_app_configuration(app_configuration *conf);
//void conf_general_get_default_mc_configuration(mcConfiguration_t *conf);
//void conf_general_read_app_configuration(app_configuration *conf);
//bool conf_general_store_app_configuration(app_configuration *conf);
//void conf_general_read_mc_configuration(mcConfiguration_t *conf);
//bool conf_general_store_mc_configuration(mcConfiguration_t *conf);


#endif /* CONF_GENERAL_H_ */
