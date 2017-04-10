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
 * app.h
 *
 *  Created on: 18 apr 2014
 *      Author: benjamin
 */

#ifndef APP_H_
#define APP_H_

#include "app_ppm.h"

// Applications to use
typedef enum {
	APP_NONE = 0,
	APP_PPM,
	APP_ADC,
	APP_UART,
	APP_PPM_UART,
	APP_ADC_UART,
	APP_NUNCHUK,
	APP_NRF,
	APP_CUSTOM
} app_use;


#ifdef __cplusplus
extern "C" {
#endif
// Standard apps
void app_init(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_H_ */
