/*
	Copyright 2015 Benjamin Vedder	benjamin@vedder.se

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
 * mc_interface.h
 *
 *  Created on: 10 okt 2015
 *      Author: benjamin
 */

#ifndef MC_INTERFACE_H_
#define MC_INTERFACE_H_

#include "conf_general.h"
#include "hw.h"

// Functions
void mc_interface_init(mc_configuration *configuration);
const volatile mc_configuration* mc_interface_get_configuration(void);
void mc_interface_set_configuration(mc_configuration *configuration);
void mc_interface_set_pwm_callback(void (*p_func)(void));
void mc_interface_lock(void);
void mc_interface_unlock(void);
void mc_interface_lock_override_once(void);




#endif /* MC_INTERFACE_H_ */
