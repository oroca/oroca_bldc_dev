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
 * mc_pwm.h
 *
 *  Created on: 13 okt 2012
 *      Author: bakchajang
 */

#ifndef _MC_PWM_H_
#define _MC_PWM_H_


//======================================================================================
//public function Declaration
#ifdef __cplusplus
extern "C" {
#endif

// Functions
void mcpwm_init(volatile mcConfiguration_t *configuration);
void mcpwm_deinit(void);

void update_timer_Duty(unsigned int duty_A,unsigned int duty_B,unsigned int duty_C);

#ifdef __cplusplus
}
#endif


#endif
