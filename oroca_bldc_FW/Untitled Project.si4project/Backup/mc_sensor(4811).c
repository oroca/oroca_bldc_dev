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
 * mcpwm.c
 *
 *  Created on: 13 okt 2012
 *      Author: bakchajang
 */

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"

#include "hw.h"
#include "mc_define.h"
#include "mc_typedef.h"

#include "mc_interface.h"
#include "mc_control.h"
#include "mc_sensor.h"
#include "mc_pwm.h"

//#include "utils.h"

#include <math.h>

//======================================================================================
//private variable Declaration

tSMC smc1;
tMeasCurrParm MeasCurrParm;
tMeasSensorValue MeasSensorValue;


bool do_dc_cal(void);



uint16_t curr0_sum;
uint16_t curr1_sum;
uint16_t curr_start_samples;
bool do_dc_cal(void)
{
	uint16_t fault_cnt=0;
	DCCAL_ON();
	
	while(IS_DRV_FAULT())
	{
		fault_cnt++;
		if(5 < fault_cnt)
		{
			return false;
		}
		
		chThdSleepMilliseconds(1000);
	};
	
	curr0_sum = 0;
	curr1_sum = 0;
	curr_start_samples = 0;
	
	chThdSleepMilliseconds(1000);

	MeasCurrParm.Offseta = curr0_sum / curr_start_samples;
	MeasCurrParm.Offsetb = curr1_sum / curr_start_samples;

	DCCAL_OFF();
	
	return true;

	//Uart3_printf(&SD3, (uint8_t *)"curr0_offset : %u\r\n",curr0_offset);//170530  
	//Uart3_printf(&SD3, (uint8_t *)"curr1_offset : %u\r\n",curr1_offset);//170530  
}





