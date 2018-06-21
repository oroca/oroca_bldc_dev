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
void SMC_HallSensor_Estimation (tSMC *s);


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



/********************************PLL loop **********************************/	

#if 0
void SMC_HallSensor_Estimation_filtered (tSMC *s)
{

	HallPLLA = ((float)ADC_Value[ADC_IND_SENS1] - 1241.0f)/ 4095.0f;
	HallPLLB = ((float)ADC_Value[ADC_IND_SENS2] - 1241.0f)/ 4095.0f;

	cos3th = cosf(3.0f * Theta);
	sin3th = sinf(3.0f * Theta);

	HallPLLA_sin3th = HallPLLA * sin3th * Gamma;
	HallPLLA_cos3th = HallPLLA * cos3th * Gamma;
	
	HallPLLB_cos3th = HallPLLB* cos3th * Gamma;
	HallPLLB_sin3th = HallPLLB * sin3th * Gamma;

	HallPLLA_cos3th_Integral += HallPLLA_cos3th;
	HallPLLA_sin3th_Integral += HallPLLA_sin3th;
	
	HallPLLB_sin3th_Integral += HallPLLB_sin3th;
	HallPLLB_cos3th_Integral += HallPLLB_cos3th;

	Asin3th= HallPLLA_sin3th_Integral * sin3th;
	Acos3th= HallPLLA_cos3th_Integral * cos3th;

	Bsin3th= HallPLLB_sin3th_Integral * sin3th;
	Bcos3th= HallPLLB_cos3th_Integral * cos3th;

	ANF_PLLA = HallPLLA - Asin3th - Acos3th;
	ANF_PLLB = HallPLLB - Bsin3th - Bcos3th;
	
	costh = cosf(Theta);
	sinth = sinf(Theta);
	
	Hall_SinCos = ANF_PLLA * costh;
	Hall_CosSin = ANF_PLLB * sinth;

	float err, tmp_kp, tmp_kpi; 									
	tmp_kp = 1.0f;
	tmp_kpi = (1.0f + 1.0f * Tsamp);
	err = Hall_SinCos - Hall_CosSin; 											
	Hall_PIout += ((tmp_kpi * err) - (tmp_kp * Hall_Err0)); 					
	Hall_PIout = Bound_limit(Hall_PIout, 10.0f);						
	Hall_Err0= err;									
	
	Theta += Hall_PIout ;
	if((2.0f * PI) < Theta) Theta = Theta - (2.0f * PI);
	else if(Theta < 0.0f) Theta = (2.0f * PI) + Theta;

	s->Theta= Theta + 0.3f;

	if((2.0f * PI) < s->Theta) s->Theta = s->Theta - (2.0f * PI);
	else if(s->Theta < 0.0f) s->Theta = (2.0f * PI) + s->Theta;

	s->Omega = Hall_PIout;
	//Futi   = Hall_PIout / (2.* PI) *Fsamp;

	//spi_dac_write_A((HallPLLA+ 1.0f) * 200.0f);
	//spi_dac_write_B((HallPLLB+ 1.0f) * 200.0f);

	//spi_dac_write_A((costh + 1.0f) * 2000.0f);
	//spi_dac_write_B((sinth + 1.0f) * 2047.0f);

	//spi_dac_write_A( (Hall_SinCos+ 1.0f) * 2048.0f);
	//spi_dac_write_B( (Hall_CosSin+ 1.0f) * 2048.0f);

	//spi_dac_write_A( (Hall_err+ 1.0f) * 2048.0f);
	//spi_dac_write_B( (Theta * 200.0f) );

	//spi_dac_write_B( Hall_PIout * 100.0f);


	//spi_dac_write_A( (ParkParm.qAngle * 200.0f) );
	//spi_dac_write_B( (smc1.Theta * 200.0f) );


	//s->Omega = Wpll;
	//s->Theta =Theta;

	//DAC_SetChannel1Data(uint32_t DAC_Align, uint16_t Data)
}

#else
void SMC_HallSensor_Estimation (tSMC *s)
{
	s->HallPLLA = ((float)ADC_Value[ADC_IND_SENS1] - 1241.0f)/ 4095.0f;
	s->HallPLLB = ((float)ADC_Value[ADC_IND_SENS2] - 1241.0f)/ 4095.0f;

	s->costh = cosf(s->Theta);
	s->sinth = sinf(s->Theta);
	
	s->Hall_SinCos = s->HallPLLA * s->costh;
	s->Hall_CosSin = s->HallPLLB * s->sinth;

	float err, tmp_kp, tmp_kpi; 									
	tmp_kp = 1.0f;
	tmp_kpi = (1.0f + 1.0f * HALL_SENSOR_PEROID);
	err = s->Hall_SinCos - s->Hall_CosSin; 											
	s->Hall_PIout += ((tmp_kpi * err) - (tmp_kp * s->Hall_Err0)); 					
	s->Hall_PIout = Bound_limit(s->Hall_PIout, 10.0f);						
	s->Hall_Err0= err;									
	
	s->Theta += s->Hall_PIout ;
	if((2.0f * PI) < s->Theta) s->Theta = s->Theta - (2.0f * PI);
	else if(s->Theta < 0.0f) s->Theta = (2.0f * PI) + s->Theta;

	s->ThetaCal= s->Theta + 0.3f;

	if((2.0f * PI) < s->ThetaCal) s->ThetaCal = s->ThetaCal - (2.0f * PI);
	else if(s->ThetaCal < 0.0f) s->ThetaCal = (2.0f * PI) + s->ThetaCal;

	s->Omega = s->Hall_PIout;


	s->trueTheta += (s->Hall_PIout /7.0f) ;
	if((2.0f * PI) < s->trueTheta) s->trueTheta = s->trueTheta - (2.0f * PI);
	else if(s->trueTheta < 0.0f) s->trueTheta = (2.0f * PI) + s->trueTheta;

	s->Futi   = s->Hall_PIout / (2.0f * PI) * HALL_SENSOR_FREQ;
	s->rpm = 120.0f * s->Futi / 7.0f;
	

	//spi_dac_write_A((HallPLLA+ 1.0f) * 200.0f);
	//spi_dac_write_B((HallPLLB+ 1.0f) * 200.0f);

	//spi_dac_write_A((costh + 1.0f) * 2000.0f);
	//spi_dac_write_B((sinth + 1.0f) * 2047.0f);

	//spi_dac_write_A( (Hall_SinCos+ 1.0f) * 2048.0f);
	//spi_dac_write_B( (Hall_CosSin+ 1.0f) * 2048.0f);

	//spi_dac_write_A( (Hall_err+ 1.0f) * 2048.0f);
	//spi_dac_write_B( (Theta * 200.0f) );

	//spi_dac_write_B( Hall_PIout * 100.0f);


	//spi_dac_write_A( (ParkParm.qAngle * 200.0f) );
	//spi_dac_write_B( (smc1.Theta * 200.0f) );


	//s->Omega = Wpll;
	//s->Theta =Theta;

	//DAC_SetChannel1Data(uint32_t DAC_Align, uint16_t Data)
}

#endif


