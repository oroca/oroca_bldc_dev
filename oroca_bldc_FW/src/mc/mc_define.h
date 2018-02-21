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

#ifndef _MCPWM_DEFINE_H_
#define _MCPWM_DEFINE_H_

#define MCPWM_DEAD_TIME_CYCLES			80		// Dead time
#define MCPWM_RPM_TIMER_FREQ			1000000.0	// Frequency of the RPM measurement timer
#define MCPWM_MIN_DUTY_CYCLE			0.005	// Minimum duty cycle
#define MCPWM_MAX_DUTY_CYCLE			0.95	// Maximum duty cycle
#define MCPWM_RAMP_STEP					0.01	// Ramping step (1000 times/sec) at maximum duty cycle
#define MCPWM_RAMP_STEP_CURRENT_MAX		0.04	// Maximum ramping step (1000 times/sec) for the current control
#define MCPWM_RAMP_STEP_RPM_LIMIT		0.0005	// Ramping step when limiting the RPM

//************** PWM and Control Timing Parameters **********
#define PWMFREQ			16000		// PWM Frequency in Hertz
#define PWMPEROID   	1.0f/PWMFREQ

#define CURR_CTRL_FREQ		8000		// current control Frequency in Hertz
#define CURR_CTRL_PEROID   	1.0f/CURR_CTRL_FREQ
#define CURR_CTRL_DIV   PWMFREQ/CURR_CTRL_FREQ


#define	SPD_CTRL_FREQ		1000		// speed control Frequency in Hertz
#define SPD_CTRL_PEROID   	1.0f/SPD_CTRL_FREQ
#define SPD_CTRL_DIV	PWMFREQ/SPD_CTRL_FREQ


#define	HALL_SENSOR_FREQ	1000		// hall sensor measure Frequency in Hertz
#define HALL_SENSOR_PEROID  1.0f/HALL_SENSOR_FREQ
#define HALL_SENSOR_DIV	PWMFREQ/HALL_SENSOR_FREQ



//************** Hardware Parameters ****************

#define RSHUNT			0.001f	
#define VDD				3.3f	

//************** PI Coefficients **************
#define	DKP        0.02
#define	DKI        0.05
#define	DKC        0.99999
#define	DOUTMAX    0.99999

#define	QKP        0.02
#define	QKI        0.05
#define	QKC        0.99999
#define	QOUTMAX    0.99999

#define	WKP       12.0
#define	WKI        2.0
#define	WKC        0.99999
#define	WOUTMAX    0.95

#define	PLLKP       2.0
#define	PLLKI        0.01
#define	PLLKC        0.99999
#define	PLLOUTMAX    0.95

#define	DQKA       0.0008058608f	// Current feedback software gain : adc*(1/resol)*(AVDD/AmpGAIN)*(1/R) 
#define	DQKB       0.0008058608f	// Current feedback software gain : adc*(1/4096)*(3.3/10)*(1/0.001)

//************** Derived Parameters ****************
#define	PI				3.14159265358979f
#define	SQRT2			1.414213562f
#define	SQRT3			1.732050808f
#define	INV_SQRT3		(float)(1./SQRT3)

#define WMd 2.0f * PI * 180.0f
#define AMd (WMd-(2./PWMPEROID))/(WMd+(2./PWMPEROID))
#define BMd WMd/(WMd+(2./PWMPEROID))

#endif /* MC_PWM_H_ */
