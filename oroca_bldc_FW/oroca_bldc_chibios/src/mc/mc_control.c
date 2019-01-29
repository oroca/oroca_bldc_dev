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

#include <math.h>

#include "hw.h"
#include "mc_define.h"
#include "mc_typedef.h"

#include "mc_interface.h"
#include "mc_control.h"
#include "mc_sensor.h"
#include "mc_encoder.h"
#include "mc_pwm.h"


//#include "utils.h"




//======================================================================================
//private variable Declaration

tPIParm PIParmD;
tPIParm PIParmQ;
tPIParm PIParmW;	
tPIParm PIParmPLL;

tMcCtrlBits McCtrlBits;
tCtrlParm CtrlParm;
tParkParm ParkParm;
tSVGenParm SVGenParm;
tFdWeakParm FdWeakParm;

void CalcPI( tPIParm *pParm);
void DoControl( void );
void InitPI( tPIParm *pParm);
void CalcSVGen( void );

//---------------------------------------------------------------------
// Executes one PI itteration for each of the three loops Id,Iq,Speed,
float qVdSquared = 0.0f;

void CurrentControl( void )
{
	//if (uGF.bit.EnTorqueMod)
	//	CtrlParm.qVqRef = CtrlParm.qVelRef;

	//CtrlParm.qVdRef = FieldWeakening(fabsf(CtrlParm.qVelRef));

	// PI control for D
	PIParmD.qInMeas = ParkParm.qId;
	PIParmD.qInRef	= CtrlParm.qVdRef;
	CalcPI(&PIParmD);

	//if(uGF.bit.EnVoltRipCo)
	//	ParkParm.qVd = VoltRippleComp(PIParmD.qOut);
	//else
		ParkParm.qVd = PIParmD.qOut;

	qVdSquared = ParkParm.qVd * ParkParm.qVd;
	PIParmQ.qOutMax = sqrtf((0.95*0.95) - qVdSquared);
	PIParmQ.qOutMin = -PIParmQ.qOutMax;

	// PI control for Q
	PIParmQ.qInMeas = ParkParm.qIq;
	PIParmQ.qInRef	= CtrlParm.qVqRef;
	CalcPI(&PIParmQ);

	// If voltage ripple compensation flag is set, adjust the output
	// of the Q controller depending on measured DC Bus voltage
	//if(uGF.bit.EnVoltRipCo)
	//	ParkParm.qVq = VoltRippleComp(PIParmQ.qOut);
	//else
		ParkParm.qVq = PIParmQ.qOut;

}

void SpeedControl( void )
{
	// Execute the velocity control loop
	PIParmW.qInMeas = smc1.Omega;
	PIParmW.qInRef	= CtrlParm.qVelRef;
	CalcPI(&PIParmW);
	//CtrlParm.qVqRef = PIParmW.qOut;
}

void InitPI( tPIParm *pParm)
{
	pParm->qdSum=0;
	pParm->qOut=0;

}
void CalcPI( tPIParm *pParm)
{
	float U,Exc,Err;
	Err  = pParm->qInRef - pParm->qInMeas;
	
	U  = pParm->qdSum + pParm->qKp * Err;

	if( U > pParm->qOutMax )          pParm->qOut = pParm->qOutMax;
	else if( U < pParm->qOutMin )    pParm->qOut = pParm->qOutMin;
	else                  pParm->qOut = U ;

	Exc = U - pParm->qOut;

	pParm->qdSum = pParm->qdSum + pParm->qKi * Err - pParm->qKc * Exc ;
	
	return;
}


void SetupControlParameters(void)
{
	MeasCurrParm.qKa    = DQKA;
	MeasCurrParm.qKb    = DQKB;

	// Initial Current offsets
	MeasCurrParm.Offseta = 0;//curr0_offset;
	MeasCurrParm.Offsetb = 0;//curr1_offset;

	// ============= SVGen ===============
	// Set PWM period to Loop Time
	SVGenParm.iPWMPeriod = TIM1->ARR;

	CtrlParm.qVelRef = 0.0f;
	CtrlParm.qVdRef = 0.0f;
	CtrlParm.qVqRef = 0.0f;

	// ============= PI D Term ===============
	PIParmD.qKp = DKP;
	PIParmD.qKi = DKI;
	PIParmD.qKc = DKC;
	PIParmD.qOutMax = DOUTMAX;
	PIParmD.qOutMin = -PIParmD.qOutMax;

	InitPI(&PIParmD);

	// ============= PI Q Term ===============
	PIParmQ.qKp = QKP;
	PIParmQ.qKi = QKI;
	PIParmQ.qKc = QKC;
	PIParmQ.qOutMax = QOUTMAX;
	PIParmQ.qOutMin = -PIParmQ.qOutMax;

	InitPI(&PIParmQ);

	// ============= PI W Term ===============
	PIParmW.qKp = WKP;
	PIParmW.qKi = WKI;
	PIParmW.qKc = WKC;
	PIParmW.qOutMax = WOUTMAX;
	PIParmW.qOutMin = -PIParmW.qOutMax;

	InitPI(&PIParmW);

	// ============= PI PLL Term ===============
	PIParmPLL.qKp = PLLKP;		 
	PIParmPLL.qKi = PLLKI;		 
	PIParmPLL.qKc = PLLKC;		 
	PIParmPLL.qOutMax = PLLOUTMAX;	 
	PIParmPLL.qOutMin = -PIParmPLL.qOutMax;

	InitPI(&PIParmPLL);

}



void CalcTimes(void)
{
	SVGenParm.T1 = ((float)SVGenParm.iPWMPeriod * SVGenParm.T1);
	SVGenParm.T2 = ((float)SVGenParm.iPWMPeriod * SVGenParm.T2);
	SVGenParm.Tc = (((float)SVGenParm.iPWMPeriod-SVGenParm.T1-SVGenParm.T2)/2);
	SVGenParm.Tb = SVGenParm.Tc + SVGenParm.T1;
	SVGenParm.Ta = SVGenParm.Tb + SVGenParm.T2 ;

	return;
}  

void CalcSVGen( void )
{ 
	if( SVGenParm.qVr1 >= 0 )
	{       
		// (xx1)
		if( SVGenParm.qVr2 >= 0 )
		{
			// (x11)
			// Must be Sector 3 since Sector 7 not allowed
			// Sector 3: (0,1,1)  0-60 degrees
			SVGenParm.T2 = SVGenParm.qVr2;
			SVGenParm.T1 = SVGenParm.qVr1;
			CalcTimes();
			update_timer_Duty(SVGenParm.Ta,SVGenParm.Tb,SVGenParm.Tc) ;
		}
		else
		{            
			// (x01)
			if( SVGenParm.qVr3 >= 0 )
			{
				// Sector 5: (1,0,1)  120-180 degrees
				SVGenParm.T2 = SVGenParm.qVr1;
				SVGenParm.T1 = SVGenParm.qVr3;
				CalcTimes();
				update_timer_Duty(SVGenParm.Tc,SVGenParm.Ta,SVGenParm.Tb) ;

			}
			else
			{
				// Sector 1: (0,0,1)  60-120 degrees
				SVGenParm.T2 = -SVGenParm.qVr2;
				SVGenParm.T1 = -SVGenParm.qVr3;
				CalcTimes();
				update_timer_Duty(SVGenParm.Tb,SVGenParm.Ta,SVGenParm.Tc) ;
			}
		}
	}
	else
	{
		// (xx0)
		if( SVGenParm.qVr2 >= 0 )
		{
			// (x10)
			if( SVGenParm.qVr3 >= 0 )
			{
				// Sector 6: (1,1,0)  240-300 degrees
				SVGenParm.T2 = SVGenParm.qVr3;
				SVGenParm.T1 = SVGenParm.qVr2;
				CalcTimes();
				update_timer_Duty(SVGenParm.Tb,SVGenParm.Tc,SVGenParm.Ta) ;
			}
			else
			{
				// Sector 2: (0,1,0)  300-0 degrees
				SVGenParm.T2 = -SVGenParm.qVr3;
				SVGenParm.T1 = -SVGenParm.qVr1;
				CalcTimes();
				update_timer_Duty(SVGenParm.Ta,SVGenParm.Tc,SVGenParm.Tb) ;
			}
		}
		else
		{            
			// (x00)
			// Must be Sector 4 since Sector 0 not allowed
			// Sector 4: (1,0,0)  180-240 degrees
			SVGenParm.T2 = -SVGenParm.qVr1;
			SVGenParm.T1 = -SVGenParm.qVr2;
			CalcTimes();
			update_timer_Duty(SVGenParm.Tc,SVGenParm.Tb,SVGenParm.Ta) ;
		}
	}

}


