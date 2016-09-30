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
 * mc_pwm.h
 *
 *  Created on: 13 okt 2012
 *      Author: benjamin
 */

#ifndef MCPWM_H_
#define MCPWM_H_

//#include "conf_general.h"
//#include "datatypes.h"
#include <stdbool.h>



#ifdef __cplusplus
extern "C" {
#endif



void spi_dac_hw_init(void);
void spi_dac_write_A( short data);
void spi_dac_write_B( short data);
void spi_dac_write_AB( short data);

#define SYSTEM_CORE_CLOCK		168000000

//Defines
typedef unsigned short WORD;
//typedef signed int SFRAC16;
typedef unsigned char  BYTE;
//typedef unsigned char  BOOL;

// Structs
typedef struct {
	float	qKa;	
	short	Offseta;

	float	qKb;   
	short	Offsetb;
} tMeasCurrParm;


typedef struct {
		float  Valpha;   		// Input: Stationary alfa-axis stator voltage
		float  Ealpha;   		// Variable: Stationary alfa-axis back EMF
		float  EalphaFinal;	// Variable: Filtered EMF for Angle calculation
		float  Zalpha;      	// Output: Stationary alfa-axis sliding control
		float  Gsmopos;    	// Parameter: Motor dependent control gain
		float  EstIalpha;   	// Variable: Estimated stationary alfa-axis stator current
		float  Fsmopos;    	// Parameter: Motor dependent plant matrix
		float  Vbeta;   		// Input: Stationary beta-axis stator voltage
		float  Ebeta;  		// Variable: Stationary beta-axis back EMF
		float  EbetaFinal;	// Variable: Filtered EMF for Angle calculation
		float  Zbeta;      	// Output: Stationary beta-axis sliding control
		float  EstIbeta;    	// Variable: Estimated stationary beta-axis stator current
		float  Ialpha;  		// Input: Stationary alfa-axis stator current
		float  IalphaError; 	// Variable: Stationary alfa-axis current error
		float  Kslide;     	// Parameter: Sliding control gain
		float  MaxSMCError;  	// Parameter: Maximum current error for linear SMC
		float  Ibeta;  		// Input: Stationary beta-axis stator current
		float  IbetaError;  	// Variable: Stationary beta-axis current error
		float  Kslf;       	// Parameter: Sliding control filter gain
		float  KslfFinal;    	// Parameter: BEMF Filter for angle calculation
		float  FiltOmCoef;   	// Parameter: Filter Coef for Omega filtered calc
		float  ThetaOffset;	// Output: Offset used to compensate rotor angle
		float  Theta;			// Output: Compensated rotor angle
		float  Omega;     	// Output: Rotor speed
		float  OmegaFltred;  	// Output: Filtered Rotor speed for speed PI
} SMC;

typedef struct {
    float   qdSum;          // 1.31 format
    float   qKp;
    float   qKi;
    float   qKc;
    float   qOutMax;
    float   qOutMin;
    float   qInRef; 
    float   qInMeas;
    float   qOut;
    } tPIParm;


typedef struct {
    float   qAngle;
    float   qSin;
    float   qCos;
    float   qIa;
    float   qIb;
    float   qIalpha;
    float   qIbeta;
    float   qId;
    float   qIq;
    float   qVd;
    float   qVq;
    float   qValpha;
    float   qVbeta;
    float   qV1;
    float   qV2;
    float   qV3;
    } tParkParm;

typedef struct {
    float   qVelRef;    // Reference velocity
    float   qVdRef;     // Vd flux reference value
    float   qVqRef;     // Vq torque reference value
    } tCtrlParm;

//------------------  C API for FdWeak routine ---------------------

typedef struct {
	float	qK1;            // < Nominal speed value
	float	qIdRef;
	float	qFwOnSpeed;
	float	qFwActiv;
	int	qIndex;
	float	qFWPercentage;
	float	qInterpolPortion;
	float		qFwCurve[16];	// Curve for magnetizing current
    } tFdWeakParm;

//------------------  C API for SVGen routine ---------------------

typedef struct {
	unsigned int   iPWMPeriod;

	float   qVr1;
	float   qVr2;
	float   qVr3;

	float T1;
	float T2;

	unsigned int Ta;
	unsigned int Tb;
	unsigned int Tc;

    } tSVGenParm;

#define False  0
#define True   1

#define MCPWM_DEAD_TIME_CYCLES			80		// Dead time
#define MCPWM_RPM_TIMER_FREQ			1000000.0	// Frequency of the RPM measurement timer
#define MCPWM_MIN_DUTY_CYCLE			0.005	// Minimum duty cycle
#define MCPWM_MAX_DUTY_CYCLE			0.95	// Maximum duty cycle
#define MCPWM_RAMP_STEP					0.01	// Ramping step (1000 times/sec) at maximum duty cycle
#define MCPWM_RAMP_STEP_CURRENT_MAX		0.04	// Maximum ramping step (1000 times/sec) for the current control
#define MCPWM_RAMP_STEP_RPM_LIMIT		0.0005	// Ramping step when limiting the RPM

#define SMC_DEFAULTS {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}



//************** PWM and Control Timing Parameters **********

#define PWMFREQUENCY	16000		// PWM Frequency in Hertz
#define SPEEDLOOPFREQ	1000		// Speed loop Frequency in Hertz. This value must
									// be an integer to avoid pre-compiler error
									// Use this value to test low speed motor

//************** Hardware Parameters ****************

#define RSHUNT			0.001f		// Value in Ohms of shunt resistors used.
#define VDD				3.3f		// VDD voltage, only used to convert torque

//*************** Optional Modes **************
//#define TORQUEMODE
//#define ENVOLTRIPPLE

//************** PI Coefficients **************

//******** D Control Loop Coefficients *******
#define     DKP        0.05
#define     DKI        0.01
#define     DKC        0.99999
#define     DOUTMAX    0.99999

//******** Q Control Loop Coefficients *******
#define     QKP        0.05
#define     QKI        0.01
#define     QKC        0.99999
#define     QOUTMAX    0.99999

//*** Velocity Control Loop Coefficients *****
#define     WKP       2.0
#define     WKI        0.01
#define     WKC        0.99999
#define     WOUTMAX    0.95


#define     DQKA       0.0008058608f	// Current feedback software gain : adc*(1/resol)*(AVDD/AmpGAIN)*(1/R) 
#define     DQKB       0.0008058608f	// Current feedback software gain : adc*(1/4096)*(3.3/10)*(1/0.001)

//************** Derived Parameters ****************


#define LOOPTIMEINSEC (1.0/PWMFREQUENCY) // PWM Period = 1.0 / PWMFREQUENCY
#define IRP_PERCALC (unsigned int)(SPEEDLOOPTIME/LOOPTIMEINSEC)	// PWM loops per velocity calculation
#define SPEEDLOOPTIME (float)(1.0/SPEEDLOOPFREQ) // Speed Control Period
#define LOOPINTCY	 TIM1->ARR


#define		PI				3.14159265358979f
#define		SQRT2			1.414213562f
#define		SQRT3			1.732050808f
#define		INV_SQRT3		(float)(1./SQRT3)

// External variables
extern  uint16_t ADC_Value[];
extern  unsigned int  switching_frequency_now;


//==============================================================
//define
#define Digital_PI_controller(out, ref, in, err0, limit, kp, ki, tsample)   \
		{						                                            \
			float err, tmp_kp, tmp_kpi;                                     \
			tmp_kp = (float)(kp);                                           \
			tmp_kpi = (float)(kp + ki*tsample);                             \
			err = ref - in;					                                \
			out += ((tmp_kpi * err) - (tmp_kp * err0));	                    \
			out = Bound_limit(out, limit);                                  \
			err0 = err;                                                     \
		}
#define Bound_limit(in,lim)	((in > (lim)) ? (lim) : ((in < -(lim)) ? -(lim) : in))
#define Bound_min_max(in, min, max)	((in > (max)) ? (max) : ((in < (min)) ? (min) : in))

#define Low_pass_filter(out, in, in_old, alpha)     \
		{												\
			float tmp;									\
			tmp = alpha*(in + in_old - (out*2)); \
			out += tmp; 								\
			in_old = in;								\
		}



// Functions
bool SetupParm(void);

void InitMeasCompCurr( short Offset_a, short Offset_b );

void SinCos(void);      // Calculate qSin,qCos from iAngle




void mcpwm_init(void);
float mcpwm_get_rpm(void);

float FieldWeakening(float qMotorSpeed);


void CalcRefVec( void );
void CorrectPhase( void );
void update_timer_Duty(unsigned int duty_A,unsigned int duty_B,unsigned int duty_C);

float VoltRippleComp(float Vdq);



void do_dc_cal(void);
void SMC_HallSensor_Estimation (SMC *s);
void CalcPI( tPIParm *pParm);
void DoControl( void );
void InitPI( tPIParm *pParm);
void SetupControlParameters(void);



// Interrupt handlers
void mcpwm_adc_int_handler(void *p, uint32_t flags);

#ifdef __cplusplus
}
#endif

#endif /* MC_PWM_H_ */
