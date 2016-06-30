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

#include "conf_general.h"
#include "datatypes.h"
#include <stdbool.h>

void spi_dac_hw_init(void);
void spi_dac_write_A( short data);
void spi_dac_write_B( short data);
void spi_dac_write_AB( short data);


//Defines
typedef unsigned short WORD;
//typedef signed int SFRAC16;
typedef unsigned char  BYTE;
//typedef unsigned char  BOOL;

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


//************** Start-Up Parameters **************

#define LOCKTIMEINSEC  0.25		// Initial rotor lock time in seconds
								// Make sure LOCKTIMEINSEC*(1.0/LOOPTIMEINSEC)
								// is less than 65535.
#define OPENLOOPTIMEINSEC 5.0	// Open loop time in seconds. This is the time that
								// will take from stand still to closed loop.
								// Optimized to overcome the brake inertia.
								// (Magtrol AHB-3 brake inertia = 6.89 kg x cm2).
#define INITIALTORQUE	1		// Initial Torque demand in Amps.
								// Enter initial torque demand in Amps using REFINAMPS() 
								// macro. Maximum Value for reference is defined by 
								// shunt resistor value and differential amplifier gain.
								// Use this equation to calculate maximum torque in 
								// Amperes:
								// 
								// Max REFINAMPS = (VDD/2)/(RSHUNT*DIFFAMPGAIN)
								//
								// For example:
								//
								// RSHUNT = 0.005
								// VDD = 3.3
								// DIFFAMPGAIN = 75
								//
								// Maximum torque reference in Amps is:
								//
								// (3.3/2)/(.005*75) = 4.4 Amperes, or REFINAMPS(4.4)
#define ENDSPEEDOPENLOOP MINSPEEDINRPM

//************** Motor Parameters **************

// Values used to test Bejing Eletechnic Motor "Dia 80-252140-220" at 320VDC input. Motor datasheet at www.eletechnic.com
#define POLEPAIRS      12      // Number of pole pairs
#define PHASERES		((float)0.15)	// Phase resistance in Ohms.
#define PHASEIND		((float)0.0006)// Phase inductance in Henrys.
#define NOMINALSPEEDINRPM 1000	// Make sure NOMINALSPEEDINRPM generates a MAXOMEGA < 1.0
		//50Hz   				// Use this formula:
								// MAXOMEGA = NOMINALSPEEDINRPM*SPEEDLOOPTIME*POLEPAIRS*2/60
								// If MAXOMEGA > 1.0, reduce NOMINALSPEEDINRPM or execute
								// speed loop faster by reducing SpeedLoopTime.
								// Maximum position of POT will set a reference of 
								// NOMINALSPEEDINRPM.
#define MINSPEEDINRPM	10	// Minimum speed in RPM. Closed loop will operate at this
		//8.47Hz						// speed. Open loop will transition to closed loop at
								// this minimum speed. Minimum POT position (CCW) will set
								// a speed reference of MINSPEEDINRPM
#define FIELDWEAKSPEEDRPM 1000	// Make sure FIELDWEAKSPEEDRPM generates a MAXOMEGA < 1.0
								// Use this formula:
								// MAXOMEGA = FIELDWEAKSPEEDRPM*SPEEDLOOPTIME*POLEPAIRS*2/60
								// If MAXOMEGA > 1.0, reduce FIELDWEAKSPEEDRPM or execute
								// speed loop faster by reducing SpeedLoopTime.
								// Maximum position of POT will set a reference of 
								// FIELDWEAKSPEEDRPM.

//************** PWM and Control Timing Parameters **********

#define PWMFREQUENCY	16000		// PWM Frequency in Hertz
#define DEADTIMESEC		0.000002f	// Deadtime in seconds
#define BUTPOLLOOPTIME	0.100f		// Button polling loop period in sec
#define SPEEDLOOPFREQ	1000		// Speed loop Frequency in Hertz. This value must
									// be an integer to avoid pre-compiler error
									// Use this value to test low speed motor

//************** Slide Mode Controller Parameters **********

#define SMCGAIN			0.85f		// Slide Mode Controller Gain (0.0 to 0.9999)
#define MAXLINEARSMC    0.005f		// If measured current - estimated current
								// is less than MAXLINEARSMC, the slide mode
								// Controller will have a linear behavior
								// instead of ON/OFF. Value from (0.0 to 0.9999)
#define FILTERDELAY		90		// Phase delay of two low pass filters for
								// theta estimation. Value in Degrees from
								// from 0 to 359.

//************** Hardware Parameters ****************

#define RSHUNT			0.001f		// Value in Ohms of shunt resistors used.
#define DIFFAMPGAIN		10		// Gain of differential amplifier.
#define VDD				3.3f		// VDD voltage, only used to convert torque
								// reference from Amps to internal variables

#define SPEEDDELAY  1000 * LOOPTIMEINSEC;	// Delay for the speed ramp.
					  // Necessary for the PI control to work properly at high speeds.

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

//************** ADC Scaling **************
// Scaling constants: Determined by calibration or hardware design. 
#define     DQK        (OMEGA10 - OMEGA1)/2.0	// POT Scaling
#define     DQKA       0.0008058608f	// Current feedback software gain : adc*(1/resol)*(AVDD/AmpGAIN)*(1/R) 
#define     DQKB       0.0008058608f	// Current feedback software gain : adc*(1/4096)*(3.3/10)*(1/0.001)

//************** Field Weakening **************
// Enter flux demand Amperes using REFINAMPS() macro. Maximum Value for
// reference is defined by shunt resistor value and differential amplifier gain.
// Use this equation to calculate maximum torque in Amperes:
// 
// Max REFINAMPS = (VDD/2)/(RSHUNT*DIFFAMPGAIN)
//
// For example:
//
// RSHUNT = 0.005
// VDD = 3.3
// DIFFAMPGAIN = 75
//
// Maximum torque reference in Amps is:
//
// (3.3/2)/(.005*75) = 4.4 Amperes, or REFINAMPS(4.4)
//
// in order to have field weakening, this reference value should be negative,
// so maximum value in this example is -4.4, or REFINAMPS(-4.4)

//****Values for Field weakening used to test Sander Motor at 160VDC input****
// For other motors, FW was not used to reach the rated speed
#define     dqKFw0  REFINAMPS(0)
#define     dqKFw1  REFINAMPS(-0.320)
#define     dqKFw2  REFINAMPS(-0.325)
#define     dqKFw3  REFINAMPS(-0.330)
#define     dqKFw4  REFINAMPS(-0.335)
#define     dqKFw5  REFINAMPS(-0.340)
#define     dqKFw6  REFINAMPS(-0.345)
#define     dqKFw7  REFINAMPS(-0.350)
#define     dqKFw8  REFINAMPS(-0.355)
#define     dqKFw9  REFINAMPS(-0.360)
#define     dqKFw10  REFINAMPS(-0.365)
#define     dqKFw11  REFINAMPS(-0.370)
#define     dqKFw12  REFINAMPS(-0.375)
#define     dqKFw13  REFINAMPS(-0.380)
#define     dqKFw14  REFINAMPS(-0.385)
#define     dqKFw15  REFINAMPS(-0.390)

//************** Derived Parameters ****************


#define LOOPTIMEINSEC (1.0/PWMFREQUENCY) // PWM Period = 1.0 / PWMFREQUENCY
#define IRP_PERCALC (unsigned int)(SPEEDLOOPTIME/LOOPTIMEINSEC)	// PWM loops per velocity calculation
#define SPEEDLOOPTIME (float)(1.0/SPEEDLOOPFREQ) // Speed Control Period
#define LOOPINTCY	 TIM1->ARR
#define LOCKTIME	(unsigned int)(LOCKTIMEINSEC*(1.0/LOOPTIMEINSEC))
// Time it takes to ramp from zero to MINSPEEDINRPM. Time represented in seconds
#define DELTA_STARTUP_RAMP	(unsigned int)(MINSPEEDINRPM*POLEPAIRS*LOOPTIMEINSEC* LOOPTIMEINSEC*65536*65536/(60*OPENLOOPTIMEINSEC))
// Number of control loops that must execute before the button routine is executed.
#define	BUTPOLLOOPCNT	(unsigned int)(BUTPOLLOOPTIME/LOOPTIMEINSEC)


#define		PI				3.14159265358979f
#define		SQRT2			1.414213562f
#define		SQRT3			1.732050808f
#define		INV_SQRT3		(float)(1./SQRT3)

#define REFINAMPS(Amperes_Value) (	Amperes_Value*DQKA*RSHUNT*DIFFAMPGAIN/(VDD/2))

// External variables
extern  uint16_t ADC_Value[];
extern   int SpeedReference;
//extern  unsigned int  switching_frequency_now;


#ifdef __cplusplus
extern "C" {
#endif

// Functions
bool SetupParm(void);

void MeasCompCurr( int curr1, int curr2 );
void InitMeasCompCurr( short Offset_a, short Offset_b );

//void InitPI( tPIParm *pParm);
//void CalcPI( tPIParm *pParm);

void SinCos(void);      // Calculate qSin,qCos from iAngle
void ClarkePark(void);  // Calculate qId,qIq from qCos,qSin,qIa,qIb
void InvPark(void);     // Calculate qValpha, qVbeta from qSin,qCos,qVd,qVq



void mcpwm_init(mc_configuration *configuration);
float mcpwm_get_rpm(void);
mc_state mcpwm_get_state(void);
mc_fault_code mcpwm_get_fault(void);
const char* mcpwm_fault_to_string(mc_fault_code fault);


//------------------------------------------------------------------------------------------------------------------------------------
//smc(sliding mode control )

//------------------------------------------------------------------------------------------------------------------------------------
//CalcRef.s 


void FWInit (void);
float FieldWeakening(float qMotorSpeed);



void CalcRefVec( void );
void CalcSVGen( void );
void CorrectPhase( void );
void update_timer_Duty(unsigned int duty_A,unsigned int duty_B,unsigned int duty_C);

float VoltRippleComp(float Vdq);

mc_rpm_dep_struct mcpwm_get_rpm_dep(void); 
const volatile mc_configuration* mcpwm_get_configuration(void);
void mcpwm_set_configuration(mc_configuration *configuration);


// Interrupt handlers
void mcpwm_adc_inj_int_handler(void);
void mcpwm_adc_int_handler(void *p, uint32_t flags);

#ifdef __cplusplus
}
#endif

#endif /* MC_PWM_H_ */
