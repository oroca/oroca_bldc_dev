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

#ifndef MCPWM_H_
#define MCPWM_H_

#include "ch.h"
#include "hal.h"

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif
//Defines

#define SYSTEM_CORE_CLOCK		168000000

#define False  0
#define True   1

typedef unsigned short WORD;
//typedef signed int SFRAC16;
typedef unsigned char  BYTE;
//typedef unsigned char  BOOL;


typedef struct {
	float	qKa;	
	short	Offseta;

	float	qKb;   
	short	Offsetb;
} tMeasCurrParm;

typedef struct {
	float HallPLLlead  ;
	float HallPLLlead1 ;
	float HallPLLlead2 ;
	float HallPLLqe    ;
	float HallPLLde    ;
	float HallPLLde1   ;
	float HallPLLdef   ;
	float HallPLLdef1  ;

	float Wpll	 		;
	float Wpll1	 		;
	float Wpllp	 		;
	float Wplli	 		;

	float Kpll      	;	// = 0.428;
	float Ipll       	;	//= 28.83;

	float Hall_KA 		;	
	float Hall_KB 		;	
	
	float Hall_PIout 	;	
	float Hall_Err0 	;	

	float HallPLLA	;	
	float HallPLLA1 ;
	float HallPLLB	;

	float HallPLLA_cos3th;
	float HallPLLA_sin3th;
	float HallPLLB_sin3th;
	float HallPLLB_cos3th;

	float HallPLLA_cos3th_Integral;
	float HallPLLA_sin3th_Integral;
	float HallPLLB_sin3th_Integral;
	float HallPLLB_cos3th_Integral;

	float HallPLLA_old ;
	float HallPLLB_old ;

	float HallPLLA_filtered;
	float HallPLLB_filtered;

	float Hall_SinCos;
	float Hall_CosSin;

	float Gamma; //= 1.0f;

	float costh;
	float sinth;

	float Asin3th	;// = 0.0f;
	float Acos3th	;// = 0.0f;
	float Bsin3th	;//= 0.0f;
	float Bcos3th	;//= 0.0f;
	float ANF_PLLA	;//= 0.0f;
	float ANF_PLLB	;//= 0.0f;

	float cos3th;
	float sin3th;

	float Theta	 	;
	float ThetaCal	;
	float trueTheta	;
	float Futi	 	;
	float Omega;     	// Output: Rotor speed
	float rpm;     	// Output: Rotor speed
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



// Data types
typedef enum {
   MC_STATE_OFF = 0,
   MC_STATE_DETECTING,
   MC_STATE_RUNNING,
   MC_STATE_FULL_BRAKE,
} mc_state;

typedef enum {
	PWM_MODE_NONSYNCHRONOUS_HISW = 0, // This mode is not recommended
	PWM_MODE_SYNCHRONOUS, // The recommended and most tested mode
	PWM_MODE_BIPOLAR // Some glitches occasionally, can kill MOSFETs
} mc_pwm_mode;

typedef enum {
	COMM_MODE_INTEGRATE = 0,
	COMM_MODE_DELAY
} mc_comm_mode;

typedef enum {
	SENSOR_MODE_SENSORLESS = 0,
	SENSOR_MODE_SENSORED,
	SENSOR_MODE_HYBRID
} mc_sensor_mode;

typedef enum {
	FOC_SENSOR_MODE_SENSORLESS = 0,
	FOC_SENSOR_MODE_ENCODER,
	FOC_SENSOR_MODE_HALL
} mc_foc_sensor_mode;

typedef enum {
	MOTOR_TYPE_BLDC = 0,
	MOTOR_TYPE_DC,
	MOTOR_TYPE_FOC
} mc_motor_type;

typedef enum {
	FAULT_CODE_NONE = 0,
	FAULT_CODE_OVER_VOLTAGE,
	FAULT_CODE_UNDER_VOLTAGE,
	FAULT_CODE_DRV8302,
	FAULT_CODE_ABS_OVER_CURRENT,
	FAULT_CODE_OVER_TEMP_FET,
	FAULT_CODE_OVER_TEMP_MOTOR
} mc_fault_code;

typedef enum {
	CONTROL_MODE_DUTY = 0,
	CONTROL_MODE_SPEED,
	CONTROL_MODE_CURRENT,
	CONTROL_MODE_CURRENT_BRAKE,
	CONTROL_MODE_POS,
	CONTROL_MODE_NONE
} mc_control_mode;

typedef enum {
	DISP_POS_MODE_NONE = 0,
	DISP_POS_MODE_INDUCTANCE,
	DISP_POS_MODE_OBSERVER,
	DISP_POS_MODE_ENCODER,
	DISP_POS_MODE_PID_POS,
	DISP_POS_MODE_PID_POS_ERROR,
	DISP_POS_MODE_ENCODER_OBSERVER_ERROR
} disp_pos_mode;

typedef enum {
	SENSOR_PORT_MODE_HALL = 0,
	SENSOR_PORT_MODE_ABI,
	SENSOR_PORT_MODE_AS5047_SPI
} sensor_port_mode;

typedef struct {
	mc_pwm_mode pwm_mode;
	mc_comm_mode comm_mode;
	mc_motor_type motor_type;
	mc_sensor_mode sensor_mode;

	// Limits
	float l_current_max;
	float l_current_min;
	float l_in_current_max;
	float l_in_current_min;
	float l_abs_current_max;
	float l_min_erpm;
	float l_max_erpm;
	float l_erpm_start;
	float l_max_erpm_fbrake;
	float l_max_erpm_fbrake_cc;
	float l_min_vin;
	float l_max_vin;
	float l_battery_cut_start;
	float l_battery_cut_end;
	bool l_slow_abs_current;
	float l_temp_fet_start;
	float l_temp_fet_end;
	float l_temp_motor_start;
	float l_temp_motor_end;
	float l_min_duty;
	float l_max_duty;
	float l_watt_max;
	float l_watt_min;
	// Overridden limits (Computed during runtime)
	float lo_current_max;
	float lo_current_min;
	float lo_in_current_max;
	float lo_in_current_min;
	float lo_current_motor_max_now;
	float lo_current_motor_min_now;


	sensor_port_mode m_sensor_port_mode;
	uint32_t m_encoder_counts;

	//******** D Control Loop Coefficients *******
	float     dkp;        //0.02
	float     dki;        //0.05
	float     dkc;        //0.99999
	float     dout_max;   //0.99999
	//******** Q Control Loop Coefficients *******
	float     qkp;        //0.02
	float     qki;        //0.05
	float     qkc;        //0.99999
	float     qout_max;    //0.99999

	//*** Velocity Control Loop Coefficients *****
	float     wkp;       //12.0
	float     wki;        //2.0
	float     wkc;        //0.99999
	float     wout_max;    //0.95

	//*** analog hallsensor Coefficients *****
	float     pll_kp;       //2.0
	float     pll_ki;	   //0.01
	float     pll_kc;       //0.99999
	float     pllout_max;    //0.95

	//*** current sensor gain *****
	float     dqk_a;       //0.0008058608f
	float     dqk_b;       //0.0008058608f

} mc_configuration;


#define MCPWM_DEAD_TIME_CYCLES			80		// Dead time
#define MCPWM_RPM_TIMER_FREQ			1000000.0	// Frequency of the RPM measurement timer
#define MCPWM_MIN_DUTY_CYCLE			0.005	// Minimum duty cycle
#define MCPWM_MAX_DUTY_CYCLE			0.95	// Maximum duty cycle
#define MCPWM_RAMP_STEP					0.01	// Ramping step (1000 times/sec) at maximum duty cycle
#define MCPWM_RAMP_STEP_CURRENT_MAX		0.04	// Maximum ramping step (1000 times/sec) for the current control
#define MCPWM_RAMP_STEP_RPM_LIMIT		0.0005	// Ramping step when limiting the RPM

//************** PWM and Control Timing Parameters **********

#define PWMFREQUENCY	16000		// PWM Frequency in Hertz
#define PWMPEROID           1.0f/PWMFREQUENCY
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
#define     DKP        0.02
#define     DKI        0.05
#define     DKC        0.99999
#define     DOUTMAX    0.99999

//******** Q Control Loop Coefficients *******
#define     QKP        0.02
#define     QKI        0.05
#define     QKC        0.99999
#define     QOUTMAX    0.99999

//*** Velocity Control Loop Coefficients *****
#define     WKP       12.0
#define     WKI        2.0
#define     WKC        0.99999
#define     WOUTMAX    0.95

#define     PLLKP       2.0
#define     PLLKI        0.01
#define     PLLKC        0.99999
#define     PLLOUTMAX    0.95

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


#define WMd      2.*3.141592654*180.
#define AMd      (WMd-(2./PWMPEROID))/(WMd+(2./PWMPEROID))
#define BMd      WMd/(WMd+(2./PWMPEROID))


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

extern tCtrlParm CtrlParm;
extern tParkParm ParkParm;
extern SMC smc1;


// Functions
bool SetupParm(void);

void mcpwm_init(volatile mc_configuration *configuration);
void mcpwm_deinit(void);
void mcpwm_set_configuration(volatile mc_configuration *configuration);

float mcpwm_get_rpm(void);

float FieldWeakening(float qMotorSpeed);


void CalcRefVec( void );
void CorrectPhase( void );
void update_timer_Duty(unsigned int duty_A,unsigned int duty_B,unsigned int duty_C);


void do_dc_cal(void);
void SMC_HallSensor_Estimation (SMC *s);
void CalcPI( tPIParm *pParm);
void DoControl( void );
void InitPI( tPIParm *pParm);
void SetupControlParameters(void);

void CalcSVGen( void );

// Interrupt handlers
void mcpwm_adc_dma_int_handler(void *p, uint32_t flags);

#ifdef __cplusplus
}
#endif

#endif /* MC_PWM_H_ */
