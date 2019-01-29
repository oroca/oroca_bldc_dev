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

#ifndef _MCPWM_TYPEDEF_H_
#define _MCPWM_TYPEDEF_H_

#ifdef __cplusplus
extern "C" {
#endif


typedef enum {
   MC_STATE_OFF = 0,
   MC_STATE_DETECTING,
   MC_STATE_RUNNING,
   MC_STATE_FULL_BRAKE,
} mc_state;

typedef enum {
	SENSOR_MODE_SENSORLESS = 0,
	SENSOR_MODE_SENSORED,
	SENSOR_MODE_HYBRID
} mc_sensor_mode;

typedef enum {
	MOTOR_TYPE_BLDC = 0,
	MOTOR_TYPE_DC
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
	CONTROL_MODE_SETUP = 0,
	CONTROL_MODE_CURRENT,
	CONTROL_MODE_SPEED,
	CONTROL_MODE_POSITION,
	CONTROL_MODE_NONE
} mc_control_mode;


typedef enum {
	SENSOR_PORT_MODE_HALL = 0,
	SENSOR_PORT_MODE_ABI,
	SENSOR_PORT_MODE_AS5047_SPI
} sensor_port_mode;

// Logged fault data
typedef struct {
	mc_fault_code fault;
	float current;
	float current_filtered;
	float voltage;
	float duty;
	float rpm;
	int16_t tacho;
	int16_t cycles_running;
	int16_t tim_val_samp;
	int16_t tim_current_samp;
	int16_t tim_top;
	int16_t comm_step;
	float temperature;
	int16_t drv8302_faults;
} mcFaultData_t;


typedef struct {
	float vdd;
	float rshunt;
	uint16_t pwmFreq;
	
	mc_motor_type motor_type;
	mc_sensor_mode sensor_mode;
	mc_control_mode control_mode;

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

}mcConfiguration_t;

//Defines

typedef struct {
	int16_t CorrADC_a;
	int16_t CorrADC_b;
	int16_t CorrADC_c;

	float	qKa;
	float	qKb; 
	
	int16_t	Offseta;
	int16_t	Offsetb;
	uint32_t curr0_sum;
	uint32_t curr1_sum;
	uint16_t curr_start_samples;
} tMeasCurrParm;

typedef struct {
	float	InputVoltage;
	float	MotorTemp;
	float	boardTemp;
} tMeasSensorValue;


typedef struct {
	//------------------------------------
	//for measure
	float AlphaMeas	;	
	float BetaMeas	;
	float ThetaMeas;

	//------------------------------------
	//for pll
	float costh;
	float sinth;
	
	float Kpll      	;
	float Ipll       	;

	float pll_PIout ;	
	float pll_Err0 	;	
	
	float SinCosTheta;
	float CosSinTheta;

	float Theta ; // 0 ~ 2PI (vector)
	float ThetaEst;
	float ThetaEst0	;
	float ThetaOffset;

	//------------------------------------
	//for 3phase hamony filter
	
	float Asin3th	;// = 0.0f;
	float Acos3th	;// = 0.0f;
	float Bsin3th	;//= 0.0f;
	float Bcos3th	;//= 0.0f;
	float ANF_PLLA	;//= 0.0f;
	float ANF_PLLB	;//= 0.0f;

	float pllA_cos3th;
	float pllA_sin3th;
	float pllB_sin3th;
	float pllB_cos3th;

	float pllA_cos3th_Integral;
	float pllA_sin3th_Integral;
	float pllB_sin3th_Integral;
	float pllB_cos3th_Integral;

	float cos3th;
	float sin3th;

	float Gamma; //= 1.0f;
	//------------------------------------
	//for EKF
	float xk1,xk2,xk3;
	float Pk11,Pk12,Pk13,
		  Pk21,Pk22,Pk23,
		  Pk31,Pk32,Pk33;

	float Q11,Q12,Q13,
	      Q21,Q22,Q23,
	      Q31,Q32,Q33;	  

	float R;
	float wb;

	float Tsc; 
	float y1,y2;
	float xp1,xp2,xp3;

	float yp1,yp2;

	float F11,F12,F13,
	      F21,F22,F23,
	      F31,F32,F33;
	
	float H11,H12,H13,
	      H21,H22,H23;

	float Pp11,Pp12,Pp13,
	      Pp21,Pp22,Pp23,
	      Pp31,Pp32,Pp33;	

	float K11,K12,
	      K21,K22;

	//------------------------------------
	//common
	float angle; // 0 ~ 2PI
	float Futi;
	float Omega;  
	float rpm; 
} tSMC;

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

	bool 	openloop;
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

	uint16_t Ta;
	uint16_t Tb;
	uint16_t Tc;

    } tSVGenParm;

typedef struct
	{
    uint16_t OpenLoop:1;
    uint16_t RunMotor:1;
    uint16_t DcCalDone:1;
} __attribute__((packed)) tMcCtrlBits;


#ifdef __cplusplus
}
#endif

#endif /* MC_PWM_H_ */
