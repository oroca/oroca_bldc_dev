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

// Default settings
#ifndef MCCONF_DEFAULT_MOTOR_TYPE
#define MCCONF_DEFAULT_MOTOR_TYPE		MOTOR_TYPE_BLDC
#endif
#ifndef MCCONF_PWM_MODE
#define MCCONF_PWM_MODE					PWM_MODE_SYNCHRONOUS // Default PWM mode
#endif
#ifndef MCCONF_SENSOR_MODE
#define MCCONF_SENSOR_MODE				SENSOR_MODE_SENSORLESS // Sensor mode
#endif
#ifndef MCCONF_COMM_MODE
#define MCCONF_COMM_MODE				COMM_MODE_INTEGRATE	// The commutation mode to use
#endif

// Limits
#ifndef MCCONF_L_CURRENT_MAX
#define MCCONF_L_CURRENT_MAX			60.0	// Current limit in Amperes (Upper)
#endif
#ifndef MCCONF_L_CURRENT_MIN
#define MCCONF_L_CURRENT_MIN			-60.0	// Current limit in Amperes (Lower)
#endif
#ifndef MCCONF_L_IN_CURRENT_MAX
#define MCCONF_L_IN_CURRENT_MAX			60.0	// Input current limit in Amperes (Upper)
#endif
#ifndef MCCONF_L_IN_CURRENT_MIN
#define MCCONF_L_IN_CURRENT_MIN			-40.0	// Input current limit in Amperes (Lower)
#endif
#ifndef MCCONF_L_MAX_ABS_CURRENT
#define MCCONF_L_MAX_ABS_CURRENT		130.0	// The maximum absolute current above which a fault is generated
#endif
#ifndef MCCONF_L_MIN_VOLTAGE
#define MCCONF_L_MIN_VOLTAGE			8.0		// Minimum input voltage
#endif
#ifndef MCCONF_L_MAX_VOLTAGE
#define MCCONF_L_MAX_VOLTAGE			57.0	// Maximum input voltage
#endif
#ifndef MCCONF_L_BATTERY_CUT_START
#define MCCONF_L_BATTERY_CUT_START		10.0	// Start limiting the positive current at this voltage
#endif
#ifndef MCCONF_L_BATTERY_CUT_END
#define MCCONF_L_BATTERY_CUT_END		8.0		// Limit the positive current completely at this voltage
#endif
#ifndef MCCONF_L_RPM_MAX
#define MCCONF_L_RPM_MAX				100000.0	// The motor speed limit (Upper)
#endif
#ifndef MCCONF_L_RPM_MIN
#define MCCONF_L_RPM_MIN				-100000.0	// The motor speed limit (Lower)
#endif
#ifndef MCCONF_L_RPM_START
#define MCCONF_L_RPM_START				0.8		// Fraction of full speed where RPM current limiting starts
#endif
#ifndef MCCONF_L_SLOW_ABS_OVERCURRENT
#define MCCONF_L_SLOW_ABS_OVERCURRENT	true	// Use the filtered (and hence slower) current for the overcurrent fault detection
#endif
#ifndef MCCONF_L_MIN_DUTY
#define MCCONF_L_MIN_DUTY				0.005	// Minimum duty cycle
#endif
#ifndef MCCONF_L_MAX_DUTY
#define MCCONF_L_MAX_DUTY				0.95	// Maximum duty cycle
#endif
#ifndef MCCONF_L_CURR_MAX_RPM_FBRAKE
#define MCCONF_L_CURR_MAX_RPM_FBRAKE	300		// Maximum electrical RPM to use full brake at
#endif
#ifndef MCCONF_L_CURR_MAX_RPM_FBRAKE_CC
#define MCCONF_L_CURR_MAX_RPM_FBRAKE_CC	1500	// Maximum electrical RPM to use full brake at with current control
#endif
#ifndef MCCONF_L_LIM_TEMP_FET_START
#define MCCONF_L_LIM_TEMP_FET_START		80.0	// MOSFET temperature where current limiting should begin
#endif
#ifndef MCCONF_L_LIM_TEMP_FET_END
#define MCCONF_L_LIM_TEMP_FET_END		100.0	// MOSFET temperature where everything should be shut off
#endif
#ifndef MCCONF_L_LIM_TEMP_MOTOR_START
#define MCCONF_L_LIM_TEMP_MOTOR_START	80.0	// MOTOR temperature where current limiting should begin
#endif
#ifndef MCCONF_L_LIM_TEMP_MOTOR_END
#define MCCONF_L_LIM_TEMP_MOTOR_END		100.0	// MOTOR temperature where everything should be shut off
#endif
#ifndef MCCONF_L_WATT_MAX
#define MCCONF_L_WATT_MAX				15000.0	// Maximum wattage output
#endif
#ifndef MCCONF_L_WATT_MIN
#define MCCONF_L_WATT_MIN				-15000.0	// Minimum wattage output (braking)
#endif

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


// Logged fault data
typedef struct {
	mc_fault_code fault;
	float current;
	float current_filtered;
	float voltage;
	float duty;
	float rpm;
	int tacho;
	int cycles_running;
	int tim_val_samp;
	int tim_current_samp;
	int tim_top;
	int comm_step;
	float temperature;
	int drv8301_faults;
} fault_data;


typedef struct {
	float vdd;
	float rshunt;
	uint16_t pwmFreq;
	
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



// Functions
void mc_interface_init(mc_configuration *configuration);
const volatile mc_configuration* mc_interface_get_configuration(void);
void mc_interface_set_configuration(mc_configuration *configuration);
void mc_interface_set_pwm_callback(void (*p_func)(void));
void mc_interface_lock(void);
void mc_interface_unlock(void);
void mc_interface_lock_override_once(void);

void mc_interface_fault_stop(mc_fault_code fault);



#endif /* MC_INTERFACE_H_ */
