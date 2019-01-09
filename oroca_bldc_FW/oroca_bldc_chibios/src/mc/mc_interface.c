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
 * mc_interface.c
 *
 *  Created on: 10 okt 2015
 *      Author: benjamin
 */
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"

#include <math.h>

#include "hw.h"
#include "mc_define.h"
#include "mc_typedef.h"

#include "mc_control.h"
#include "mc_sensor.h"
#include "mc_pwm.h"
#include "mc_encoder.h"

#include "eeprom.h"
#include "ledpwm.h"
#include "utils.h"

#include "comm_usb_serial.h"

#include "conf_general.h"
#include "mc_interface.h"

#include "mavlink_proc.h"

#include "usart1_print.h"


// Global variables


// Private variables
static volatile mcConfiguration_t m_conf;
static mc_fault_code m_fault_now;
static int m_ignore_iterations;
static volatile bool m_lock_enabled;


// Private functions
static void update_override_limits(volatile mcConfiguration_t *conf);

// Function pointers
static void(*pwn_done_func)(void) = 0;

// Threads
static THD_WORKING_AREA(sample_send_thread_wa, 1024);
static THD_FUNCTION(sample_send_thread, arg);
static thread_t *sample_send_tp;


// User defined default motor configuration file
#ifdef MCCONF_DEFAULT_USER
#include MCCONF_DEFAULT_USER
#endif

// Default configuration parameters that can be overridden
//#include "mcconf_default.h"
//#include "appconf_default.h"

// EEPROM settings
#define EEPROM_BASE_MCCONF		1000

// Global variables
extern uint16_t VirtAddVarTab[NB_OF_VAR];

// Private variables
mcConfiguration_t mcconf, mcconf_old;



void mcconf_general_init(void) {
	// First, make sure that all relevant virtual addresses are assigned for page swapping.
	memset(VirtAddVarTab, 0, sizeof(VirtAddVarTab));

	int ind = 0;
	for (unsigned int i = 0;i < (sizeof(app_configuration) / 2);i++)
	{
		VirtAddVarTab[ind++] = EEPROM_BASE_MCCONF + i;
	}

	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
	EE_Init();
}


/**
 * Load the compiled default mcConfiguration_t.
 *
 * @param conf
 * A pointer to store the default configuration to.
 */
void mcconf_general_get_default_mc_configuration(mcConfiguration_t *conf) {
	memset(conf, 0, sizeof(mcConfiguration_t));
	conf->pwm_mode = MCCONF_PWM_MODE;
	conf->comm_mode = MCCONF_COMM_MODE;
	conf->motor_type = MCCONF_DEFAULT_MOTOR_TYPE;
	conf->sensor_mode = MCCONF_SENSOR_MODE;

	conf->l_current_max = MCCONF_L_CURRENT_MAX;
	conf->l_current_min = MCCONF_L_CURRENT_MIN;
	conf->l_in_current_max = MCCONF_L_IN_CURRENT_MAX;
	conf->l_in_current_min = MCCONF_L_IN_CURRENT_MIN;
	conf->l_abs_current_max = MCCONF_L_MAX_ABS_CURRENT;
	conf->l_min_erpm = MCCONF_L_RPM_MIN;
	conf->l_max_erpm = MCCONF_L_RPM_MAX;
	conf->l_max_erpm_fbrake = MCCONF_L_CURR_MAX_RPM_FBRAKE;
	conf->l_max_erpm_fbrake_cc = MCCONF_L_CURR_MAX_RPM_FBRAKE_CC;
	conf->l_min_vin = MCCONF_L_MIN_VOLTAGE;
	conf->l_max_vin = MCCONF_L_MAX_VOLTAGE;
	conf->l_battery_cut_start = MCCONF_L_BATTERY_CUT_START;
	conf->l_battery_cut_end = MCCONF_L_BATTERY_CUT_END;
	conf->l_slow_abs_current = MCCONF_L_SLOW_ABS_OVERCURRENT;
	conf->l_temp_fet_start = MCCONF_L_LIM_TEMP_FET_START;
	conf->l_temp_fet_end = MCCONF_L_LIM_TEMP_FET_END;
	conf->l_temp_motor_start = MCCONF_L_LIM_TEMP_MOTOR_START;
	conf->l_temp_motor_end = MCCONF_L_LIM_TEMP_MOTOR_END;
	conf->l_min_duty = MCCONF_L_MIN_DUTY;
	conf->l_max_duty = MCCONF_L_MAX_DUTY;

	conf->lo_current_max = conf->l_current_max;
	conf->lo_current_min = conf->l_current_min;
	conf->lo_in_current_max = conf->l_in_current_max;
	conf->lo_in_current_min = conf->l_in_current_min;
	/*

	conf->sl_min_erpm = MCCONF_SL_MIN_RPM;
	conf->sl_max_fullbreak_current_dir_change = MCCONF_SL_MAX_FB_CURR_DIR_CHANGE;
	conf->sl_min_erpm_cycle_int_limit = MCCONF_SL_MIN_ERPM_CYCLE_INT_LIMIT;
	conf->sl_cycle_int_limit = MCCONF_SL_CYCLE_INT_LIMIT;
	conf->sl_phase_advance_at_br = MCCONF_SL_PHASE_ADVANCE_AT_BR;
	conf->sl_cycle_int_rpm_br = MCCONF_SL_CYCLE_INT_BR;
	conf->sl_bemf_coupling_k = MCCONF_SL_BEMF_COUPLING_K;

	conf->hall_table[0] = MCCONF_HALL_TAB_0;
	conf->hall_table[1] = MCCONF_HALL_TAB_1;
	conf->hall_table[2] = MCCONF_HALL_TAB_2;
	conf->hall_table[3] = MCCONF_HALL_TAB_3;
	conf->hall_table[4] = MCCONF_HALL_TAB_4;
	conf->hall_table[5] = MCCONF_HALL_TAB_5;
	conf->hall_table[6] = MCCONF_HALL_TAB_6;
	conf->hall_table[7] = MCCONF_HALL_TAB_7;
	conf->hall_sl_erpm = MCCONF_HALL_ERPM;

	conf->foc_current_kp = MCCONF_FOC_CURRENT_KP;
	conf->foc_current_ki = MCCONF_FOC_CURRENT_KI;
	conf->foc_f_sw = MCCONF_FOC_F_SW;
	conf->foc_dt_us = MCCONF_FOC_DT_US;
	conf->foc_encoder_inverted = MCCONF_FOC_ENCODER_INVERTED;
	conf->foc_encoder_offset = MCCONF_FOC_ENCODER_OFFSET;
	conf->foc_encoder_ratio = MCCONF_FOC_ENCODER_RATIO;
	conf->foc_sensor_mode = MCCONF_FOC_SENSOR_MODE;
	conf->foc_pll_kp = MCCONF_FOC_PLL_KP;
	conf->foc_pll_ki = MCCONF_FOC_PLL_KI;
	conf->foc_motor_l = MCCONF_FOC_MOTOR_L;
	conf->foc_motor_r = MCCONF_FOC_MOTOR_R;
	conf->foc_motor_flux_linkage = MCCONF_FOC_MOTOR_FLUX_LINKAGE;
	conf->foc_observer_gain = MCCONF_FOC_OBSERVER_GAIN;
	conf->foc_duty_dowmramp_kp = MCCONF_FOC_DUTY_DOWNRAMP_KP;
	conf->foc_duty_dowmramp_ki = MCCONF_FOC_DUTY_DOWNRAMP_KI;
	conf->foc_openloop_rpm = MCCONF_FOC_OPENLOOP_RPM;
	conf->foc_sl_openloop_hyst = MCCONF_FOC_SL_OPENLOOP_HYST;
	conf->foc_sl_openloop_time = MCCONF_FOC_SL_OPENLOOP_TIME;
	conf->foc_sl_d_current_duty = MCCONF_FOC_SL_D_CURRENT_DUTY;
	conf->foc_sl_d_current_factor = MCCONF_FOC_SL_D_CURRENT_FACTOR;
	conf->foc_hall_table[0] = MCCONF_FOC_HALL_TAB_0;
	conf->foc_hall_table[1] = MCCONF_FOC_HALL_TAB_1;
	conf->foc_hall_table[2] = MCCONF_FOC_HALL_TAB_2;
	conf->foc_hall_table[3] = MCCONF_FOC_HALL_TAB_3;
	conf->foc_hall_table[4] = MCCONF_FOC_HALL_TAB_4;
	conf->foc_hall_table[5] = MCCONF_FOC_HALL_TAB_5;
	conf->foc_hall_table[6] = MCCONF_FOC_HALL_TAB_6;
	conf->foc_hall_table[7] = MCCONF_FOC_HALL_TAB_7;
	conf->foc_sl_erpm = MCCONF_FOC_SL_ERPM;

	conf->s_pid_kp = MCCONF_S_PID_KP;
	conf->s_pid_ki = MCCONF_S_PID_KI;
	conf->s_pid_kd = MCCONF_S_PID_KD;
	conf->s_pid_min_erpm = MCCONF_S_PID_MIN_RPM;

	conf->p_pid_kp = MCCONF_P_PID_KP;
	conf->p_pid_ki = MCCONF_P_PID_KI;
	conf->p_pid_kd = MCCONF_P_PID_KD;
	conf->p_pid_ang_div = MCCONF_P_PID_ANG_DIV;

	conf->cc_startup_boost_duty = MCCONF_CC_STARTUP_BOOST_DUTY;
	conf->cc_min_current = MCCONF_CC_MIN_CURRENT;
	conf->cc_gain = MCCONF_CC_GAIN;
	conf->cc_ramp_step_max = MCCONF_CC_RAMP_STEP;

	conf->m_fault_stop_time_ms = MCCONF_M_FAULT_STOP_TIME;
	conf->m_duty_ramp_step = MCCONF_M_RAMP_STEP;
	conf->m_duty_ramp_step_rpm_lim = MCCONF_M_RAMP_STEP_RPM_LIM;
	conf->m_current_backoff_gain = MCCONF_M_CURRENT_BACKOFF_GAIN;
	conf->m_encoder_counts = MCCONF_M_ENCODER_COUNTS;*/
	conf->m_sensor_port_mode = MCCONF_M_SENSOR_PORT_MODE;
}



/**
 * Read mcConfiguration_t from EEPROM. If this fails, default values will be used.
 *
 * @param conf
 * A pointer to a mcConfiguration_t struct to write the read configuration to.
 */
void mcconf_general_read_mc_configuration(mcConfiguration_t *conf)
{
	bool is_ok = true;
	uint8_t *conf_addr = (uint8_t*)conf;
	uint16_t var;

	for (unsigned int i = 0;i < (sizeof(mcConfiguration_t) / 2);i++) 
	{
		if (EE_ReadVariable(EEPROM_BASE_MCCONF + i, &var) == 0) 
		{
			conf_addr[2 * i] = (var >> 8) & 0xFF;
			conf_addr[2 * i + 1] = var & 0xFF;
		}
		else
		{
			is_ok = false;
			break;
		}
	}

	//chvprintf(&SDU1, (uint8_t *)"to conf_general.c : conf_general_read_mc_configuration\r\n");

	//mavlink_dbgString(0,(uint8_t *)"to conf_general.c : conf_general_read_mc_configuration\r\n");



	if (!is_ok) 
	{
		//chvprintf(&SD1, (uint8_t *)"conf_ok\r\n");
		mcconf_general_get_default_mc_configuration(conf);
	}

	
}

/**
 * Write mcConfiguration_t to EEPROM.
 *
 * @param conf
 * A pointer to the configuration that should be stored.
 */
bool mcconf_general_store_mc_configuration(mcConfiguration_t *conf) 
{

	mavlink_dbgString(0,"conf_general_store_mc_configuration");


	mc_interface_unlock();
	//mc_interface_release_motor();

	utils_sys_lock_cnt();
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, DISABLE);

	bool is_ok = true;
	uint8_t *conf_addr = (uint8_t*)conf;
	uint16_t var;

	FLASH_ClearFlag(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |	FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	for (unsigned int i = 0;i < (sizeof(mcConfiguration_t) / 2);i++) 
	{
		var = (conf_addr[2 * i] << 8) & 0xFF00;
		var |= conf_addr[2 * i + 1] & 0xFF;

		if (EE_WriteVariable(EEPROM_BASE_MCCONF + i, var) != FLASH_COMPLETE) 
		{
			is_ok = false;
			break;
		}
	}

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);
	utils_sys_unlock_cnt();

	mavlink_dbgString(0,"finish");
	return is_ok;
}




void mc_interface_init(mcConfiguration_t *configuration) 
{
	//chvprintf(&SD1, (uint8_t *)"mc_interface_init\r\n");
	
	m_conf = *configuration;
	m_fault_now = FAULT_CODE_NONE;
	m_lock_enabled = false;

	m_conf.m_sensor_port_mode = SENSOR_PORT_MODE_AS5047_SPI;


	// Start threads
	chThdCreateStatic(sample_send_thread_wa, sizeof(sample_send_thread_wa), NORMALPRIO - 1, sample_send_thread, NULL);
	

	// Initialize encoder
#if !WS2811_ENABLE
	switch (m_conf.m_sensor_port_mode) {
		case SENSOR_PORT_MODE_ABI:
			encoder_init_abi(m_conf.m_encoder_counts);
			break;

		case SENSOR_PORT_MODE_AS5047_SPI:
			encoder_init_as5047p_spi();
			break;

		default:
			break;
	}
#endif

//	m_conf.motor_type = MOTOR_TYPE_BLDC;
	mcpwm_init(&m_conf);

	// Initialize selected implementation
/*	switch (m_conf.motor_type) {
		case MOTOR_TYPE_BLDC:
		case MOTOR_TYPE_DC:
			mcpwm_init(&m_conf);
			break;

		case MOTOR_TYPE_FOC:
			//mcpwm_foc_init(&m_conf);
			break;

		default:
			break;
	}
*/

}

const volatile mcConfiguration_t* mc_interface_get_configuration(void) {
	return &m_conf;
}

void mc_interface_set_configuration(mcConfiguration_t *configuration) {
#if !WS2811_ENABLE
	if (m_conf.m_sensor_port_mode != configuration->m_sensor_port_mode) {
		encoder_deinit();
		switch (configuration->m_sensor_port_mode) {
		case SENSOR_PORT_MODE_ABI:
			encoder_init_abi(configuration->m_encoder_counts);
			break;

		case SENSOR_PORT_MODE_AS5047_SPI:
			encoder_init_as5047p_spi();
			break;

		default:
			break;
		}
	}

	if (configuration->m_sensor_port_mode == SENSOR_PORT_MODE_ABI) {
		encoder_set_counts(configuration->m_encoder_counts);
	}
#endif

	if (m_conf.motor_type == MOTOR_TYPE_FOC	&& configuration->motor_type != MOTOR_TYPE_FOC)
	{
		//mcpwm_foc_deinit();
		m_conf = *configuration;
		mcpwm_init(&m_conf);
	}
	else if (m_conf.motor_type != MOTOR_TYPE_FOC && configuration->motor_type == MOTOR_TYPE_FOC)
	{
		mcpwm_deinit();
		m_conf = *configuration;
		//mcpwm_foc_init(&m_conf);
	}
	else
	{
		m_conf = *configuration;
	}

	update_override_limits(&m_conf);

	switch (m_conf.motor_type) {
	case MOTOR_TYPE_BLDC:
	case MOTOR_TYPE_DC:
//		mcpwm_set_configuration(&m_conf);
		break;

	case MOTOR_TYPE_FOC:
		//mcpwm_foc_set_configuration(&m_conf);
		break;

	default:
		break;
	}
}

bool mc_interface_dccal_done(void) {
	bool ret = false;
	switch (m_conf.motor_type) {
	case MOTOR_TYPE_BLDC:
	case MOTOR_TYPE_DC:
		//ret = mcpwm_is_dccal_done();
		break;

	case MOTOR_TYPE_FOC:
		//ret = mcpwm_foc_is_dccal_done();
		break;

	default:
		break;
	}

	return ret;
}

/**
 * Set a function that should be called after each PWM cycle.
 *
 * @param p_func
 * The function to be called. 0 will not call any function.
 */
void mc_interface_set_pwm_callback(void (*p_func)(void)) {
	pwn_done_func = p_func;
}

/**
 * Lock the control by disabling all control commands.
 */
void mc_interface_lock(void) {
	m_lock_enabled = true;
}

/**
 * Unlock all control commands.
 */
void mc_interface_unlock(void) {
	m_lock_enabled = false;
}



mc_fault_code mc_interface_get_fault(void) {
	return m_fault_now;
}




/**
 * Update the override limits for a configuration based on MOSFET temperature etc.
 *
 * @param conf
 * The configaration to update.
 */
static void update_override_limits(volatile mcConfiguration_t *conf)
{
	const float temp = NTC_TEMP(ADC_IND_TEMP_MOS2);
	const float v_in = GET_INPUT_VOLTAGE();

	// Temperature
	if (temp < conf->l_temp_fet_start) 
	{
		conf->lo_current_min = conf->l_current_min;
		conf->lo_current_max = conf->l_current_max;
	}
	else if (temp > conf->l_temp_fet_end) 
	{
		conf->lo_current_min = 0.0;
		conf->lo_current_max = 0.0;
		mc_interface_fault_stop(FAULT_CODE_OVER_TEMP_FET);
	}
	else
	{
		float maxc = fabsf(conf->l_current_max);
		if (fabsf(conf->l_current_min) > maxc)
		{
			maxc = fabsf(conf->l_current_min);
		}

		maxc = utils_map(temp, conf->l_temp_fet_start, conf->l_temp_fet_end, maxc, 0.0);

		if (fabsf(conf->l_current_max) > maxc)
		{
			conf->lo_current_max = SIGN(conf->l_current_max) * maxc;
		}

		if (fabsf(conf->l_current_min) > maxc) 
		{
			conf->lo_current_min = SIGN(conf->l_current_min) * maxc;
		}
	}

	// Battery cutoff
	if (v_in > conf->l_battery_cut_start)
	{
		conf->lo_in_current_max = conf->l_in_current_max;	
	} 
	else if (v_in < conf->l_battery_cut_end) 
	{
		conf->lo_in_current_max = 0.0;
	}
	else 
	{
		conf->lo_in_current_max = utils_map(v_in, conf->l_battery_cut_start,
		conf->l_battery_cut_end, conf->l_in_current_max, 0.0);
	}

	conf->lo_in_current_min = conf->l_in_current_min;
}


void mc_interface_fault_stop(mc_fault_code fault) 
{
/*	if (m_fault_now == fault) {
		return;
	}

	if (mc_interface_dccal_done() && m_fault_now == FAULT_CODE_NONE) 
	{
		// Sent to terminal fault logger so that all faults and their conditions
		// can be printed for debugging.
		utils_sys_lock_cnt();
		volatile int val_samp = TIM8->CCR1;
		volatile int current_samp = TIM1->CCR4;
		volatile int tim_top = TIM1->ARR;
		utils_sys_unlock_cnt();

		mcFaultData_t fdata;
		fdata.fault = fault;
		fdata.current = mc_interface_get_tot_current();
		fdata.current_filtered = mc_interface_get_tot_current_filtered();
		fdata.voltage = GET_INPUT_VOLTAGE();
		fdata.duty = mc_interface_get_duty_cycle_now();
		fdata.rpm = mc_interface_get_rpm();
		fdata.tacho = mc_interface_get_tachometer_value(false);
		fdata.tim_val_samp = val_samp;
		fdata.tim_current_samp = current_samp;
		fdata.tim_top = tim_top;
		fdata.comm_step = mcpwm_get_comm_step();
		fdata.temperature = NTC_TEMP(ADC_IND_TEMP_MOS);
#ifdef HW_HAS_DRV8301
		if (fault == FAULT_CODE_DRV) {
			fdata.drv8301_faults = drv8301_read_faults();
		}
#endif
		terminal_add_fault_data(&fdata);
	}


	switch (m_conf.motor_type) {
	case MOTOR_TYPE_BLDC:
	case MOTOR_TYPE_DC:
		mcpwm_stop_pwm();
		break;

	case MOTOR_TYPE_FOC:
		mcpwm_foc_stop_pwm();
		break;

	default:
		break;
	}

	m_fault_now = fault;
	*/
}


void mc_interface_set_velocity(uint16_t vel)
{
	CtrlParm.qVelRef = (float)vel/1000; 
}

float mc_interface_get_angle(void)
{
	return encoder_read_deg();  
}


static volatile mc_state state;

void mc_setConfiguration(mcConfiguration_t configuration)
{
	stop_pwm_hw();//stop_pwm_ll();

	utils_sys_lock_cnt();
	m_conf = configuration;
	//mcpwm_init_hall_table((int8_t*)conf->hall_table);
	//update_sensor_mode();
	utils_sys_unlock_cnt();
}

mcConfiguration_t mc_getConfiguration(void)
{
	stop_pwm_hw();//stop_pwm_ll();
	
 	return m_conf;
}

float mc_rpm_to_omega(uint16_t RPM)
{
	//return RPM * 2.0f * PI / 60.0f;
	return (float)RPM * 0.104719755f;
}

float mc_rpm_to_freq(uint16_t RPM)
{
	return (float)RPM / 60.0f;
}



static THD_FUNCTION(sample_send_thread, arg) {
	(void)arg;

	chRegSetThreadName("SampleSender");

	sample_send_tp = chThdGetSelfX();

	//chvprintf(&SD1, (uint8_t *)"to mc_interface -> SampleSender\r\n");


	for(;;) 
	{
		chEvtWaitAny((eventmask_t) 1);

	}
}
