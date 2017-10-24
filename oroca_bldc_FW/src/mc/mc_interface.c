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

#include "mc_interface.h"
#include "mcpwm.h"
#include "ledpwm.h"
#include "stm32f4xx_conf.h"
#include "hw.h"
#include "utils.h"
#include "ch.h"
#include "hal.h"
//#include "commands.h"
#include "encoder.h"
#include <math.h>

// Global variables
volatile int ADC_curr_norm_value[3];

// Private variables
static volatile mc_configuration m_conf;
static mc_fault_code m_fault_now;
static int m_ignore_iterations;
static volatile unsigned int m_cycles_running;
static volatile bool m_lock_enabled;
static volatile bool m_lock_override_once;
static volatile float m_motor_current_sum;
static volatile float m_input_current_sum;
static volatile float m_motor_current_iterations;
static volatile float m_input_current_iterations;
static volatile float m_amp_seconds;
static volatile float m_amp_seconds_charged;
static volatile float m_watt_seconds;
static volatile float m_watt_seconds_charged;
static volatile float m_position_set;

// Sampling variables
#define ADC_SAMPLE_MAX_LEN		2000
static volatile int16_t m_curr0_samples[ADC_SAMPLE_MAX_LEN];
static volatile int16_t m_curr1_samples[ADC_SAMPLE_MAX_LEN];
static volatile int16_t m_ph1_samples[ADC_SAMPLE_MAX_LEN];
static volatile int16_t m_ph2_samples[ADC_SAMPLE_MAX_LEN];
static volatile int16_t m_ph3_samples[ADC_SAMPLE_MAX_LEN];
static volatile int16_t m_vzero_samples[ADC_SAMPLE_MAX_LEN];
static volatile uint8_t m_status_samples[ADC_SAMPLE_MAX_LEN];
static volatile int16_t m_curr_fir_samples[ADC_SAMPLE_MAX_LEN];
static volatile int16_t m_f_sw_samples[ADC_SAMPLE_MAX_LEN];
static volatile int m_sample_len;
static volatile int m_sample_int;
static volatile int m_sample_ready;
static volatile int m_sample_now;
static volatile int m_sample_at_start;
static volatile int m_start_comm;
static volatile float m_last_adc_duration_sample;

// Private functions
static void update_override_limits(volatile mc_configuration *conf);

// Function pointers
static void(*pwn_done_func)(void) = 0;

// Threads
static THD_WORKING_AREA(timer_thread_wa, 1024);
static THD_FUNCTION(timer_thread, arg);
static THD_WORKING_AREA(sample_send_thread_wa, 1024);
static THD_FUNCTION(sample_send_thread, arg);
static thread_t *sample_send_tp;

void mc_interface_init(mc_configuration *configuration) {
	m_conf = *configuration;
	m_fault_now = FAULT_CODE_NONE;
	m_ignore_iterations = 0;
	m_cycles_running = 0;
	m_lock_enabled = false;
	m_lock_override_once = false;
	m_motor_current_sum = 0.0;
	m_input_current_sum = 0.0;
	m_motor_current_iterations = 0.0;
	m_input_current_iterations = 0.0;
	m_amp_seconds = 0.0;
	m_amp_seconds_charged = 0.0;
	m_watt_seconds = 0.0;
	m_watt_seconds_charged = 0.0;
	m_position_set = 0.0;
	m_last_adc_duration_sample = 0.0;

	m_sample_len = 1000;
	m_sample_int = 1;
	m_sample_ready = 1;
	m_sample_now = 0;
	m_sample_at_start = 0;
	m_start_comm = 0;

	// Start threads
	chThdCreateStatic(timer_thread_wa, sizeof(timer_thread_wa), NORMALPRIO, timer_thread, NULL);
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

	// Initialize selected implementation
	switch (m_conf.motor_type) {
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
}

const volatile mc_configuration* mc_interface_get_configuration(void) {
	return &m_conf;
}

void mc_interface_set_configuration(mc_configuration *configuration) {
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
		mcpwm_set_configuration(&m_conf);
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

/**
 * Allow just one motor control command in the locked state.
 */
void mc_interface_lock_override_once(void) {
	m_lock_override_once = true;
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
static void update_override_limits(volatile mc_configuration *conf) {
	const float temp = NTC_TEMP(ADC_IND_TEMP_MOS2);
	const float v_in = GET_INPUT_VOLTAGE();

	// Temperature
/*	if (temp < conf->l_temp_fet_start) {
		conf->lo_current_min = conf->l_current_min;
		conf->lo_current_max = conf->l_current_max;
	} else if (temp > conf->l_temp_fet_end) {
		conf->lo_current_min = 0.0;
		conf->lo_current_max = 0.0;
		mc_interface_fault_stop(FAULT_CODE_OVER_TEMP_FET);
	} else {
		float maxc = fabsf(conf->l_current_max);
		if (fabsf(conf->l_current_min) > maxc) {
			maxc = fabsf(conf->l_current_min);
		}

		maxc = utils_map(temp, conf->l_temp_fet_start, conf->l_temp_fet_end, maxc, 0.0);

		if (fabsf(conf->l_current_max) > maxc) {
			conf->lo_current_max = SIGN(conf->l_current_max) * maxc;
		}

		if (fabsf(conf->l_current_min) > maxc) {
			conf->lo_current_min = SIGN(conf->l_current_min) * maxc;
		}
	}

	// Battery cutoff
	if (v_in > conf->l_battery_cut_start) {
		conf->lo_in_current_max = conf->l_in_current_max;
	} else if (v_in < conf->l_battery_cut_end) {
		conf->lo_in_current_max = 0.0;
	} else {
		conf->lo_in_current_max = utils_map(v_in, conf->l_battery_cut_start,
				conf->l_battery_cut_end, conf->l_in_current_max, 0.0);
	}

	conf->lo_in_current_min = conf->l_in_current_min;*/
}

static THD_FUNCTION(timer_thread, arg) {
	(void)arg;

	chRegSetThreadName("mcif timer");

	for(;;) {

		update_override_limits(&m_conf);

		chThdSleepMilliseconds(1);
	}
}

static THD_FUNCTION(sample_send_thread, arg) {
	(void)arg;

	chRegSetThreadName("SampleSender");

	sample_send_tp = chThdGetSelfX();

	for(;;) {
		chEvtWaitAny((eventmask_t) 1);

		for (int i = 0;i < m_sample_len;i++) {
			uint8_t buffer[20];
			int index = 0;

			buffer[index++] = m_curr0_samples[i] >> 8;
			buffer[index++] = m_curr0_samples[i];
			buffer[index++] = m_curr1_samples[i] >> 8;
			buffer[index++] = m_curr1_samples[i];
			buffer[index++] = m_ph1_samples[i] >> 8;
			buffer[index++] = m_ph1_samples[i];
			buffer[index++] = m_ph2_samples[i] >> 8;
			buffer[index++] = m_ph2_samples[i];
			buffer[index++] = m_ph3_samples[i] >> 8;
			buffer[index++] = m_ph3_samples[i];
			buffer[index++] = m_vzero_samples[i] >> 8;
			buffer[index++] = m_vzero_samples[i];
			buffer[index++] = m_status_samples[i];
			buffer[index++] = m_curr_fir_samples[i] >> 8;
			buffer[index++] = m_curr_fir_samples[i];
			buffer[index++] = m_f_sw_samples[i] >> 8;
			buffer[index++] = m_f_sw_samples[i];

			//commands_send_samples(buffer, index);
		}
	}
}