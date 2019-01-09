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
 * app.h
 *
 *  Created on: 18 apr 2014
 *      Author: benjamin
 */

#ifndef APP_H_
#define APP_H_

//#include "conf_general.h"
#include <stdint.h>
#include <stdbool.h>

#include "rf.h"

// Applications to use
typedef enum {
	APP_NONE = 0,
	APP_PPM,
	APP_ADC,
	APP_UART,
	APP_PPM_UART,
	APP_ADC_UART,
	APP_NUNCHUK,
	APP_NRF,
	APP_CUSTOM
} app_use;

// PPM control types
typedef enum {
	PPM_CTRL_TYPE_NONE = 0,
	PPM_CTRL_TYPE_CURRENT,
	PPM_CTRL_TYPE_CURRENT_NOREV,
	PPM_CTRL_TYPE_CURRENT_NOREV_BRAKE,
	PPM_CTRL_TYPE_DUTY,
	PPM_CTRL_TYPE_DUTY_NOREV,
	PPM_CTRL_TYPE_PID,
	PPM_CTRL_TYPE_PID_NOREV
} ppm_control_type;

typedef struct {
	ppm_control_type ctrl_type;
	float pid_max_erpm;
	float hyst;
	float pulse_start;
	float pulse_end;
	bool median_filter;
	bool safe_start;
	float rpm_lim_start;
	float rpm_lim_end;
	bool multi_esc;
	bool tc;
	float tc_max_diff;
} ppm_config;

// ADC control types
typedef enum {
	ADC_CTRL_TYPE_NONE = 0,
	ADC_CTRL_TYPE_CURRENT,
	ADC_CTRL_TYPE_CURRENT_REV_CENTER,
	ADC_CTRL_TYPE_CURRENT_REV_BUTTON,
	ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER,
	ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_BUTTON,
	ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC,
	ADC_CTRL_TYPE_DUTY,
	ADC_CTRL_TYPE_DUTY_REV_CENTER,
	ADC_CTRL_TYPE_DUTY_REV_BUTTON
} adc_control_type;

typedef struct {
	adc_control_type ctrl_type;
	float hyst;
	float voltage_start;
	float voltage_end;
	bool use_filter;
	bool safe_start;
	bool cc_button_inverted;
	bool rev_button_inverted;
	bool voltage_inverted;
	float rpm_lim_start;
	float rpm_lim_end;
	bool multi_esc;
	bool tc;
	float tc_max_diff;
	uint32_t update_rate_hz;
} adc_config;

// Nunchuk control types
typedef enum {
	CHUK_CTRL_TYPE_NONE = 0,
	CHUK_CTRL_TYPE_CURRENT,
	CHUK_CTRL_TYPE_CURRENT_NOREV
} chuk_control_type;

typedef struct {
	chuk_control_type ctrl_type;
	float hyst;
	float rpm_lim_start;
	float rpm_lim_end;
	float ramp_time_pos;
	float ramp_time_neg;
	float stick_erpm_per_s_in_cc;
	bool multi_esc;
	bool tc;
	float tc_max_diff;
} chuk_config;



typedef struct {
	// Settings
	uint8_t controller_id;
	uint32_t timeout_msec;
	float timeout_brake_current;
	bool send_can_status;
	uint32_t send_can_status_rate_hz;

	// Application to use
	app_use app_to_use;

	// PPM application settings
	ppm_config app_ppm_conf;

	// ADC application settings
	adc_config app_adc_conf;

	// UART application settings
	uint32_t app_uart_baudrate;

	// Nunchuk application settings
	chuk_config app_chuk_conf;

	// NRF application settings
	nrf_config app_nrf_conf;
} app_configuration;

typedef struct {
	int js_x;
	int js_y;
	int acc_x;
	int acc_y;
	int acc_z;
	bool bt_c;
	bool bt_z;
} chuck_data;

// Functions
void app_init(app_configuration *conf);
const app_configuration* app_get_configuration(void);
void app_set_configuration(app_configuration *conf);

// Standard apps
void app_ppm_start(void);
void app_ppm_configure(ppm_config *conf);
void app_adc_start(bool use_rx_tx);
void app_adc_configure(adc_config *conf);
float app_adc_get_decoded_level(void);
float app_adc_get_voltage(void);
float app_adc_get_decoded_level2(void);
float app_adc_get_voltage2(void);
void app_uartcomm_start(void);
void app_uartcomm_configure(uint32_t baudrate);
void app_nunchuk_start(void);
void app_nunchuk_configure(chuk_config *conf);
float app_nunchuk_get_decoded_chuk(void);
void app_nunchuk_update_output(chuck_data *data);

// Custom apps
void app_sten_init(void);

#endif /* APP_H_ */
