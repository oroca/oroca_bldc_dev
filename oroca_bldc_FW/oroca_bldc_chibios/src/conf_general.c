/*
 * conf_general.c
 *
 *  Created on: 14 sep 2014
 *      Author: benjamin
 */


#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"

#include "eeprom.h"
#include "mc_typedef.h"
#include "mc_interface.h"
#include "hw.h"
#include "utils.h"
#include "timeout.h"

#include <string.h>
#include <math.h>

#include "conf_general.h"

//#include "comm_usb.h"
#include "comm_usb_serial.h"//for debug

#include "mavlink_proc.h"



// User defined default motor configuration file
#ifdef MCCONF_DEFAULT_USER
#include MCCONF_DEFAULT_USER
#endif

// User defined default app configuration file
#ifdef APPCONF_DEFAULT_USER
#include APPCONF_DEFAULT_USER
#endif

// Default configuration parameters that can be overridden
//#include "mcconf_default.h"
//#include "appconf_default.h"

// EEPROM settings
#define EEPROM_BASE_APPCONF		2000

// Global variables
extern uint16_t VirtAddVarTab[NB_OF_VAR];



void conf_general_init(void) {
	// First, make sure that all relevant virtual addresses are assigned for page swapping.
	memset(VirtAddVarTab, 0, sizeof(VirtAddVarTab));

	int ind = 0;

	for (unsigned int i = 0;i < (sizeof(app_configuration) / 2);i++) 
	{
		VirtAddVarTab[ind++] = EEPROM_BASE_APPCONF + i;
	}

	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
	EE_Init();
}

/**
 * Load the compiled default app_configuration.
 *
 * @param conf
 * A pointer to store the default configuration to.
 */
void conf_general_get_default_app_configuration(app_configuration *conf) {
	memset(conf, 0, sizeof(app_configuration));
/*	conf->controller_id = APPCONF_CONTROLLER_ID;
	conf->timeout_msec = APPCONF_TIMEOUT_MSEC;
	conf->timeout_brake_current = APPCONF_TIMEOUT_BRAKE_CURRENT;
	conf->send_can_status = APPCONF_SEND_CAN_STATUS;
	conf->send_can_status_rate_hz = APPCONF_SEND_CAN_STATUS_RATE_HZ;

	conf->app_to_use = APPCONF_APP_TO_USE;

	conf->app_ppm_conf.ctrl_type = APPCONF_PPM_CTRL_TYPE;
	conf->app_ppm_conf.pid_max_erpm = APPCONF_PPM_PID_MAX_ERPM;
	conf->app_ppm_conf.hyst = APPCONF_PPM_HYST;
	conf->app_ppm_conf.pulse_start = APPCONF_PPM_PULSE_START;
	conf->app_ppm_conf.pulse_end = APPCONF_PPM_PULSE_END;
	conf->app_ppm_conf.median_filter = APPCONF_PPM_MEDIAN_FILTER;
	conf->app_ppm_conf.safe_start = APPCONF_PPM_SAFE_START;
	conf->app_ppm_conf.rpm_lim_start = APPCONF_PPM_RPM_LIM_START;
	conf->app_ppm_conf.rpm_lim_end = APPCONF_PPM_RPM_LIM_END;
	conf->app_ppm_conf.multi_esc = APPCONF_PPM_MULTI_ESC;
	conf->app_ppm_conf.tc = APPCONF_PPM_TC;
	conf->app_ppm_conf.tc_max_diff = APPCONF_PPM_TC_MAX_DIFF;

	conf->app_adc_conf.ctrl_type = APPCONF_ADC_CTRL_TYPE;
	conf->app_adc_conf.hyst = APPCONF_ADC_HYST;
	conf->app_adc_conf.voltage_start = APPCONF_ADC_VOLTAGE_START;
	conf->app_adc_conf.voltage_end = APPCONF_ADC_VOLTAGE_END;
	conf->app_adc_conf.use_filter = APPCONF_ADC_USE_FILTER;
	conf->app_adc_conf.safe_start = APPCONF_ADC_SAFE_START;
	conf->app_adc_conf.cc_button_inverted = APPCONF_ADC_CC_BUTTON_INVERTED;
	conf->app_adc_conf.rev_button_inverted = APPCONF_ADC_REV_BUTTON_INVERTED;
	conf->app_adc_conf.voltage_inverted = APPCONF_ADC_VOLTAGE_INVERTED;
	conf->app_adc_conf.rpm_lim_start = APPCONF_ADC_RPM_LIM_START;
	conf->app_adc_conf.rpm_lim_end = APPCONF_ADC_RPM_LIM_END;
	conf->app_adc_conf.multi_esc = APPCONF_ADC_MULTI_ESC;
	conf->app_adc_conf.tc = APPCONF_ADC_TC;
	conf->app_adc_conf.tc_max_diff = APPCONF_ADC_TC_MAX_DIFF;
	conf->app_adc_conf.update_rate_hz = APPCONF_ADC_UPDATE_RATE_HZ;

	conf->app_uart_baudrate = APPCONF_UART_BAUDRATE;

	conf->app_chuk_conf.ctrl_type = APPCONF_CHUK_CTRL_TYPE;
	conf->app_chuk_conf.hyst = APPCONF_CHUK_HYST;
	conf->app_chuk_conf.rpm_lim_start = APPCONF_CHUK_RPM_LIM_START;
	conf->app_chuk_conf.rpm_lim_end = APPCONF_CHUK_RPM_LIM_END;
	conf->app_chuk_conf.ramp_time_pos = APPCONF_CHUK_RAMP_TIME_POS;
	conf->app_chuk_conf.ramp_time_neg = APPCONF_CHUK_RAMP_TIME_NEG;
	conf->app_chuk_conf.stick_erpm_per_s_in_cc = APPCONF_STICK_ERPM_PER_S_IN_CC;
	conf->app_chuk_conf.multi_esc = APPCONF_CHUK_MULTI_ESC;
	conf->app_chuk_conf.tc = APPCONF_CHUK_TC;
	conf->app_chuk_conf.tc_max_diff = APPCONF_CHUK_TC_MAX_DIFF;

	conf->app_nrf_conf.speed = APPCONF_NRF_SPEED;
	conf->app_nrf_conf.power = APPCONF_NRF_POWER;
	conf->app_nrf_conf.crc_type = APPCONF_NRF_CRC;
	conf->app_nrf_conf.retry_delay = APPCONF_NRF_RETR_DELAY;
	conf->app_nrf_conf.retries = APPCONF_NRF_RETRIES;
	conf->app_nrf_conf.channel = APPCONF_NRF_CHANNEL;
	conf->app_nrf_conf.address[0] = APPCONF_NRF_ADDR_B0;
	conf->app_nrf_conf.address[1] = APPCONF_NRF_ADDR_B1;
	conf->app_nrf_conf.address[2] = APPCONF_NRF_ADDR_B2;
	conf->app_nrf_conf.send_crc_ack = APPCONF_NRF_SEND_CRC_ACK;*/
}



/**
 * Read app_configuration from EEPROM. If this fails, default values will be used.
 *
 * @param conf
 * A pointer to a app_configuration struct to write the read configuration to.
 */
void conf_general_read_app_configuration(app_configuration *conf) {
	bool is_ok = true;
	uint8_t *conf_addr = (uint8_t*)conf;
	uint16_t var;

	for (unsigned int i = 0;i < (sizeof(app_configuration) / 2);i++) {
		if (EE_ReadVariable(EEPROM_BASE_APPCONF + i, &var) == 0) {
			conf_addr[2 * i] = (var >> 8) & 0xFF;
			conf_addr[2 * i + 1] = var & 0xFF;
		} else {
			is_ok = false;
			break;
		}
	}

	// Set the default configuration
	if (!is_ok) {
		conf_general_get_default_app_configuration(conf);
	}
}

/**
 * Write app_configuration to EEPROM.
 *
 * @param conf
 * A pointer to the configuration that should be stored.
 */
bool conf_general_store_app_configuration(app_configuration *conf) {
	mc_interface_unlock();
	//mc_interface_release_motor();

	utils_sys_lock_cnt();
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, DISABLE);

	bool is_ok = true;
	uint8_t *conf_addr = (uint8_t*)conf;
	uint16_t var;

	FLASH_ClearFlag(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
			FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	for (unsigned int i = 0;i < (sizeof(app_configuration) / 2);i++) {
		var = (conf_addr[2 * i] << 8) & 0xFF00;
		var |= conf_addr[2 * i + 1] & 0xFF;

		if (EE_WriteVariable(EEPROM_BASE_APPCONF + i, var) != FLASH_COMPLETE) {
			is_ok = false;
			break;
		}
	}

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);
	utils_sys_unlock_cnt();

	return is_ok;
}

