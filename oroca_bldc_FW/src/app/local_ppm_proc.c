/*
	Copyright 2012-2015 OROCA ESC Project 	www.oroca.org

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
 * app_ppm.c
 *
 *  Created on: 18 apr 2014
 *      Author: bakchajang
 */

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"

#include "servo_dec.h"
#include "mcpwm.h"
#include "timeout.h"
#include "utils.h"

#include <math.h>
#include <chthreads.h>
#include <Chvt.h>

#include "user_interface_app.h"
#include "local_ppm_proc.h"
#include "../core/uart3.h"

// Settings
#define MAX_CAN_AGE						0.1
#define MIN_PULSES_WITHOUT_POWER		50

// Threads
static THD_FUNCTION(ppm_thread, arg);
static THD_WORKING_AREA(ppm_thread_wa, 1024);
static thread_t*ppm_tp; 
virtual_timer_t vt; 

// Private functions
static void servodec_func(void);

// Private variables
static volatile bool is_running = false;
static volatile ppm_config config;
static volatile int pulses_without_power = 0;

// Private functions
static void update(void *p);

void app_ppm_configure(ppm_config *conf) {
	config = *conf;
	pulses_without_power = 0;

	if (is_running) {
		servodec_set_pulse_options(config.pulse_start, config.pulse_end, config.median_filter);
	}
}

void app_ppm_start(void) {
	chThdCreateStatic(ppm_thread_wa, sizeof(ppm_thread_wa), NORMALPRIO, ppm_thread, NULL);


	//Uart3_printf(&SD3, (uint8_t *)"app_ppm_start.....\r\n");  //170530  

	chSysLock();
	chVTSetI(&vt, MS2ST(1), update, NULL);
	chSysUnlock();
}

static void servodec_func(void) {
	chSysLockFromISR();
	timeout_reset();

	chEvtSignalI(ppm_tp, (eventmask_t) 1);
	chSysUnlockFromISR();
}


static void update(void *p) {
	chSysLockFromISR();
	chVTSetI(&vt, MS2ST(2), update, p);

	chEvtSignalI(ppm_tp, (eventmask_t) 1);
	chSysUnlockFromISR();
}

static THD_FUNCTION(ppm_thread, arg)
{
	(void)arg;

	chRegSetThreadName("APP_PPM");
	ppm_tp = chThdGetSelfX();

	//servodec_set_pulse_options(config.pulse_start, config.pulse_end, config.median_filter);
	servodec_set_pulse_options(1.0f, 2.0f, false);
	servodec_init(servodec_func);
	is_running = true;

	for(;;) {

		chEvtWaitAny((eventmask_t) 1);

		float servo_val = servodec_get_servo(0);

		//--------------------------------------------------------------------------------
		//test code
		//Uart3_printf(&SD3, (uint8_t *)"servo : %f\r\n",(float)servo_val);    //170530
		CtrlParm.qVelRef=servo_val/100.0f;
		//--------------------------------------------------------------------------------

		ui_events |= EVT_PPM;
	}

}
