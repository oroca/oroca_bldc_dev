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
 * timeout.c
 *
 *  Created on: 20 sep 2014
 *      Author: benjamin
 */

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"

#include "timeout.h"
#include "mcpwm.h"

// Private variables
static volatile systime_t timeout_msec;
static volatile systime_t last_update_time;
static volatile float timeout_brake_current;
static volatile bool has_timeout;

// Threads
static THD_WORKING_AREA(timeout_thread_wa, 512);
//static msg_t timeout_thread(void *arg);


void timeout_configure(systime_t timeout, float brake_current) {
	timeout_msec = timeout;
	timeout_brake_current = brake_current;
}

void timeout_reset(void) {
	last_update_time = chVTGetSystemTimeX();
}

bool timeout_has_timeout(void) {
	return has_timeout;
}

systime_t timeout_get_timeout_msec(void) {
	return timeout_msec;
}

static msg_t timeout_thread(void *arg) {
	(void)arg;

	chRegSetThreadName("Timeout");

	for(;;) {
#if 0
		if (timeout_msec != 0 && chTimeElapsedSince(last_update_time) > MS2ST(timeout_msec)) {
			mcpwm_set_brake_current(timeout_brake_current);
			has_timeout = true;
		} else {
			has_timeout = false;
		}
#endif
		chThdSleepMilliseconds(10);
	}

	return 0;
}

void timeout_init(void) {
	timeout_msec = 1000;
	last_update_time = 0;
	timeout_brake_current = 0.0;
	has_timeout = false;

	chThdCreateStatic(timeout_thread_wa, sizeof(timeout_thread_wa), NORMALPRIO, timeout_thread, NULL);
}

