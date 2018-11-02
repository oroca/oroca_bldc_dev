/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef _MC_ENCODER_H_
#define _MC_ENCODER_H_

//#include "conf_general.h"
#include <stdint.h>
#include <stdbool.h>

extern tSMC smc1;
//extern float last_enc_angle;

typedef enum {
	ENCODER_MODE_NONE = 0,
	ENCODER_MODE_ABI,
	ENCODER_MODE_AS5047P_SPI,
	ENCODER_MODE_AHALL,
	ENCODER_MODE_PWM
} encoder_mode;

extern encoder_mode EncMode;


// Functions
void encoder_deinit(void);
void encoder_init_abi(uint32_t counts);
void encoder_init_as5047p_spi(void);
bool encoder_is_configured(void);
float encoder_read_deg(void);
void encoder_reset(void);
void encoder_tim_isr(void);
void encoder_set_counts(uint32_t counts);
bool encoder_index_found(void);

void encoder_3HarmonicFilter(tSMC *s);
void encoder_PLLThetaEstimation(tSMC *s);


#endif /* ENCODER_H_ */
