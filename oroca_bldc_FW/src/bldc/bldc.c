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
	OROCA BLDC PROJECT.
*/
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"

#include "hw.h"
#include "bldc.h"
#include "Mcpwm.h"
#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "../core/uart3.h"



/*
 * Timers used:
 * TIM7: servo
 * TIM1: mcpwm
 * TIM2: mcpwm
 * TIM12: mcpwm
 * TIM8: mcpwm
 * TIM3: servo_dec/Encoder (HW_R2)
 * TIM4: WS2811/WS2812 LEDs/Encoder (other HW)
 *
 * DMA/stream	Device		Function
 * 1, 2			I2C1		Nunchuk, temp on rev 4.5
 * 1, 7			I2C1		Nunchuk, temp on rev 4.5
 * 1, 1			UART3		HW_R2
 * 1, 3			UART3		HW_R2
 * 2, 2			UART6		Other HW
 * 2, 7			UART6		Other HW
 * 2, 4			ADC			mcpwm
 * 1, 0			TIM4		WS2811/WS2812 LEDs CH1 (Ch 1)
 * 1, 3			TIM4		WS2811/WS2812 LEDs CH2 (Ch 2)
 *
 */

static THD_WORKING_AREA(periodic_thread_wa, 128);
static THD_FUNCTION(periodic_thread, arg) 
{
	(void)arg;

	chRegSetThreadName("BLDC periodic");

	//Uart3_printf(&SD3, (uint8_t *)"periodic_thread\r\n");//170530  

	for(;;)
	{
		LED_GREEN_ON();
		chThdSleepMilliseconds(500);
		LED_GREEN_OFF();
		chThdSleepMilliseconds(500);
	}
}



int bldc_init(void)
{
	halInit();
	chSysInit();

	chThdSleepMilliseconds(1000);

	hw_init_gpio();

	//spi_dac_hw_init();
	//spi_dac_write_A( 100) ;

	mcpwm_init();

	//chThdSleepMilliseconds(1000);

	bldc_start();

	return 0;
}


float qVelRef = 0.01f;
float dbg_fTheta;
float dbg_fMea;
uint16_t dbg_AccumTheta;

int bldc_start(void)
{
	chThdCreateStatic(periodic_thread_wa, sizeof(periodic_thread_wa), NORMALPRIO, periodic_thread, NULL);

	//CtrlParm.qVelRef=-0.01f;

	return 0;

}

