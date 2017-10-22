/*
	OROCA BLDC PROJECT.



*/
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "hw.h"
#include "timeout.h"

#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "main.h"

#include "src/core/uart3.h"

/*
 * Timers used:
 * TIM7: servo
 * TIM1: mcpwm
 * TIM2: mcpwm
 * TIM12: mcpwm
 * TIM8: mcpwm
 * TIM3: servo_dec/Encoder (HW_R2)/servo_simple
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

static THD_WORKING_AREA(periodic_thread_wa, 1024);
static THD_WORKING_AREA(timer_thread_wa, 128);

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

static THD_FUNCTION(timer_thread, arg) {
	(void)arg;

	chRegSetThreadName("msec_timer");

	for(;;) {
		//packet_timerfunc();
		chThdSleepMilliseconds(1);
	}
}

/*---------------------------------------------------------------------------
     TITLE   : main
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
int main(void)
{
	halInit();
	chSysInit();

	chThdSleepMilliseconds(1000);

	hw_init_gpio();

	mcpwm_init();

	//user_interface_configure();

	timeout_init();
	timeout_configure(1000);

	// Threads
	chThdCreateStatic(periodic_thread_wa, sizeof(periodic_thread_wa), NORMALPRIO, periodic_thread, NULL);
	chThdCreateStatic(timer_thread_wa, sizeof(timer_thread_wa), NORMALPRIO, timer_thread, NULL);

//=================================
	for(;;)
	{
		chThdSleepMilliseconds(10);
	}

	return 0;
}

