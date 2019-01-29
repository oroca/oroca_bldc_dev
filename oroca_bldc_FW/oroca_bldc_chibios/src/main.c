/*
	OROCA BLDC PROJECT.
*/


#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"

#include <stdio.h>
//#include <math.h>
//#include <mavlink/mavlink_proc.h>
#include <string.h>
#include <stdlib.h>

#include "mc_interface.h"
//#include "conf_general.h"

#include "ledpwm.h"
#include "hw.h"
//#include "app.h"
#include "timeout.h"
//#include "comm_can.h"
//#include "ws2811.h"
//#include "led_external.h"
//#include "mc_encoder.h"
//#include "servo.h"
//#include "servo_simple.h"
#include "utils.h"

//#include "comm_usb.h"
//#include "comm_usb_serial.h"

#include "usart1_print.h"



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
 */

static THD_WORKING_AREA(periodic_thread_wa, 128);
static THD_WORKING_AREA(timer_thread_wa, 128);

static THD_FUNCTION(periodic_thread, arg)
{
	(void)arg;

	chRegSetThreadName("BLDC periodic");


	for(;;)
	{
		LED_GREEN_ON();		chThdSleepMilliseconds(50);
		LED_GREEN_OFF();	chThdSleepMilliseconds(50);
		LED_GREEN_ON();		chThdSleepMilliseconds(50);
		LED_GREEN_OFF();	chThdSleepMilliseconds(850);



		//LED_RED_ON();	chThdSleepMilliseconds(50);
		//LED_RED_OFF();	chThdSleepMilliseconds(50);
		//LED_GREEN_ON();	chThdSleepMilliseconds(50);
		//LED_GREEN_OFF();	chThdSleepMilliseconds(850);

		//chvprintf(&SD1, (uint8_t *)"\x1b[2J\x1b[0;0H");
		//chvprintf(&SD1, (uint8_t *)"%f/r/n",mc_interface_get_angle());

	}
}

static THD_FUNCTION(timer_thread, arg) {
	(void)arg;

	chRegSetThreadName("msec_timer");

	//chvprintf(&SD1, (uint8_t *)"to main -> timer_thread\r\n");

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
	
	ledpwm_init();

#if USB_SERIAL_ENABLE
	comm_usb_init();
	chThdSleepMilliseconds(1000);
	
	//chvprintf(&SDU1, (uint8_t *)"\x1b[2J\x1b[0;0H");
	//chvprintf(&SDU1, (uint8_t *)"USB Serial ready...\r\n\r\n\r\n");

	//chvprintf(&SDU1, (uint8_t *)"Project : OROCA BLDC\r\n");
	//chvprintf(&SDU1, (uint8_t *)"by BakChaJang\r\n");
	//chvprintf(&SDU1, (uint8_t *)"date : 2017/11/15\r\n\r\n");

#endif

#if CAN_ENABLE
		comm_can_init();
#endif


	Usart1_print_init();

	mcconf_general_init();

	mcConfiguration_t mcconf;
	mcconf_general_read_mc_configuration(&mcconf);
	mc_interface_init(&mcconf);

	//app_configuration appconf;
	//conf_general_read_app_configuration(&appconf);
	//app_init(&appconf);

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

