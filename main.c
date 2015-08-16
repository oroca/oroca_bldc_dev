/*
	Copyright 2012-2015 Benjamin Vedder	benjamin@vedder.se

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

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "main.h"
#include "mcpwm.h"
#include "ledpwm.h"
#include "comm_usb.h"
#include "ledpwm.h"
#include "terminal.h"
#include "hw.h"
#include "app.h"
#include "packet.h"
#include "commands.h"
#include "timeout.h"
#include "comm_can.h"
#include "ws2811.h"
#include "led_external.h"
#include "encoder.h"

//jsyoon
#include<stdarg.h>
#include "chprintf.h"
#include "memstreams.h"

//jsyoon
#ifdef _TEST_STM32F4_DISCOVERY
void _LED_init();
void _LED_on_off( int led);
void _SD2_init();
int debug_print( const char *fmt, ...);
void debuf_out_( const char *pFunctionName, const int lineNumber, int continueFlag);
#endif //_TEST_STM32F4_DISCOVERY




//int debug_print_usb( const char *fmt, ...);

int debug_print_usb( const char *fmt, ...){
	extern SerialUSBDriver SDU1;

#define DEBUG_USB_BUFFER_MAX	1024
	static char debug_usb_buff[DEBUG_USB_BUFFER_MAX];

	int     ret=0;
	char *str = debug_usb_buff;
	size_t size =  sizeof(debug_usb_buff);


	va_list ap;
	MemoryStream ms;
	BaseSequentialStream *chp;

	/* Memory stream object to be used as a string writer.*/
	msObjectInit(&ms, (uint8_t *)str, size, 0);

	/* Performing the print operation using the common code.*/
	chp = (BaseSequentialStream *)&ms;
	va_start(ap, fmt);
	chvprintf(chp, fmt, ap);
	va_end(ap);

	/* Final zero and size return.*/
	chSequentialStreamPut(chp, 0);
	ret =  ms.eos - 1;


	//#define sdAsynchronousWrite(sdp, b, n)   chOQWriteTimeout(&(sdp)->oqueue, b, n, TIME_IMMEDIATE)
	//sdAsynchronousWrite( &SDU1,  debug_usb_buff, ret );
	chOQWriteTimeout( &(SDU1.oqueue), debug_usb_buff, ret , TIME_IMMEDIATE);

	return ret;
}


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

/*
 * Notes:
 *
 * Disable USB VBUS sensing:
 * ChibiOS-RT-master/os/hal/platforms/STM32/OTGv1/usb_lld.c
 *
 * change
 * otgp->GCCFG = GCCFG_VBUSASEN | GCCFG_VBUSBSEN | GCCFG_PWRDWN;
 * to
 * otgp->GCCFG = GCCFG_NOVBUSSENS | GCCFG_PWRDWN;
 *
 * This should be handled automatically with the latest version of
 * ChibiOS since I have added an option to the makefile.
 *
 */

// Private variables
#define ADC_SAMPLE_MAX_LEN		2000
static volatile int16_t curr0_samples[ADC_SAMPLE_MAX_LEN];
static volatile int16_t curr1_samples[ADC_SAMPLE_MAX_LEN];
static volatile int16_t ph1_samples[ADC_SAMPLE_MAX_LEN];
static volatile int16_t ph2_samples[ADC_SAMPLE_MAX_LEN];
static volatile int16_t ph3_samples[ADC_SAMPLE_MAX_LEN];
static volatile int16_t vzero_samples[ADC_SAMPLE_MAX_LEN];
static volatile uint8_t status_samples[ADC_SAMPLE_MAX_LEN];
static volatile int16_t curr_fir_samples[ADC_SAMPLE_MAX_LEN];
static volatile int16_t f_sw_samples[ADC_SAMPLE_MAX_LEN];

static volatile int sample_len = 1000;
static volatile int sample_int = 1;
static volatile int sample_ready = 1;
static volatile int sample_now = 0;
static volatile int sample_at_start = 0;
static volatile int was_start_sample = 0;
static volatile int start_comm = 0;
static volatile float main_last_adc_duration = 0.0;

static WORKING_AREA(periodic_thread_wa, 1024);
static WORKING_AREA(sample_send_thread_wa, 1024);
static WORKING_AREA(timer_thread_wa, 128);

static Thread *sample_send_tp;

static msg_t periodic_thread(void *arg) {
	(void)arg;

	chRegSetThreadName("Main periodic");

	int fault_print = 0;

	for(;;) {

		if (mcpwm_get_state() == MC_STATE_RUNNING) {
			ledpwm_set_intensity(LED_GREEN, 1.0);
		} else {
			ledpwm_set_intensity(LED_GREEN, 0.2);
		}

		mc_fault_code fault = mcpwm_get_fault();
		if (fault != FAULT_CODE_NONE) {
			if (!fault_print && AUTO_PRINT_FAULTS) {
				fault_print = 1;
				commands_printf("%s\n", mcpwm_fault_to_string(mcpwm_get_fault()));
			}

			for (int i = 0;i < (int)fault;i++) {
				ledpwm_set_intensity(LED_RED, 1.0);
				chThdSleepMilliseconds(250);
				ledpwm_set_intensity(LED_RED, 0.0);
				chThdSleepMilliseconds(250);
			}

			chThdSleepMilliseconds(500);
		} else {
			ledpwm_set_intensity(LED_RED, 0.0);
			fault_print = 0;
		}

		if (mcpwm_get_state() == MC_STATE_DETECTING) {
			//--->commands_send_rotor_pos(mcpwm_get_detect_pos());
		}

#if ENCODER_ENABLE
//		commands_send_rotor_pos(encoder_read_deg());
//		comm_can_set_pos(0, encoder_read_deg());
#endif

		chThdSleepMilliseconds(10);
	}

	return 0;
}

static msg_t sample_send_thread(void *arg) {
	(void)arg;

	chRegSetThreadName("Main sample");

	sample_send_tp = chThdSelf();

	for(;;) {
		chEvtWaitAny((eventmask_t) 1);

		for (int i = 0;i < sample_len;i++) {
			uint8_t buffer[20];
			int index = 0;

			buffer[index++] = curr0_samples[i] >> 8;
			buffer[index++] = curr0_samples[i];
			buffer[index++] = curr1_samples[i] >> 8;
			buffer[index++] = curr1_samples[i];
			buffer[index++] = ph1_samples[i] >> 8;
			buffer[index++] = ph1_samples[i];
			buffer[index++] = ph2_samples[i] >> 8;
			buffer[index++] = ph2_samples[i];
			buffer[index++] = ph3_samples[i] >> 8;
			buffer[index++] = ph3_samples[i];
			buffer[index++] = vzero_samples[i] >> 8;
			buffer[index++] = vzero_samples[i];
			buffer[index++] = status_samples[i];
			buffer[index++] = curr_fir_samples[i] >> 8;
			buffer[index++] = curr_fir_samples[i];
			buffer[index++] = f_sw_samples[i] >> 8;
			buffer[index++] = f_sw_samples[i];

			commands_send_samples(buffer, index);
		}
	}

	return 0;
}

static msg_t timer_thread(void *arg) {
	(void)arg;

	chRegSetThreadName("msec_timer");

	for(;;) {
		packet_timerfunc();
		chThdSleepMilliseconds(1);
	}

	return 0;
}

/*
 * Called every time new ADC values are available. Note that
 * the ADC is initialized from mcpwm.c
 */
void main_dma_adc_handler(void) {
	//ledpwm_update_pwm();


	//		main_last_adc_duration = mcpwm_get_last_adc_isr_duration();

}

float main_get_last_adc_isr_duration(void) {
	return main_last_adc_duration;
}

void main_sample_print_data(bool at_start, uint16_t len, uint8_t decimation) {


}

int main(void) {
	halInit();
	chSysInit();

	chThdSleepMilliseconds(1000);

	conf_general_init();
	hw_init_gpio();

	ledpwm_init();

//jsyoon
#ifdef _TEST_STM32F4_DISCOVERY
	_LED_init();
	_SD2_init();
	debuf_out_( __FUNCTION__, __LINE__, 0); //
#endif //_TEST_STM32F4_DISCOVERY


	mc_configuration mcconf;
	conf_general_read_mc_configuration(&mcconf);
	mcpwm_init(&mcconf);

	commands_init();
	comm_usb_init();

	app_configuration appconf;
	conf_general_read_app_configuration(&appconf);
	app_init(&appconf);

	timeout_init();
	timeout_configure(appconf.timeout_msec, appconf.timeout_brake_current);

#if CAN_ENABLE
	comm_can_init();
#endif

#if WS2811_ENABLE
	ws2811_init();
	led_external_init();
#endif

#if ENCODER_ENABLE
	encoder_init();
#endif

	// Threads
	chThdCreateStatic(periodic_thread_wa, sizeof(periodic_thread_wa), NORMALPRIO, periodic_thread, NULL);
	chThdCreateStatic(sample_send_thread_wa, sizeof(sample_send_thread_wa), NORMALPRIO - 1, sample_send_thread, NULL);
	chThdCreateStatic(timer_thread_wa, sizeof(timer_thread_wa), NORMALPRIO, timer_thread, NULL);


	for(;;) {
		static int debug_out_cnt=0;

		#ifdef _TEST_STM32F4_DISCOVERY
		
		static int led=0;
		_LED_on_off( led);
		led= ~led;
		#endif // _TEST_STM32F4_DISCOVERY


		debug_print_usb("TTTT =%d \n\r", debug_out_cnt);
		//debug_out_cnt += 1234567;
		debug_out_cnt += 12;


		//palSetPad(GPIOA, 7);
		//chThdSleepMilliseconds(5000);
		chThdSleepMilliseconds(200);
		//palClearPad(GPIOA, 7);

		
	}
}

//jsyoon
#ifdef _TEST_STM32F4_DISCOVERY
static SerialConfig ser_cfg_1 = {
    115200,
    0,
    0,
    0,
};

void _LED_init()
{
	palSetPadMode(GPIOD, GPIOD_LED3, PAL_MODE_OUTPUT_PUSHPULL );

	//palClearPad(GPIOD, GPIOD_LED3); // OFF
	palSetPad(GPIOD, GPIOD_LED3);       // ON
}
void _LED_on_off( int led)
{
	if( !led)
		palClearPad(GPIOD, GPIOD_LED3);
	else
		palSetPad(GPIOD, GPIOD_LED3);       /* Orange.  */
}


void _SD2_init()
{
	//static int debugCnt_i = 0;
	sdStart(&SD2, &ser_cfg_1 );
	palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));
	palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));

	//chprintf((BaseSequentialStream *)&SD2, "@@@@ (%d)\r\n", debugCnt_i ++);
}
int debug_print( const char *fmt, ...)
{
	  va_list ap;
  int     ret=0;

  va_start(ap, fmt);
  chvprintf((BaseSequentialStream *)&SD2, fmt, ap);
  va_end(ap);

  return ret;
}

void debuf_out_( const char *pFunctionName, const int lineNumber, int continueFlag)
{
	//jsyoon
	static int debugCnt_i = 0;

	do
	{
		chprintf((BaseSequentialStream *)&SD2, "@@@@ %s:%d (%d)\r\n", pFunctionName, lineNumber, debugCnt_i ++);
		//chThdSleepMilliseconds(1000);
		chThdSleepMilliseconds(10);
	}while( continueFlag );
}

#endif //_TEST_STM32F4_DISCOVERY
