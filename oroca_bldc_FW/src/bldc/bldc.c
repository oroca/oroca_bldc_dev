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
/*
	OROCA BLDC PROJECT.


 */

#include "bldc.h"
#include "usb_uart.h"
#include "../../mavlink/oroca_bldc/mavlink.h"



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

int send( uint8_t data );
bool recv( uint8_t ch );



static THD_WORKING_AREA(periodic_thread_wa, 1024);
static THD_WORKING_AREA(uart_thread_wa, 128);



static msg_t periodic_thread(void *arg) {
	(void)arg;

	chRegSetThreadName("Main periodic");

	int fault_print = 0;

	for(;;)
	{

		chThdSleepMilliseconds(10);
	}

	return 0;
}





int bldc_init(void)
{
	halInit();
	chSysInit();

	chThdSleepMilliseconds(1000);

	hw_init_gpio();

	mc_configuration mcconf;
	mcpwm_init(&mcconf);

	comm_usb_init();

	return 0;
}


uint8_t Ch;

static msg_t uart_process_thread(void *arg) {
	(void)arg;

	chRegSetThreadName("uart rx process");

	//process_tp = chThdSelf();

	for(;;)
	{
		//chThdSleepMilliseconds(1);

		Ch = usb_uart_getch();
		if( recv( Ch ) )
		{
			send( 1 );
		}
	}


	return 0;
}


float qVelRef = 0.01f;
float dbg_fTheta;
float dbg_fMea;
uint16_t dbg_AccumTheta;

int bldc_start(void)
{



	//-- 스레드 생성
	//
	chThdCreateStatic(periodic_thread_wa, sizeof(periodic_thread_wa), NORMALPRIO, periodic_thread, NULL);
	chThdCreateStatic(uart_thread_wa, sizeof(uart_thread_wa), NORMALPRIO, uart_process_thread, NULL);



	//-- IDLE
	//
	for(;;)
	{
		//palSetPad(GPIOA, 7);
		chThdSleepMilliseconds(1);
		//palClearPad(GPIOA, 7);

		//Ch = usb_uart_getch();

		/*
		if( Ch == 'q' )
		{
			Ch = 0;
			qVelRef += 0.01;
			//debug_print_usb("Enter q : %f\r\n", qVelRef);

		}
		if( Ch == 'a' )
		{
			Ch = 0;
			qVelRef -= 0.01;
			//debug_print_usb("Enter a : %f\r\n", qVelRef);
		}

		//debug_print_usb("8 %f 0\r\n", dbg_fTheta );
		//debug_print_usb("%d\r\n", dbg_AccumTheta );
		usb_uart_printf("500 %f %f 0\r\n", qVelRef*10000, dbg_fMea*10000 );
		*/
	}
}


int send( uint8_t data )
{
    mavlink_message_t msg; // Mavlink 메세지 구조체
    uint8_t buf[1024];           // 메세지의 엔코딩된 전송 프레임

    // TEST_CMD 메세지에 대한 자동으로 생성된 함수,
    // 인자 1은 System ID  ==> ex)같은 메세지도 여러 보드로 구분할때 사용.
    // 인자 2는 컴포넌트 ID  ==> ex) 한 보드내에 여러 구룹이 있을때 구분용으로 사용 .
    // 인자 4 부터는 메세지 정의시 필드 항목 - cmd_1, arg1,arg2
    mavlink_msg_test_cmd_pack( 9, 121, &msg, data ,92,93);

    // 다음 함수내부에서 전송할 프레임을 완성함, CRC 생성등
    int len = mavlink_msg_to_send_buffer(buf, &msg);

   // 시리얼 포트로 버퍼 포인터는 buf, 길이는 len 내용을 전달 하면 완성됨

    usb_uart_write(buf, len);
}


bool recv( uint8_t ch )
{
	bool ret = false;


	mavlink_message_t msg; // 로컬변수로 선언해도 잘 수행 되는데 아마도 실제 자료구조는 static 으로 구현 되는거 같아요.
	mavlink_status_t status; // 현재 수신된 데이터 파싱한 상태 리턴값.

	if (mavlink_parse_char(MAVLINK_COMM_0, ch, &msg, &status) == MAVLINK_FRAMING_OK)
	{
		if( MAVLINK_MSG_ID_TEST_CMD == msg.msgid ) // 메세지 ID 가  TEST_CMD 라면 해석
		{
			mavlink_test_cmd_t test_cmd;
			mavlink_msg_test_cmd_decode( &msg, &test_cmd); // 메세지 디코딩

			//Serial.print("seq= ");
			//Serial.println(test_cmd.arg1);
			ret = true;
		}
    }

	return ret;
}
