/*
	Copyright 2012-2014 OROCA ESC Project 	www.oroca.org

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
 * hw_40.6
 *
 *  Created on: 22 nov 2014
 *      Author: bakchajang
 */

#ifndef HW_OROCA_H_
#define HW_OROCA_H_

#define SYSTEM_CORE_CLOCK		168000000

// Macros
#define ENABLE_GATE()			palSetPad(GPIOC, 10)
#define DISABLE_GATE()			palClearPad(GPIOC, 10)
#define DCCAL_ON()				palSetPad(GPIOB, 12)
#define DCCAL_OFF()				palClearPad(GPIOB, 12)
#define IS_DRV_FAULT()			(!palReadPad(GPIOC, 12))

#define LED_GREEN_ON()				palSetPad(GPIOC, 4)
#define LED_GREEN_OFF()				palClearPad(GPIOC, 4)
#define LED_RED_ON()				palSetPad(GPIOC, 5)
#define LED_RED_OFF()				palClearPad(GPIOC, 5)

/*
 * ADC Vector
 *
 * 0:	IN0		SENS3
 * 1:	IN1		SENS2
 * 2:	IN2		SENS1
 * 3:	IN8		CURR2
 * 4:	IN9		CURR1
 * 5:	IN3		TEMP_MOSFET
 * 6:	Vrefint
 * 7:	IN6		ADC_EXT2
 * 8:	IN12	AN_IN
 * 9:	IN4		TX_SDA_NSS
 * 10:	IN5     ADC_EXT
 * 11:	IN3 	TEMP_MOTOR
 */

#define HW_ADC_CHANNELS				12
#define HW_ADC_NBR_CONV				4

// ADC Indexes
#define ADC_IND_SENS1				2
#define ADC_IND_SENS2				1
#define ADC_IND_SENS3				0
#define ADC_IND_CURR1				3
#define ADC_IND_CURR2				4
#define ADC_IND_VIN_SENS			8
#define ADC_IND_EXT					10
#define ADC_IND_EXT2				7
#define ADC_IND_TEMP_MOS1			5
#define ADC_IND_TEMP_MOS2			5
#define ADC_IND_TEMP_MOS3			5
#define ADC_IND_TEMP_MOS4			5
#define ADC_IND_TEMP_MOS5			5
#define ADC_IND_TEMP_MOS6			5
#define ADC_IND_TEMP_PCB			5
#define ADC_IND_VREFINT				6

// ADC macros and settings

// Component parameters (can be overridden)
// Correction factor for computations that depend on the old resistor division factor
#define VDIV_CORR			((VIN_R2 / (VIN_R2 + VIN_R1)) / (2.2 / (2.2 + 33.0)))

// Actual voltage on 3.3V net based on internal reference
//#define V_REG				(1.21 / ((float)ADC_Value[ADC_IND_VREFINT] / 4095.0))
#define V_REG				3.3

#ifndef VIN_R1
#define VIN_R1				100000.0
#endif
#ifndef VIN_R2
#define VIN_R2				1000.0
#endif
#ifndef CURRENT_AMP_GAIN
#define CURRENT_AMP_GAIN	10.0
#endif
#ifndef CURRENT_SHUNT_RES
#define CURRENT_SHUNT_RES	0.001
#endif

// Input voltage
#define GET_INPUT_VOLTAGE()	((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VIN_SENS] * ((VIN_R1 + VIN_R2) / VIN_R2))

// Voltage on ADC channel
#define ADC_VOLTS(ch)		((float)ADC_Value[ch] / 4095.0 * V_REG)

// NTC Termistors
//#define NTC_RES(adc_val)	(10000.0 / ((4096.0 / (float)adc_val) - 1.0))
#define NTC_RES(adc_val)	((4095.0 * 10000.0) / adc_val - 10000.0)
#define NTC_TEMP(adc_ind)	(1.0 / ((logf(NTC_RES(ADC_Value[adc_ind]) / 10000.0) / 3434.0) + (1.0 / 298.15)) - 273.15)

// Double samples in beginning and end for positive current measurement.
// Useful when the shunt sense traces have noise that causes offset.
#ifndef CURR1_DOUBLE_SAMPLE
#define CURR1_DOUBLE_SAMPLE	0
#endif
#ifndef CURR2_DOUBLE_SAMPLE
#define CURR2_DOUBLE_SAMPLE	0
#endif

// Number of servo outputs
#define HW_SERVO_NUM		2

// UART Peripheral
#define HW_UART_DEV			UARTD6
#define HW_UART_GPIO_AF		GPIO_AF_USART6
#define HW_UART_TX_PORT		GPIOC
#define HW_UART_TX_PIN		6
#define HW_UART_RX_PORT		GPIOC
#define HW_UART_RX_PIN		7

// ICU Peripheral for servo decoding
#define HW_ICU_DEV			ICUD3
#define HW_ICU_CHANNEL		ICU_CHANNEL_2
#define HW_ICU_GPIO_AF		GPIO_AF_TIM3
#define HW_ICU_GPIO			GPIOB
#define HW_ICU_PIN			5

// I2C Peripheral
#define HW_I2C_DEV			I2CD2
#define HW_I2C_GPIO_AF		GPIO_AF_I2C2
#define HW_I2C_SCL_PORT		GPIOB
#define HW_I2C_SCL_PIN		10
#define HW_I2C_SDA_PORT		GPIOB
#define HW_I2C_SDA_PIN		11

// Hall/encoder pins
#define HW_HALL_ENC_GPIO1		GPIOB
#define HW_HALL_ENC_PIN1		6
#define HW_HALL_ENC_GPIO2		GPIOB
#define HW_HALL_ENC_PIN2		7
#define HW_HALL_ENC_GPIO3		GPIOC
#define HW_HALL_ENC_PIN3		11
#define HW_ENC_TIM				TIM4
#define HW_ENC_TIM_AF			GPIO_AF_TIM4
#define HW_ENC_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE)
#define HW_ENC_EXTI_PORTSRC		EXTI_PortSourceGPIOC
#define HW_ENC_EXTI_PINSRC		EXTI_PinSource11
#define HW_ENC_EXTI_CH			EXTI15_10_IRQn
#define HW_ENC_EXTI_LINE		EXTI_Line11
#define HW_ENC_EXTI_ISR_VEC		EXTI15_10_IRQHandler
#define HW_ENC_TIM_ISR_CH		TIM4_IRQn
#define HW_ENC_TIM_ISR_VEC		TIM4_IRQHandler

// NRF pins
#define NRF_PORT_CSN			GPIOA
#define NRF_PIN_CSN				4
#define NRF_PORT_SCK			GPIOA
#define NRF_PIN_SCK				5
#define NRF_PORT_MOSI			GPIOA
#define NRF_PIN_MOSI			7
#define NRF_PORT_MISO			GPIOA
#define NRF_PIN_MISO			6

// SPI pins
#define HW_SPI_DEV				SPID1
#define HW_SPI_GPIO_AF			GPIO_AF_SPI1
#define HW_SPI_PORT_NSS			GPIOA
#define HW_SPI_PIN_NSS			4
#define HW_SPI_PORT_SCK			GPIOA
#define HW_SPI_PIN_SCK			5
#define HW_SPI_PORT_MOSI		GPIOA
#define HW_SPI_PIN_MOSI			7
#define HW_SPI_PORT_MISO		GPIOA
#define HW_SPI_PIN_MISO			6

extern  uint16_t ADC_Value[];
// Measurement macros
#define ADC_V_L1				ADC_Value[ADC_IND_SENS1]
#define ADC_V_L2				ADC_Value[ADC_IND_SENS2]
#define ADC_V_L3				ADC_Value[ADC_IND_SENS3]
#define ADC_V_ZERO				(ADC_Value[ADC_IND_VIN_SENS] / 2)

// Macros
#define READ_HALL1()			palReadPad(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1)
#define READ_HALL2()			palReadPad(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2)
#define READ_HALL3()			palReadPad(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)

#endif /* HW_40_H_ */
