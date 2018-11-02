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


#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"

#include "hw.h"
#include "mc_define.h"
#include "mc_typedef.h"

#include "mc_control.h"
#include "mc_encoder.h"

#include "usart1_print.h"

#include <math.h>


// Defines
#define AS5047_USE_HW_SPI_PINS		1

#define AS5047P_READ_ANGLECOM		(0x3FFF | 0x4000 | 0x8000) // This is just ones
#define AS5047_SAMPLE_RATE_HZ		10000

#if AS5047_USE_HW_SPI_PINS
#ifdef HW_SPI_DEV
#define SPI_SW_MISO_GPIO			HW_SPI_PORT_MISO
#define SPI_SW_MISO_PIN				HW_SPI_PIN_MISO
#define SPI_SW_MOSI_GPIO			HW_SPI_PORT_MOSI
#define SPI_SW_MOSI_PIN				HW_SPI_PIN_MOSI
#define SPI_SW_SCK_GPIO				HW_SPI_PORT_SCK
#define SPI_SW_SCK_PIN				HW_SPI_PIN_SCK
#define SPI_SW_CS_GPIO				HW_SPI_PORT_NSS
#define SPI_SW_CS_PIN				HW_SPI_PIN_NSS
#else
// Note: These values are hardcoded.
#define SPI_SW_MISO_GPIO			GPIOB
#define SPI_SW_MISO_PIN				4
#define SPI_SW_MOSI_GPIO			GPIOB
#define SPI_SW_MOSI_PIN				5
#define SPI_SW_SCK_GPIO				GPIOB
#define SPI_SW_SCK_PIN				3
#define SPI_SW_CS_GPIO				GPIOB
#define SPI_SW_CS_PIN				0
#endif
#else
#define SPI_SW_MISO_GPIO			HW_HALL_ENC_GPIO2
#define SPI_SW_MISO_PIN				HW_HALL_ENC_PIN2
#define SPI_SW_SCK_GPIO				HW_HALL_ENC_GPIO1
#define SPI_SW_SCK_PIN				HW_HALL_ENC_PIN1
#define SPI_SW_CS_GPIO				HW_HALL_ENC_GPIO3
#define SPI_SW_CS_PIN				HW_HALL_ENC_PIN3
#endif

// Private types


// Private variables
static bool index_found = false;
static uint32_t enc_counts = 10000;
encoder_mode EncMode = ENCODER_MODE_NONE;
//float last_enc_angle = 0.0;

tSMC smc1;

// Private functions
static void spi_transfer(uint16_t *in_buf, const uint16_t *out_buf, int length);
static void spi_begin(void);
static void spi_end(void);
static void spi_delay(void);


void encoder_deinit(void) {
	nvicDisableVector(HW_ENC_EXTI_CH);
	nvicDisableVector(HW_ENC_TIM_ISR_CH);

	TIM_DeInit(HW_ENC_TIM);

	palSetPadMode(SPI_SW_MISO_GPIO, SPI_SW_MISO_PIN, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(SPI_SW_SCK_GPIO, SPI_SW_SCK_PIN, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(SPI_SW_CS_GPIO, SPI_SW_CS_PIN, PAL_MODE_INPUT_PULLUP);

	palSetPadMode(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2, PAL_MODE_INPUT_PULLUP);

	index_found = false;
	EncMode = ENCODER_MODE_NONE;
	//last_enc_angle = 0.0;
}

void encoder_init_abi(uint32_t counts) {
	EXTI_InitTypeDef   EXTI_InitStructure;

	// Initialize variables
	index_found = false;
	enc_counts = counts;

	palSetPadMode(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1, PAL_MODE_ALTERNATE(HW_ENC_TIM_AF));
	palSetPadMode(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2, PAL_MODE_ALTERNATE(HW_ENC_TIM_AF));
//	palSetPadMode(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3, PAL_MODE_ALTERNATE(HW_ENC_TIM_AF));

	// Enable timer clock
	HW_ENC_TIM_CLK_EN();

	// Enable SYSCFG clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	TIM_EncoderInterfaceConfig (HW_ENC_TIM, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
	TIM_SetAutoreload(HW_ENC_TIM, enc_counts - 1);

	// Filter
	HW_ENC_TIM->CCMR1 |= 6 << 12 | 6 << 4;
	HW_ENC_TIM->CCMR2 |= 6 << 4;

	TIM_Cmd(HW_ENC_TIM, ENABLE);

	// Interrupt on index pulse

	// Connect EXTI Line to pin
	SYSCFG_EXTILineConfig(HW_ENC_EXTI_PORTSRC, HW_ENC_EXTI_PINSRC);

	// Configure EXTI Line
	EXTI_InitStructure.EXTI_Line = HW_ENC_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	// Enable and set EXTI Line Interrupt to the highest priority
	nvicEnableVector(HW_ENC_EXTI_CH, 0);

	EncMode = ENCODER_MODE_ABI;
}

void encoder_init_as5047p_spi(void) {
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	palSetPadMode(SPI_SW_MISO_GPIO, SPI_SW_MISO_PIN, PAL_MODE_INPUT);
	palSetPadMode(SPI_SW_SCK_GPIO, SPI_SW_SCK_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(SPI_SW_CS_GPIO, SPI_SW_CS_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);

	// Set MOSI to 1
#if AS5047_USE_HW_SPI_PINS
	palSetPadMode(SPI_SW_MOSI_GPIO, SPI_SW_MOSI_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	palSetPad(SPI_SW_MOSI_GPIO, SPI_SW_MOSI_PIN);
#endif

	// Enable timer clock
	HW_ENC_TIM_CLK_EN();

	// Time Base configuration
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = ((168000000 / 2 / AS5047_SAMPLE_RATE_HZ) - 1);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(HW_ENC_TIM, &TIM_TimeBaseStructure);

	// Enable overflow interrupt
	TIM_ITConfig(HW_ENC_TIM, TIM_IT_Update, ENABLE);

	// Enable timer
	TIM_Cmd(HW_ENC_TIM, ENABLE);

	nvicEnableVector(HW_ENC_TIM_ISR_CH, 6);

	EncMode = ENCODER_MODE_AS5047P_SPI;
	index_found = true;
}

bool encoder_is_configured(void) {
	return EncMode != ENCODER_MODE_NONE;
}

float encoder_read_deg(void) {
	static float angle = 0.0;

	switch (EncMode) {
	case ENCODER_MODE_ABI:
		angle = ((float)HW_ENC_TIM->CNT * 360.0) / (float)enc_counts;
		break;

	case ENCODER_MODE_AS5047P_SPI:
		angle = smc1.angle;
		break;

	default:
		break;
	}

	return angle;
}

/**
 * Reset the encoder counter. Should be called from the index interrupt.
 */
void encoder_reset(void) {
	// Only reset if the pin is still high to avoid too short pulses, which
	// most likely are noise.
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	if (palReadPad(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)) {
		const unsigned int cnt = HW_ENC_TIM->CNT;
		static int bad_pulses = 0;
		const unsigned int lim = enc_counts / 20;

		if (index_found) {
			// Some plausibility filtering.
			if (cnt > (enc_counts - lim) || cnt < lim) {
				HW_ENC_TIM->CNT = 0;
				bad_pulses = 0;
			} else {
				bad_pulses++;

				if (bad_pulses > 5) {
					index_found = 0;
				}
			}
		} else {
			HW_ENC_TIM->CNT = 0;
			index_found = true;
			bad_pulses = 0;
		}
	}
}

/**
 * Timer interrupt
 */
uint16_t cap1_cnt=0, cap1_r_new=0, cap1_r_old=0, cap1_f=0;
uint16_t pul1_width=0,pul1_period=0;
int16_t encpos=0,encpos1=0;
int16_t delta_pos=0,delta_pos1=0;

float enc_sum = 0.0f;
uint16_t enc_sum_cnt=0;



void encoder_tim_isr(void) {
	uint16_t pos;
	float vecpos;

	//LED_RED_ON();
	if(EncMode == ENCODER_MODE_AS5047P_SPI)
	{
		spi_begin();
		spi_transfer(&pos, 0, 1);
		spi_end();

		pos &= 0x3FFF;
		vecpos = fmodf((float)pos, 2340.428571f);
		smc1.ThetaMeas = ((float)vecpos * TWOPI) / 2340.428571f;

		smc1.AlphaMeas = sinf(smc1.ThetaMeas);
		smc1.BetaMeas = cosf(smc1.ThetaMeas);

		encoder_PLLThetaEstimation(&smc1);

		smc1.Theta  = smc1.ThetaEst - PI - (0.017453293f * 25.0f);

	}
	else if (EncMode == ENCODER_MODE_AHALL)
	{
		smc1.AlphaMeas = ((float)ADC_Value[ADC_IND_SENS1] - 1241.0f)/ 4095.0f;
		smc1.BetaMeas = ((float)ADC_Value[ADC_IND_SENS2] - 1241.0f)/ 4095.0f;
	
		encoder_PLLThetaEstimation(&smc1);

		smc1.Theta = smc1.ThetaEst + 0.3f;
		smc1.Futi  = smc1.pll_PIout / TWOPI * HALL_SENSOR_FREQ;
		smc1.rpm 	= 120.0f * smc1.Futi / 7.0f;
	}
	else if(EncMode == ENCODER_MODE_PWM)
	{
		if((TIM4->SR & TIM_IT_CC1)&&(TIM4->DIER & TIM_IT_CC1))
		{
			cap1_cnt++;
			TIM4->SR = (uint16_t)~TIM_IT_CC1;	// clear flag
			if((GPIOA->IDR >> HW_HALL_ENC_PIN1) & 1U)
			{	
				// Timer4 Ch1 pin is High
				cap1_r_new = TIM4->CCR1; // read capture data
				pul1_period = (uint32_t)(cap1_r_new - cap1_r_old);
				cap1_r_old = cap1_r_new;
				TIM4->CCER &= ~TIM_CCER_CC1P;	// to falling edge
			}
			else
			{	// Timer4 Ch1 pin is Low
				cap1_f = TIM4->CCR1; // read capture data
				pul1_width = (uint32_t)(cap1_f - cap1_r_new);
				TIM4->CCER |= TIM_CCER_CC1P;	// to rising edge
			}
		}
	}



	//LED_RED_OFF();

	
}

/**
 * Set the number of encoder counts.
 *
 * @param counts
 * The number of encoder counts
 */
void encoder_set_counts(uint32_t counts) {
	if (counts != enc_counts) {
		enc_counts = counts;
		TIM_SetAutoreload(HW_ENC_TIM, enc_counts - 1);
		index_found = false;
	}
}

/**
 * Check if the index pulse is found.
 *
 * @return
 * True if the index is found, false otherwise.
 */
bool encoder_index_found(void) {
	return index_found;
}

// Software SPI
static void spi_transfer(uint16_t *in_buf, const uint16_t *out_buf, int length) {
	for (int i = 0;i < length;i++) {
		uint16_t send = out_buf ? out_buf[i] : 0xFFFF;
		uint16_t recieve = 0;

		for (int bit = 0;bit < 16;bit++) {
			//palWritePad(HW_SPI_PORT_MOSI, HW_SPI_PIN_MOSI, send >> 15);
			send <<= 1;

			spi_delay();
			palSetPad(SPI_SW_SCK_GPIO, SPI_SW_SCK_PIN);
			spi_delay();

			int r1, r2, r3;
			r1 = palReadPad(SPI_SW_MISO_GPIO, SPI_SW_MISO_PIN);
			__NOP();
			r2 = palReadPad(SPI_SW_MISO_GPIO, SPI_SW_MISO_PIN);
			__NOP();
			r3 = palReadPad(SPI_SW_MISO_GPIO, SPI_SW_MISO_PIN);

			recieve <<= 1;
			if (utils_middle_of_3_int(r1, r2, r3)) {
				recieve |= 1;
			}

			palClearPad(SPI_SW_SCK_GPIO, SPI_SW_SCK_PIN);
			spi_delay();
		}

		if (in_buf) {
			in_buf[i] = recieve;
		}
	}
}

static void spi_begin(void) {
	palClearPad(SPI_SW_CS_GPIO, SPI_SW_CS_PIN);
}

static void spi_end(void) {
	palSetPad(SPI_SW_CS_GPIO, SPI_SW_CS_PIN);
}

static void spi_delay(void) {
	__NOP();
	__NOP();
	__NOP();
	__NOP();
}


void encoder_init_pwm(void) {
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;

	// Enable timer clock
	HW_ENC_TIM_CLK_EN();

    /* TIM3_ch1(PA6),TIM3_ch2(PA7) configuration */
	palSetPadMode(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1, PAL_MODE_ALTERNATE(HW_ENC_TIM_AF));

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 65535;
    TIM_TimeBaseStructure.TIM_Prescaler = 72-1; // 1 usec for 72MHz clock
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);

    /* TIM enable counter */
    TIM_Cmd(TIM4, ENABLE);
    
    /* Enable the CC2 Interrupt Request */
    TIM_ITConfig(TIM4, TIM_IT_CC1, ENABLE);

	/* Enable the TIM3 global Interrupt */
	nvicEnableVector(HW_ENC_TIM_ISR_CH, 6);

	EncMode = ENCODER_MODE_AS5047P_SPI;
	index_found = true;
}





void encoder_3HarmonicFilter(tSMC *s)
{
	s->cos3th = cosf(3.0f * s->Theta);
	s->sin3th = sinf(3.0f * s->Theta);

	s->pllA_sin3th = s->AlphaMeas * s->sin3th * s->Gamma;
	s->pllA_cos3th = s->AlphaMeas * s->cos3th * s->Gamma;
	
	s->pllB_cos3th = s->BetaMeas * s->cos3th * s->Gamma;
	s->pllB_sin3th = s->BetaMeas * s->sin3th * s->Gamma;

	s->pllA_cos3th_Integral += s->pllA_cos3th;
	s->pllA_sin3th_Integral += s->pllA_sin3th;
	
	s->pllB_sin3th_Integral += s->pllB_sin3th;
	s->pllB_cos3th_Integral += s->pllB_cos3th;

	s->Asin3th= s->pllA_sin3th_Integral * s->sin3th;
	s->Acos3th= s->pllA_cos3th_Integral * s->cos3th;

	s->Bsin3th= s->pllB_sin3th_Integral * s->sin3th;
	s->Bcos3th= s->pllB_cos3th_Integral * s->cos3th;

	s->ANF_PLLA = s->AlphaMeas - s->Asin3th - s->Acos3th;
	s->ANF_PLLB = s->BetaMeas - s->Bsin3th - s->Bcos3th;
	
	s->costh = cosf(s->Theta);
	s->sinth = sinf(s->Theta);
	
	s->SinCosTheta = s->ANF_PLLA * s->costh;
	s->CosSinTheta = s->ANF_PLLB * s->sinth;

}




void encoder_PLLThetaEstimation(tSMC *s)
{
	s->Kpll =  2.0f;     	;
	s->Ipll =  1.0f / (float)AS5047_SAMPLE_RATE_HZ;

	s->costh = cosf(s->ThetaEst0);
	s->sinth = sinf(s->ThetaEst0);
	
	s->SinCosTheta = s->AlphaMeas * s->costh;
	s->CosSinTheta = s->BetaMeas * s->sinth;
	
	float err, tmp_kp, tmp_kpi; 									
	tmp_kp = s->Kpll;
	tmp_kpi = s->Kpll + s->Ipll;//(kp+ki)err
	err = s->SinCosTheta - s->CosSinTheta;											
	s->pll_PIout += ((tmp_kpi * err) - (tmp_kp * s->pll_Err0));					
	s->pll_PIout = Bound_limit(s->pll_PIout, 10.0f);						
	s->pll_Err0= err;									
	
	s->Omega = s->pll_PIout;
	
	s->ThetaEst += s->Omega ;
	if(TWOPI <s->ThetaEst) s->ThetaEst = s->ThetaEst - TWOPI;
	else if(s->ThetaEst < 0.0f) s->ThetaEst = TWOPI + s->ThetaEst;
	
	s->ThetaEst0 = s->ThetaEst; 

}


