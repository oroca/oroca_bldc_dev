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
 * mcpwm.c
 *
 *  Created on: 13 okt 2012
 *      Author: bakchajang
 */

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"

#include "utils.h"

#include "hw.h"
#include "mc_define.h"
#include "mc_typedef.h"

#include "mc_interface.h"
#include "mc_control.h"
#include "mc_sensor.h"
#include "mc_pwm.h"



//======================================================================================
//internal function Declaration

void SetupControlParameters(void);
void update_timer_Duty(unsigned int duty_A,unsigned int duty_B,unsigned int duty_C);

void mcpwm_adc_dma_int_handler(void *p, uint32_t flags);// Interrupt handlers

static THD_WORKING_AREA(timer_thread_wa, 1024);
static THD_FUNCTION(timer_thread, arg);

//======================================================================================
//external function 
void mcpwm_init(volatile mcConfiguration_t *configuration)
{

	utils_sys_lock_cnt();

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

	TIM_DeInit(TIM1);
	TIM_DeInit(TIM8);
	TIM1->CNT = 0;
	TIM8->CNT = 0;

	// TIM1 clock enable
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	// Time Base configuration
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
	TIM_TimeBaseStructure.TIM_Period = SYSTEM_CORE_CLOCK  / (int)PWMFREQ /2;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 1;

	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	// Channel 1, 2 and 3 Configuration in PWM mode
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = TIM1->ARR / 2;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;

	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

	// Automatic Output enable, Break, dead time and lock configuration
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	TIM_BDTRInitStructure.TIM_DeadTime = MCPWM_DEAD_TIME_CYCLES;
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;

	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
	TIM_CCPreloadControl(TIM1, ENABLE);
	TIM_ARRPreloadConfig(TIM1, ENABLE);

	/*
	 * ADC!
	 */
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	// Clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | RCC_APB2Periph_ADC3, ENABLE);

	dmaStreamAllocate(STM32_DMA_STREAM(STM32_DMA_STREAM_ID(2, 4)),3,(stm32_dmaisr_t)mcpwm_adc_dma_int_handler,(void *)0);

	// DMA for the ADC
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC_Value;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC->CDR;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = HW_ADC_CHANNELS;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream4, &DMA_InitStructure);

	// DMA2_Stream0 enable
	DMA_Cmd(DMA2_Stream4, ENABLE);

	// Enable transfer complete interrupt
	DMA_ITConfig(DMA2_Stream4, DMA_IT_TC, ENABLE);

	// ADC Common Init
	// Note that the ADC is running at 42MHz, which is higher than the
	// specified 36MHz in the data sheet, but it works.
	ADC_CommonInitStructure.ADC_Mode = ADC_TripleMode_RegSimult;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	// Channel-specific settings
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Falling;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T8_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = HW_ADC_NBR_CONV;

	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_Init(ADC2, &ADC_InitStructure);
	ADC_Init(ADC3, &ADC_InitStructure);

	hw_setup_adc_channels();

	// Enable DMA request after last transfer (Multi-ADC mode)
	ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);

	// Enable ADC
	ADC_Cmd(ADC1, ENABLE);
	ADC_Cmd(ADC2, ENABLE);
	ADC_Cmd(ADC3, ENABLE);

	// ------------- Timer8 for ADC sampling ------------- //
	// Time Base configuration
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = SYSTEM_CORE_CLOCK  / (int)PWMFREQ;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = TIM1->ARR;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM8, ENABLE);
	TIM_CCPreloadControl(TIM8, ENABLE);

	// PWM outputs have to be enabled in order to trigger ADC on CCx
	TIM_CtrlPWMOutputs(TIM8, ENABLE);

	// TIM1 Master and TIM8 slave
	TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);
	TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);
	TIM_SelectInputTrigger(TIM8, TIM_TS_ITR0);
	TIM_SelectSlaveMode(TIM8, TIM_SlaveMode_Reset);

	// Enable TIM8
	TIM_Cmd(TIM8, ENABLE);

	// Enable TIM1
	TIM_Cmd(TIM1, ENABLE);

	// Main Output Enable
	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	// 32-bit timer for RPM measurement
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	uint16_t PrescalerValue = (uint16_t) ((SYSTEM_CORE_CLOCK / 2) / MCPWM_RPM_TIMER_FREQ) - 1;

	// Time base configuration
	TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	// TIM2 enable counter
	TIM_Cmd(TIM2, ENABLE);

	// ADC sampling locations
	//stop_pwm_hw();

	// Disable preload register updates
	TIM1->CR1 |= TIM_CR1_UDIS;
	TIM8->CR1 |= TIM_CR1_UDIS;

	TIM8->CCR1 = TIM1->ARR;//for vdc
	TIM8->CCR2 = TIM1->ARR;//for Ib
	TIM8->CCR3 = TIM1->ARR;//for Ia

	// Enables preload register updates
	TIM1->CR1 &= ~TIM_CR1_UDIS;
	TIM8->CR1 &= ~TIM_CR1_UDIS;

	utils_sys_unlock_cnt();

	// Various time measurements
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
	PrescalerValue = (uint16_t) ((SYSTEM_CORE_CLOCK / 2) / 10000000) - 1;

	// Time base configuration
	TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM12, &TIM_TimeBaseStructure);

	// TIM3 enable counter
	TIM_Cmd(TIM12, ENABLE);

	// WWDG configuration
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);
	WWDG_SetPrescaler(WWDG_Prescaler_1);
	WWDG_SetWindowValue(255);
	WWDG_Enable(100);

	//--------------------------------------------------------------------------
	SetupControlParameters();

	// Calibrate current offset
	ENABLE_GATE();
	DCCAL_OFF();

	McCtrlBits.OpenLoop = 0;
	McCtrlBits.DcCalDone = do_dc_cal();
	

	chThdCreateStatic(timer_thread_wa, sizeof(timer_thread_wa), NORMALPRIO, timer_thread, NULL);

}

void mcpwm_deinit(void) 
{
	WWDG_DeInit();

	//timer_thd_stop = true;
	//rpm_thd_stop = true;

	//while (timer_thd_stop || rpm_thd_stop)
	//{
	//	chThdSleepMilliseconds(1);
	//}

	TIM_DeInit(TIM1);
	TIM_DeInit(TIM2);
	TIM_DeInit(TIM8);
	TIM_DeInit(TIM12);
	ADC_DeInit();
	DMA_DeInit(DMA2_Stream4);
	nvicDisableVector(ADC_IRQn);
	dmaStreamRelease(STM32_DMA_STREAM(STM32_DMA_STREAM_ID(2, 4)));
}



//======================================================================================
//private function 

void stop_pwm_hw(void)
{
	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive);
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);

	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
}


void update_timer_Duty(unsigned int duty_A,unsigned int duty_B,unsigned int duty_C)
{
	utils_sys_lock_cnt();

	// Disable preload register updates
	TIM1->CR1 |= TIM_CR1_UDIS;
	TIM8->CR1 |= TIM_CR1_UDIS;

	TIM1->CCR1 = duty_A;
	TIM1->CCR2 = duty_B;
	TIM1->CCR3 = duty_C;

	// Enables preload register updates
	TIM1->CR1 &= ~TIM_CR1_UDIS;
	TIM8->CR1 &= ~TIM_CR1_UDIS;

	utils_sys_unlock_cnt();
}


/*
 * New ADC samples ready. Do commutation!
 */
uint32_t MCCtrlCnt = 0;	
void mcpwm_adc_dma_int_handler(void *p, uint32_t flags)
{
	(void)p;
	(void)flags;

	MCCtrlCnt++;

	if(MCCtrlCnt % HALL_SENSOR_DIV == 2) //speed ctrl
	{
		SMC_HallSensor_Estimation (&smc1);

	}
	
	if(MCCtrlCnt % SPD_CTRL_DIV == 1) //speed ctrl
	{
		//LED_GREEN_ON();
		// Execute the velocity control loop
		PIParmW.qInMeas = smc1.Omega;
		PIParmW.qInRef	= CtrlParm.qVelRef;
		CalcPI(&PIParmW);
		CtrlParm.qVqRef = PIParmW.qOut;
		
		//LED_GREEN_OFF();
	}

	if(MCCtrlCnt % CURR_CTRL_DIV == 0)//current ctrl
	{
		//LED_RED_ON();

		TIM12->CNT = 0;

		if(!McCtrlBits.DcCalDone)
		{
			MeasCurrParm.curr_start_samples++;
			MeasCurrParm.curr0_sum += ADC_Value[ADC_IND_CURR1] ;
			MeasCurrParm.curr1_sum += ADC_Value[ADC_IND_CURR2] ;
		}

		// Calculate qIa,qIb
		MeasCurrParm.CorrADC_a = ADC_Value[ADC_IND_CURR1] - MeasCurrParm.Offseta;
		MeasCurrParm.CorrADC_b = ADC_Value[ADC_IND_CURR2] - MeasCurrParm.Offsetb;
		MeasCurrParm.CorrADC_c = -(MeasCurrParm.CorrADC_a + MeasCurrParm.CorrADC_b);

		// Calculate control values
		if(McCtrlBits.OpenLoop)
		{

			//ParkParm.qAngle = 0.0f;

			ParkParm.qAngle -= 0.01f;
			if(  ParkParm.qAngle < 0)ParkParm.qAngle=2*PI;
			
			//ParkParm.qAngle += 0.002f;
			//if(2*PI <  ParkParm.qAngle)ParkParm.qAngle=2*PI - ParkParm.qAngle;

			ParkParm.qVd = 0.5f;
			ParkParm.qVq = 0.0f;

		}
		else
		{
			ParkParm.qAngle = smc1.Theta;
		
			ParkParm.qIa = MeasCurrParm.qKa * (float)MeasCurrParm.CorrADC_a;
			ParkParm.qIb = MeasCurrParm.qKb * (float)MeasCurrParm.CorrADC_b;

			// Calculate qId,qIq from qSin,qCos,qIa,qIb
			ParkParm.qIalpha = ParkParm.qIa;
			ParkParm.qIbeta = ParkParm.qIa*INV_SQRT3 + 2*ParkParm.qIb*INV_SQRT3;
			// Ialpha and Ibeta have been calculated. Now do rotation.
			// Get qSin, qCos from ParkParm structure
			
			ParkParm.qId = -ParkParm.qIalpha*cosf(ParkParm.qAngle) + ParkParm.qIbeta*sinf(ParkParm.qAngle);
			ParkParm.qIq = ParkParm.qIalpha*sinf(ParkParm.qAngle) + ParkParm.qIbeta*cosf(ParkParm.qAngle);
		
			DoControl();
		}

		// Calculate qValpha, qVbeta from qSin,qCos,qVd,qVq
		ParkParm.qValpha =  ParkParm.qVd*cosf(ParkParm.qAngle) - ParkParm.qVq*sinf(ParkParm.qAngle);
		ParkParm.qVbeta  =  ParkParm.qVd*sinf(ParkParm.qAngle) + ParkParm.qVq*cosf(ParkParm.qAngle);

		// Calculate Vr1,Vr2,Vr3 from qValpha, qVbeta
		SVGenParm.qVr1 = ParkParm.qVbeta;
		SVGenParm.qVr2 = (-ParkParm.qVbeta + SQRT3 * ParkParm.qValpha)/2;
		SVGenParm.qVr3 = (-ParkParm.qVbeta - SQRT3 * ParkParm.qValpha)/2;

		CalcSVGen();

		// Reset the watchdog

		//LED_RED_OFF();
	}

	WWDG_SetCounter(100);

}


static THD_FUNCTION(timer_thread, arg) {
	(void)arg;

	chRegSetThreadName("mc_timer");

	//chvprintf(&SDU1, (uint8_t *)"to mc_interface -> mcif timer\r\n");


	for(;;) {

		LED_RED_ON();
		chThdSleepMilliseconds(250);

		//Target DC Bus, without sign.
		MeasSensorValue.InputVoltage = GET_INPUT_VOLTAGE();
		MeasSensorValue.MotorTemp = NTC_TEMP(ADC_IND_TEMP_PCB);
		LED_RED_OFF();

		chThdSleepMilliseconds(750);
		
		
	}
}

