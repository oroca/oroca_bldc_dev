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
 * mcpwm.c
 *
 *  Created on: 13 okt 2012
 *      Author: benjamin
 */

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "hw.h"

//#include "main.h"
#include "mcpwm.h"
#include "utils.h"
#include "uart3_print.h"

#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>




union{
        struct {
		unsigned OpenLoop:1;	// Indicates if motor is running in open or closed loop
		unsigned RunMotor:1;	// If motor is running, or stopped.
		unsigned EnTorqueMod:1;	// This bit enables Torque mode when running closed loop
		unsigned EnVoltRipCo:1;	// Bit that enables Voltage Ripple Compensation
		unsigned ChangeMode:1;	// This flag indicates that a transition from open to closed
								// loop, or closed to open loop has happened. This
								// causes DoControl subroutine to initialize some variables
								// before executing open or closed loop for the first time
		unsigned ChangeSpeed:1;	// This flag indicates a step command in speed reference.
								// This is mainly used to analyze step response
		unsigned    :10;
            }bit;
        	WORD Word;
 } uGF;


SMC smc1;
tParkParm ParkParm;

tPIParm     PIParmD;	// Structure definition for Flux component of current, or Id
tPIParm     PIParmQ;	// Structure definition for Torque component of current, or Iq
tPIParm     PIParmW;	// Structure definition for Speed, or Omega
tPIParm     PIParmPLL;

tCtrlParm CtrlParm;
tSVGenParm SVGenParm;
tFdWeakParm FdWeakParm;
static volatile tMeasCurrParm MeasCurrParm;

static volatile bool dccal_done;

int VelReq = 0;

// Global variables
uint16_t ADC_Value[HW_ADC_CHANNELS];

unsigned int  switching_frequency_now = PWMFREQUENCY;

// Speed Calculation Variables

float AccumTheta = 0;	// Accumulates delta theta over a number of times
WORD AccumThetaCnt = 0;	// Counter used to calculate motor speed. Is incremented
						// in SMC_Position_Estimation() subroutine, and accumulates
						// delta Theta. After N number of accumulations, Omega is
						// calculated. This N is diIrpPerCalc which is defined in
						// UserParms.h.

// Vd and Vq vector limitation variables

static volatile float qVdSquared = 0;	// This variable is used to know what is left from the VqVd vector
						// in order to have maximum output PWM without saturation. This is
						// done before executing Iq control loop at the end of DoControl()

static volatile float DCbus = 0;		// DC Bus measured continuously and stored in this variable
						// while motor is running. Will be compared with TargetDCbus
						// and Vd and Vq will be compensated depending on difference
						// between DCbus and TargetDCbus

static volatile float TargetDCbus = 0;// DC Bus is measured before running motor and stored in this
						// variable. Any variation on DC bus will be compared to this value
						// and compensated linearly.


void mcpwm_init(void)
{

	Uart3_printf(&SD3, (uint8_t *)"mcpwm_init....\r\n");
	utils_sys_lock_cnt();

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
	//NVIC_InitTypeDef NVIC_InitStructure;

	// Initialize variables
	dccal_done = false;

	TIM_DeInit(TIM1);
	TIM_DeInit(TIM8);
	TIM1->CNT = 0;
	TIM8->CNT = 0;

	// TIM1 clock enable
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	// Time Base configuration
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
	TIM_TimeBaseStructure.TIM_Period = SYSTEM_CORE_CLOCK  / (int)switching_frequency_now /2;
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

	dmaStreamAllocate(STM32_DMA_STREAM(STM32_DMA_STREAM_ID(2, 4)),3,(stm32_dmaisr_t)mcpwm_adc_int_handler,(void *)0);

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
	TIM_TimeBaseStructure.TIM_Period = SYSTEM_CORE_CLOCK  / (int)switching_frequency_now;
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
	utils_sys_lock_cnt();

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

	// Calibrate current offset
	ENABLE_GATE();
	DCCAL_OFF();
	GAIN_FULLDN();
	do_dc_cal();
	//Uart3_printf(&SD3, (uint8_t *)"5-1\r\n");
	// Enable transfer complete interrupt


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


//--------------------------------------------------
//main ctrl setup

	SetupParm();
	SetupControlParameters();


	//dmaStreamAllocate(STM32_DMA_STREAM(STM32_DMA_STREAM_ID(2, 4)),3,(stm32_dmaisr_t)mcpwm_adc_int_handler,(void *)0);
	//DMA_ITConfig(DMA2_Stream4, DMA_IT_TC, ENABLE);


	uGF.Word = 0;                   // clear flags
	#ifdef TORQUEMODE
    	uGF.bit.EnTorqueMod = 1;
	#endif

	#ifdef ENVOLTRIPPLE
    	uGF.bit.EnVoltRipCo = 1;
	#endif

	uGF.bit.RunMotor = 1;

	
//
}


static volatile int curr0_sum;
static volatile int curr1_sum;
static volatile int curr_start_samples;
static volatile int curr0_offset;
static volatile int curr1_offset;
void do_dc_cal(void)
{
	DCCAL_ON();
	while(IS_DRV_FAULT()){};
	chThdSleepMilliseconds(1000);
	curr0_sum = 0;
	curr1_sum = 0;
	curr_start_samples = 0;
	while(curr_start_samples < 4000) {};
	curr0_offset = curr0_sum / curr_start_samples;
	curr1_offset = curr1_sum / curr_start_samples;
	DCCAL_OFF();
	dccal_done = true;

	Uart3_printf(&SD3, (uint8_t *)"curr0_offset : %u\r\n",curr0_offset);
	Uart3_printf(&SD3, (uint8_t *)"curr1_offset : %u\r\n",curr1_offset);
}


void mcpwm_adc_int_prehandler(void *p, uint32_t flags) 
{
	(void)p;
	(void)flags;
	LED_RED_ON();

	curr_start_samples++;
	curr0_sum += ADC_Value[ADC_IND_CURR1] ;
	curr1_sum += ADC_Value[ADC_IND_CURR2] ;

	LED_RED_OFF();

	// Reset the watchdog
	WWDG_SetCounter(100);	
}


/*
 * New ADC samples ready. Do commutation!
 */
void mcpwm_adc_int_handler(void *p, uint32_t flags) 
{
	(void)p;
	(void)flags;

	TIM12->CNT = 0;

	curr_start_samples++;
	curr0_sum += ADC_Value[ADC_IND_CURR1] ;
	curr1_sum += ADC_Value[ADC_IND_CURR2] ;

	SMC_HallSensor_Estimation (&smc1);

	// Check for faults that should stop the motor
	uGF.bit.RunMotor = 1;
	if( uGF.bit.RunMotor )
		{
			ENABLE_GATE();
			LED_RED_ON();
	
			// Calculate qIa,qIb
			 int CorrADC1, CorrADC2;

			 CorrADC1 = ADC_Value[ADC_IND_CURR1] - MeasCurrParm.Offseta;
			 CorrADC2 = ADC_Value[ADC_IND_CURR2] - MeasCurrParm.Offsetb;
			// ADC_curr_norm_value[2] = -(ADC_curr_norm_value[0] + ADC_curr_norm_value[1]);

			 ParkParm.qIa = MeasCurrParm.qKa * (float)CorrADC1;
			 ParkParm.qIb = MeasCurrParm.qKb * (float)CorrADC2;	
	
			//Uart3_printf(&SD3,  "%f,%d,%d\r\n",ParkParm.qAngle ,ParkParm.qIa,CorrADC2);

			// Calculate commutation angle using estimator
			ParkParm.qAngle = smc1.Theta;
	
			//ParkParm.qAngle = (float)IN[2];
			//smc1.Omega = (float)IN[3] *LOOPTIMEINSEC * IRP_PERCALC * POLEPAIRS/PI;
	
			AccumThetaCnt++;
			if (AccumThetaCnt == IRP_PERCALC)
			{
				AccumThetaCnt = 0;
			}
	
	
			// Calculate qId,qIq from qSin,qCos,qIa,qIb
			ParkParm.qIalpha = ParkParm.qIa;
			ParkParm.qIbeta = ParkParm.qIa*INV_SQRT3 + 2*ParkParm.qIb*INV_SQRT3;
			// Ialpha and Ibeta have been calculated. Now do rotation.
			// Get qSin, qCos from ParkParm structure

			ParkParm.qId = -ParkParm.qIalpha*cosf(ParkParm.qAngle) + ParkParm.qIbeta*sinf(ParkParm.qAngle);
			ParkParm.qIq = ParkParm.qIalpha*sinf(ParkParm.qAngle) + ParkParm.qIbeta*cosf(ParkParm.qAngle);
	
			// Calculate control values
			DoControl();
	
	//=============================================================================
	//for open loop test
			//ParkParm.qVd =0.5f;
			//ParkParm.qVq = 0.0f;
	
			//ParkParm.qAngle = 0.0f;
	
			//ParkParm.qAngle -= 0.002f;
			//if(  ParkParm.qAngle < 0)ParkParm.qAngle=2*PI;
	
			//ParkParm.qAngle += 0.002f;
			//if(2*PI <  ParkParm.qAngle)ParkParm.qAngle=2*PI - ParkParm.qAngle;
	//==============================================================================
	
			// Calculate qValpha, qVbeta from qSin,qCos,qVd,qVq
			ParkParm.qValpha =  ParkParm.qVd*cosf(ParkParm.qAngle) - ParkParm.qVq*sinf(ParkParm.qAngle);
			ParkParm.qVbeta  =  ParkParm.qVd*sinf(ParkParm.qAngle) + ParkParm.qVq*cosf(ParkParm.qAngle);
	
			// Calculate Vr1,Vr2,Vr3 from qValpha, qVbeta
			SVGenParm.qVr1 = ParkParm.qVbeta;
			SVGenParm.qVr2 = (-ParkParm.qVbeta + SQRT3 * ParkParm.qValpha)/2;
			SVGenParm.qVr3 = (-ParkParm.qVbeta	- SQRT3 * ParkParm.qValpha)/2;


			CalcSVGen();
	
			LED_RED_OFF();
			//DISABLE_GATE();
	
				
		}
		else
		{
			DISABLE_GATE();
		}

	// Reset the watchdog
	WWDG_SetCounter(100);

}


bool SetupParm(void)
{
	// ============= ADC - Measure Current & Pot ======================

	MeasCurrParm.qKa    = DQKA;
	MeasCurrParm.qKb    = DQKB;

	// Initial Current offsets
	InitMeasCompCurr( curr0_offset, curr1_offset ); 

	// Target DC Bus, without sign.
	TargetDCbus = GET_INPUT_VOLTAGE();

	// ============= SVGen ===============
	// Set PWM period to Loop Time
	SVGenParm.iPWMPeriod = LOOPINTCY;



	return False;
}


//---------------------------------------------------------------------
// Executes one PI itteration for each of the three loops Id,Iq,Speed,

void DoControl( void )
{
	if(AccumThetaCnt == 0)
	{
		// Execute the velocity control loop
		PIParmW.qInMeas = smc1.Omega;
		PIParmW.qInRef	= CtrlParm.qVelRef;
		CalcPI(&PIParmW);
		CtrlParm.qVqRef = PIParmW.qOut;
	}

	if (uGF.bit.EnTorqueMod)
		CtrlParm.qVqRef = CtrlParm.qVelRef;

	//CtrlParm.qVdRef = FieldWeakening(fabsf(CtrlParm.qVelRef));

	// PI control for D
	PIParmD.qInMeas = ParkParm.qId;
	//PIParmD.qInRef	= CtrlParm.qVdRef;
	PIParmD.qInRef	= 0.0f;
	CalcPI(&PIParmD);

	if(uGF.bit.EnVoltRipCo)
		ParkParm.qVd = VoltRippleComp(PIParmD.qOut);
	else
		ParkParm.qVd = PIParmD.qOut;

	qVdSquared = ParkParm.qVd * ParkParm.qVd;
	PIParmQ.qOutMax = sqrtf((0.95*0.95) - qVdSquared);
	PIParmQ.qOutMin = -PIParmQ.qOutMax;

	// PI control for Q
	PIParmQ.qInMeas = ParkParm.qIq;
	PIParmQ.qInRef	= CtrlParm.qVqRef;
	CalcPI(&PIParmQ);

	// If voltage ripple compensation flag is set, adjust the output
	// of the Q controller depending on measured DC Bus voltage
	if(uGF.bit.EnVoltRipCo)
		ParkParm.qVq = VoltRippleComp(PIParmQ.qOut);
	else
		ParkParm.qVq = PIParmQ.qOut;

}

void InitPI( tPIParm *pParm)
{
	pParm->qdSum=0;
	pParm->qOut=0;

}
void CalcPI( tPIParm *pParm)
{
	float U,Exc,Err;
	Err  = pParm->qInRef - pParm->qInMeas;
	
	U  = pParm->qdSum + pParm->qKp * Err;

	if( U > pParm->qOutMax )          pParm->qOut = pParm->qOutMax;
	else if( U < pParm->qOutMin )    pParm->qOut = pParm->qOutMin;
	else                  pParm->qOut = U ;

	Exc = U - pParm->qOut;

	pParm->qdSum = pParm->qdSum + pParm->qKi * Err - pParm->qKc * Exc ;
	
	return;
}
void SetupControlParameters(void)
{

	// ============= PI D Term ===============
	PIParmD.qKp = DKP;
	PIParmD.qKi = DKI;
	PIParmD.qKc = DKC;
	PIParmD.qOutMax = DOUTMAX;
	PIParmD.qOutMin = -PIParmD.qOutMax;

	InitPI(&PIParmD);

	// ============= PI Q Term ===============
	PIParmQ.qKp = QKP;
	PIParmQ.qKi = QKI;
	PIParmQ.qKc = QKC;
	PIParmQ.qOutMax = QOUTMAX;
	PIParmQ.qOutMin = -PIParmQ.qOutMax;

	InitPI(&PIParmQ);

	// ============= PI W Term ===============
	PIParmW.qKp = WKP;
	PIParmW.qKi = WKI;
	PIParmW.qKc = WKC;
	PIParmW.qOutMax = WOUTMAX;
	PIParmW.qOutMin = -PIParmW.qOutMax;

	InitPI(&PIParmW);

	// ============= PI PLL Term ===============
	PIParmPLL.qKp = PLLKP;		 
	PIParmPLL.qKi = PLLKI;		 
	PIParmPLL.qKc = PLLKC;		 
	PIParmPLL.qOutMax = PLLOUTMAX;	 
	PIParmPLL.qOutMin = -PIParmPLL.qOutMax;

	InitPI(&PIParmPLL);

	return;
}

// NOTE:
//
// If Input power supply has switching frequency noise, for example if a
// switch mode power supply is used, Voltage Ripple Compensation is not
// recommended, since it will generate spikes on Vd and Vq, which can
// potentially make the controllers unstable.

float VoltRippleComp(float Vdq)
{
	float CompVdq;
	// DCbus is already updated with new DC Bus measurement
	// in ReadSignedADC0 subroutine.
	//
	// If target DC Bus is greater than what we measured last sample, adjust
	// output as follows:
	//
	//                  TargetDCbus - DCbus
	// CompVdq = Vdq + --------------------- * Vdq
	//                         DCbus
	//
	// If Measured DCbus is greater than target, then the following compensation
	// is implemented:
	//
	//            TargetDCbus 
	// CompVdq = ------------- * Vdq
	//               DCbus
	//
	// If target and measured are equal, no operation is made.
	//
	if (TargetDCbus > DCbus)
		CompVdq = Vdq + (((TargetDCbus - DCbus)/ DCbus)* Vdq);
	else if (DCbus > TargetDCbus)
		CompVdq = ((TargetDCbus/ DCbus)* Vdq);
	else
		CompVdq = Vdq;

	return CompVdq;
}

void InitMeasCompCurr( short Offset_a, short Offset_b )
{
	MeasCurrParm.Offseta = Offset_a;
	MeasCurrParm.Offsetb = Offset_b;
	return;
}


void CalcTimes(void)
{
	SVGenParm.iPWMPeriod = LOOPINTCY;	  

	SVGenParm.T1 = ((float)SVGenParm.iPWMPeriod * SVGenParm.T1);
	SVGenParm.T2 = ((float)SVGenParm.iPWMPeriod * SVGenParm.T2);
	SVGenParm.Tc = (((float)SVGenParm.iPWMPeriod-SVGenParm.T1-SVGenParm.T2)/2);
	SVGenParm.Tb = SVGenParm.Tc + SVGenParm.T1;
	SVGenParm.Ta = SVGenParm.Tb + SVGenParm.T2 ;

	return;
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

	//debug_print_usb( "%u,%u,%u\r\n",duty_A ,duty_B,duty_C);
	//TIM1->CCR4 = 100;
	//TIM8->CCR1 = duty_A;
	//TIM8->CCR2 = duty_A;
	//TIM8->CCR3 = duty_C;


	// Enables preload register updates
	TIM1->CR1 &= ~TIM_CR1_UDIS;
	TIM8->CR1 &= ~TIM_CR1_UDIS;

	utils_sys_unlock_cnt();
}

void CalcSVGen( void )
{ 
	if( SVGenParm.qVr1 >= 0 )
	{       
		// (xx1)
		if( SVGenParm.qVr2 >= 0 )
		{
			// (x11)
			// Must be Sector 3 since Sector 7 not allowed
			// Sector 3: (0,1,1)  0-60 degrees
			SVGenParm.T2 = SVGenParm.qVr2;
			SVGenParm.T1 = SVGenParm.qVr1;
			CalcTimes();
			update_timer_Duty(SVGenParm.Ta,SVGenParm.Tb,SVGenParm.Tc) ;
		}
		else
		{            
			// (x01)
			if( SVGenParm.qVr3 >= 0 )
			{
				// Sector 5: (1,0,1)  120-180 degrees
				SVGenParm.T2 = SVGenParm.qVr1;
				SVGenParm.T1 = SVGenParm.qVr3;
				CalcTimes();
				update_timer_Duty(SVGenParm.Tc,SVGenParm.Ta,SVGenParm.Tb) ;

			}
			else
			{
				// Sector 1: (0,0,1)  60-120 degrees
				SVGenParm.T2 = -SVGenParm.qVr2;
				SVGenParm.T1 = -SVGenParm.qVr3;
				CalcTimes();
				update_timer_Duty(SVGenParm.Tb,SVGenParm.Ta,SVGenParm.Tc) ;
			}
		}
	}
	else
	{
		// (xx0)
		if( SVGenParm.qVr2 >= 0 )
		{
			// (x10)
			if( SVGenParm.qVr3 >= 0 )
			{
				// Sector 6: (1,1,0)  240-300 degrees
				SVGenParm.T2 = SVGenParm.qVr3;
				SVGenParm.T1 = SVGenParm.qVr2;
				CalcTimes();
				update_timer_Duty(SVGenParm.Tb,SVGenParm.Tc,SVGenParm.Ta) ;
			}
			else
			{
				// Sector 2: (0,1,0)  300-0 degrees
				SVGenParm.T2 = -SVGenParm.qVr3;
				SVGenParm.T1 = -SVGenParm.qVr1;
				CalcTimes();
				update_timer_Duty(SVGenParm.Ta,SVGenParm.Tc,SVGenParm.Tb) ;
			}
		}
		else
		{            
			// (x00)
			// Must be Sector 4 since Sector 0 not allowed
			// Sector 4: (1,0,0)  180-240 degrees
			SVGenParm.T2 = -SVGenParm.qVr1;
			SVGenParm.T1 = -SVGenParm.qVr2;
			CalcTimes();
			update_timer_Duty(SVGenParm.Tc,SVGenParm.Tb,SVGenParm.Ta) ;
		}
	}

}


/********************************PLL loop **********************************/	

#if 0
void SMC_HallSensor_Estimation (SMC *s)
{

	HallPLLA = ((float)ADC_Value[ADC_IND_SENS1] - 1241.0f)/ 4095.0f;
	HallPLLB = ((float)ADC_Value[ADC_IND_SENS2] - 1241.0f)/ 4095.0f;

	cos3th = cosf(3.0f * Theta);
	sin3th = sinf(3.0f * Theta);

	HallPLLA_sin3th = HallPLLA * sin3th * Gamma;
	HallPLLA_cos3th = HallPLLA * cos3th * Gamma;
	
	HallPLLB_cos3th = HallPLLB* cos3th * Gamma;
	HallPLLB_sin3th = HallPLLB * sin3th * Gamma;

	HallPLLA_cos3th_Integral += HallPLLA_cos3th;
	HallPLLA_sin3th_Integral += HallPLLA_sin3th;
	
	HallPLLB_sin3th_Integral += HallPLLB_sin3th;
	HallPLLB_cos3th_Integral += HallPLLB_cos3th;

	Asin3th= HallPLLA_sin3th_Integral * sin3th;
	Acos3th= HallPLLA_cos3th_Integral * cos3th;

	Bsin3th= HallPLLB_sin3th_Integral * sin3th;
	Bcos3th= HallPLLB_cos3th_Integral * cos3th;

	ANF_PLLA = HallPLLA - Asin3th - Acos3th;
	ANF_PLLB = HallPLLB - Bsin3th - Bcos3th;
	
	costh = cosf(Theta);
	sinth = sinf(Theta);
	
	Hall_SinCos = ANF_PLLA * costh;
	Hall_CosSin = ANF_PLLB * sinth;

	float err, tmp_kp, tmp_kpi; 									
	tmp_kp = 1.0f;
	tmp_kpi = (1.0f + 1.0f * Tsamp);
	err = Hall_SinCos - Hall_CosSin; 											
	Hall_PIout += ((tmp_kpi * err) - (tmp_kp * Hall_Err0)); 					
	Hall_PIout = Bound_limit(Hall_PIout, 10.0f);						
	Hall_Err0= err;									
	
	Theta += Hall_PIout ;
	if((2.0f * PI) < Theta) Theta = Theta - (2.0f * PI);
	else if(Theta < 0.0f) Theta = (2.0f * PI) + Theta;

	s->Theta= Theta + 0.3f;

	if((2.0f * PI) < s->Theta) s->Theta = s->Theta - (2.0f * PI);
	else if(s->Theta < 0.0f) s->Theta = (2.0f * PI) + s->Theta;

	s->Omega = Hall_PIout;
	//Futi   = Hall_PIout / (2.* PI) *Fsamp;

	//spi_dac_write_A((HallPLLA+ 1.0f) * 200.0f);
	//spi_dac_write_B((HallPLLB+ 1.0f) * 200.0f);

	//spi_dac_write_A((costh + 1.0f) * 2000.0f);
	//spi_dac_write_B((sinth + 1.0f) * 2047.0f);

	//spi_dac_write_A( (Hall_SinCos+ 1.0f) * 2048.0f);
	//spi_dac_write_B( (Hall_CosSin+ 1.0f) * 2048.0f);

	//spi_dac_write_A( (Hall_err+ 1.0f) * 2048.0f);
	//spi_dac_write_B( (Theta * 200.0f) );

	//spi_dac_write_B( Hall_PIout * 100.0f);


	//spi_dac_write_A( (ParkParm.qAngle * 200.0f) );
	//spi_dac_write_B( (smc1.Theta * 200.0f) );


	//s->Omega = Wpll;
	//s->Theta =Theta;

	//DAC_SetChannel1Data(uint32_t DAC_Align, uint16_t Data)
}

#else
void SMC_HallSensor_Estimation (SMC *s)
{


	s->HallPLLA = ((float)ADC_Value[ADC_IND_SENS1] - 1241.0f)/ 4095.0f;
	s->HallPLLB = ((float)ADC_Value[ADC_IND_SENS2] - 1241.0f)/ 4095.0f;

	s->costh = cosf(s->Theta);
	s->sinth = sinf(s->Theta);
	
	s->Hall_SinCos = s->HallPLLA * s->costh;
	s->Hall_CosSin = s->HallPLLB * s->sinth;

	float err, tmp_kp, tmp_kpi; 									
	tmp_kp = 1.0f;
	tmp_kpi = (1.0f + 1.0f * PWMPEROID);
	err = s->Hall_SinCos - s->Hall_CosSin; 											
	s->Hall_PIout += ((tmp_kpi * err) - (tmp_kp * s->Hall_Err0)); 					
	s->Hall_PIout = Bound_limit(s->Hall_PIout, 10.0f);						
	s->Hall_Err0= err;									
	
	s->Theta += s->Hall_PIout ;
	if((2.0f * PI) < s->Theta) s->Theta = s->Theta - (2.0f * PI);
	else if(s->Theta < 0.0f) s->Theta = (2.0f * PI) + s->Theta;

	s->ThetaCal= s->Theta + 0.3f;

	if((2.0f * PI) < s->ThetaCal) s->ThetaCal = s->ThetaCal - (2.0f * PI);
	else if(s->ThetaCal < 0.0f) s->ThetaCal = (2.0f * PI) + s->ThetaCal;

	s->Omega = s->Hall_PIout;


	s->trueTheta += (s->Hall_PIout /7.0f) ;
	if((2.0f * PI) < s->trueTheta) s->trueTheta = s->trueTheta - (2.0f * PI);
	else if(s->trueTheta < 0.0f) s->trueTheta = (2.0f * PI) + s->trueTheta;

	s->Futi   = s->Hall_PIout / (2.* PI) * PWMFREQUENCY;
	s->rpm = 120.0f * s->Futi / 7.0f;
	

	//spi_dac_write_A((HallPLLA+ 1.0f) * 200.0f);
	//spi_dac_write_B((HallPLLB+ 1.0f) * 200.0f);

	//spi_dac_write_A((costh + 1.0f) * 2000.0f);
	//spi_dac_write_B((sinth + 1.0f) * 2047.0f);

	//spi_dac_write_A( (Hall_SinCos+ 1.0f) * 2048.0f);
	//spi_dac_write_B( (Hall_CosSin+ 1.0f) * 2048.0f);

	//spi_dac_write_A( (Hall_err+ 1.0f) * 2048.0f);
	//spi_dac_write_B( (Theta * 200.0f) );

	//spi_dac_write_B( Hall_PIout * 100.0f);


	//spi_dac_write_A( (ParkParm.qAngle * 200.0f) );
	//spi_dac_write_B( (smc1.Theta * 200.0f) );


	//s->Omega = Wpll;
	//s->Theta =Theta;

	//DAC_SetChannel1Data(uint32_t DAC_Align, uint16_t Data)
}

#endif

void stop_pwm_hw(void) {
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
