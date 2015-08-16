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
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "mcpwm.h"
#include "digital_filter.h"
#include "utils.h"
#include "ledpwm.h"
#include "hw.h"
#include "terminal.h"
#include "encoder.h"

#include <errno.h>
#include <unistd.h>

//jsyoon
#include <stdarg.h>
extern int debug_print_usb( const char *fmt, ...);


// Structs
union{
        struct {
		unsigned OpenLoop:1;	// Indicates if motor is running in open or closed loop
		unsigned RunMotor:1;	// If motor is running, or stopped.
		unsigned EnTorqueMod:1;	// This bit enables Torque mode when running closed loop
		unsigned EnVoltRipCo:1;	// Bit that enables Voltage Ripple Compensation
		unsigned Btn1Pressed:1;	// Button 1 has been pressed.
		unsigned ChangeMode:1;	// This flag indicates that a transition from open to closed
								// loop, or closed to open loop has happened. This
								// causes DoControl subroutine to initialize some variables
								// before executing open or closed loop for the first time
		unsigned ChangeSpeed:1;	// This flag indicates a step command in speed reference.
								// This is mainly used to analyze step response
		unsigned    :9;
            }bit;
        	WORD Word;
 } uGF;

struct {

	unsigned int LockTime;

	unsigned long EndSpeed;

} MotorParm;

typedef struct {
	float	qKa;	
	short	Offseta;

	float	qKb;   
	short	Offsetb;
} tMeasCurrParm;

typedef struct {
		float  Valpha;   		// Input: Stationary alfa-axis stator voltage
		float  Ealpha;   		// Variable: Stationary alfa-axis back EMF
		float  EalphaFinal;	// Variable: Filtered EMF for Angle calculation
		float  Zalpha;      	// Output: Stationary alfa-axis sliding control
		float  Gsmopos;    	// Parameter: Motor dependent control gain
		float  EstIalpha;   	// Variable: Estimated stationary alfa-axis stator current
		float  Fsmopos;    	// Parameter: Motor dependent plant matrix
		float  Vbeta;   		// Input: Stationary beta-axis stator voltage
		float  Ebeta;  		// Variable: Stationary beta-axis back EMF
		float  EbetaFinal;	// Variable: Filtered EMF for Angle calculation
		float  Zbeta;      	// Output: Stationary beta-axis sliding control
		float  EstIbeta;    	// Variable: Estimated stationary beta-axis stator current
		float  Ialpha;  		// Input: Stationary alfa-axis stator current
		float  IalphaError; 	// Variable: Stationary alfa-axis current error
		float  Kslide;     	// Parameter: Sliding control gain
		float  MaxSMCError;  	// Parameter: Maximum current error for linear SMC
		float  Ibeta;  		// Input: Stationary beta-axis stator current
		float  IbetaError;  	// Variable: Stationary beta-axis current error
		float  Kslf;       	// Parameter: Sliding control filter gain
		float  KslfFinal;    	// Parameter: BEMF Filter for angle calculation
		float  FiltOmCoef;   	// Parameter: Filter Coef for Omega filtered calc
		float  ThetaOffset;	// Output: Offset used to compensate rotor angle
		float  Theta;			// Output: Compensated rotor angle
		float  Omega;     	// Output: Rotor speed
		float  OmegaFltred;  	// Output: Filtered Rotor speed for speed PI
} SMC;
typedef SMC *SMC_handle;

typedef struct {
    float    qdSum;          // 1.31 format
    float   qKp;
    float   qKi;
    float   qKc;
    float   qOutMax;
    float   qOutMin;
    float   qInRef; 
    float   qInMeas;
    float   qOut;
    } tPIParm;

typedef struct {
    float   qAngle;
    float   qSin;
    float   qCos;
    float   qIa;
    float   qIb;
    float   qIalpha;
    float   qIbeta;
    float   qId;
    float   qIq;
    float   qVd;
    float   qVq;
    float   qValpha;
    float   qVbeta;
    float   qV1;
    float   qV2;
    float   qV3;
    } tParkParm;

typedef struct {
    float   qVelRef;    // Reference velocity
    float   qVdRef;     // Vd flux reference value
    float   qVqRef;     // Vq torque reference value
    } tCtrlParm;

//------------------  C API for FdWeak routine ---------------------

typedef struct {
	float	qK1;            // < Nominal speed value
	float	qIdRef;
	float	qFwOnSpeed;
	float	qFwActiv;
	int	qIndex;
	float	qFWPercentage;
	float	qInterpolPortion;
	float		qFwCurve[16];	// Curve for magnetizing current
    } tFdWeakParm;

//------------------  C API for SVGen routine ---------------------

typedef struct {
	unsigned int   iPWMPeriod;

	float   qVr1;
	float   qVr2;
	float   qVr3;

	float T1;
	float T2;

	unsigned int Ta;
	unsigned int Tb;
	unsigned int Tc;

    } tSVGenParm;


// Private variables
int count = 0; // delay for ramping the reference velocity 
int VelReq = 0; 

unsigned long Startup_Ramp = 0;	/* Start up ramp in open loop. This variable
								is incremented in CalculateParkAngle()
								subroutine, and it is assigned to 
								ParkParm.qAngle as follows:
								ParkParm.qAngle += (int)(Startup_Ramp >> 16);*/

unsigned int Startup_Lock = 0;	/* This is a counter that is incremented in
								CalculateParkAngle() every time it is called. 
								Once this counter has a value of LOCK_TIME, 
								then theta will start increasing moving the 
								motor in open loop. */



static volatile tMeasCurrParm MeasCurrParm;
SMC smc1 = SMC_DEFAULTS;

// Global variables
uint16_t ADC_Value[HW_ADC_CHANNELS];

tParkParm ParkParm;

  tPIParm     PIParmD;	// Structure definition for Flux component of current, or Id
  tPIParm     PIParmQ;	// Structure definition for Torque component of current, or Iq
  tPIParm     PIParmW;	// Structure definition for Speed, or Omega

  tCtrlParm CtrlParm;
  tSVGenParm SVGenParm;
  tFdWeakParm FdWeakParm;

  int SpeedReference = 0;
  unsigned int  switching_frequency_now = PWMFREQUENCY;

  // Speed Calculation Variables

  WORD iADCisrCnt = 0;	// This Counter is used as a timeout for polling the push buttons
  						// in main() subroutine. It will be reset to zero when it matches
  						// dButPolLoopCnt defined in UserParms.h
  float PrevTheta = 0;	// Previous theta which is then substracted from Theta to get
  						// delta theta. This delta will be accumulated in AccumTheta, and
  						// after a number of accumulations Omega is calculated.
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

  static volatile float Theta_error = 0;// This value is used to transition from open loop to closed looop.
  						// At the end of open loop ramp, there is a difference between
  						// forced angle and estimated angle. This difference is stored in
  						// Theta_error, and added to estimated theta (smc1.Theta) so the
  						// effective angle used for commutating the motor is the same at
  						// the end of open loop, and at the begining of closed loop.
  						// This Theta_error is then substracted from estimated theta
  						// gradually in increments of 0.05 degrees until the error is less
  						// than 0.05 degrees.

// Private functions
void SMC_Position_Estimation (SMC *s);
void SMCInit(SMC *s);
void CalcPI( tPIParm *pParm);
void DoControl( void );
void InitPI( tPIParm *pParm);
void SetupControlParameters(void);
void CalculateParkAngle(void);
void update_timer_Duty(unsigned int duty_A,unsigned int duty_B,unsigned int duty_C);

// Threads
static WORKING_AREA(timer_thread_wa, 2048);
static msg_t timer_thread(void *arg);
//static WORKING_AREA(rpm_thread_wa, 1024);
//static msg_t rpm_thread(void *arg);

void mcpwm_init(mc_configuration *configuration) {
	utils_sys_lock_cnt();

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;


	TIM_DeInit(TIM1);
	//TIM_DeInit(TIM8);
	TIM1->CNT = 0;
	//TIM8->CNT = 0;

	// TIM1 clock enable
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	// Time Base configuration
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
	TIM_TimeBaseStructure.TIM_Period = SYSTEM_CORE_CLOCK / (int)switching_frequency_now;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

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
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = 0;
	ADC_Init(ADC2, &ADC_InitStructure);
	ADC_Init(ADC3, &ADC_InitStructure);

	hw_setup_adc_channels();

	// Enable DMA request after last transfer (Multi-ADC mode)
	ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);

	// Injected channels for current measurement at end of cycle
	ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T8_CC2);
	ADC_ExternalTrigInjectedConvConfig(ADC2, ADC_ExternalTrigInjecConv_T8_CC2);
	ADC_ExternalTrigInjectedConvEdgeConfig(ADC1, ADC_ExternalTrigInjecConvEdge_Falling);
	ADC_ExternalTrigInjectedConvEdgeConfig(ADC2, ADC_ExternalTrigInjecConvEdge_Falling);
	ADC_InjectedSequencerLengthConfig(ADC1, 1);
	ADC_InjectedSequencerLengthConfig(ADC2, 1);

	// Interrupt
	ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Enable ADC1
	ADC_Cmd(ADC1, ENABLE);

	// Enable ADC2
	ADC_Cmd(ADC2, ENABLE);

	// Enable ADC3
	ADC_Cmd(ADC3, ENABLE);

#if 1
	// ------------- Timer8 for ADC sampling ------------- //
	// Time Base configuration
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 500;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);

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
#endif
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

	TIM8->CCR1 = 500;//for vdc
	TIM8->CCR2 = TIM1->ARR /2;//for Ib
	//TIM1->CCR4 = 100;//for Ia
	

//	TIM1->CCR1 = 100;
//	TIM1->CCR2 = 200;
//	TIM1->CCR3 = 300;
	

	// Enables preload register updates
	TIM1->CR1 &= ~TIM_CR1_UDIS;
	TIM8->CR1 &= ~TIM_CR1_UDIS;

	utils_sys_unlock_cnt();


	// Calibrate current offset
	ENABLE_GATE();
	//DISABLE_GATE();
	DCCAL_OFF();
	//do_dc_cal();


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

	// Start threads
	chThdCreateStatic(timer_thread_wa, sizeof(timer_thread_wa), NORMALPRIO, timer_thread, NULL);
	////chThdCreateStatic(rpm_thread_wa, sizeof(rpm_thread_wa), NORMALPRIO, rpm_thread, NULL);

	// WWDG configuration
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);
	WWDG_SetPrescaler(WWDG_Prescaler_1);
	WWDG_SetWindowValue(255);
	WWDG_Enable(100);

	SMCInit(&smc1);
	SetupControlParameters();
	FWInit();

	uGF.Word = 0;                   // clear flags
	#ifdef TORQUEMODE
    	uGF.bit.EnTorqueMod = 1;
	#endif

	#ifdef ENVOLTRIPPLE
    	uGF.bit.EnVoltRipCo = 1;
	#endif


}
void do_dc_cal(void)
{
	DCCAL_ON();
	while(IS_DRV_FAULT()){};
	chThdSleepMilliseconds(1000);
	DCCAL_OFF();

}


int fputc(int ch, FILE *f)
{
    return(ITM_SendChar(ch));
}

void mcpwm_adc_inj_int_handler(void) 
{
	TIM12->CNT = 0;

	// jsyoon TEST
	/*
	static int led=0;
	static int led_cnt=0;

	led_cnt++;

	//if( led_cnt > 200)
	if( led_cnt > 2)
	{
		led_cnt =0;
		_LED_on_off( led);
		led= ~led;
		//debug_print_usb( " Debug Test \r\n");
		debug_print_usb( " Debug Test Debug Test Debug Test Debug Test Debug Test Debug Test Debug Test Debug Test Debug Test Debug Test Debug Test Debug Test Debug Test Debug Test Debug Test Debug Test 12345 \r\n");
	}
	*/
	debug_print_usb( "iT ");
	


	uGF.bit.RunMotor = 1;
	if( uGF.bit.RunMotor )
	{
		palSetPad(GPIOC, 4);
		// Calculate qIa,qIb
		MeasCompCurr();

		// Calculate commutation angle using estimator
		//CalculateParkAngle();

		//printf("Hallo World!\n");
		//ParkParm.qAngle = (float)IN[2];
		//smc1.Omega = (float)IN[3] *LOOPTIMEINSEC * IRP_PERCALC * POLEPAIRS/PI;

		//AccumThetaCnt++;
		//if (AccumThetaCnt == IRP_PERCALC)
		//{
		//	AccumThetaCnt = 0;
		//}


		// Calculate qId,qIq from qSin,qCos,qIa,qIb
		//ClarkePark();

		// Calculate control values
		//DoControl();


		ParkParm.qVd =1.0f;
		ParkParm.qVq = 0.0f;

		//ParkParm.qAngle-= 0.002f;
		//if(  ParkParm.qAngle < 0)ParkParm.qAngle=2*PI;

		ParkParm.qAngle += 0.002f;
		if(2*PI <  ParkParm.qAngle)ParkParm.qAngle=0.0f;


		// Calculate qValpha, qVbeta from qSin,qCos,qVd,qVq
		InvPark();

		// Calculate Vr1,Vr2,Vr3 from qValpha, qVbeta
		CalcRefVec();

		// Calculate and set PWM duty cycles from Vr1,Vr2,Vr3
		CalcSVGen();

		palClearPad(GPIOC, 4);
			
		} 
}

/*
 * New ADC samples ready. Do commutation!
 */
void mcpwm_adc_int_handler(void *p, uint32_t flags) {
	(void)p;
	(void)flags;

	TIM12->CNT = 0;
	palSetPad(GPIOA, 7);	
	palClearPad(GPIOA, 7);


	// Reset the watchdog
	WWDG_SetCounter(100);

}
static msg_t timer_thread(void *arg) 
{
	(void)arg;

	chRegSetThreadName("mcpwm timer");

	int state = 0;

	for(;;)
	{
#if 0
		if(state == 0)
		{
			uGF.bit.ChangeSpeed = 1;
			// init Mode
			uGF.bit.OpenLoop = 1;			// start in openloop
			
			// init user specified parms and stop on error
			if( SetupParm() )
			{
				// Error
				uGF.bit.RunMotor=0;
				return 0;
			}
			
			// zero out i sums 
			PIParmD.qdSum = 0;
			PIParmQ.qdSum = 0;
			PIParmW.qdSum = 0;
		 
			state = 1;
		}
		else if(state == 1)
		{
			///---------------------------------------motor start code
			//if(start cmd)
			{	
				SetupParm();
				uGF.bit.RunMotor = 1;				//then start motor
							// Run the motor
				uGF.bit.ChangeMode = 1; // Ensure variable initialization when open loop is
									// executed for the first time
				state = 2;
			}
			
		}
		else if(state == 2)
		{
			//if(stop cmd)
			{
				uGF.bit.RunMotor = 0;
				state = 0;
			}
		}
#else 
		//jsyoon
		chThdSleepMilliseconds(200);
#endif
	}

	return 0;
}

//==========================================================================================================


bool SetupParm(void)
{

    // Setup required parameters
 
// ============= Open Loop ======================
	// Motor End Speed Calculation
	// MotorParm.EndSpeed = ENDSPEEDOPENLOOP * POLEPAIRS * LOOPTIMEINSEC * 65536 * 65536 / 60.0;
	// Then, * 65536 which is a right shift done in "void CalculateParkAngle(void)"
	// ParkParm.qAngle += (int)(Startup_Ramp >> 16);
	MotorParm.EndSpeed = ENDSPEEDOPENLOOP * POLEPAIRS * LOOPTIMEINSEC * 65536 * 65536 / 60.0;
	MotorParm.LockTime = LOCKTIME;

// ============= ADC - Measure Current & Pot ======================

    MeasCurrParm.qKa    = DQKA;    
    MeasCurrParm.qKb    = DQKB;   

    // Initial Current offsets
	InitMeasCompCurr( ADC_Value[ADC_IND_CURR1], ADC_Value[ADC_IND_CURR1] ); 

	// Target DC Bus, without sign.
	TargetDCbus = GET_INPUT_VOLTAGE();

// ============= SVGen ===============
    // Set PWM period to Loop Time 
    SVGenParm.iPWMPeriod = LOOPINTCY;      


    return False;
}
void CalculateParkAngle(void)
{
	smc1.Ialpha = ParkParm.qIalpha;
	smc1.Ibeta = ParkParm.qIbeta;
	smc1.Valpha = ParkParm.qValpha;
	smc1.Vbeta = ParkParm.qVbeta;

	SMC_Position_Estimation(&smc1);

	if(uGF.bit.OpenLoop)	
	{
		if (Startup_Lock < MotorParm.LockTime)
			Startup_Lock += 1;	// This variable is incremented until
								// lock time expires, them the open loop
								// ramp begins
		else if (Startup_Ramp < MotorParm.EndSpeed)
			// Ramp starts, and increases linearly until EndSpeed is reached.
			// After ramp, estimated theta is used to commutate motor.
			Startup_Ramp += DELTA_STARTUP_RAMP;
		else
		{
			// This section enables closed loop, right after open loop ramp.
			uGF.bit.ChangeMode = 1;
			uGF.bit.OpenLoop = 0;
			// Difference between force angle and estimated theta is saved,
			// so a soft transition is made when entering closed loop.
			Theta_error = ParkParm.qAngle - smc1.Theta;
		}
		ParkParm.qAngle += (int)(Startup_Ramp >> 16);
	}
	else
	{
		// This value is used to transition from open loop to closed looop. 
		// At the end of open loop ramp, there is a difference between 
		// forced angle and estimated angle. This difference is stored in 
		// Theta_error, and added to estimated theta (smc1.Theta) so the 
		// effective angle used for commutating the motor is the same at 
		// the end of open loop, and at the begining of closed loop. 
		// This Theta_error is then substracted from estimated theta 
		// gradually in increments of 0.05 degrees until the error is less
		// than 0.05 degrees.  0.05/180=0.0002777
		ParkParm.qAngle = smc1.Theta + Theta_error;
		if (fabsf(Theta_error) > 0.0002777)
		{
			if (Theta_error < 0)
				Theta_error += 0.00027777;
			else
				Theta_error -= 0.00027777;
		}
	}
	return;
}
void ClarkePark(void)
{
	ParkParm.qIalpha = ParkParm.qIa;
	ParkParm.qIbeta = ParkParm.qIa*INV_SQRT3 + 2*ParkParm.qIb*INV_SQRT3;
	// Ialpha and Ibeta have been calculated. Now do rotation.
	// Get qSin, qCos from ParkParm structure

	ParkParm.qId =  ParkParm.qIalpha*cosf(ParkParm.qAngle) + ParkParm.qIbeta*sinf(ParkParm.qAngle);
	ParkParm.qIq = -ParkParm.qIalpha*sinf(ParkParm.qAngle) + ParkParm.qIbeta*cosf(ParkParm.qAngle);

	return;
}
//---------------------------------------------------------------------
// Executes one PI itteration for each of the three loops Id,Iq,Speed,

void DoControl( void )
	{

		if( uGF.bit.OpenLoop )
			{
			// OPENLOOP:	force rotating angle, and control Iq and Id
			//				Also limits Vs vector to ensure maximum PWM duty
			//				cycle and no saturation
	
			// This If statement is executed only the first time we enter open loop,
			// everytime we run the motor
			if( uGF.bit.ChangeMode )
			{
				// just changed to openloop
				uGF.bit.ChangeMode = 0;
				// synchronize angles
	
				// VqRef & VdRef not used
				CtrlParm.qVqRef = 0;
				CtrlParm.qVdRef = 0;
				CtrlParm.qVelRef = 0;
				Startup_Lock = 0;
				Startup_Ramp = 0;
				// Initialize SMC
				smc1.Valpha = 0;
				smc1.Ealpha = 0;
				smc1.EalphaFinal = 0;
				smc1.Zalpha = 0;
				smc1.EstIalpha = 0;
				smc1.Vbeta = 0;
				smc1.Ebeta = 0;
				smc1.EbetaFinal = 0;
				smc1.Zbeta = 0;
				smc1.EstIbeta = 0;
				smc1.Ialpha = 0;
				smc1.IalphaError = 0;
				smc1.Ibeta = 0;
				smc1.IbetaError = 0;
				smc1.Theta = 0;
				smc1.Omega = 0;
			}
	
			// Enter initial torque demand in Amps using REFINAMPS() macro.
			// Maximum Value for reference is defined by shunt resistor value and 
			// differential amplifier gain. Use this equation to calculate 
			// maximum torque in Amperes:
			// 
			// Max REFINAMPS = (VDD/2)/(RSHUNT*DIFFAMPGAIN)
			//
			// For example:
			//
			// RSHUNT = 0.005
			// VDD = 3.3
			// DIFFAMPGAIN = 75
			//
			// Maximum torque reference in Amps is:
			//
			// (3.3/2)/(.005*75) = 4.4 Amperes, or REFINAMPS(4.4)
			//
			// If motor requires more torque than Maximum torque to startup, user
			// needs to change either shunt resistors installed on the board,
			// or differential amplifier gain.
	
			CtrlParm.qVqRef = REFINAMPS(INITIALTORQUE);
	
			if(AccumThetaCnt == 0)
			{
				PIParmW.qInMeas = smc1.Omega;
			}
	
			// PI control for D
			PIParmD.qInMeas = ParkParm.qId;
			PIParmD.qInRef	= CtrlParm.qVdRef;
			CalcPI(&PIParmD);
			ParkParm.qVd	= PIParmD.qOut;
	
			// Vector limitation
			// Vd is not limited
			// Vq is limited so the vector Vs is less than a maximum of 95%.
			// The 5% left is needed to be able to measure current through
			// shunt resistors.
			// Vs = SQRT(Vd^2 + Vq^2) < 0.95
			// Vq = SQRT(0.95^2 - Vd^2)
			qVdSquared = PIParmD.qOut * PIParmD.qOut;
			PIParmQ.qOutMax = sqrtf((0.95*0.95) - qVdSquared);
			PIParmQ.qOutMin = -PIParmQ.qOutMax;
	
			// PI control for Q
			PIParmQ.qInMeas = ParkParm.qIq;
			PIParmQ.qInRef	= CtrlParm.qVqRef;
			CalcPI(&PIParmQ);
			ParkParm.qVq	= PIParmQ.qOut;
			
		}
	
		else
		// Closed Loop Vector Control
		{
			// Pressing one of the push buttons, speed reference (or torque reference
			// if enabled) will be doubled. This is done to test transient response
			// of the controllers
			if( ++count == SPEEDDELAY ) 
			{
				VelReq = (SpeedReference* DQK) + ((OMEGA10 + OMEGA1)/2.0);
	
				if (CtrlParm.qVelRef <= VelReq)
				{
					 CtrlParm.qVelRef += SPEEDDELAY; 
				}
				else 
				{
					CtrlParm.qVelRef -= SPEEDDELAY;
				}
				
				count = 0;
			}
			
			// When it first transition from open to closed loop, this If statement is
			// executed
			if( uGF.bit.ChangeMode )
			{
				// just changed from openloop
				uGF.bit.ChangeMode = 0;
				// An initial value is set for the speed controller accumulation.
				//
				// The first time the speed controller is executed, we want the output
				// to be the same as it was the last time open loop was executed. So,
				// last time open loop was executed, torque refefernce was constant,
				// and set to CtrlParm.qVqRef.
				//
				// First time in closed loop, CtrlParm.qVqRef = PIParmW.qdSum >> 16
				// assuming the error is zero at time zero. This is why we set 
				// PIParmW.qdSum = (long)CtrlParm.qVqRef << 16.
				PIParmW.qdSum = CtrlParm.qVqRef ;
				Startup_Lock = 0;
				Startup_Ramp = 0;
					//velocity reference ramp begins at minimum speed
				CtrlParm.qVelRef = OMEGA0;
			
			}  
	
			// Check to see if new velocity information is available by comparing
			// the number of interrupts per velocity calculation against the
			// number of velocity count samples taken.	If new velocity info
			// is available, calculate the new velocity value and execute
			// the speed control loop.
	
			if(AccumThetaCnt == 0)
			{
				// Execute the velocity control loop
				PIParmW.qInMeas = smc1.Omega;
				PIParmW.qInRef	= CtrlParm.qVelRef;
				CalcPI(&PIParmW);
				CtrlParm.qVqRef = PIParmW.qOut;
			}
			 
			// If the application is running in torque mode, the velocity
			// control loop is bypassed.  The velocity reference value, read
			// from the potentiometer, is used directly as the torque 
			// reference, VqRef. This feature is enabled automatically only if
			// #define TORQUEMODE is defined in UserParms.h. If this is not
			// defined, uGF.bit.EnTorqueMod bit can be set in debug mode to enable
			// torque mode as well.
	
			if (uGF.bit.EnTorqueMod)
				CtrlParm.qVqRef = CtrlParm.qVelRef;
	
			// Get Id reference from Field Weakening table. If Field weakening
			// is not needed or user does not want to enable this feature, 
			// let NOMINALSPEEDINRPM be equal to FIELDWEAKSPEEDRPM in
			// UserParms.h
			CtrlParm.qVdRef = FieldWeakening(fabsf(CtrlParm.qVelRef));
	
			// PI control for D
			PIParmD.qInMeas = ParkParm.qId;
			PIParmD.qInRef	= CtrlParm.qVdRef;
			CalcPI(&PIParmD);
	
			// If voltage ripple compensation flag is set, adjust the output
			// of the D controller depending on measured DC Bus voltage. This 
			// feature is enabled automatically only if #define ENVOLTRIPPLE is 
			// defined in UserParms.h. If this is not defined, uGF.bit.EnVoltRipCo
			// bit can be set in debug mode to enable voltage ripple compensation.
			//
			// NOTE:
			//
			// If Input power supply has switching frequency noise, for example if a
			// switch mode power supply is used, Voltage Ripple Compensation is not
			// recommended, since it will generate spikes on Vd and Vq, which can
			// potentially make the controllers unstable.
			if(uGF.bit.EnVoltRipCo)
				ParkParm.qVd = VoltRippleComp(PIParmD.qOut);
			else
				ParkParm.qVd = PIParmD.qOut;
	
			// Vector limitation
			// Vd is not limited
			// Vq is limited so the vector Vs is less than a maximum of 95%. 
			// Vs = SQRT(Vd^2 + Vq^2) < 0.95
			// Vq = SQRT(0.95^2 - Vd^2)
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
	
			// Limit, if motor is stalled, stop motor commutation
			if (smc1.OmegaFltred < 0)
			{
				uGF.bit.RunMotor = 0;
			}
		}
	}
void InitPI( tPIParm *pParm)
{
	pParm->qdSum=0;
	pParm->qOut=0;

	//pParm->qInMeas=0;
	//pParm->qInRef=0;
	//pParm->qKc=0;
	//pParm->qKi=0;
	//pParm->qKp=0;
	//pParm->qOutMax=0;
	//pParm->qOutMin=0;
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
	return;
}
void DebounceDelay(void)
{
	long i;
	for (i = 0;i < 100000;i++)
		;
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
void MeasCompCurr( void )
{
	 int CorrADC1, CorrADC2;

	 int curr1 = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
	 int curr2 = ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_1);
	 
	 CorrADC1 = curr1 - MeasCurrParm.Offseta;
	 CorrADC2 = curr2 - MeasCurrParm.Offsetb;
	// ADC_curr_norm_value[2] = -(ADC_curr_norm_value[0] + ADC_curr_norm_value[1]);

	 ParkParm.qIa = MeasCurrParm.qKa * (float)CorrADC1;
	 ParkParm.qIb = MeasCurrParm.qKb * (float)CorrADC2;

	 return;
}
void InitMeasCompCurr( short Offset_a, short Offset_b )
{
	MeasCurrParm.Offseta = Offset_a;
	MeasCurrParm.Offsetb = Offset_b;
	return;
}
void InvPark(void)
{
	ParkParm.qValpha =  ParkParm.qVd*cosf(ParkParm.qAngle) - ParkParm.qVq*sinf(ParkParm.qAngle);
	ParkParm.qVbeta  =  ParkParm.qVd*sinf(ParkParm.qAngle) + ParkParm.qVq*cosf(ParkParm.qAngle);
	return;
}
void CalcRefVec(void)
{
     //SVGenParm.qVr1 =ParkParm.qValpha;
     //SVGenParm.qVr2 = (-ParkParm.qValpha + SQRT3 * ParkParm.qVbeta)/2;
     //SVGenParm.qVr3 = (-ParkParm.qValpha  - SQRT3 * ParkParm.qVbeta)/2;

    SVGenParm.qVr1 =ParkParm.qVbeta;
    SVGenParm.qVr2 = (-ParkParm.qVbeta + SQRT3 * ParkParm.qValpha)/2;
    SVGenParm.qVr3 = (-ParkParm.qVbeta  - SQRT3 * ParkParm.qValpha)/2;

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
	//TIM8->CR1 |= TIM_CR1_UDIS;

	TIM1->CCR1 = duty_A;
	TIM1->CCR2 = duty_B;
	TIM1->CCR3 = duty_C;
	//TIM1->CCR4 = 100;


	// Enables preload register updates
	TIM1->CR1 &= ~TIM_CR1_UDIS;
	//TIM8->CR1 &= ~TIM_CR1_UDIS;

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

void SMCInit(SMC *s)
{
	//				  R * Ts
	// Fsmopos = 1 - --------
	//					L
	//			  Ts
	// Gsmopos = ----
	//			  L
	// Ts = Sampling Period. If sampling at PWM, Ts = 50 us
	// R = Phase Resistance. If not provided by motor datasheet,
	//	   measure phase to phase resistance with multimeter, and
	//	   divide over two to get phase resistance. If 4 Ohms are
	//	   measured from phase to phase, then R = 2 Ohms
	// L = Phase inductance. If not provided by motor datasheet,
	//	   measure phase to phase inductance with multimeter, and
	//	   divide over two to get phase inductance. If 2 mH are
	//	   measured from phase to phase, then L = 1 mH

	if ((PHASERES * LOOPTIMEINSEC) > PHASEIND)
		s->Fsmopos = 0.0;
	else
		s->Fsmopos = (1 - PHASERES * LOOPTIMEINSEC / PHASEIND);

	if (LOOPTIMEINSEC > PHASEIND)
		s->Gsmopos = 0.99999;
	else
		s->Gsmopos = LOOPTIMEINSEC / PHASEIND;

	s->Kslide = SMCGAIN;
	s->MaxSMCError = MAXLINEARSMC;
	s->FiltOmCoef = (OMEGA0 * PI / IRP_PERCALC); // Cutoff frequency for omega filter
											 // is minimum omega, or OMEGA0
	return;
}	
void SMC_Position_Estimation (SMC *s)
{
	// Sliding mode current observer
	
	s->EstIalpha = s->Gsmopos * s->Valpha - s->Gsmopos * s->Ealpha	- s->Gsmopos * s->Zalpha	 + s->Fsmopos * s->EstIalpha;
	s->EstIbeta = s->Gsmopos * s->Vbeta - s->Gsmopos * s->Ebeta - s->Gsmopos * s->Zbeta + s->Fsmopos * s->EstIbeta;

	s->IalphaError = s->EstIalpha - s->Ialpha;
	s->IbetaError = s->EstIbeta - s->Ibeta;
		
	// Sliding control calculator

	if (fabsf(s->IalphaError) < s->MaxSMCError)
	{
		// s->Zalpha = (s->Kslide * s->IalphaError) / s->MaxSMCError
		// If we are in the linear range of the slide mode controller,
		// then correction factor Zalpha will be proportional to the
		// error (Ialpha - EstIalpha) and slide mode gain, Kslide.
		s->Zalpha = (s->Kslide * s->IalphaError) / s->MaxSMCError;
	}
	else if (s->IalphaError > 0)
		s->Zalpha = s->Kslide;
	else
		s->Zalpha = -s->Kslide;

	if (fabsf(s->IbetaError) < s->MaxSMCError)
	{
		// s->Zbeta = (s->Kslide * s->IbetaError) / s->MaxSMCError
		// If we are in the linear range of the slide mode controller,
		// then correction factor Zbeta will be proportional to the
		// error (Ibeta - EstIbeta) and slide mode gain, Kslide.
		s->Zbeta = (s->Kslide * s->IbetaError) / s->MaxSMCError;
	}
	else if (s->IbetaError > 0)
		s->Zbeta = s->Kslide;
	else
		s->Zbeta = -s->Kslide;
	
		
	// Sliding control filter -> back EMF calculator
	 s->Ealpha = s->Ealpha + s->Kslf * s->Zalpha - s->Kslf * s->Ealpha;
	 s->Ebeta = s->Ebeta + s->Kslf * s->Zbeta - s->Kslf * s->Ebeta;
	 s->EalphaFinal = s->EalphaFinal + s->KslfFinal * s->Ealpha  - s->KslfFinal * s->EalphaFinal;
	 s->EbetaFinal = s->EbetaFinal + s->KslfFinal * s->Ebeta - s->KslfFinal * s->EbetaFinal;
	
		
	// Rotor angle calculator -> Theta = atan(-EalphaFinal,EbetaFinal)

	s->Theta = atan2f(-s->EalphaFinal,s->EbetaFinal);
		
	AccumTheta += s->Theta - PrevTheta;
	PrevTheta = s->Theta;
	
	AccumThetaCnt++;
	if (AccumThetaCnt == IRP_PERCALC)
	{
		s->Omega = AccumTheta;
		AccumThetaCnt = 0;
		AccumTheta = 0;
	}
	//					  Q15(Omega) * 60
	// Speed RPMs = -----------------------------
	//				 SpeedLoopTime * Motor Poles
	// For example:
	//	  Omega = 0.5
	//	  SpeedLoopTime = 0.001
	//	  Motor Poles (pole pairs * 2) = 10
	// Then:
	//	  Speed in RPMs is 3,000 RPMs

	// s->OmegaFltred = s->OmegaFltred + FilterCoeff * s->Omega   - FilterCoeff * s->OmegaFltred

	 s->OmegaFltred = s->OmegaFltred + s->FiltOmCoef * s->Omega - s->FiltOmCoef * s->OmegaFltred;
		

	// Adaptive filter coefficients calculation
	// Cutoff frequency is defined as 2*_PI*electrical RPS
	//
	//		Wc = 2*_PI*Fc.
	//		Kslf = Tpwm*2*_PI*Fc
	//
	// Fc is the cutoff frequency of our filter. We want the cutoff frequency
	// be the frequency of the drive currents and voltages of the motor, which
	// is the electrical revolutions per second, or eRPS.
	//
	//		Fc = eRPS = RPM * Pole Pairs / 60
	//
	// Kslf is then calculated based on user parameters as follows:
	// First of all, we have the following equation for RPMS:
	//
	//		RPM = (Q15(Omega) * 60) / (SpeedLoopTime * Motor Poles)
	//		Let us use: Motor Poles = Pole Pairs * 2
	//		eRPS = RPM * Pole Pairs / 60), or
	//		eRPS = (Q15(Omega) * 60 * Pole Pairs) / (SpeedLoopTime * Pole Pairs * 2 * 60)
	//	Simplifying eRPS
	//		eRPS = Q15(Omega) / (SpeedLoopTime * 2)
	//	Using this equation to calculate Kslf
	//		Kslf = Tpwm*2*_PI*Q15(Omega) / (SpeedLoopTime * 2)
	//	Using diIrpPerCalc = SpeedLoopTime / Tpwm
	//		Kslf = Tpwm*2*Q15(Omega)*_PI / (diIrpPerCalc * Tpwm * 2)
	//	Simplifying:
	//		Kslf = Q15(Omega)*_PI/diIrpPerCalc
	//
	// We use a second filter to get a cleaner signal, with the same coefficient
	//
	//		Kslf = KslfFinal = Q15(Omega)*_PI/diIrpPerCalc
	//
	// What this allows us at the end is a fixed phase delay for theta compensation
	// in all speed range, since cutoff frequency is changing as the motor speeds up.
	// 
	// Phase delay: Since cutoff frequency is the same as the input frequency, we can
	// define phase delay as being constant of -45 DEG per filter. This is because
	// the equation to calculate phase shift of this low pass filter is 
	// arctan(Fin/Fc), and Fin/Fc = 1 since they are equal, hence arctan(1) = 45 DEG.
	// A total of -90 DEG after the two filters implemented (Kslf and KslfFinal).
	
	s->Kslf = s->KslfFinal = s->OmegaFltred * (PI / IRP_PERCALC);
		
	// Since filter coefficients are dynamic, we need to make sure we have a minimum
	// so we define the lowest operation speed as the lowest filter coefficient

	if (s->Kslf < (OMEGA0 * PI / IRP_PERCALC))
	{
		s->Kslf = (OMEGA0 * PI / IRP_PERCALC);
		s->KslfFinal = (OMEGA0 * PI / IRP_PERCALC);
	}
	s->ThetaOffset = CONSTANT_PHASE_SHIFT;
	s->Theta = s->Theta + s->ThetaOffset;

	return;
}

void FWInit(void)
{
	/* initialize magnetizing curve values */
	FdWeakParm.qFwOnSpeed = OMEGANOMINAL;
	FdWeakParm.qFwCurve[0]	= dqKFw0;
	FdWeakParm.qFwCurve[1]	= dqKFw1;
	FdWeakParm.qFwCurve[2]	= dqKFw2;
	FdWeakParm.qFwCurve[3]	= dqKFw3;
	FdWeakParm.qFwCurve[4]	= dqKFw4;
	FdWeakParm.qFwCurve[5]	= dqKFw5;
	FdWeakParm.qFwCurve[6]	= dqKFw6;
	FdWeakParm.qFwCurve[7]	= dqKFw7;
	FdWeakParm.qFwCurve[8]	= dqKFw8;
	FdWeakParm.qFwCurve[9]	= dqKFw9;
	FdWeakParm.qFwCurve[10]	= dqKFw10;
	FdWeakParm.qFwCurve[11]	= dqKFw11;
	FdWeakParm.qFwCurve[12]	= dqKFw12;
	FdWeakParm.qFwCurve[13]	= dqKFw13;
	FdWeakParm.qFwCurve[14]	= dqKFw14;
	FdWeakParm.qFwCurve[15]	= dqKFw15;
	return;
}
float FieldWeakening(float qMotorSpeed)
{
    /* if the speed is less than one for activating the FW */
	if (qMotorSpeed <= FdWeakParm.qFwOnSpeed)
	{
		/* set Idref as first value in magnetizing curve */
		FdWeakParm.qIdRef = FdWeakParm.qFwCurve[0];
	} 
	else
	{
		// Index in FW-Table. The result is left shifted 11 times because
		// we have a field weakening table of 16 (4 bits) values, and the result
		// of the division is 15 bits (16 bits, with no sign). So
		// Result (15 bits) >> 11 -> Index (4 bits).
		FdWeakParm.qFWPercentage = (qMotorSpeed-FdWeakParm.qFwOnSpeed)/ (OMEGAFIELDWK-OMEGANOMINAL+1);
		FdWeakParm.qIndex = FdWeakParm.qFWPercentage;

		// Interpolation betwen two results from the Table. First mask 11 bits,
		// then left shift 4 times to get 15 bits again.
		//------->FdWeakParm.qInterpolPortion = (FdWeakParm.qFWPercentage & 0x07FF) << 4;

		//------->FdWeakParm.qIdRef = FdWeakParm.qFwCurve[FdWeakParm.qIndex] - (FdWeakParm.qFwCurve[FdWeakParm.qIndex] - FdWeakParm.qFwCurve[FdWeakParm.qIndex+1] ) /FdWeakParm.qInterpolPortion);

	}

	return FdWeakParm.qIdRef;
}

static void stop_pwm_hw(void) {
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
float mcpwm_get_rpm(void) {
	return 0.0;
}
mc_state mcpwm_get_state(void) {
	return 0;
}
mc_fault_code mcpwm_get_fault(void) {
	return 0;
}
const char* mcpwm_fault_to_string(mc_fault_code fault) {
	switch (fault) {
	case FAULT_CODE_NONE: return "FAULT_CODE_NONE"; break;
	case FAULT_CODE_OVER_VOLTAGE: return "FAULT_CODE_OVER_VOLTAGE"; break;
	case FAULT_CODE_UNDER_VOLTAGE: return "FAULT_CODE_UNDER_VOLTAGE"; break;
	case FAULT_CODE_DRV8302: return "FAULT_CODE_DRV8302"; break;
	case FAULT_CODE_ABS_OVER_CURRENT: return "FAULT_CODE_ABS_OVER_CURRENT"; break;
	case FAULT_CODE_OVER_TEMP_FET: return "FAULT_CODE_OVER_TEMP_FET"; break;
	case FAULT_CODE_OVER_TEMP_MOTOR: return "FAULT_CODE_OVER_TEMP_MOTOR"; break;
	default: return "FAULT_UNKNOWN"; break;
	}
}
mc_rpm_dep_struct mcpwm_get_rpm_dep(void) {
	mc_rpm_dep_struct temp;

	return temp;
}

static volatile mc_configuration conf;
const volatile mc_configuration* mcpwm_get_configuration(void) {
	return &conf;
}
void mcpwm_set_configuration(mc_configuration *configuration) {
	// Stop everything first to be safe

}
