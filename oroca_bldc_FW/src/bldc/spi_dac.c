
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"

#include "uart3_print.h"

// pin 
/*
PA15 ==> SPI_NSS ,GPIO
PB3  ==> SPI_SCK , AF5, SPI1_SCK
PB4  ==> SPI_MISO, GPIO, /LOADDACS
PB5  ==> SPI_MOSI , AF5, SPI1_MOSI
*/

#define GPIOA_PIN15_SPI_NSS			15
#define GPIOB_PIN4_LOADDACS         4

#define GPIOB_PIN3_SCK              3
#define GPIOB_PIN5_MOSI             5


static void gpt_cb(GPTDriver *gptp) {

  (void)gptp;
  //palSetPad(IOPORT3, GPIOC_LED);
}


//TIM5 �̸� APB1, 42MHZ �� ����� �ֳ�!!!, 32bit �̿�
static const GPTConfig gpt_cfg = {
  1000000,    // 1MHz timer clock.
  //100000,    // 100KHz timer clock.
  //10000,    // 10KHz timer clock.
  gpt_cb,   // Timer callback. ������ �־�� �ϳ�!!!
  0
};


void initGPT( void)
{
	//gptStart(&GPTD7, &gpt_cfg);
	gptStart(&GPTD5, &gpt_cfg);
	gptStartContinuous(&GPTD5, 0xFFFFFFFF);
}

// TIM2,TIM5 �� 32bit �̰�, �������� ��� , 16bit ��
uint32_t get_GPT_value( void)
{
	 //return GPTD7.tim->CNT;
	 return GPTD5.tim->CNT;
}

void _udelay( uint32_t udelay)
{
	uint32_t  t1 = get_GPT_value();
	uint32_t  t2 = get_GPT_value();
	while( (t2-t1) < udelay )
	{
		t2 = get_GPT_value();
	}
}




void spi_hw_init_gpio(void) {
	// SPI clock enable

	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);


	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, DISABLE);

	

	palSetPadMode(GPIOA, GPIOA_PIN15_SPI_NSS,	PAL_MODE_OUTPUT_PUSHPULL |	PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(GPIOB, GPIOB_PIN4_LOADDACS,	PAL_MODE_OUTPUT_PUSHPULL |	PAL_STM32_OSPEED_HIGHEST);
	
	//palClearPad(GPIOA, GPIOA_PIN15_SPI_NSS);
	palSetPad( GPIOA, GPIOA_PIN15_SPI_NSS);

	palSetPad(GPIOB, GPIOB_PIN4_LOADDACS);
	
	

	//PWM ( GPIOA Configuration: Channel 1 to 3 as alternate function push-pull)
	//palSetPadMode(GPIOA, GPIOA_PIN15_SPI_NSS, PAL_MODE_ALTERNATE(GPIO_AF_SPI1) |PAL_STM32_OSPEED_HIGHEST |	PAL_STM32_PUDR_FLOATING);
	//palSetPadMode(GPIOA, GPIOA_PIN15_SPI_NSS, PAL_MODE_ALTERNATE(GPIO_AF_SPI1) |PAL_STM32_OSPEED_HIGHEST |	PAL_STM32_PUDR_PULLUP);

	palSetPadMode(GPIOB, GPIOB_PIN3_SCK, PAL_MODE_ALTERNATE(GPIO_AF_SPI1) |PAL_STM32_OSPEED_HIGHEST |	PAL_STM32_PUDR_FLOATING);

	// ����� ���� �������� ��ȣ�� �̻���
	//palSetPadMode(GPIOB, GPIOB_PIN5_MOSI, PAL_MODE_ALTERNATE(GPIO_AF_SPI1) |PAL_STM32_OSPEED_HIGHEST |	PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, GPIOB_PIN5_MOSI, PAL_MODE_ALTERNATE(GPIO_AF_SPI1) |PAL_STM32_OSPEED_HIGHEST |	PAL_STM32_PUDR_PULLUP);
	//palSetPadMode(GPIOB, GPIOB_PIN5_MOSI, PAL_MODE_ALTERNATE(GPIO_AF_SPI1) |PAL_STM32_OSPEED_HIGHEST |	PAL_STM32_PUDR_PULLDOWN);

	
}





void SPI_Init_for_DAC_7612(SPI_TypeDef* SPIx)
{
	uint16_t tmpreg = 0;
	
	// Transmit-only mode is similar to full-duplex mode (BIDIMODE=0, RXONLY=0): 
	// RXNE �� �̿��ؼ� LOADDACS ���� ����ϱ� ���ؼ�

	// CPOL = 1, CPHA =1
	// 16-bit data frame format
	// APB2 84 MHz �ִ� 20Mhz �̹Ƿ� �ּ� 1/4 �� �ؾ���
	// 001: fPCLK/4, 010: fPCLK/8, 011: fPCLK/16, 100: fPCLK/32

	
	tmpreg |= SPI_CR1_DFF; //16 bit

	//tmpreg |= SPI_CR1_BR_1; // 1/8,  10MHZ
	//tmpreg |= SPI_CR1_BR_1 | SPI_CR1_BR_0  ; // 1/16, , 5MHZ
	tmpreg |= SPI_CR1_BR_2; // 1/32, , 2.5MHZ

	tmpreg |= SPI_CR1_MSTR; // Master
	tmpreg |= SPI_CR1_CPOL | SPI_CR1_CPHA ;

	SPIx->CR1 = tmpreg;

	//Uart3_printf( &SD3,"CR1=%X,%X \r\n",  SPIx->CR1, tmpreg  );//170530  


	SPIx->CR2  = SPI_CR2_SSOE; // �ϵ���� NSS ���


	SPIx->CR1  |= SPI_CR1_SPE;


}

void spi_dac_hw_init(void) 
{
	spi_hw_init_gpio();
	initGPT();
  
	SPI_Init_for_DAC_7612( SPI1);
}

static void _spi_dac_write( short data) 
{
	volatile short data_temp;
	SPI_TypeDef* SPIx = SPI1;


	//Uart3_printf(&SD3, "CR1=%X \r\n",  SPIx->CR1 );//170530 
	//Uart3_printf(&SD3, "CR2=%X \r\n",  SPIx->CR2 );
	//Uart3_printf(&SD3, "SR=%X \r\n",  SPIx->SR );

	
	while( (SPIx->SR & SPI_SR_TXE) == 0); // ������ �� ���� ������ ���


	// ����� ���� ���� ���� �ֱ� ���ؼ�
	palClearPad( GPIOA, GPIOA_PIN15_SPI_NSS);
	_udelay( 1);
	palSetPad( GPIOA, GPIOA_PIN15_SPI_NSS);
	_udelay( 1);
	palClearPad( GPIOA, GPIOA_PIN15_SPI_NSS);

	
	data_temp = SPIx->DR ; // for Clear RXNE

	//palClearPad(GPIOA, GPIOA_PIN15_SPI_NSS); // CS LOW
	//SPIx->DR = data;
	//SPIx->DR = data | 0x8000; // ����� ���� ���� ���� �ֱ� ���ؼ�
	SPIx->DR = data | 0x4000; // ����� ���� ���� ���� �ֱ� ���ؼ�


	
	while( (SPIx->SR & SPI_SR_RXNE) == 0);

	_udelay( 1);

	palSetPad( GPIOA, GPIOA_PIN15_SPI_NSS);
	_udelay( 1);


	
	palClearPad(GPIOB, GPIOB_PIN4_LOADDACS);
	// �ּ� 20ns �ʿ���
	_udelay( 1);
	palSetPad(GPIOB, GPIOB_PIN4_LOADDACS);
	

}

void spi_dac_write_AB( short data) 
{
	data &= 0xFFF;
	_spi_dac_write( data);
}
void spi_dac_write_A( short data) 
{
	data &= 0xFFF;
	_spi_dac_write( data | 0x2000);
}
void spi_dac_write_B( short data) 
{
	data &= 0xFFF;
	_spi_dac_write( data | 0x3000);
}
