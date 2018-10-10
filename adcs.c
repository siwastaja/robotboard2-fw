#include <stdint.h>
#include "ext_include/stm32h7xx.h"
#include "stm32_cmsis_extension.h"
#include "misc.h"
#define DEFINE_VARS
#include "adcs.h"
#undef DEFINE_VARS
#include "own_std.h"

static char printbuf[128];

volatile adc1_group_t adc1 __attribute__((aligned(4))) __attribute__((section(".sram3_bss")));
volatile adc2_group_t adc2 __attribute__((aligned(4))) __attribute__((section(".sram3_bss")));
volatile adc3_group_t adc3 __attribute__((aligned(4))) __attribute__((section(".sram3_bss")));


void adc12_inthandler()
{
	if(ADC1->ISR & (1UL<<7)) // ADC1 AWD1
	{
		uart_print_string_blocking("\r\nADC1 AWD1: Vbat out of range\r\n");
		ADC1->ISR = 1UL<<7;	
	}

	if(ADC1->ISR & (1UL<<8)) // ADC1 AWD2
	{
		uart_print_string_blocking("\r\nADC1 AWD2: Charger Vinbus out of range\r\n");	
		ADC1->ISR = 1UL<<8;
	}

	if(ADC1->ISR & (1UL<<9)) // ADC1 AWD3
	{
		uart_print_string_blocking("\r\nADC1 AWD3\r\n");	
		ADC1->ISR = 1UL<<9;
	}

	if(ADC2->ISR & (1UL<<7)) // ADC2 AWD1
	{
		uart_print_string_blocking("\r\nADC2 AWD1\r\n");	
		ADC2->ISR = 1UL<<7;

	}

	if(ADC2->ISR & (1UL<<8)) // ADC2 AWD2
	{
		uart_print_string_blocking("\r\nADC2 AWD2\r\n");	
		ADC2->ISR = 1UL<<8;
	}

	if(ADC2->ISR & (1UL<<9)) // ADC2 AWD3
	{
		uart_print_string_blocking("\r\nADC2 AWD3\r\n");	
		ADC2->ISR = 1UL<<9;
	}

	__DSB();

	if((ADC1->ISR & (1UL<<4)) || (ADC2->ISR & (1UL<<4)) )
	{
		// Overrun is the only real error condition

		uart_print_string_blocking("\r\n\r\nSTOPPED: ");
		uart_print_string_blocking(__func__);
		uart_print_string_blocking("\r\n");

		uart_print_string_blocking("\r\nADC1 intflags = "); o_btoa16_fixed(ADC1->ISR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
		uart_print_string_blocking("\r\nADC2 intflags = "); o_btoa16_fixed(ADC2->ISR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");

		uart_print_string_blocking("DMA for ADC1: ");

		if(ADC1_DMA_STREAM->CR & 1) uart_print_string_blocking("STREAM ON"); else uart_print_string_blocking("STREAM OFF"); 
		uart_print_string_blocking("\r\nintflags = "); o_btoa8_fixed(DMA_INTFLAGS(ADC1_DMA, ADC1_DMA_STREAM_NUM), printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
		uart_print_string_blocking("NDTR  = "); o_utoa16(ADC1_DMA_STREAM->NDTR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");

		uart_print_string_blocking("DMA for ADC2: ");
		if(ADC2_DMA_STREAM->CR & 1) uart_print_string_blocking("STREAM ON"); else uart_print_string_blocking("STREAM OFF"); 
		uart_print_string_blocking("\r\nintflags = "); o_btoa8_fixed(DMA_INTFLAGS(ADC2_DMA, ADC2_DMA_STREAM_NUM), printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
		uart_print_string_blocking("NDTR  = "); o_utoa16(ADC2_DMA_STREAM->NDTR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");


		uart_print_string_blocking("\r\n");
		error(20);

	}
}


void adc3_inthandler()
{
	uart_print_string_blocking("\r\n\r\nSTOPPED: ");
	uart_print_string_blocking(__func__);
	uart_print_string_blocking("\r\n");

	uart_print_string_blocking("\r\nADC3 intflags = "); o_btoa16_fixed(ADC3->ISR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");

	uart_print_string_blocking("DMA for ADC3: ");
	if(ADC3_DMA_STREAM->CR & 1) uart_print_string_blocking("STREAM ON"); else uart_print_string_blocking("STREAM OFF"); 
	uart_print_string_blocking("\r\nintflags = "); o_btoa8_fixed(DMA_INTFLAGS(ADC3_DMA, ADC1_DMA_STREAM_NUM), printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("NDTR  = "); o_utoa16(ADC3_DMA_STREAM->NDTR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");

	uart_print_string_blocking("\r\n");
	error(20);
}

void adc1_dma_errhandler()
{
	uart_print_string_blocking("\r\n\r\nSTOPPED: ");
	uart_print_string_blocking(__func__);
	uart_print_string_blocking("\r\n");

	if(ADC1_DMA_STREAM->CR & 1) uart_print_string_blocking("STREAM ON"); else uart_print_string_blocking("STREAM OFF"); 
	uart_print_string_blocking("\r\nintflags = "); o_btoa8_fixed(DMA_INTFLAGS(ADC1_DMA, ADC1_DMA_STREAM_NUM), printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("NDTR  = "); o_utoa16(ADC1_DMA_STREAM->NDTR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	error(20);
}

void adc2_dma_errhandler()
{
	uart_print_string_blocking("\r\n\r\nSTOPPED: ");
	uart_print_string_blocking(__func__);
	uart_print_string_blocking("\r\n");
	if(ADC2_DMA_STREAM->CR & 1) uart_print_string_blocking("STREAM ON"); else uart_print_string_blocking("STREAM OFF"); 
	uart_print_string_blocking("\r\nintflags = "); o_btoa8_fixed(DMA_INTFLAGS(ADC2_DMA, ADC2_DMA_STREAM_NUM), printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("NDTR  = "); o_utoa16(ADC2_DMA_STREAM->NDTR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	error(20);
}

void adc3_dma_errhandler()
{
	uart_print_string_blocking("\r\n\r\nSTOPPED: ");
	uart_print_string_blocking(__func__);
	uart_print_string_blocking("\r\n");
	if(ADC3_DMA_STREAM->CR & 1) uart_print_string_blocking("STREAM ON"); else uart_print_string_blocking("STREAM OFF"); 
	uart_print_string_blocking("\r\nintflags = "); o_btoa8_fixed(DMA_INTFLAGS(ADC3_DMA, ADC1_DMA_STREAM_NUM), printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("NDTR  = "); o_utoa16(ADC3_DMA_STREAM->NDTR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	error(20);
}


static inline void CONF_ADC_SEQ(ADC_TypeDef* adc, int len,
	int s1, int s2, int s3, int s4, int s5, int s6, int s7, int s8,
	int s9, int s10, int s11, int s12, int s13, int s14, int s15, int s16)
{
	adc->SQR1 = (s4)<<24 | (s3)<<18 | (s2)<<12 | (s1)<<6 | (len-1);
	adc->SQR2 = (s9)<<24 | (s8)<<18 | (s7)<<12 | (s6)<<6 | (s5);
	adc->SQR3 = (s14)<<24 | (s13)<<18 | (s12)<<12 | (s11)<<6 | (s10);
	adc->SQR4 = (s16)<<6 | (s15);
}

static inline void CONF_ADC_SMPTIMES(ADC_TypeDef* adc,
	int c0, int c1, int c2, int c3, int c4, int c5, int c6, int c7, int c8, int c9,
	int c10, int c11, int c12, int c13, int c14, int c15, int c16, int c17, int c18, int c19)
{
	adc->SMPR1 = c9<<27 | c8<<24 | c7<<21 | c6<<18 | c5<<15 | c4<<12 | c3<<9 | c2<<6 | c1<<3 | c0;
	adc->SMPR2 = c19<<27 | c18<<24 | c17<<21 | c16<<18 | c15<<15 | c14<<12 | c13<<9 | c12<<6 | c11<<3 | c10;
}

// If differential measurements will be used, remember to add a calibration cycle for diff meas.
static void init_calib_adc(ADC_TypeDef *adc)
{
	adc->CR = 0; // DEEPPWD (deep power down) mode off (reset state is bit29 high).
	adc->CR = 1UL<<28; // Voltage regulator on
	__DSB();
	delay_us(100); // minimum required 10us per datasheet, but let's be sure to get the calibration values accurate.
	adc->CR = 1UL<<28 /* Keep Vreg on*/ | 1UL<<16 /* Calibrate for nonlinearity as well */;
	adc->CR |= 1UL<<31; // Start calibration
	__DSB();
	delay_us(1);
	while(adc->CR & (1UL<<31)) ; // Wait for calibration success
	adc->ISR = 1UL; // Clear ADRDY
	adc->CR |= 1UL; // Enable ADC
	__DSB();
	while(adc->ISR & 1UL) ; // Wait for enable success
	adc->CR |= 1UL<<8 /*BOOST bit for fADC>20MHz*/;
}

#define TRIG_SW         (0b00UL<<10)
#define TRIG_RISING     (0b01UL<<10)
#define TRIG_FALLING    (0b10UL<<10)
#define TRIG_BOTH       (0b11UL<<10)
#define TRIG_EVENT(_x_) ((_x_)<<5)
#define CONTINUOUS      (1UL<<13)

#define EN_AWD1         (1UL<<23)
#define AWD1_ON_SINGLE_CHAN (1UL<<22)
#define AWD1_CHAN(_x_) ((_x_)<<26)

#define DISCON          (1UL<<16)
#define DISCLEN(x_)     ((x_)<<17)

#define RESO_16B (0b000UL<<2)
#define RESO_14B (0b001UL<<2)
#define RESO_12B (0b010UL<<2)
#define RESO_10B (0b011UL<<2)
#define RESO_8B  (0b100UL<<2)

#define DMA_CIRCULAR 0b11UL
#define DMA_ONESHOT 0b01UL

#define AWD1IE (1UL<<7)
#define AWD2IE (1UL<<8)
#define AWD3IE (1UL<<9)

#define ADSTART (1UL<<2)

void adc_test()
{

	uart_print_string_blocking("\r\nADC1:\r\n");
	for(int i=0; i < ADC1_SEQ_LEN; i++)
	{
		uart_print_string_blocking(adc1_names[i]);
		uart_print_string_blocking(" = ");
		o_utoa16(adc1.b[i], printbuf); uart_print_string_blocking(printbuf);
		uart_print_string_blocking("\r\n");
	}

	uart_print_string_blocking("\r\nADC2:\r\n");
	for(int i=0; i < ADC2_SEQ_LEN; i++)
	{
		uart_print_string_blocking(adc2_names[i]);
		uart_print_string_blocking(" = ");
		o_utoa16(adc2.b[i], printbuf); uart_print_string_blocking(printbuf);
		uart_print_string_blocking("\r\n");
	}

	uart_print_string_blocking("\r\nADC3:\r\n");
	for(int i=0; i < ADC3_SEQ_LEN; i++)
	{
		uart_print_string_blocking(adc3_names[i]);
		uart_print_string_blocking(" = ");
		o_utoa16(adc3.b[i], printbuf); uart_print_string_blocking(printbuf);
		uart_print_string_blocking("\r\n");
	}
	delay_ms(100);
}

void init_adcs()
{

	RCC->AHB1ENR |= 1UL<<5; // ADC1,2
	RCC->AHB4ENR |= 1UL<<24; // ADC3
	__DSB();

	IO_TO_ANALOG(GPIOF,11);
	IO_TO_ANALOG(GPIOA,7);
	IO_TO_ANALOG(GPIOA,0);
	IO_TO_ANALOG(GPIOA,1);
	IO_TO_ANALOG(GPIOC,4);
	IO_TO_ANALOG(GPIOA,3);
	IO_TO_ANALOG(GPIOA,2);
	IO_TO_ANALOG(GPIOA,4);
	IO_TO_ANALOG(GPIOC,2);
	IO_TO_ANALOG(GPIOC,5);
	IO_TO_ANALOG(GPIOB,0);
	IO_TO_ANALOG(GPIOB,1);
	IO_TO_ANALOG(GPIOF,3);
	IO_TO_ANALOG(GPIOF,10);
	IO_TO_ANALOG(GPIOC,0);
	IO_TO_ANALOG(GPIOC,3);
	IO_TO_ANALOG(GPIOH,3);
	IO_TO_ANALOG(GPIOH,4);
	IO_TO_ANALOG(GPIOH,5);
	IO_TO_ANALOG(GPIOH,2);


	// Clear DEEPPWD
	// ADVREGEN = 1
	// Wait for TADCVREG_STUP (10 us min)
	
	// ADCALDIF = 0
	// ADCALLIN = 1
	// ADCAL = 1
	// Wait until ADCAL goes 0
	// ADC_ISR write 1 to ADRDY
	// ADEN = 1
	// Wait until ADRDY goes 1


	init_calib_adc(ADC1);
	CONF_ADC_SEQ(ADC1, ADC1_SEQ_LEN, ADC1_SEQ);
	CONF_ADC_SMPTIMES(ADC1, ADC1_SMPTIMES);
	ADC1->PCSEL = ADC1_CHANNELS_IN_USE;

	ADC1->LTR1 = AWD_VBAT_LO<<(16-ADC_BITS);
	ADC1->HTR1 = AWD_VBAT_HI<<(16-ADC_BITS);

	ADC1->CFGR = 
		EN_AWD1 |
		AWD1_ON_SINGLE_CHAN |
		AWD1_CHAN(18) |  // VBAT
//		CONTINUOUS |
//		TRIG_SW |
		TRIG_RISING |
		TRIG_EVENT(0b01010) | // TIM1_TRGO2
		RESO_14B |
		DISCON |
		DISCLEN(ADC1_DISCONTINUOUS_GROUP_LEN) |
		DMA_CIRCULAR;


	// Analog watchdog 2 enabled on channels (bit index = channel number):
	ADC1->AWD2CR = 1UL<<8; // ch8 = vinbus_meas
	ADC1->LTR2 = AWD_CHA_VINBUS_LO<<(16-ADC_BITS);
	ADC1->HTR2 = AWD_CHA_VINBUS_HI<<(16-ADC_BITS);

	// AWD3 reserved for MC overcurrents
	ADC1->AWD3CR = 0; 
	ADC1->LTR3 = 0;
	ADC1->HTR3 = 65535;


	ADC1->IER = 1UL<<4 /*overrun*/ | AWD1IE | AWD2IE;

	NVIC_SetPriority(ADC1_DMA_STREAM_IRQ, 1);
	NVIC_EnableIRQ(ADC1_DMA_STREAM_IRQ);

	init_calib_adc(ADC2);
	CONF_ADC_SEQ(ADC2, ADC2_SEQ_LEN, ADC2_SEQ);
	CONF_ADC_SMPTIMES(ADC2, ADC2_SMPTIMES);
	ADC2->PCSEL = ADC2_CHANNELS_IN_USE;

	ADC2->CFGR = 
//		EN_AWD1 |
//		AWD1_ON_SINGLE_CHAN |
//		AWD1_CHAN(0) |
//		CONTINUOUS |
		TRIG_RISING |
		TRIG_EVENT(0b10000) | // HRTIM_ADCTRG1
		RESO_14B |
		DISCON |
		DISCLEN(ADC2_DISCONTINUOUS_GROUP_LEN) |
		DMA_CIRCULAR;

	// Analog watchdog 2 enabled on channels (bit index = channel number):
	ADC2->AWD2CR = 0; 
	ADC2->LTR2 = 0;
	ADC2->HTR2 = 65535;

	ADC2->AWD3CR = 0; 
	ADC2->LTR3 = 0;
	ADC2->HTR3 = 65535;

	ADC2->IER = 1UL<<4 /*overrun*/;

	NVIC_SetPriority(ADC_IRQn, 1);
	NVIC_EnableIRQ(ADC_IRQn);

	NVIC_SetPriority(ADC2_DMA_STREAM_IRQ, 1);
	NVIC_EnableIRQ(ADC2_DMA_STREAM_IRQ);

	

	init_calib_adc(ADC3);
	CONF_ADC_SEQ(ADC3, ADC3_SEQ_LEN, ADC3_SEQ);
	CONF_ADC_SMPTIMES(ADC3, ADC3_SMPTIMES);
	ADC3->PCSEL = ADC3_CHANNELS_IN_USE;

	ADC3->CFGR = 
//		EN_AWD1 |
//		AWD1_ON_SINGLE_CHAN |
//		AWD1_CHAN(0) |
		CONTINUOUS |
		TRIG_SW |
//		TRIG_EVENT(0) |
		RESO_14B |
//		DISCON |
		DMA_CIRCULAR;

	// Analog watchdog 2 enabled on channels (bit index = channel number):
	ADC3->AWD2CR = 0; 
	ADC3->LTR2 = 0;
	ADC3->HTR2 = 65535;

	ADC3->AWD3CR = 0; 
	ADC3->LTR3 = 0;
	ADC3->HTR3 = 65535;

	ADC3->IER = 1UL<<4 /*overrun*/;

	NVIC_SetPriority(ADC3_IRQn, 1);
	NVIC_EnableIRQ(ADC3_IRQn);
	NVIC_SetPriority(ADC3_DMA_STREAM_IRQ, 1);
	NVIC_EnableIRQ(ADC3_DMA_STREAM_IRQ);


	ADC1_DMA_STREAM->M0AR = (uint32_t)&adc1.b[0];
	ADC1_DMA_STREAM->PAR = (uint32_t)&ADC1->DR;
	ADC1_DMA_STREAM->NDTR = ADC1_SEQ_LEN;
	ADC1_DMA_STREAM->CR =
		0b01UL<<16 /*med prio*/ | 0b01UL<<13 /*16-bit mem*/ | 0b01UL<<11 /*16-bit periph*/ |
		1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/ | 0b110 /*err interrupts*/ | 1UL<<8 /*circular*/;
	ADC1_DMAMUX();
	DMA_CLEAR_INTFLAGS(ADC1_DMA, ADC1_DMA_STREAM_NUM);
	ADC1_DMA_STREAM->CR |= 1UL;
	__DSB();
	ADC1->CR |= ADSTART;

	ADC2_DMA_STREAM->M0AR = (uint32_t)&adc2.b[0];
	ADC2_DMA_STREAM->PAR = (uint32_t)&ADC2->DR;
	ADC2_DMA_STREAM->NDTR = ADC2_SEQ_LEN;
	ADC2_DMA_STREAM->CR =
		0b01UL<<16 /*med prio*/ | 0b01UL<<13 /*16-bit mem*/ | 0b01UL<<11 /*16-bit periph*/ |
		1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/ | 0b110 /*err interrupts*/ | 1UL<<8 /*circular*/;
	ADC2_DMAMUX();
	DMA_CLEAR_INTFLAGS(ADC2_DMA, ADC2_DMA_STREAM_NUM);
	ADC2_DMA_STREAM->CR |= 1UL;
	__DSB();
	ADC2->CR |= ADSTART;

	ADC3_DMA_STREAM->M0AR = (uint32_t)&adc3.b[0];
	ADC3_DMA_STREAM->PAR = (uint32_t)&ADC3->DR;
	ADC3_DMA_STREAM->NDTR = ADC3_SEQ_LEN;
	ADC3_DMA_STREAM->CR =
		0b01UL<<16 /*med prio*/ | 0b01UL<<13 /*16-bit mem*/ | 0b01UL<<11 /*16-bit periph*/ |
		1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/ | 0b110 /*err interrupts*/ | 1UL<<8 /*circular*/;
	ADC3_DMAMUX();
	DMA_CLEAR_INTFLAGS(ADC3_DMA, ADC3_DMA_STREAM_NUM);
	ADC3_DMA_STREAM->CR |= 1UL;
	__DSB();
	ADC3->CR |= ADSTART;

}
