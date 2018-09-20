#include <stdint.h>
#include "ext_include/stm32h7xx.h"
#include "stm32_cmsis_extension.h"
#include "misc.h"
#include "flash.h"

/*
define SPI_DMA_1BYTE_AT_TIME to force the SPI communication peripheral & DMA use 8-bit transfers. 
Bus bandwidth usage will be 4 times more, but transfer sizes not multiples of four will work
perfectly.

If not defined, 32-bit accessess will be used. For maximized simplicity, message sizes need to be
multiplies of four bytes, in this case. Nothing dramatic will happen with non-conforming messages,
but the reported rx length is rounded down %4. Overindexing of the buffers may happen if you are
not careful, in this case. For example, if the transfer length is 5, 8 bytes will be transferred
by the DMA.

*/
//#define SPI_DMA_1BYTE_AT_TIME

enum {RASPI=0, ODROID=1} sbc_iface;

volatile uint8_t raspi_tx[32768] __attribute__((aligned(4)));
volatile uint8_t raspi_rx[32768] __attribute__((aligned(4)));

#if !defined(SBC_RASPI) && !defined(SBC_ODROID) && !defined(SBC_AUTODETECT)
	#error "You need to define SBC_RASPI, SBC_ODROID or SBC_AUTODETECT"
#endif

#if defined(SBC_RASPI) && defined(SBC_ODROID)
	#error "Define either SBC_RASPI or SBC_ODROID, not both"
#endif

#if defined(SBC_AUTODETECT) && (defined(SBC_RASPI) || defined(SBC_ODROID))
	#error "With SBC_AUTODETECT defined, don't define SBC_RASPI or SBC_ODROID"
#endif

#ifdef SBC_AUTODETECT
static void wait_detect_sbc()
{
	// Chip select signals are pulled down:
	IO_TO_GPI(GPIOG, 10);
	IO_TO_GPI(GPIOA, 15);
	IO_PULLDOWN_ON(GPIOG, 10);
	IO_PULLDOWN_ON(GPIOA, 15);

	// Then, we'll see which one is pulled high (inactive level) by the connected single-board computer:

	int blink_cnt = 0;
	while(1)
	{
		delay_ms(1);
		blink_cnt++;
		if(blink_cnt == 1)
			LED_ON();
		else if(blink_cnt == 100)
			LED_OFF();
		else if(blink_cnt == 200)
			blink_cnt = 0;

		int raspi_high = IN(GPIOG, 10);
		int odroid_high = IN(GPIOA, 15);

		if(raspi_high && !odroid_high)
		{
			sbc_iface = RASPI;
			break;
		}
		else if(odroid_high && !raspi_high)
		{
			sbc_iface = ODROID;
			break;
		}
		// else: keep waiting for valid situation (only either one high)
	}

	LED_OFF();

	IO_PULLUPDOWN_OFF(GPIOG, 10);
	IO_PULLUPDOWN_OFF(GPIOA, 15);
}
#endif

volatile int spi_test_cnt2;

void sbc_spi_eot_inthandler()
{
	SPI1->IFCR = 1UL<<3; // clear the intflag
	__DSB();
	spi_test_cnt2++;
}

/*
	Okay, so STM32 SPI _still_ doesn't _actually_ support any kind of nSS pin hardware management in slave mode, even when they are talking about
	it like that.

	This was the case with the previous incarnation of the SPI - now they have made it more complex, added a separate event/interrupt
	source exactly to signify end-of-transfer in slave mode as well - but for some reason, they still don't delimit the transfer by nCS
	(which is exactly designed for this purpose!), go figure. Instead, you need to know the transfer size beforehand, and a counter is
	used to give the interrupt. Of course, with our variable-length messaging, this isn't going to work.

	This is why we still use a simple EXTI interrupt from the nCS pin going high, and we still need to circumvent the whole SPI/DMA transfer
	management.

	They have fixed something - now you don't need to reset the SPI through the RCC registers anymore. Disabling SPI through the SPE bit
	flushes the FIFOs and resets the internal logic. Great!
	


	When the SPI is enabled with DMA, it always generates DMA requests right away to fill its TX fifo. When the
	nSS is asserted half a year later, the first data are from half a year ago, rest is DMA'ed from the memory at that point.
	With our communication FIFO scheme, this isn't a problem, but do remember this.

*/

volatile int spi_test_cnt;
volatile int new_rx;
volatile int new_rx_len;

volatile int spi_dbg1, spi_dbg2;

void sbc_spi_cs_end_inthandler()
{
	spi_dbg1 = DMA1_Stream0->NDTR;
	spi_dbg2 = DMA1_Stream1->NDTR;
	// Triggered when cs goes high

	// Clear the EXTI interrupt pending bit:
	if(sbc_iface == ODROID)
		EXTI_D1->PR1 = 1UL<<15;
	else // RASPI
		EXTI_D1->PR1 = 1UL<<10;

	spi_test_cnt++;

	SPI1->CR1 = 0; // Disable and reset the SPI - FIFO flush happens

	// Reference manual wants us to keep TX DMA EN bit disabled, and enable it at the precise moment (see below).
	// Zero problems so far doing it just at once (in init). But if you are having issues such as excess TX DMA events,
	// this is a possible cause.
//#ifdef SPI_DMA_1BYTE_AT_TIME
//	SPI1->CFG1 = 1UL<<14 /*RX DMA EN*/ |(1UL -1UL)<<5 /*FIFO threshold*/ | 0b00111UL /*8-bit data*/;  // don't set TX DMA EN yet 
//#else
//	SPI1->CFG1 = 1UL<<14 /*RX DMA EN*/ |(4UL -1UL)<<5 /*FIFO threshold*/ | 0b00111UL /*8-bit data*/;  // don't set TX DMA EN yet 
//#endif
	__DSB();

	// Disable the DMAs:
	DMA1_Stream0->CR = 0b01UL<<16 /*med prio*/ | 0UL<<8 /*circular OFF*/ |
		#ifdef SPI_DMA_1BYTE_AT_TIME
			   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
		#else
			   0b10UL<<13 /*32-bit mem*/ | 0b10UL<<11 /*32-bit periph*/ |
		#endif
	                   1UL<<10 /*mem increment*/ | 0b01UL<<6 /*mem-to-periph*/;

	DMA1_Stream1->CR = 0b01UL<<16 /*med prio*/ | 0UL<<8 /*circular OFF*/ |
		#ifdef SPI_DMA_1BYTE_AT_TIME
			   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
		#else
			   0b10UL<<13 /*32-bit mem*/ | 0b10UL<<11 /*32-bit periph*/ |
		#endif
	                   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/;

	__DSB();

	while(DMA1_Stream0->CR & 1UL) ;
	while(DMA1_Stream1->CR & 1UL) ;

	new_rx = 1;


	// Check if there is the maintenance magic code:

	if(*((volatile uint32_t*)&raspi_rx[0]) == 0x9876fedb)
	{
		run_flasher();
	}

	// TX DMA
	#ifdef SPI_DMA_1BYTE_AT_TIME
		DMA1_Stream0->NDTR = sizeof(raspi_tx);
	#else
		DMA1_Stream0->NDTR = sizeof(raspi_tx)/4;
	#endif

	DMA_CLEAR_INTFLAGS(DMA1, 0);
	DMA1_Stream0->CR |= 1; // Enable DMA

	// RX DMA

	#ifdef SPI_DMA_1BYTE_AT_TIME
		new_rx_len = sizeof(raspi_rx) - DMA1_Stream1->NDTR;
		DMA1_Stream1->NDTR = sizeof(raspi_rx);
	#else
		new_rx_len = sizeof(raspi_rx) - DMA1_Stream1->NDTR*4;
		DMA1_Stream1->NDTR = sizeof(raspi_rx)/4;
	#endif
	__DSB();
	DMA_CLEAR_INTFLAGS(DMA1, 1);
	DMA1_Stream1->CR |= 1; // Enable DMA

//	SPI1->CFG1 |= 1UL<<15 /*TX DMA ena*/; // not earlier, if you want to follow the refman advice


	SPI1->CR1 = 1UL; // Enable in slave mode
	__DSB();
}


void init_sbc_comm()
{

	uint8_t tmp = 0x42;
	for(int i=0; i<sizeof(raspi_tx); i++)
		raspi_tx[i] = tmp++;

	RCC->APB2ENR |= 1UL<<12; // SPI1

	#ifdef SBC_AUTODETECT
		wait_detect_sbc();
	#endif

	#ifdef SBC_ODROID
		sbc_iface = ODROID;
	#endif

	#ifdef SBC_RASPI
		sbc_iface = RASPI;
	#endif

	if(sbc_iface == ODROID)
	{
		IO_ALTFUNC(GPIOB,5,  5);
		IO_ALTFUNC(GPIOB,4,  5);
		IO_ALTFUNC(GPIOB,3,  5);
		IO_ALTFUNC(GPIOA,15, 5);
		IO_SPEED(GPIOB,4, 3);

		IO_PULLUP_ON(GPIOA, 15);
	}
	else // RASPI
	{
		IO_ALTFUNC(GPIOG,9,  5);
		IO_ALTFUNC(GPIOD,7,  5);
		IO_ALTFUNC(GPIOG,11, 5);
		IO_ALTFUNC(GPIOG,10, 5);
		IO_SPEED(GPIOG,9, 3);

		IO_PULLUP_ON(GPIOG, 10);
	}


	// Initialization order from reference manual:
#ifdef SPI_DMA_1BYTE_AT_TIME
	SPI1->CFG1 = 1UL<<14 /*RX DMA EN*/ |(1UL -1UL)<<5 /*FIFO threshold*/ | 0b00111UL /*8-bit data*/;  // don't set TX DMA EN yet 
#else
	SPI1->CFG1 = 1UL<<14 /*RX DMA EN*/ |(4UL -1UL)<<5 /*FIFO threshold*/ | 0b00111UL /*8-bit data*/;  // don't set TX DMA EN yet 
#endif
	// SPI1->CFG2 defaults good

	// SPI1->CR2 zero - transfer length unknown

	// TX DMA
	DMA1_Stream0->PAR = (uint32_t)&(SPI1->TXDR);
	DMA1_Stream0->M0AR = (uint32_t)&raspi_tx;
	DMA1_Stream0->CR = 0b01UL<<16 /*med prio*/ | 0UL<<8 /*circular OFF*/ |
		#ifdef SPI_DMA_1BYTE_AT_TIME
			   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
		#else
			   0b10UL<<13 /*32-bit mem*/ | 0b10UL<<11 /*32-bit periph*/ |
		#endif
	                   1UL<<10 /*mem increment*/ | 0b01UL<<6 /*mem-to-periph*/;

	#ifdef SPI_DMA_1BYTE_AT_TIME
		DMA1_Stream0->NDTR = sizeof(raspi_tx);
	#else
		DMA1_Stream0->NDTR = sizeof(raspi_tx)/4;
	#endif
	DMAMUX1_Channel0->CCR = 38;
	DMA_CLEAR_INTFLAGS(DMA1, 0);
	DMA1_Stream0->CR |= 1; // Enable TX DMA

	// RX DMA

	DMA1_Stream1->PAR = (uint32_t)&(SPI1->RXDR);
	DMA1_Stream1->M0AR = (uint32_t)(raspi_rx);
	DMA1_Stream1->CR = 0b01UL<<16 /*med prio*/ | 0UL<<8 /*circular OFF*/ |
		#ifdef SPI_DMA_1BYTE_AT_TIME
			   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
		#else
			   0b10UL<<13 /*32-bit mem*/ | 0b10UL<<11 /*32-bit periph*/ |
		#endif
	                   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/;

	#ifdef SPI_DMA_1BYTE_AT_TIME
		DMA1_Stream1->NDTR = sizeof(raspi_rx);
	#else
		DMA1_Stream1->NDTR = sizeof(raspi_rx)/4;
	#endif
	DMAMUX1_Channel1->CCR = 37;
	DMA_CLEAR_INTFLAGS(DMA1, 1);
	DMA1_Stream1->CR |= 1; // Enable RX DMA

	SPI1->CFG1 |= 1UL<<15 /*TX DMA ena*/; // not earlier!

//	SPI1->IER |= 1UL<<3; // EOT,SUSP,TXC interrupt enable
//	NVIC_SetPriority(SPI1_IRQn, 2);
//	NVIC_EnableIRQ(SPI1_IRQn);

//	SPI1->TSIZE = 65528;
	SPI1->CR1 = 1UL; // Enable in slave mode


	// Chip select is hardware managed for rx start - but ending must be handled by software. We use EXTI for that.
	// Maybe we won't use nCS to delimit the packet end, since it clearly isn't supported properly in the HW
	// For now, let's not use EXTI. The code is here, anyway:

	if(sbc_iface == ODROID)
	{
		// nCS is PA15, so EXTI15 must be used.
		REG_WRITE_PART(SYSCFG->EXTICR[/*refman idx:*/4   -1], 12, 0b1111UL, 0b0000UL);
		EXTI->RTSR1 |= 1UL<<15; // Get the rising edge interrupt
		EXTI_D1->IMR1 |= 1UL<<15; // Enable CPU interrupt
	}
	else // RASPI
	{
		// nCS is PG10, so EXTI10 must be used.
		REG_WRITE_PART(SYSCFG->EXTICR[/*refman idx:*/3   -1], 8, 0b1111UL, 0b0110UL);
		EXTI->RTSR1 |= 1UL<<10; // Get the rising edge interrupt
		EXTI_D1->IMR1 |= 1UL<<10; // Enable CPU interrupt
	}

	// The interrupt priority must be fairly high, to quickly reconfigure the DMA so that if the master pulls CSn low
	// again very quickly to make a new transaction, we have the DMA up and running for that before the super small 4-byte
	// FIFO in the SPI is exhausted.
	// Both RASPI (EXTI10) and ODROID (EXTI15) happen to coincide to the same multiplexed EXTI interrupt group (15,14,13,12,11,10).
	NVIC_SetPriority(EXTI15_10_IRQn, 2);
	NVIC_EnableIRQ(EXTI15_10_IRQn);

}

void deinit_sbc_comm()
{
	IO_TO_GPI(GPIOB,5);
	IO_TO_GPI(GPIOB,4);
	IO_TO_GPI(GPIOB,3);
	IO_TO_GPI(GPIOA,15);
	IO_TO_GPI(GPIOG,9);
	IO_TO_GPI(GPIOD,7);
	IO_TO_GPI(GPIOG,11);
	IO_TO_GPI(GPIOG,10);

	EXTI->RTSR1 &= ~(1UL<<15 | 1UL<<10);
	EXTI_D1->IMR1 &= ~(1UL<<15 | 1UL<<10);

}
