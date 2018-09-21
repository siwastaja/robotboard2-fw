#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "ext_include/stm32h7xx.h"
#include "stm32_cmsis_extension.h"

#include "misc.h"
#include "flash.h"
#include "tof_muxing.h"
#include "tof_process.h"
#include "micronavi.h"
#include "adcs.h"
#include "pwrswitch.h"
#include "charger.h"
#include "bldc.h"
#include "imu.h"
#include "audio.h"
#include "sbc_comm.h"
#include "timebase.h"

#include "own_std.h"

#define DBG_UART

#ifdef DBG_UART
void uart_print_string_blocking(const char *buf)
{
	while(buf[0] != 0)
	{
		while((USART1->ISR & (1UL<<7)) == 0) ;
		USART1->TDR = buf[0];
		buf++;
	}
}
#endif


void error(int code)
{
	__disable_irq();
	// Disable all other interrupts, but leave the one required for handling reflashing SPI message.
	__enable_irq();

	int i = 0;
	int o = 0;
	while(1)
	{
		LED_ON();
		delay_ms(120);
		LED_OFF();
		delay_ms(120);

		i++;
		if(i >= code)
		{
			i = 0;
			delay_ms(600);
			o++;

			if(o > 10)
			{
				NVIC_SystemReset();
				while(1);
			}
		}

	}
}

void delay_us(uint32_t i) __attribute__((section(".text_itcm")));
void delay_us(uint32_t i)
{
	i *= 90;
	i -= 7;
	while(i--)
		__asm__ __volatile__ ("nop");
}

void delay_ms(uint32_t i) __attribute__((section(".text_itcm")));
void delay_ms(uint32_t i)
{
	while(i--)
	{
		delay_us(1000);
	}
}

void sbc_comm_test();

void main()
{

	RCC->AHB4ENR |= 0b111111111; // enable GPIOA to GPIOI (J and K do not exist on the device)

	IO_TO_GPO(GPIOC, 13); // LED

	LED_ON();
	delay_ms(20);
	LED_OFF();
	delay_ms(10);

/*
	sys_ck (for CPU, etc.) is pll1_p_ck
	Fvco = Fref * divN
	Fout = Fvco / (postdiv_reg+1)
	VCO must be between 192 and 836MHz
	Multiplier (divN) must be from 4 to 512
	
	Fractional PLL mode not needed - initialize in integer mode
	Input clock = 8 MHz

	Let:
	AHB1 aka hclk1 = 200 MHz   HPRE    = 2
	AHB2 aka hclk2 = 200 MHz  (HPRE    = 2)
	AHB3 aka hclk3 = 200 MHz  (HPRE    = 2)
	APB1 aka pclk1 = 100 MHz   D2PPRE1 = 2
	APB2 aka pclk2 = 100 MHz   D2PPRE2 = 2
	APB3 aka pclk3 = 100 MHz   D1PPRE  = 2
	APB4 aka pclk4 = 100 MHz   D3PPRE  = 2

	Some APB* peripheral kernel clocks are 100MHz max, and cannot be connected to
	PLL1. So, if 200MHz bus on APB1 or APB2 is ever needed, PLL2 needs to be configured
	to drive the kernel clocks for such peripherals. For example, SPI4,5.

	For now, it's simplest to drive these peripherals directly on the bus clock, so we
	use 100MHz bus. It's highly unlikely this would never be an performance issue.

	The most busy/biggest bandwidth SPI is SPI1, which is in APB2. Consider increasing APB2
	to 200MHz.


	PLL1_P: 400MHz: sysck (cpu, etc.)
	PLL1_Q: 100MHz: SPI1,2,3
	PLL2_P: 33.33MHz:  ADC1,2, ADC3
	
*/
	RCC->CR |= 1UL<<16; // HSE clock on
	while(!(RCC->CR & 1UL<<17)) ; // Wait for HSE oscillator stabilization
	delay_ms(1);
	// M prescalers - set to 0 when PLL is disabled. 1 = div by 1 (bypass)
	RCC->PLLCKSELR = 0UL<<20 /* PLL3 M(prescaler)*/ | 
			 1UL<<12 /* PLL2 M(prescaler)*/ | 
			 1UL<<4 /* PLL1 M(prescaler)*/ |
			 0b10UL /*HSE as PLL source*/;
			
	RCC->PLLCFGR =  0b000UL<<22 /*PLL3 divider output enables: R,Q,P */ |
			0b001UL<<19 /*PLL2 divider output enables: R,Q,P */ |
			0b011UL<<16 /*PLL1 divider output enables: R,Q,P */ |
			0b10UL<<10 /*PLL3 input between 4..8MHz*/ |
			0b10UL<<6  /*PLL2 input between 4..8MHz*/ |
			0b10UL<<2  /*PLL1 input between 4..8MHz*/;
	

	RCC->PLL1DIVR = 0UL<<24 /*R*/ | (4UL  -1UL)<<16 /*Q ->pll1_q_ck (100MHz)*/ | (1UL  -1UL)<<9 /*P ->sys_ck (400MHz)  */ | (50UL   -1UL)<<0 /*N  8MHz->400MHz*/;
	RCC->PLL2DIVR = 0UL<<24 /*R*/ | 0UL<<16 /*Q                             */ | (12UL  -1UL)<<9 /*P ->ADC   (33.33MHz)*/ | (50UL   -1UL)<<0 /*N  8MHz->400MHz*/;

	RCC->CR |= 1UL<<24; // PLL1 on
	// Configure other registers while waiting PLL1 stabilization:

	FLASH->ACR = 0b10UL<<4 /*WRHIGHFREQ programming delay: 185..210MHz AXI bus in VOS1 */ |
		     2UL<<0    /*LATENCY = 2 wait states: 185..210MHz in VOS1 */;


	RCC->D1CFGR = 0UL<<8 /*D1CPRE = sysck divider = 1*/ | 0b100UL<<4 /*D1PPRE = 2*/ | 0b1000UL<<0 /*HPRE = 2*/;
	RCC->D2CFGR = 0b100UL<<8 /*D2PPRE2 = 2*/ | 0b100UL<<4 /*D2PPRE1 = 2*/;
	RCC->D3CFGR = 0b100UL<<4 /*D3PPRE = 2*/;

	RCC->AHB1ENR |= 1UL<<1 /*DMA2*/ | 1UL<<0 /*DMA1*/;


	while(!(RCC->CR & 1UL<<25)) ; // Wait for PLL1 ready

	RCC->CFGR |= 0b011; // Change PLL to system clock
	while((RCC->CFGR & (0b111UL<<3)) != (0b011UL<<3)) ; // Wait for switchover to PLL.

	RCC->CR |= 1UL<<26; // PLL2 on
	while(!(RCC->CR & 1UL<<27)) ; // Wait for PLL2 ready

//	RCC->CR |= 1UL<<28; // PLL3 on
//	while(!(RCC->CR & 1UL<<29)) ; // Wait for PLL3 ready

	
	#ifdef DBG_UART

		/*
			USART1 = the raspi USART @ APB2 = 100 MHz
		*/

		RCC->APB2ENR |= 1UL<<4;
		IO_TO_ALTFUNC(GPIOB, 14);
		IO_TO_ALTFUNC(GPIOB, 15);
		IO_SET_ALTFUNC(GPIOB, 14, 4);
		IO_SET_ALTFUNC(GPIOB, 15, 4);
		USART1->BRR = 100000000/115200;
		USART1->CR1 = 0UL<<5 /*RX interrupt*/ | 1UL<<3 /*TX ena*/ | 1UL<<2 /*RX ena*/ |  1UL /*USART ENA*/;

	#endif

	// Enable FPU

//	SCB->CPACR |= 0b1111UL<<20;
//	__DSB();
//	__ISB();


	/*
		Interrupts will have 16 levels of pre-emptive priority.

		Interrupt with more urgent (lower number) pre-emptive priority will interrupt another interrupt running.

		Priority 0 is the highest quick-safety-shutdown level which won't be disabled for atomic operations.
	*/
	NVIC_SetPriorityGrouping(0);
//	NVIC_SetPriority(PVD_IRQn, 0b0000);
//	NVIC_EnableIRQ(PVD_IRQn);


	init_sbc_comm();

	uart_print_string_blocking("No terse\r\n\r\n"); 
	sbc_comm_test();
/*	while(1)
	{

		char printbuf[128];

		extern volatile int spi_test_cnt;
		extern volatile int spi_test_cnt2;
		extern volatile int spi_dbg1, spi_dbg2;
		extern volatile int new_rx_len;

		uart_print_string_blocking("SPI SR = "); o_btoa16_fixed(SPI1->SR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
		uart_print_string_blocking("SPI TSIZE = "); o_utoa16(SPI1->TSIZE, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
		uart_print_string_blocking("tx NDTR = "); o_utoa16(DMA1_Stream0->NDTR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
		uart_print_string_blocking("rx NDTR = "); o_utoa16(DMA1_Stream1->NDTR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
		uart_print_string_blocking("spi_test_cnt = "); o_utoa16(spi_test_cnt, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
		uart_print_string_blocking("spi_test_cnt2 = "); o_utoa16(spi_test_cnt2, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
		uart_print_string_blocking("spi_dbg1 = "); o_utoa16(spi_dbg1, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
		uart_print_string_blocking("spi_dbg2 = "); o_utoa16(spi_dbg2, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
		uart_print_string_blocking("rx_len = "); o_utoa16(new_rx_len, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");

		uart_print_string_blocking("\r\n\r\n");
		delay_ms(100);		
	}*/

}
