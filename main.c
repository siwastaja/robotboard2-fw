#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "ext_include/stm32h7xx.h"
#include "stm32_cmsis_extension.h"

#define LED_ON()  HI(GPIOC, 13);
#define LED_OFF() LO(GPIOC, 13);

void main()
{

	AXI_TARG7_FN_MOD |= 1UL<<0; // From device errata sheet: SRAM connectivity is broken and data will be randomly corrupted unless this bit is set.

	RCC->AHB4ENR |= 0b111111111; // GPIOA to GPIOI (J and K do not exist on the device)

	IO_TO_GPO(GPIOC, 13);

	while(1)
	{
		LED_ON();
		delay_ms(100);
		LED_OFF();
		delay_ms(100);
	}

#if 0
	RCC->APB1ENR |= 1UL<<28; // Power interface clock enable - needed to configure PWR registers
	RCC->PLLCFGR = 9UL<<24 /*Q*/ | 1UL<<22 /*HSE as source*/ | 0b00UL<<16 /*P=2*/ | 216UL<<6 /*N*/ | 4UL /*M*/;

	RCC->CR |= 1UL<<16; // HSE clock on
	RCC->CR |= 1UL<<24; // PLL on

	PWR->CR1 |= 1UL<<16; // Overdrive on
	while(!(PWR->CSR1 & (1UL<<16))) ; // Wait for overdrive ready
	PWR->CR1 |= 1UL<<17; // Switch to overdrive
	while(!(PWR->CSR1 & (1UL<<17))) ; // Wait for overdrive switching ready


	FLASH->ACR = 1UL<<9 /* ART accelerator enable (caches) */ | 1UL<<8 /*prefetch enable*/ | 7UL /*7 wait states*/;
	RCC->CFGR = 0b100UL<<13 /*APB2 div 2*/ | 0b101UL<<10 /*APB1 div 4*/;
	RCC->DCKCFGR2 = 0b01UL<<4 /*USART3 = sysclk*/ | 0b00UL<<22 /*I2C4 = APB1clk*/ | 0b00UL<<20 /*I2C3 = APB1clk*/;

	while(!(RCC->CR & 1UL<<25)) ; // Wait for PLL
	RCC->CFGR |= 0b10; // Change PLL to system clock
	while((RCC->CFGR & (0b11UL<<2)) != (0b10UL<<2)) ; // Wait for switchover to PLL.

#endif

	// Enable FPU

//	SCB->CPACR |= 0b1111UL<<20;
//	__DSB();
//	__ISB();


	/*
		Interrupts will have 4 levels of pre-emptive priority, and 4 levels of sub-priority.

		Interrupt with more urgent (lower number) pre-emptive priority will interrupt another interrupt running.

		If interrupts with similar or lower urgency happen during another ISR, the subpriority level will decide
		the order they will be run after finishing the more urgent ISR first.
	*/
//	NVIC_SetPriorityGrouping(2);
//	NVIC_SetPriority(PVD_IRQn, 0b0000);
//	NVIC_EnableIRQ(PVD_IRQn);

}
