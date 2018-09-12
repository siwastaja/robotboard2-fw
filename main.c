#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "ext_include/stm32h7xx.h"
#include "stm32_cmsis_extension.h"

#define LED_ON()  HI(GPIOC, 13);
#define LED_OFF() LO(GPIOC, 13);


void error(int code)
{
	while(1);
}

//void delay_us(uint32_t i) __attribute__((section(".text_itcm")));
void delay_us(uint32_t i)
{
	if(i==0) return;
	i *= 90;
	i -= 7;
	while(i--)
		__asm__ __volatile__ ("nop");
}

//void delay_ms(uint32_t i) __attribute__((section(".text_itcm")));
void delay_ms(uint32_t i)
{
	while(i--)
	{
		delay_us(1000);
	}
}

void main()
{

	RCC->AHB4ENR |= 0b111111111; // enable GPIOA to GPIOI (J and K do not exist on the device)

	IO_TO_GPO(GPIOC, 13);

	LED_ON();
	delay_ms(20);
	LED_OFF();
	delay_ms(10);


	// sys_ck (for CPU, etc.) is pll1_p_ck
	// Fvco = Fref * divN
	// Fout = Fvco / (postdiv_reg+1)
	// VCO must be between 192 and 836MHz
	// Multiplier (divN) must be from 4 to 512
	
	// Fractional PLL mode not needed - initialize in integer mode
	// Input clock = 8 MHz

	RCC->CR |= 1UL<<16; // HSE clock on
	while(!(RCC->CR & 1UL<<17)) ; // Wait for HSE oscillator stabilization
	delay_ms(1);
	// M prescalers - set to 0 when PLL is disabled. 1 = div by 1 (bypass)
	RCC->PLLCKSELR = 0UL<<20 /* PLL3 M(prescaler)*/ | 
			 0UL<<12 /* PLL2 M(prescaler)*/ | 
			 0UL<<4 /* PLL1 M(prescaler)*/ |
			 0b10UL /*HSE as PLL source*/;
			
	RCC->PLLCFGR =  0b000UL<<22 /*PLL3 divider output enables: R,Q,P */ |
			0b000UL<<19 /*PLL2 divider output enables: R,Q,P */ |
			0b001UL<<16 /*PLL1 divider output enables: R,Q,P */ |
			0b10UL<<10 /*PLL3 input between 4..8MHz*/ |
			0b10UL<<6  /*PLL2 input between 4..8MHz*/ |
			0b10UL<<2  /*PLL1 input between 4..8MHz*/;
	

	RCC->PLL1DIVR = 0UL<<24 /*R*/ | 2UL<<16 /*Q ->pll1_q_ck (200MHz)*/ | 1UL<<9 /*P ->sys_ck (400MHz)*/ | (50UL   -1UL)<<0 /*N  8MHz->400MHz*/;

	RCC->CR |= 1UL<<24; // PLL1 on
	// Configure other registers while waiting PLL1 stabilization:

	FLASH->ACR = 0b10UL<<4 /*WRHIGHFREQ programming delay: 185..210MHz AXI bus in VOS1 */ |
		     2UL<<0    /*LATENCY = 2 wait states: 185..210MHz in VOS1 */;

	while(!(RCC->CR & 1UL<<25)) ; // Wait for PLL1 ready

	RCC->CFGR |= 0b011; // Change PLL to system clock
	while((RCC->CFGR & (0b111UL<<3)) != (0b011UL<<2)) ; // Wait for switchover to PLL.



//	RCC->CR |= 1UL<<26; // PLL2 on
//	while(!(RCC->CR & 1UL<<27)) ; // Wait for PLL2 ready

//	RCC->CR |= 1UL<<28; // PLL2 on
//	while(!(RCC->CR & 1UL<<29)) ; // Wait for PLL2 ready

	



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
