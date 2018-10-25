/*

Timebase: 10kHz handler

*/

#include <stdint.h>
#include "ext_include/stm32h7xx.h"
#include "stm32_cmsis_extension.h"
#include "misc.h"
#include "charger.h"
#include "pwrswitch.h"

volatile uint32_t ms_cnt;

volatile int main_power_enabled = 2;

void timebase_inthandler() __attribute__((section(".text_itcm")));
void timebase_inthandler()
{
	static int pwrswitch_cnt;
	static int cnt;
	TIM5->SR = 0;
	__DSB();

	cnt++;

	// For performance, reuse this loop to run the charge pumps as well:

	// See test report in pwrswitch.c - (250us HI, 500us LO) provided the highest Vgs. We'll round that up to 300us HI, 700us LO)

	if(cnt == 7)
	{
		if(main_power_enabled) 
			PLAT_CP_HI();

		if(PWRSWITCH_PRESSED)
		{
			pwrswitch_cnt++;
			if(main_power_enabled>1 && pwrswitch_cnt > 100)
			{
				main_power_enabled = 1;
			}
			else if(pwrswitch_cnt > 1000)
			{
				main_power_enabled = 0;
			}
		}
		else
		{
			if(pwrswitch_cnt > 16) pwrswitch_cnt-= 16;
		}

		if(main_power_enabled > 1)
		{
			LED_ON();
		}
		else if(main_power_enabled == 1)
		{
			if(ms_cnt & (1UL<<8))
				LED_ON();
			else
				LED_OFF();
		}
		else
		{
			LED_OFF();
		}



	}
	else if(cnt >= 10)
	{
		PLAT_CP_LO();
		ms_cnt++;
		cnt = 0;
	}


	// Keep all functions in ITCM!
	charger_10khz();
}

void init_timebase()
{

	/*
		TIM5 @ APB1. D2PPRE1 = 0b100 (/2), and TIMPRE=0, hence
		counter runs at double the APB1 clk = 100*2 = 200 MHz.

		Note that if the bus clock is doubled to 200MHz by reducing
		the D2PPRE1 divider to 1, everything works automatically,
		and the counter still runs at 200MHz.
	*/

	RCC->APB1LENR |= 1UL<<3;

	TIM5->DIER |= 1UL; // Update interrupt
	TIM5->ARR = 20000-1; // 200MHz -> 10 kHz
	TIM5->CR1 |= 1UL; // Enable

	NVIC_SetPriority(TIM5_IRQn, 10);
	NVIC_EnableIRQ(TIM5_IRQn);
}

void deinit_timebase()
{
	TIM5->DIER = 0;
	TIM5->CR1 = 0;
	TIM5->SR = 0;

	NVIC_DisableIRQ(TIM5_IRQn);
	RCC->APB1LENR &= ~(1UL<<3);
}
