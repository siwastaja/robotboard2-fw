/*

Timebase: 10kHz handler for running multitasking state machines

*/

#include <stdint.h>
#include "ext_include/stm32h7xx.h"
#include "stm32_cmsis_extension.h"

void timebase_handler()
{

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
	TIM5->ARR = 19999; // 200MHz -> 10 kHz
	TIM5->CR1 |= 1UL; // Enable

	NVIC_SetPriority(TIM5_IRQn, 14);
	NVIC_EnableIRQ(TIM5_IRQn);
}

void deinit_timebase()
{
	TIM5->DIER = 0;
	TIM5->CR1 = 0;
	NVIC_DisableIRQ(TIM5_IRQn);
	RCC->APB1LENR &= ~(1UL<<3);
}
