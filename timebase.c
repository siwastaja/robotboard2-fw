/*

Timebase: 10kHz handler

*/

#include <stdint.h>
#include "ext_include/stm32h7xx.h"
#include "stm32_cmsis_extension.h"
#include "misc.h"
#include "charger.h"
#include "pwrswitch.h"
#include "adcs.h"
#ifdef EXT_VACUUM
	#include "ext_vacuum_boost.h"
#endif
#include "audio.h"

volatile uint32_t cnt_100us;

static int cnt;

extern volatile int main_power_enabled;

void timebase_alternative_inthandler() __attribute__((section(".text_itcm")));
void timebase_alternative_inthandler()
{
	TIM5->SR = 0;
	__DSB();

	cnt++;

	if(cnt == 7)
	{
		if(main_power_enabled) 
			PLAT_CP_HI();
		pwrswitch_1khz();
	}
	else if(cnt >= 10)
	{
		PLAT_CP_LO();
		cnt = 0;
	}
}


void timebase_inthandler() __attribute__((section(".text_itcm")));
void timebase_inthandler()
{
	TIM5->SR = 0;
	__DSB();

	cnt++;

	// For performance, this loop runs the charge pumps as well:
	// See test report in pwrswitch.c - (250us HI, 500us LO) provided the highest Vgs. We'll round that up to 300us HI, 700us LO

	extern volatile int app_power_enabled;
	if(cnt == 6)
	{
		#ifdef EXT_VACUUM
			ext_vacuum_fsm();
		#endif
	}
	else if(cnt == 7)
	{
		if(main_power_enabled) 
			PLAT_CP_HI();

		if(app_power_enabled)
			APP_CP_HI();

		pwrswitch_1khz();
	}
	else if(cnt == 8)
	{
	}
	else if(cnt == 9)
	{
		charger_1khz();

		if(app_power_enabled)
			ADC3_CONV_INJECTED();
	}
	else if(cnt >= 10)
	{
		PLAT_CP_LO();
		if(app_power_enabled)
		{
			APP_CP_LO();
			int vg = VGAPP_MEAS_TO_MV(ADC3_VGAPP_DATAREG);
			int vbat = VBAT_MEAS_TO_MV(adc1.s.vbat_meas);
			if(vg < vbat+7000)
			{
				app_power_enabled = 0;
			}
		}
		cnt = 0;
	}

	cnt_100us++;

	// Keep all 10kHz functions in ITCM!
	charger_10khz();
	audio_10khz();
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

	NVIC_SetPriority(TIM5_IRQn, INTPRIO_TIMEBASE);
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
