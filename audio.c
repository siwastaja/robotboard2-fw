#include <stdint.h>
#include "ext_include/stm32h7xx.h"
#include "stm32_cmsis_extension.h"
#include "misc.h"

#define AUDIO_DAC (DAC1->DHR12R2)

void init_audio()
{
	RCC->APB1LENR |= 1UL<<29; // Enable DAC1,2 clock. Only DAC2 is used by us; DAC1 may be used by others (charger 
				  // when writing this comment)

	IO_TO_GPO(GPIOH, 12);

	// DAC2: Audio (don't touch channel 1, beware: configuration registers are shared)
//	DAC1->MCR |= 0b000UL<<16 /*Channel 1: Output buffer*/;
	AUDIO_DAC = 2048;
	DAC1->CR |= 1UL<<16; // Enable channel 1

	IO_TO_ANALOG(GPIOA, 5);

}


static int state;
static int vol;
static int len_100us;
static int cnt_100us;
static int cur_interval;
static int interval_step;

void audio_10khz() __attribute__((section(".text_itcm")));
void audio_10khz()
{
	static int interval_cnt;
	static int dir;
	if(state == 1)
	{
		HI(GPIOH, 12);
		interval_cnt = 0;
		dir = 0;
		AUDIO_DAC = 2048;
		state++;
	}
	else if(state > 1 && state < 100)
	{
		state++;
	}
	else if(state == 100)
	{
		cnt_100us++;
		if(cnt_100us >= len_100us)
		{
			LO(GPIOH, 12);
			state = 0;
		}
		else
		{
			int cur_interval_cmp = cur_interval>>16;
			if(cur_interval_cmp < 1) cur_interval_cmp = 1;

			interval_cnt++;
			if(dir==0 && interval_cnt >= cur_interval_cmp)
			{
				AUDIO_DAC = 2048-vol;
				dir = 1;
				interval_cnt = 0;
			}
			else if(dir==1 && interval_cnt >= cur_interval_cmp)
			{
				AUDIO_DAC = 2048+vol;
				dir = 0;
				interval_cnt = 0;
			}

			cur_interval += interval_step;
		}
	}
}


void beep(int len_ms, int hz_start, int sweep, int volume) __attribute__((section(".text_itcm")));
void beep(int len_ms, int hz_start, int sweep, int volume) // len milliseconds, hz initial freq, sweep: positive sweeps down, negative sweeps up, volume 0-100
{
	volume *= 10;
	if(volume > 700) volume = 700; else if(volume < 50) volume = 50;
	state = 1;

	vol = volume;
	cur_interval = (5000*65536)/hz_start;
	len_100us = len_ms*10;
	interval_step = sweep;
	cnt_100us = 0;
}


void beep_test()
{
	while(1)
	{
		beep(500, 200, 0, 600);
		delay_ms(1000);
		beep(500, 200, 0, 600);
		delay_ms(1000);
		beep(500, 200, +20, 600);
		delay_ms(1000);
		beep(500, 200, -20, 600);
		delay_ms(1000);
		beep(500, 200, +80, 600);
		delay_ms(1000);
		beep(500, 200, -80, 600);
		delay_ms(1000);
		beep(500, 200, +320, 600);
		delay_ms(1000);
		beep(500, 200, -320, 600);

		delay_ms(1000);

		beep(500, 800, 0, 600);
		delay_ms(1000);
		beep(500, 800, 0, 600);
		delay_ms(1000);
		beep(500, 800, +20, 600);
		delay_ms(1000);
		beep(500, 800, -20, 600);
		delay_ms(1000);
		beep(500, 800, +80, 600);
		delay_ms(1000);
		beep(500, 800, -80, 600);
		delay_ms(1000);
		beep(500, 800, +320, 600);
		delay_ms(1000);
		beep(500, 800, -320, 600);
		delay_ms(1000);
	}
}

void beep_blocking(int pulses, int us, int volume)
{
	volume /= 2;
	if(volume > 700) volume = 700; else if(volume < 50) volume = 50;

	AUDIO_DAC = 2048;
	HI(GPIOH, 12);
	delay_ms(50);
	for(int i=0; i<pulses; i++)
	{
		AUDIO_DAC = 2048-volume;
		delay_us(us);
		AUDIO_DAC = 2048+volume;
		delay_us(us);
	}
	LO(GPIOH, 12);
}
