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

void beep_blocking(int pulses, int us, int volume)
{
	if(volume > 2000) volume = 2000; else if(volume < 50) volume = 50;

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
