#include <stdint.h>
#include "adcs.h"
#include "ext_include/stm32h7xx.h"
#include "stm32_cmsis_extension.h"
#include "misc.h"


void pwrswitch_init()
{
	RCC->AHB4ENR |= 1UL<<1;
	LO(GPIOB, 2);
	IO_TO_GPO(GPIOB, 2);
}


// Vgs decays below 7.0V (25.0V, Vbat=18.0V) in 9ms after the pulse generation stops.
// Before the proper timer is initialize, call this frequently enough.
void chargepump_initial_pulsetrain()
{

	// First scope trace: 100 pulses at 10kHz (delay value 8)
	// Scond: 50 pulses at 5kHz (delay value 16)
	// third: 25 pulses at 2.5kHz (delay value 32)
	// 4th: 13 pulses at 1.0kHz (delay value 80)

	// This runs at the low CPU clock (64MHz)
	// The delay functions are calculated for 400MHz CPU.

	// Steady state gate V ripple measurements for different pulses
	// Base voltage = 18.0V
	// Rise times measured to Vgs_min=8V (26.0V)
	// Ripple min&max:
	// 80,80 = 27.2V 29.4V  7.6ms
	// 40,80 = 27.9V 29.3V  7.9ms
	// 20,80 = 27.9V 29.2V  9.8ms
	// 8,80  = 27.9V 28.8V  11.2ms

	// 32,32 = 27.1V 28.5V  4.4ms
	// 16,32 = 27.6V 28.6V  5.3ms
	// 8,32  = 27.7V 28.6V  7.5ms
	// First number is HI-time (FET on, power consumed in pull-up resistor)
	// Second number is LO-time
	// Unit: delay_us argument: 8 units = 50us.

	// Low power consumption is important to minimize drop in precharge resistors.
	// Quick turn-on is preferable, but not paramount.
	// Maximized Vgs is unimportant; the actual timer takes over soon.
	// --> Use 16,32 (100us HI, 200us LO, period=300us, freq=3.33kHz)
	// Min. num pulses after Vgs_min=8V: 18 -> use 30

	// All info based on prototype measurements


	for(int i=0; i<30; i++)
	{
		HI(GPIOB, 2);
		delay_us(16);
		LO(GPIOB, 2);
		delay_us(32);
	}
}

// Shorter pulsetrain to keep the gate charged
// Calling this assumes that the gate is already charget over minimum usable threshold
// Fewer pulses are given than driving from zero.
void chargepump_replenish_pulsetrain()
{
	for(int i=0; i<18; i++)
	{
		HI(GPIOB, 2);
		delay_us(16);
		LO(GPIOB, 2);
		delay_us(32);
	}
}


// 10% duty at 1kHz; charges the gate slower, but keeps it charged with very little power.
// specify the delay length in ms
void chargepump_pulsetrain_low_power(uint32_t del_ms)
{
	while(del_ms--)
	{
		HI(GPIOB, 2);
		delay_us(16);
		LO(GPIOB, 2);
		delay_us(144);
	}
}


static inline void alive_platform_0() __attribute__((always_inline));
static inline void alive_platform_0()
{
	LO(GPIOB, 2);
}

static inline void alive_platform_1() __attribute__((always_inline));
static inline void alive_platform_1()
{
	// Check the actual gate voltage - if below Vbat, it's being pulled to GND by
	// the protection circuit. In this case, don't fight back, stop the charge pump
	// by constant low level.

//	if(recent_adcs.v_bms_mainfet_gate /* */ < 1000 /* */)
//		LO(GPIOB, 2);
//	else
		HI(GPIOB, 2);
}



