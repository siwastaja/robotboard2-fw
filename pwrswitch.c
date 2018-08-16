#include <stdint.h>
#include "adcs.h"

void pwrswitch_init()
{
	LO(GPIOB, 2);
	IO_TO_GPO(GPIOB, 2);
}

void alive_platform_0()
{
	LO(GPIOB, 2);
}

void alive_platform_1()
{
	// Check the actual gate voltage - if below Vbat, it's being pulled to GND by
	// the protection circuit. In this case, don't fight back, stop the charge pump
	// by constant low level.

	if(recent_adcs.v_bms_mainfet_gate /* */ < 1000 /* */)
		LO(GPIOB, 2);
	else
		HI(GPIOB, 2);
}
