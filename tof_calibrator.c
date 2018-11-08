#include <stdint.h>

#include "ext_include/stm32h7xx.h"
#include "stm32_cmsis_extension.h"

#include "misc.h"


void tof_calibrator_ambient_lvl(int lvl)
{
	if(lvl&1) HI(GPIOE, 8); else LO(GPIOE, 8); 
	if(lvl&2) HI(GPIOE, 7); else LO(GPIOE, 7); 
	if(lvl&4) HI(GPIOF, 1); else LO(GPIOF, 1); 
	if(lvl&8) HI(GPIOF, 0); else LO(GPIOF, 0); 

}

void init_tof_calibrator_ambient()
{
	IO_TO_GPO(GPIOE, 8); // Header pin 2  "TX"  Ambient B0
	IO_TO_GPO(GPIOE, 7); // Header pin 4  "RX"  Ambient B1
	IO_TO_GPO(GPIOF, 1); // Header pin 14 "SCL" Ambient B2
	IO_TO_GPO(GPIOF, 0); // Header pin 16 "SDA" Ambient B3
}
