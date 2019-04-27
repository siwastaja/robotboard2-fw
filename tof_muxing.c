#include <stdint.h>
#include "ext_include/stm32h7xx.h"
#include "stm32_cmsis_extension.h"

#include "tof_muxing.h"

// All pins are the same for REV2A, REV2B

#define MUX0S0_PORT GPIOH
#define MUX0S0_BIT  (11)
#define MUX0S1_PORT GPIOH
#define MUX0S1_BIT  (10)

#define MUX1S0_PORT GPIOD
#define MUX1S0_BIT  (4)
#define MUX1S1_PORT GPIOD
#define MUX1S1_BIT  (5)

#define MUX2S0_PORT GPIOD
#define MUX2S0_BIT  (2)
#define MUX2S1_PORT GPIOD
#define MUX2S1_BIT  (6)

#define MUX3S0_PORT GPIOE
#define MUX3S0_BIT  (3)
#define MUX3S1_PORT GPIOE
#define MUX3S1_BIT  (4)

// If you change the ports in PCB layout, in addition to updating these defines, see also:
// - clearing all bits efficiently at the start of tof_mux_select()

void tof_mux_init()
{
	tof_mux_all_off();

	IO_TO_GPO(MUX0S0_PORT, MUX0S0_BIT);
	IO_TO_GPO(MUX0S1_PORT, MUX0S1_BIT);
	IO_TO_GPO(MUX1S0_PORT, MUX1S0_BIT);
	IO_TO_GPO(MUX1S1_PORT, MUX1S1_BIT);
	IO_TO_GPO(MUX2S0_PORT, MUX2S0_BIT);
	IO_TO_GPO(MUX2S1_PORT, MUX2S1_BIT);
	IO_TO_GPO(MUX3S0_PORT, MUX3S0_BIT);
	IO_TO_GPO(MUX3S1_PORT, MUX3S1_BIT);
}

/*
	SN74CBT16214 3-to-1 multiplexer:

	S2 line tied permanently high

	S1	S0	state
	0	0	disconnect
	0	1	A = B3
	1	0	A = B1
	1	1	A = B2
*/

// These functions only set high, and expect zeroed registers
static inline void MUX0_B1()
{
	HI(MUX0S1_PORT, MUX0S1_BIT);
}

static inline void MUX0_B2()
{
	HI(MUX0S0_PORT, MUX0S0_BIT);
	HI(MUX0S1_PORT, MUX0S1_BIT);
}

static inline void MUX0_B3()
{
	HI(MUX0S0_PORT, MUX0S0_BIT);
}

static inline void MUX1_B1()
{
	HI(MUX1S1_PORT, MUX1S1_BIT);
}

static inline void MUX1_B2()
{
	HI(MUX1S0_PORT, MUX1S0_BIT);
	HI(MUX1S1_PORT, MUX1S1_BIT);
}

static inline void MUX1_B3()
{
	HI(MUX1S0_PORT, MUX1S0_BIT);
}

static inline void MUX2_B1()
{
	HI(MUX2S1_PORT, MUX2S1_BIT);
}

static inline void MUX2_B2()
{
	HI(MUX2S0_PORT, MUX2S0_BIT);
	HI(MUX2S1_PORT, MUX2S1_BIT);
}

static inline void MUX2_B3()
{
	HI(MUX2S0_PORT, MUX2S0_BIT);
}

static inline void MUX3_B1()
{
	HI(MUX3S1_PORT, MUX3S1_BIT);
}

static inline void MUX3_B2()
{
	HI(MUX3S0_PORT, MUX3S0_BIT);
	HI(MUX3S1_PORT, MUX3S1_BIT);
}

static inline void MUX3_B3()
{
	HI(MUX3S0_PORT, MUX3S0_BIT);
}


void tof_mux_all_off()
{
	GPIOH->BSRR = (1UL<<(16+10)) | (1UL<<(16+11));
	GPIOD->BSRR = (1UL<<(16+4)) | (1UL<<(16+5)) | (1UL<<(16+2)) | (1UL<<(16+6));
	GPIOE->BSRR = (1UL<<(16+3)) | (1UL<<(16+4));
}

void tof_mux_select(int idx)
{
	tof_mux_all_off();
	// Only set bits to high
	switch(idx)
	{
		case 0:
		MUX0_B1();
		MUX1_B3();
		break;

		case 1:
		MUX0_B2();
		MUX1_B3();
		break;

		case 2:
		MUX0_B3();
		MUX1_B3();
		break;

		case 3:
		MUX1_B1();
		break;

		case 4:
		MUX1_B2();
		break;

		case 5:
		MUX2_B1();
		break;

		case 6:
		MUX2_B2();
		break;

		case 7:
		MUX3_B1();
		MUX2_B3();
		break;

		case 8:
		MUX3_B2();
		MUX2_B3();
		break;

		case 9:
		MUX3_B3();
		MUX2_B3();
		break;

		default:
		break;
	}
}

