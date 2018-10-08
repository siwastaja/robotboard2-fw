/*

	The Charger

	Since a proper IC solution does not exist, the complete DC/DC converter was designed from scratch from
	simple basic building blocks:
	- MOSFETs,
	- FET gate drivers
	- Current sense amplifiers

	Additionally, the MCU implements:

	- Comparators, DACs, ADCs

	Together, these parts create a nice so-called "software defined" converter. This means, the software
	does not "bit bang" the converter, but the MCU includes the necessary analog peripherals to actually
	create the converter. Software is mostly used just to /configure/ these peripherals (what they do,
	and the connectivity between them).


	Synchronous buck takes fairly massive input current spikes, because the
	input current is discontinuous; full inductor current is taken from the input capacitors defined by
	the inverse of duty cycle time. This means, if the output is 20V,20A, and input is 40V,10A respectively,
	the input peaks are actually 20A for 50% of the time. This is why the controller is dual phase: now it's
	two 20V,10A controllers in parallel, the input peaks being at 10A. If the two converters are run in
	sync, with 180 degree phase shift, the input peaks happen at different times, giving smoother input
	current.

	Peak current mode control scheme is trivial and works great to directly produce the desired triangle current
	waveform with no overshoots. However, when driven with fixed start interval timing, it suffers from
	subharmonic oscillation. This means, if we start every pulse at say, 400kHz fixed timebase, and end the
	pulse when the current limit is exceeded, once the pulse duty cycle goes over 50%, short and long pulses
	start to oscillate: every other pulse is shorter, every other is longer. Worst case, this causes the
	switching frequency to halve, with obvious disadvantages.

	Freerunning the conversion with no fixed timebase fixes the issue; it can be a hysteretic controller (oscillating
	between minimum and maximum inductor current), or a constant off-time, or a constant on-time converter. However,
	it will be nontrivial to synchronize two such converters as they will be running slightly at different frequencies.

	In any case, we implement a constant off-time converter, even though the phase synchronization is non-trivial.
 	Because the input voltage, output voltage and output current are very close between the phases, and components
	are nearly identical, the frequencies will be close enough so we can measure and calculate the phase difference
	which will be close enough to run another phase.

	The constant off-time control scheme will work like this:

	For the master phase (PHA):

	- Cycle starts. SWitch node (SW) is driven high. Current starts increasing. Aux counter starts measuring the on time.
	- Current exceeds the maximum setpoint (set by a DAC) and trips the comparator
	- Comparator forces the SW to low, and starts the off-time counter. On-time counter is stopped and read out.
	- When the off-time counter expires, the cycle starts again.

	For the slave phase (PHB):

	- When the master phase trips the current setpoint, the slave phase starts its cycle. This way, the phase
	  difference would be 180 degrees only with 50% duty cycle, but it doesn't matter too much; as long as the
	  duty is 50% or less, there will be no overlap, guaranteed. With duty cycle over 50%, there will be the
	  minimum amount of unavoidable overlap.
	- When the slave phase trips its current setpoint (comparator), its SW is forced low. But, there is no off-time
	  counter. It's not needed.

	ADC is triggered when the off-time starts.

	

	Protections:

	Input overvoltage:
	Synchronous buck can convert from the lower output voltage back to the input higher voltage (work in bidirectional
	mode). Here, it's undesirable; there is no sink in the input, so the input voltage starts rising. This happens
	if there is any problem in the control scheme, so that the duty cycle is too low or cannot react to load changes.
	Input overvoltage protection drives the gate driver enable signals low, driving both lo- and hiside FETs off,
	causing the inductor current to decay through the freewheeling diodes. The input capacitance limits the voltage rise
	during this incident (i.e., the input capacitors can store all the energy stored in the inductors without too much
	voltage rise).



*/

#include <stdint.h>
#include "ext_include/stm32h7xx.h"
#include "stm32_cmsis_extension.h"
#include "misc.h"
#include "charger.h"
#include "adcs.h"
#include "own_std.h"

#define CHARGER_CURRENT_COMPARATOR_HYSTERESIS (1UL)  // 1 (low), 2 (medium) or 3 (high)

#if CHARGER_CURRENT_COMPARATOR_HYSTERESIS < 0 || CHARGER_CURRENT_COMPARATOR_HYSTERESIS > 3
#error "Invalid CHARGER_CURRENT_COMPARATOR_HYSTERESIS"
#endif

#define CHARGER_OFFTIME 1234
#if CHARGER_OFFTIME < 1000 || CHARGER_OFFTIME > 50000
#error "Invalid CHARGER_OFFTIME"
#endif


// Gate driver enables: when disabled, both fets of the relevant half bridge are off. When enabled, one is on at a time.
#define en_gate_pha()  do{ HI(GPIOG, 1); }while(0)
#define en_gate_phb()  do{ HI(GPIOG, 0); }while(0)
#define dis_gate_pha() do{ LO(GPIOG, 1); }while(0)
#define dis_gate_phb() do{ LO(GPIOG, 0); }while(0)

// Input "ideal diode" backflow-preventing MOSFET is N-channel type and requires higher-than-Vin gate drive voltage,
// which is generated by toggling a charge pump.

#define infet_chargepump_0() do{ LO(GPIOE, 10); }while(0)
#define infet_chargepump_1() do{ HI(GPIOE, 10); }while(0)

// Gives a current-limited output pulse, to communicate with charging station:
#define pulseout_0() do{ LO(GPIOE, 11); }while(0)
#define pulseout_1() do{ HI(GPIOE, 11); }while(0)


/*
	Current ADC (14-bit): 1 LSB = 0.806mA
	Current limit DAC (12-bit) 1 LSB = 3.223 mA

*/
#define CURRLIM_DAC DAC1->DHR12R1

#define ADC_TO_MA(x_) ((x_)*806/1000)


static char printbuf[128];


void charger_test()
{

	uart_print_string_blocking("PHA curr = ");
	o_utoa16_fixed(ADC_TO_MA(adc2.s.cha_currmeasa[0]), printbuf); uart_print_string_blocking(printbuf);
	uart_print_string_blocking("  ");
	o_utoa16_fixed(ADC_TO_MA(adc2.s.cha_currmeasa[1]), printbuf); uart_print_string_blocking(printbuf);
	uart_print_string_blocking("\r\n");

	uart_print_string_blocking("PHB curr = ");
	o_utoa16_fixed(ADC_TO_MA(adc2.s.cha_currmeasb[0]), printbuf); uart_print_string_blocking(printbuf);
	uart_print_string_blocking("  ");
	o_utoa16_fixed(ADC_TO_MA(adc2.s.cha_currmeasb[1]), printbuf); uart_print_string_blocking(printbuf);
	uart_print_string_blocking("\r\n");

	uart_print_string_blocking("PHA COMP = ");
	o_utoa16(COMP12->SR&1, printbuf); uart_print_string_blocking(printbuf);
	uart_print_string_blocking("\r\n");

	uart_print_string_blocking("PHA COMP = ");
	o_utoa16((COMP12->SR&2)>>1, printbuf); uart_print_string_blocking(printbuf);
	uart_print_string_blocking("\r\n");

	uart_print_string_blocking("cha_vin_meas = ");
	o_utoa32(CHA_VIN_MEAS_TO_MV(adc1.s.cha_vin_meas), printbuf); uart_print_string_blocking(printbuf);
	uart_print_string_blocking("  raw = ");
	o_utoa32(adc1.s.cha_vin_meas, printbuf); uart_print_string_blocking(printbuf);
	uart_print_string_blocking("\r\n");

	uart_print_string_blocking("cha_vinbus_meas = ");
	o_utoa32(CHA_VINBUS_MEAS_TO_MV(adc1.s.cha_vinbus_meas), printbuf); uart_print_string_blocking(printbuf);
	uart_print_string_blocking("  raw = ");
	o_utoa32(adc1.s.cha_vinbus_meas, printbuf); uart_print_string_blocking(printbuf);
	uart_print_string_blocking("\r\n");

	uart_print_string_blocking("vbat_meas = ");
	o_utoa32(VBAT_MEAS_TO_MV(adc1.s.vbat_meas), printbuf); uart_print_string_blocking(printbuf);
	uart_print_string_blocking("  raw = ");
	o_utoa32(adc1.s.vbat_meas, printbuf); uart_print_string_blocking(printbuf);
	uart_print_string_blocking("\r\n");

	delay_ms(50);

}

void init_charger()
{

	RCC->APB2ENR  |= 1UL<<29; // Enable HRTIM clock
	RCC->APB4ENR  |= 1UL<<14; // Enable COMP1,2 clock
	RCC->APB1LENR |= 1UL<<29; // Enable DAC1,2 clock. Only DAC1 is used by us; DAC2 may be used by others (audio 
				  // when writing this comment)

	

	dis_gate_pha();
	dis_gate_phb();
	IO_TO_GPO(GPIOG, 0);
	IO_TO_GPO(GPIOG, 1);

	infet_chargepump_0();
	IO_TO_GPO(GPIOE, 10);

	pulseout_0();
	IO_TO_GPO(GPIOE, 11);


	// We expect that the ADC is already configured.

	// DAC1: Current setpoint (don't touch channel 2, beware: configuration registers are shared)
	DAC1->MCR |= 0b011UL<<0 /*Channel 1: Connected to on-chip peripherals only, with buffer disabled*/;
	CURRLIM_DAC = 500;
	DAC1->CR |= 1UL<<0; // Enable channel 1

	// Comparator 1: PHA current
	COMP1->CFGR = 0UL<<24 /*blanking source*/ | 0UL<<20 /*in+ = PB0*/ | 0b100UL<<16 /*in- = DAC1*/ |
		CHARGER_CURRENT_COMPARATOR_HYSTERESIS<<8 | 1UL /* enable the thing!*/;

	// Comparator 2: PHB current
	COMP2->CFGR = 0UL<<24 /*blanking source*/ | 0UL<<20 /*in+ = PE9*/ | 0b100UL<<16 /*in- = DAC1*/ |
		CHARGER_CURRENT_COMPARATOR_HYSTERESIS<<8 | 1UL /* enable the thing!*/;

	// The HRTIM peripheral controls the gate signals:

#define HRTIM_MASTER HRTIM1->sMasterRegs
#define HRTIM_CHA    HRTIM1->sTimerxRegs[0]
#define HRTIM_CHB    HRTIM1->sTimerxRegs[1]
#define HRTIM_CHC    HRTIM1->sTimerxRegs[2]
#define HRTIM_CHD    HRTIM1->sTimerxRegs[3]
#define HRTIM_CHE    HRTIM1->sTimerxRegs[4]

	HRTIM_CHE.PERxR = CHARGER_OFFTIME;  // PHA = CHE
	HRTIM_CHD.PERxR = CHARGER_OFFTIME;


	IO_ALTFUNC(GPIOG,  6,  2); // CHE1: PHA LOFET
	IO_ALTFUNC(GPIOG,  7,  2); // CHE2: PHA HIFET
	IO_ALTFUNC(GPIOA, 11,  2); // CHD1: PHB LOFET
	IO_ALTFUNC(GPIOA, 12,  2); // CHD2: PHB HIFET

	while(1)
		charger_test();

}


void deinit_charger()
{
	dis_gate_pha();
	dis_gate_phb();
	infet_chargepump_0();
	pulseout_0();

	// disconnect gate signals from HRTIM:
	IO_TO_GPI(GPIOG, 6);
	IO_TO_GPI(GPIOG, 7);
	IO_TO_GPI(GPIOA, 11);
	IO_TO_GPI(GPIOA, 12);

	COMP1->CFGR = 0;
	COMP2->CFGR = 0;

	DAC1->CR &= ~(1UL<<0); // Disable DAC channel 1, without touching channel 2

	RCC->APB2ENR &= ~(1UL<<29); // HRTIM clock
	RCC->APB4ENR &= ~(1UL<<14); // COMP1,2 clock
	// We won't turn the DACs off due to other uses for DAC2.

}




