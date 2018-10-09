/*

	STM32 CONFUSION WARNING:

	The HRTIM in STM23H7 series is a completely different thing than the earlier HRTIM, for example, in
	STM32F334. The new HRTIM has been completely stripped away of the actual core of the HRTIM - the high
	resolution part! Only the external functionality looks the same.

	So it's just a regular timer, but sophisticated and flexible one.

	To compensate somewhat, 400MHz input clock (2.5 ns resolution) can be used. While it's good, it's of
	course nothing compared to the old 4.6GHz-equivalent clock (217 ps resolution).

	Some copy-pasta confusion exists in the reference manual.

	Luckily, this doens't matter in our case at all, and is actually a simplification.
	Just be careful when designing on these parts!



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


#define HRTIM_MASTER HRTIM1->sMasterRegs
#define HRTIM_CHA    HRTIM1->sTimerxRegs[0]
#define HRTIM_CHB    HRTIM1->sTimerxRegs[1]
#define HRTIM_CHC    HRTIM1->sTimerxRegs[2]
#define HRTIM_CHD    HRTIM1->sTimerxRegs[3]
#define HRTIM_CHE    HRTIM1->sTimerxRegs[4]
#define HRTIM        HRTIM1->sCommonRegs


#define CHARGER_CURRENT_COMPARATOR_HYSTERESIS (1UL)  // 1 (low), 2 (medium) or 3 (high)

#if CHARGER_CURRENT_COMPARATOR_HYSTERESIS < 0 || CHARGER_CURRENT_COMPARATOR_HYSTERESIS > 3
#error "Invalid CHARGER_CURRENT_COMPARATOR_HYSTERESIS"
#endif

#define INITIAL_OFFTIME (1000.0F) // ns
uint16_t charger_offtime = INITIAL_OFFTIME/2.5;

#define SAFETY_MAX_OFFTIME (5000.0F) // ns
 
#define SAFETY_MAX_OFFTIME_REG (SAFETY_MAX_OFFTIME/2.5)

#define DEADTIME_FALLING_NS (100.0F)
#define DEADTIME_FALLING_REG ((int)((DEADTIME_FALLING_NS/2.5)+0.5))

#define DEADTIME_RISING_NS (100.0F)
#define DEADTIME_RISING_REG ((int)((DEADTIME_RISING_NS/2.5)+0.5))


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

void charger_safety_errhandler()
{
	dis_gate_pha();
	dis_gate_phb();
	IO_TO_GPO(GPIOG, 1);
	IO_TO_GPO(GPIOG, 0);

	infet_chargepump_0();
	IO_TO_GPO(GPIOE, 10);

	HRTIM.OENR = 0;

	uart_print_string_blocking("\r\n\r\nSTOPPED: ");
	uart_print_string_blocking(__func__);
	uart_print_string_blocking("\r\n");

	uart_print_string_blocking("MASTER ISR = ");
	o_utoa32_hex(HRTIM_MASTER.MISR, printbuf); uart_print_string_blocking(printbuf);
	uart_print_string_blocking("\r\n");

	uart_print_string_blocking("TIMA ISR = ");
	o_utoa32_hex(HRTIM_CHA.TIMxISR, printbuf); uart_print_string_blocking(printbuf);
	uart_print_string_blocking("\r\n");

	uart_print_string_blocking("TIMB ISR = ");
	o_utoa32_hex(HRTIM_CHB.TIMxISR, printbuf); uart_print_string_blocking(printbuf);
	uart_print_string_blocking("\r\n");

	uart_print_string_blocking("TIMC ISR = ");
	o_utoa32_hex(HRTIM_CHC.TIMxISR, printbuf); uart_print_string_blocking(printbuf);
	uart_print_string_blocking("\r\n");

	uart_print_string_blocking("TIMD ISR = ");
	o_utoa32_hex(HRTIM_CHD.TIMxISR, printbuf); uart_print_string_blocking(printbuf);
	uart_print_string_blocking("\r\n");

	uart_print_string_blocking("TIME ISR = ");
	o_utoa32_hex(HRTIM_CHE.TIMxISR, printbuf); uart_print_string_blocking(printbuf);
	uart_print_string_blocking("\r\n");

	if(ADC1->ISR & (1UL<<7)) // ADC1 AWD1
		uart_print_string_blocking("\r\nADC1 AWD1\r\n");
	if(ADC1->ISR & (1UL<<8)) // ADC1 AWD2
		uart_print_string_blocking("\r\nADC1 AWD2\r\n");	
	if(ADC1->ISR & (1UL<<9)) // ADC1 AWD3
		uart_print_string_blocking("\r\nADC1 AWD3\r\n");	
	if(ADC2->ISR & (1UL<<7)) // ADC2 AWD1
		uart_print_string_blocking("\r\nADC2 AWD1\r\n");	
	if(ADC2->ISR & (1UL<<8)) // ADC2 AWD2
		uart_print_string_blocking("\r\nADC2 AWD2\r\n");	
	if(ADC2->ISR & (1UL<<9)) // ADC2 AWD3
		uart_print_string_blocking("\r\nADC2 AWD3\r\n");	

	uart_print_string_blocking("\r\nADC1 intflags = "); o_btoa16_fixed(ADC1->ISR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("\r\nADC2 intflags = "); o_btoa16_fixed(ADC2->ISR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");

	uart_print_string_blocking("DMA for ADC1: ");

	if(ADC1_DMA_STREAM->CR & 1) uart_print_string_blocking("STREAM ON"); else uart_print_string_blocking("STREAM OFF"); 
	uart_print_string_blocking("\r\nintflags = "); o_btoa8_fixed(DMA_INTFLAGS(ADC1_DMA, ADC1_DMA_STREAM_NUM), printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("NDTR  = "); o_utoa16(ADC1_DMA_STREAM->NDTR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");

	uart_print_string_blocking("DMA for ADC2: ");
	if(ADC2_DMA_STREAM->CR & 1) uart_print_string_blocking("STREAM ON"); else uart_print_string_blocking("STREAM OFF"); 
	uart_print_string_blocking("\r\nintflags = "); o_btoa8_fixed(DMA_INTFLAGS(ADC2_DMA, ADC2_DMA_STREAM_NUM), printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("NDTR  = "); o_utoa16(ADC2_DMA_STREAM->NDTR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");


	uart_print_string_blocking("\r\n");

	error(15);
}


void charger_10khz()
{
	static int cnt = 0;
	cnt++;

	if(cnt&1)
		infet_chargepump_0();
	else
	{
		// Keep the charge pump running whenever the Vin is at least 500mV below the Vinbus
		if(CHA_VIN_MEAS_TO_MV(adc1.s.cha_vin_meas) > CHA_VINBUS_MEAS_TO_MV(adc1.s.cha_vinbus_meas)-500)
			infet_chargepump_1();
	}

}

void start_pha()
{
	DIS_IRQ();
	en_gate_pha();
	__DSB();
	delay_us(1);
	HRTIM_CHE.SETx1R |= 1UL;
	HRTIM.CR2 |= (1UL<<13); // Software reset CHE
	__DSB();
	delay_us(2);
	HRTIM_CHE.SETx1R |= 1UL;
	HRTIM.CR2 |= (1UL<<13); // Software reset CHE
	__DSB();
	delay_us(2);
	HRTIM_CHE.SETx1R |= 1UL;
	HRTIM.CR2 |= (1UL<<13); // Software reset CHE
	__DSB();
	delay_us(2);
	HRTIM_CHE.SETx1R |= 1UL;
	HRTIM.CR2 |= (1UL<<13); // Software reset CHE
	__DSB();
	delay_us(2);
	HRTIM_CHE.SETx1R |= 1UL;
	HRTIM.CR2 |= (1UL<<13); // Software reset CHE
	__DSB();
	delay_us(2);
	HRTIM_CHE.SETx1R |= 1UL;
	HRTIM.CR2 |= (1UL<<13); // Software reset CHE
	__DSB();
	dis_gate_pha();

	ENA_IRQ();
}

void start_phb()
{
	DIS_IRQ();
	en_gate_phb();
	__DSB();

	delay_us(2);

	HRTIM_CHD.SETx1R |= 1UL;
	__DSB();
	delay_us(1);

	HRTIM_CHD.RSTx1R |= 1UL;
	__DSB();
	delay_us(1);


	HRTIM_CHD.SETx1R |= 1UL;
	__DSB();
	delay_us(1);

	HRTIM_CHD.RSTx1R |= 1UL;
	__DSB();
	delay_us(1);


	dis_gate_phb();

	ENA_IRQ();
}


/*
	IO_ALTFUNC(GPIOG,  6,  2); // CHE1: PHA LOFET
	IO_ALTFUNC(GPIOG,  7,  2); // CHE2: PHA HIFET
	IO_ALTFUNC(GPIOA, 11,  2); // CHD1: PHB LOFET
	IO_ALTFUNC(GPIOA, 12,  2); // CHD2: PHB HIFET
*/
#define A_LO_0() do{LO(GPIOG,6);__DSB();} while(0)
#define A_LO_1() do{HI(GPIOG,6);__DSB();} while(0)
#define A_HI_0() do{LO(GPIOG,7);__DSB();} while(0)
#define A_HI_1() do{HI(GPIOG,7);__DSB();} while(0)
#define B_LO_0() do{LO(GPIOA,11);__DSB();} while(0)
#define B_LO_1() do{HI(GPIOA,11);__DSB();} while(0)
#define B_HI_0() do{LO(GPIOA,12);__DSB();} while(0)
#define B_HI_1() do{HI(GPIOA,12);__DSB();} while(0)

/*
	MP1907 gate driver errata:

	MP1907 is either broken-by-design, or they just didn't bother to specify how it works.

	(I guess the actual datasheet with actual information could be available under NDA???)
	Almost all info regarding the EN pin (timing, how it works) is missing from the datasheet.

	Anyway, through trial and error:
	If the INH is high when the EN turns on, the chip enters some kind of lock-up error condition,
	and doesn't reply to any INH/INL changes after that point. I didn't bother testing whether this
	requires a power cycle to resolve, or if EN low -> back high is enough. Nevertheless, thanks
	MPS for wasting my time and going through my PCB design, soldering and code over and over again
	while there was no error anywhere whatsoever.

	Hope this helps others with this malfunctioning chip.
*/
void gatedriver_debug()
{
	dis_gate_pha();
	dis_gate_phb();
	IO_TO_GPO(GPIOG, 0);
	IO_TO_GPO(GPIOG, 1);

	A_LO_0();
	B_LO_0();
	A_HI_0();
	B_HI_0();
	IO_TO_GPO(GPIOG, 6);
	IO_TO_GPO(GPIOG, 7);
	IO_TO_GPO(GPIOA, 11);
	IO_TO_GPO(GPIOA, 12);

	while(1)
	{
		delay_ms(50);
		B_LO_0();
		B_HI_1(); // <- comment this out to make it work
		delay_ms(50);
		en_gate_phb();
		delay_us(1);

		for(int i=0; i<3; i++)
		{
			B_HI_0();
			__asm__ __volatile__ ("nop");
			__asm__ __volatile__ ("nop");
			__asm__ __volatile__ ("nop");
			B_LO_1();
			delay_us(1);
			B_LO_0();
			__asm__ __volatile__ ("nop");
			__asm__ __volatile__ ("nop");
			__asm__ __volatile__ ("nop");
			B_HI_1();
			delay_us(1);
		}
		B_HI_0();
		delay_us(1);
		dis_gate_phb();		
	}
}

void charger_test()
{
	static int cnt;
	uart_print_string_blocking("\r\n\r\n");

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


	cnt++;

	if(cnt > 10)
	{
		start_phb();

		cnt = 0;
	}

	delay_ms(99);

}


void init_charger()
{

	RCC->CFGR |= 1UL<<14; // Use CPU clock (400MHz) as HRTIM clock.
	__DSB();

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
	CURRLIM_DAC = 250;
	DAC1->CR |= 1UL<<0; // Enable channel 1

	// Comparator 1: PHA current
	COMP1->CFGR = 0UL<<24 /*blanking source*/ | 0UL<<20 /*in+ = PB0*/ | 0b100UL<<16 /*in- = DAC1*/ |
		CHARGER_CURRENT_COMPARATOR_HYSTERESIS<<8 | 1UL /* enable the thing!*/;

	// Comparator 2: PHB current
	COMP2->CFGR = 0UL<<24 /*blanking source*/ | 0UL<<20 /*in+ = PE9*/ | 0b100UL<<16 /*in- = DAC1*/ |
		CHARGER_CURRENT_COMPARATOR_HYSTERESIS<<8 | 1UL /* enable the thing!*/;

	// The HRTIM peripheral controls the gate signals:
/*
	For the master phase (PHA):

	- Cycle starts. SWitch node (SW) is driven high. Current starts increasing. Aux counter starts measuring the on time.
	- Current exceeds the maximum setpoint (set by a DAC) and trips the comparator
	- Comparator forces the SW to low, and starts the off-time counter. On-time counter is stopped and read out.
	- When the off-time counter expires, the cycle starts again.


	COMPARATOR EVENT COMP1 (high level on PHA comparator):
	- HIFET	OFF, LOFET ON, aka.  set CHE1 high (CHE2 follows automatically with deadtime)
	- Start counter CHE (offtime counter)

	CHE PERiod EVENT: (offtime ends)
	- HIFET ON, LOFET OFF, aka. set CHE1 low (CHE2 follows automatically)

	For the slave phase (PHB):

	- When the master phase trips the current setpoint, the slave phase starts its cycle. This way, the phase
	  difference would be 180 degrees only with 50% duty cycle, but it doesn't matter too much; as long as the
	  duty is 50% or less, there will be no overlap, guaranteed. With duty cycle over 50%, there will be the
	  minimum amount of unavoidable overlap.
	- When the slave phase trips its current setpoint (comparator), its SW is forced low. But, there is no off-time
	  counter. It's not needed.

	COMPARATOR EVENT COMP2 (high level on PHB comparator)
	- HIFET OFF, LOFET ON, aka. set CHD1 high (CHD2 follows automatically with deadtime)

	COMPARATOR EVENT COMP1 (high level on PHA comparator)
	- HIFET ON, LOFET OFF, aka. set CHD1 low (CHD2 follows automatically)

	ADC is triggered when the off-time starts.

*/

	// prescaler bitfield must be written before compare and period registers
	// period and compare registers must be between 3 and FFFD.


	// PHA overcurr: COMP1 -> EEV6 (SRC2)
	// PHB overcurr: COMP2 -> EEV7 (SRC2)

#define EV_LEVEL   0b00
#define EV_RISING  0b01
#define EV_FALLING 0b10
#define EV_BOTH    0b11

#define EV_ACTIVEHI 0UL
#define EV_ACTIVELO 1UL

	HRTIM_CHE.DTxR = DEADTIME_FALLING_REG<<16 | DEADTIME_RISING_REG<<0 | 0b011<<10 /*prescaler = 1*/;
	HRTIM_CHD.DTxR = DEADTIME_FALLING_REG<<16 | DEADTIME_RISING_REG<<0 | 0b011<<10 /*prescaler = 1*/;

	HRTIM_MASTER.MCR = 1UL<<27 /*preload*/ | 0b101 /*prescaler = 1*/;
	HRTIM_CHE.TIMxCR = 1UL<<27 /*preload*/ | 0b101 /*prescaler = 1*/ | 1UL<<4 /*retrig*/;
	HRTIM_CHD.TIMxCR = 1UL<<27 /*preload*/ | 0b101 /*prescaler = 1*/ | 1UL<<4 /*retrig*/;

	/*
		The event mapping documentation is completely fucked up by random
		name changes, incoherent, and partially non-documented. By reverse-engineering
		by trial and error, we found out the following:
		COMP1: EEV6, source regval=1
		
	*/
	HRTIM.EECR2 =
		1UL<<0 /*EEV6 source*/ | EV_ACTIVEHI<<2 | EV_LEVEL<<3 |
		1UL<<6 /*EEV7 source*/ | EV_ACTIVEHI<<8 | EV_LEVEL<<9;

	HRTIM_CHE.OUTxR =
		0b10UL<<20 /*OUT2 inactive in fault*/ | 0UL<<19 /*OUT2 inactive in idle*/ |
		0b10UL<<4  /*OUT1 inactive in fault*/ | 0UL<<3  /*OUT1 inactive in idle*/ |
		0UL<<17    /*OUT2 active high*/       | 0UL<<1  /*OUT1 active high*/ |
		1UL<<8 /*Deadtime enable - OUT2 commands ignored, OUT1 is in command*/;

	HRTIM_CHD.OUTxR =
		0b10UL<<20 /*OUT2 inactive in fault*/ | 0UL<<19 /*OUT2 inactive in idle*/ |
		0b10UL<<4  /*OUT1 inactive in fault*/ | 0UL<<3  /*OUT1 inactive in idle*/ |
		0UL<<17    /*OUT2 active high*/       | 0UL<<1  /*OUT1 active high*/ |
		1UL<<8 /*Deadtime enable - OUT2 commands ignored, OUT1 is in command*/;


	// Confusion warning: RST1R is _output_ turn-off register. RSTR is counter reset register
	HRTIM_CHE.SETx1R = 1UL<<26 /*EEV6*/;  // Overcurrent turns on bottom FET
	HRTIM_CHE.RSTxR  = 1UL<<14 /*EEV6*/;  // Overcurrent resets the offtime counter
	HRTIM_CHE.PERxR  = INITIAL_OFFTIME/2.5;
	HRTIM_CHE.RSTx1R = 1UL<<2 /*PER (of CHE)*/; // turn current-increasing


/*
	COMPARATOR EVENT COMP2 (high level on PHB comparator)
	- HIFET OFF, LOFET ON, aka. set CHD1 high (CHD2 follows automatically with deadtime)

	COMPARATOR EVENT COMP1 (high level on PHA comparator)
	- HIFET ON, LOFET OFF, aka. set CHD1 low (CHD2 follows automatically)
*/
	HRTIM_CHD.SETx1R = 1UL<<27 /*EEV7*/; // Overcurrent turns on bottom FET
	HRTIM_CHD.RSTx1R = 1UL<<26 /*EEV6*/; // Overcurrent of the master phase turns off the bottom FET, starting current-increasing cycle on the slave phase
	HRTIM_CHD.RSTxR  = 1UL<<15 /*EEV7*/; // Our own overcurrent also starts a backup safety off-time counter
	HRTIM_CHD.PERxR  = SAFETY_MAX_OFFTIME_REG;
	HRTIM_CHD.CMP1xR = SAFETY_MAX_OFFTIME_REG-1;
	HRTIM_CHD.TIMxDIER = 1UL; // Compare 1 interrupt

	IO_ALTFUNC(GPIOG,  6,  2); // CHE1: PHA LOFET
	IO_ALTFUNC(GPIOG,  7,  2); // CHE2: PHA HIFET
	IO_ALTFUNC(GPIOA, 11,  2); // CHD1: PHB LOFET
	IO_ALTFUNC(GPIOA, 12,  2); // CHD2: PHB HIFET


	HRTIM.CR2 = 0b11111100111111UL; // Force software update on all timers & reset counters.

	NVIC_SetPriority(HRTIM1_TIMD_IRQn, 0);
//	NVIC_EnableIRQ(HRTIM1_TIMD_IRQn);



	// Output enables - aka RUN:
	HRTIM.OENR = 0b11UL<<8 /*CHE*/ | 0b11UL<<6 /*CHD*/;

//	HRTIM_CHE.PERxR = CHARGER_OFFTIME;  // PHA = CHE
//	HRTIM_CHD.PERxR = CHARGER_OFFTIME;

	HRTIM_MASTER.MCR |= 1UL<<21 /*Enable CHE*/ | 1UL<<20 /*Enable CHD*/;
	while(1)
	{

		charger_test();
	}

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

