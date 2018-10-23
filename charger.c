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


#define HRTIM_OUTEN_PHA()  do{HRTIM.OENR  = 0b11UL<<8;}while(0)
#define HRTIM_OUTDIS_PHA() do{HRTIM.ODISR = 0b11UL<<8;}while(0)
#define HRTIM_OUTEN_PHB()  do{HRTIM.OENR  = 0b11UL<<6;}while(0)
#define HRTIM_OUTDIS_PHB() do{HRTIM.ODISR = 0b11UL<<6;}while(0)


#define CHARGER_CURRENT_COMPARATOR_HYSTERESIS (0UL)  // 0 (no), 1 (low), 2 (medium) or 3 (high)

#if CHARGER_CURRENT_COMPARATOR_HYSTERESIS < 0 || CHARGER_CURRENT_COMPARATOR_HYSTERESIS > 3
#error "Invalid CHARGER_CURRENT_COMPARATOR_HYSTERESIS"
#endif


#define SAFETY_MAX_OFFTIME (5000.0F) // ns
 
#define SAFETY_MAX_OFFTIME_REG (SAFETY_MAX_OFFTIME/2.5)

// -6.4V .. +48.0V Vin=31.6V, 30ns deadtime  QuickPrint5
// -12.0V .. +48.8V Vin=31.6V, 15ns deadtime  QuickPrint6


//15ns: undershoot -12.0V at 8A, Vin=31.6V, Vout=16.0V
//17.5ns: undershoot -6.8V
//20ns: undershoot -6.4V
//30ns: undershoot -6.4V
#define DEADTIME_FALLING_NS (20.0F)
#define DEADTIME_FALLING_REG ((int)((DEADTIME_FALLING_NS/2.5)+0.5))

//30ns: overshoot 47.6V (quickprint7), quickprint12
//20ns: 47.2V (quickprint8)
//15ns: 47.2V (quickprint10)
//10ns: quickprint11
//7.5ns: quickprint14
#define DEADTIME_RISING_NS (12.5F)
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
	Current limit DAC (12-bit) 1 LSB = 3.223 mA
	From mA to DAC: multiplier = 1/3.223
	Multiplier for shift by 12: 1270.87

	Current ADC (14-bit): 1 LSB = 0.806mA
	Current ADC (14-bit, oversampled by 3x): 1 LSB = 0.269mA

	Current ADC (10-bit): 1 LSB = 12.8906mA
	Current ADC (10-bit, oversampled by 3x): 1 LSB = 4.2969mA
	Multiplier for shift by 12: 17600.10

	From mA to ADC: multiplier = 1/4.2969 = 0.232726
	Multiplier for shift by 12: 953.2454
*/
#define CURRLIM_DAC DAC1->DHR12R1

#define ADC_TO_MA(x_) (((x_)*17600)>>12)
#define MA_TO_ADC(x_) (((x_)*953)>>12)
#define MA_TO_DAC(x_) (((x_)*1271)>>12)

void start_phab();
void stop_phab();

volatile int run_pump = 0;



static char printbuf[128];

void charger_safety_shutdown() __attribute__((section(".text_itcm")));
void charger_safety_shutdown()
{
	dis_gate_pha();
	dis_gate_phb();
//	HRTIM_OUTDIS_PHA();
//	HRTIM_OUTDIS_PHB();
	IO_TO_GPO(GPIOG, 1);
	IO_TO_GPO(GPIOG, 0);

	run_pump = 0;
	infet_chargepump_0();
	IO_TO_GPO(GPIOE, 10);

//	HRTIM.OENR = 0;
}

void charger_safety_errhandler()
{
	SAFETY_SHUTDOWN();

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
		uart_print_string_blocking("\r\nADC1 AWD1: Vbat out of range\r\n");
	if(ADC1->ISR & (1UL<<8)) // ADC1 AWD2
		uart_print_string_blocking("\r\nADC1 AWD2: Charger Vinbus out of range\r\n");	
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

	uart_print_string_blocking("\r\nADC1 DR = "); o_utoa16_fixed(ADC1->DR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("\r\nADC2 DR = "); o_utoa16_fixed(ADC2->DR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");

	uart_print_string_blocking("DMA for ADC1: ");

	if(ADC1_DMA_STREAM->CR & 1) uart_print_string_blocking("STREAM ON"); else uart_print_string_blocking("STREAM OFF"); 
	uart_print_string_blocking("\r\nintflags = "); o_btoa8_fixed(DMA_INTFLAGS(ADC1_DMA, ADC1_DMA_STREAM_NUM), printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("NDTR  = "); o_utoa16(ADC1_DMA_STREAM->NDTR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");

	uart_print_string_blocking("DMA for ADC2: ");
	if(ADC2_DMA_STREAM->CR & 1) uart_print_string_blocking("STREAM ON"); else uart_print_string_blocking("STREAM OFF"); 
	uart_print_string_blocking("\r\nintflags = "); o_btoa8_fixed(DMA_INTFLAGS(ADC2_DMA, ADC2_DMA_STREAM_NUM), printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("NDTR  = "); o_utoa16(ADC2_DMA_STREAM->NDTR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");


	uart_print_string_blocking("\r\n");

	error(11);
}

void charger_10khz()
{
	static int cnt = 0;
	cnt++;

	// Keep the charge pump running whenever enabled, and the Vin is at least 500mV below the Vinbus
	if(cnt&1)
	{
		infet_chargepump_0();
	}
	else if( run_pump && 
	         (CHA_VIN_MEAS_TO_MV(adc1.s.cha_vin_meas) > CHA_VINBUS_MEAS_TO_MV(adc1.s.cha_vinbus_meas)-500) )
	{
		infet_chargepump_1();
	}
}

void stop_pump()
{
	run_pump = 0;
	infet_chargepump_0();
}

void start_pump()
{
	run_pump = 1;
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

	MP1907 is either broken-by-design, or they just didn't bother to specify how they designed it to work, which
	is a very nonstandard way for a gate driver.

	(I guess the actual datasheet with actual information could be available under NDA???)
	Almost all info regarding the EN pin (timing, how it works) is missing from the datasheet.

	Anyway, this is what I found out by reverse-engineering through trial and error:
	If either the INH or INL is high when the EN turns on, the chip enters some kind of lock-up error condition,
	and doesn't reply to any INH/INL changes after that point. I didn't bother testing whether this
	requires a power cycle to resolve, or if EN low -> back high is enough. Nevertheless, thanks
	MPS for wasting my time and going through my PCB design, prototype soldering and code over and over again
	while there was no error anywhere whatsoever on my part.

	This bug / "feature" makes working with this chip really cumbersome, as the enable pin cannot be used
	as an independent way to start the driver. Robustness of the system configuration is reduced as the
	PWM cannot simply run in the standard deadtime'd configuration, but needs extra states when "enabling"
	the device.

	Additionally, there seems to be a HUUUUUUUUUUUUUUUUUUGE delay of the EN pin responding: measured at
	around 60 us. Disabling seems to happen instantly, thank god...

	If you try to change the inputs during this massive wait, sometimes it starts working after around 60us,
	sometimes strange glitches appear on the waveform!

	This random, undocumented and unexpected behavior cripples the usability of this expensive chip.

	Hope this helps others with this malfunctioning chip.

	Update: Found another design bug!
	When giving the first low input pulse after enabling the chip (to charge the bootstrap cap), if this
	pulse length is below about 1.5us, the high-side driver stays disabled and completely misses the
	first high-level input!!!!! This isn't due to bootstrap UVLO; the bootstrap cap charges to over 10V,
	well above the turn-on threshold. The delay (minimum off-time at start) is the same regardless of
	whether the capacitor is 220n (charging to around 7V in 1us) or 100n (charging to over 10V in 1us) -
	the high-FET just doesn't turn on at all!

*/

/*
	HRTIM errata:
	Timers cannot be shut down by zeroing MCR bits. This is non-documented and unexpected.

*/

/*

	Charger fsw = 333.3kHz
	Charger period = 3 us (1200 units)
	Min duty cycle is 16V/52V = 31%
	Max duty cycle is 25.2V/33V = 76%

	Actual duty cycle is always more (current driven into the battery, unidirectional losses)

	-> min design duty cycle = 30% (on: 900ns (360 units), off: 2100ns (840 units))
	-> max design duty cycle = 90% (on: 2700ns (1080 units), off: 300ns (120 units))

	Duty cycles of the phases are tied together, with small variance allowed. If significant
	deviation is seen, the converter is stopped. This guarantees the ADC triggers won't overlap,
	simplifying the design, and also catches some unexpected errors.

	3 ADC conversions fit in the minimum on-time of 900ns. The conversions are centered in the
	middle of the on-time.


1 character = 300ns (120 units)
1 ADC conversion = 300ns (120 units)

At min duty:
PHA:    ---_______---_______---_______---__
PHB:         ---_______---_______---_______
ADC:    AAA..BBB..AAA..BBB..AAA..BBB..AAA..

At max duty:
PHA:    ---------_---------_---------_
PHB:         ---------_---------_---------_
           AAA..BBB..AAA..BBB..AAA..BBB..AAA..

*/

/*
	These could be calculated automatically from floats, using nanoseconds conveniently, like
	I did originally.
	I feel that we keep a better grip of the actual resolution and the time steps doing it
	with direct timer units.

	1 unit = 2.5ns (timer runs at 400MHz)

*/
#define PERIOD       1200
#define MAX_DUTY_ON  1080
#define MAX_DUTY_OFF 120
#define MIN_DUTY_ON  360
#define MIN_DUTY_OFF 840

/*
	Maximum difference in duty cycles between the two phases allowed before erroring out.
	This has to be small enough, because the ADC measurements are centered on the mid-ontime.
	If the duty cycles are different, mid-on-time is different, ADCs are not triggered at equivalent
	intervals. If the difference is massive, the ADC misses a trigger because it's still converting.
	(There is 600ns free time between ADC conversions, so the actual difference allowed would be 1200ns,
	minus safety margin)
	But, even a much lower difference between the phases would signify strange issues.
	This number is in the time units: it applies equally to either on-time or off-time, and both
	as a negative and positive limit.
*/
#define MAX_PHASE_DIFFERENCE 100


#if (    ((MAX_DUTY_ON + MAX_DUTY_OFF) != PERIOD) || ((MIN_DUTY_ON + MIN_DUTY_OFF) != PERIOD)      )
#error Check duty cycle defines.
#endif

//uint16_t pha_duty = 

// 2 us to charge the bootstrap capacitor properly
// 1 us really isn't enough.

// 800 (2us):
// Phase A: QuickPrint1: Amp saturation recovery from the end-of-bootstrap-charge-pulse: 22.4us
// Phase B: QuickPrint2: Recovery: 23.0us

// 600 (1.5us)
// Phase A: QuickPrint3: Revocery: 18.8us
// Phase B: QuickPrint4: Revocery: 19.7us

// Bootstrap capacitor changed from 220n to 100n, verified no difference at 1.5us as expected.
// Now, we can safety go down to 1us:

// 400 (1.0us)
// Phase A: QuickPrint5: Revocery: 18.8us
// Phase B: QuickPrint6: Revocery: 19.7us


#define INITIAL_DUTY_OFF 100


// Gives the ontime

//374ns
//433ns after calibration multipliers in variables instead :(.
//To 378ns after optimizing by using a common calibration multiplier :).
//Down to 328ns after optimizing for 32-bit load operation!
//409ns after enabling all ADC intflag safety checks again
//398ns after combining all possible error sources to a single check.
//Whoopsie: calc_check_offtime() was in flash not itcm. Inlined it, 250ns.

// Original super-slow:
//#define CALC_DUTY() (  ((PERIOD*(VBAT_MEAS_TO_MV(adc1.s.vbat_meas))) / CHA_VINBUS_MEAS_TO_MV(adc1.s.cha_vinbus_meas))  )

// Quicker:
//#define CALC_DUTY() ((((vbat_per_vinbus_mult*adc1.s.vbat_meas) / adc1.s.cha_vinbus_meas)*PERIOD)>>13)

/*
	The quickest:

	This optimization is here for a good reason.

	This produces just a few lines of asm. I need to read vbat_meas and cha_vinbus_meas from the ADC memory structure,
	and loading them separately as two 16-bit loads is taking too much time and preventing optimizations.
	Now, because the ADC struct needs to be volatile, the compiler doesn't optimize the 16-bit accesses together.
	So, through an alised union, a 32-bit access is made. To made this a bit more readable, I want to use a documentative
	union right here.

	GCC is utterly stupid not to inline this function without always_inline even though it's just a few instructions,
	and only called from two places.
*/
static inline int CALC_DUTY() __attribute__((always_inline));
static inline int CALC_DUTY()
{
	union
	{
		struct __attribute__((packed))
		{
			uint16_t vbat_meas;
			uint16_t cha_vinbus_meas;
		} singles;
		uint32_t both;
	} access_32b;

	access_32b.both = adc1.quick.vbat_and_vinbus;

	return (((vbat_per_vinbus_mult*access_32b.singles.vbat_meas) / access_32b.singles.cha_vinbus_meas)*PERIOD)>>13;
}

static inline int calc_check_offtime(int finetune) __attribute__((always_inline));
static inline int calc_check_offtime(int finetune) 
{
	int new_offtime = PERIOD - CALC_DUTY() + finetune;
	
	if(new_offtime < MAX_DUTY_OFF || new_offtime > MIN_DUTY_OFF)
	{
		SAFETY_SHUTDOWN();

		uart_print_string_blocking("\r\nSTOPPED: calculated offtime = "); o_itoa32(new_offtime, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");

		error(12);
		return 0;
	}

	return new_offtime;
}

/*
	ADC trigger happens 900ns/2 (360units/2), ie. 180 units before the on-time midpoint.

	Off-time is first in HRTIM cycle.
	On-time starts at CMP1 (SET_OFFTIME argument value)
	On-time lasts for PERIOD-offtime
	On-time halfway is at offtime + (PERIOD-offtime)/2
	ADC trigger is at offtime + (PERIOD-offtime)/2 - 180

*/

#define SET_OFFTIME_PHA(x_) do{HRTIM_CHE.CMP1xR = (x_); HRTIM_CHE.CMP2xR = (x_) + (PERIOD-(x_))/2 - 180;}while(0)
#define SET_OFFTIME_PHB(x_) do{HRTIM_CHD.CMP1xR = (x_); HRTIM_CHD.CMP2xR = (x_) + (PERIOD-(x_))/2 - 180;}while(0)

int phase;

int32_t current_setpoint = MA_TO_ADC(2000);
volatile int interrupt_count;


void set_current(int ma)
{
	if(ma < 1000 || ma > 8000)
		error(27);
	current_setpoint = MA_TO_ADC(ma);

	int dac = MA_TO_DAC(ma+4000);
	if(dac > 4090) dac = 4090;
	CURRLIM_DAC = dac;
}

volatile int latest_cur_pha, latest_cur_phb;

#ifdef ENABLE_TRACE
	#define TRACE_LEN 256
	volatile int traces[4][TRACE_LEN];
	volatile int trace_at;
#endif

/*
	The "theoretical" duty cycle can be calculated from the ratio of input and output voltage, and adding the
	losses to the equation, the current can be modeled as an imaginary duty cycle offset.

	This cannot be, and doesn't need to be accurate. Typically, something like this isn't done /at all/; instead,
	typically a dumb feedback loop is used. By using an approximation directly feedforwarded from both input and
	output voltage, we are really close to the real duty cycle, and can feedback (using PI loop) a fine-tuning
	parameter only!
*/


int pha_lowcurr_fail_cnt;
int phb_lowcurr_fail_cnt;

//int pha_finetune;
int phb_finetune;

int pha_finetune_fail_cnt;
int phb_finetune_fail_cnt;

#define INITIAL_FINETUNE (-150)
#define INITIAL_ERR_INTEGRAL (-15*256)

int pha_err_integral;
int phb_err_integral;

void charger_adc2_phb_inthandler() __attribute__((section(".text_itcm")));

static int pid_i = 2;
static int pid_p = 64;

#define ERR_INTEGRAL_MIN (-30*256)
#define ERR_INTEGRAL_MAX (10*256)

void charger_adc2_pha_inthandler() __attribute__((section(".text_itcm")));
void charger_adc2_pha_inthandler()
{
	LED_ON();
	interrupt_count++;
	uint32_t sr1 = ADC1->ISR;
	uint32_t sr2 = ADC2->ISR;

	int32_t cur = ADC2->DR;

	if(sr1 & 0b01110010000 /*any AWD or OVERRUN*/ || sr2 & 0b01110011000 /*any AWD or OVERRUN, or End-of-Seq in unexpected state*/)
	{
		SAFETY_SHUTDOWN();
		uart_print_string_blocking("\r\nCharger ISR multi-error 1\r\n");
		DBG_PR_VAR_U32_HEX(sr1);
		DBG_PR_VAR_U32_HEX(sr2);
		charger_safety_errhandler();
	}

	SET_ADC12_VECTOR(charger_adc2_phb_inthandler);


	if(cur < MA_TO_ADC(300))
	{
		if(++pha_lowcurr_fail_cnt > 8)
		{
			SAFETY_SHUTDOWN();
			uart_print_string_blocking("\r\nLow-current failure 1\r\n");
			charger_safety_errhandler();
		}
	}
	else
	{
		pha_lowcurr_fail_cnt = 0;
	}

	int err = cur - current_setpoint;

	pha_err_integral += err;
	if(pha_err_integral < ERR_INTEGRAL_MIN)
	{
		pha_err_integral = ERR_INTEGRAL_MIN;
		pha_finetune_fail_cnt++;
	}
	else if(pha_err_integral > ERR_INTEGRAL_MAX)
	{
		pha_err_integral = ERR_INTEGRAL_MAX;
		pha_finetune_fail_cnt++;
	}
	else
		pha_finetune_fail_cnt = 0;


	if(pha_finetune_fail_cnt > 7)
	{
		SAFETY_SHUTDOWN();
		uart_print_string_blocking("\r\nCharger PID integral windup 1\r\n");
		DBG_PR_VAR_I32(pha_err_integral);
		charger_safety_errhandler();
	}

	int pha_finetune = pid_p*err + pid_i*pha_err_integral;


	int vratio_offtime = calc_check_offtime(pha_finetune>>8);

	SET_OFFTIME_PHA(vratio_offtime);

	#ifdef ENABLE_TRACE
		traces[2][trace_at] = pha_finetune;
		traces[0][trace_at] = cur;
	#endif

	LED_OFF();
}

int phb_disabled = 1;

void charger_adc2_phb_inthandler() __attribute__((section(".text_itcm")));
void charger_adc2_phb_inthandler()
{
	LED_ON();
	interrupt_count++;
	uint32_t sr1 = ADC1->ISR;
	uint32_t sr2 = ADC2->ISR;

	int32_t cur = ADC2->DR;

	if(sr1 & 0b01110010000 /*any AWD or OVERRUN*/ || sr2 & 0b01110010000 /*any AWD or OVERRUN*/) // || !(sr2 & 1UL<<3))
	{	
		// Wrong end-of-sequence not checked for. If it's really out of sync, the next cycle
		// reveals it, and that's fast enough. We get the check for free there.
		SAFETY_SHUTDOWN();
		uart_print_string_blocking("\r\nCharger ISR multi-error 2\r\n");
		DBG_PR_VAR_U32_HEX(sr1);
		DBG_PR_VAR_U32_HEX(sr2);
		charger_safety_errhandler();
	}

	SET_ADC12_VECTOR(charger_adc2_pha_inthandler);
	ADC2->ISR = 1UL<<3; // Clear the end-of-seq flag: it's used for failure detection

	if(phb_disabled)
	{
		goto PHB_SKIP_CTRL;
	}

	if(cur < MA_TO_ADC(300))
	{
		if(++phb_lowcurr_fail_cnt > 8)
		{
			SAFETY_SHUTDOWN();
			uart_print_string_blocking("\r\nLow-current failure 2\r\n");
			charger_safety_errhandler();
		}
	}
	else
	{
		phb_lowcurr_fail_cnt = 0;
	}


	int err = cur - current_setpoint;

	phb_err_integral += err;
	if(phb_err_integral < ERR_INTEGRAL_MIN)
	{
		phb_err_integral = ERR_INTEGRAL_MIN;
		phb_finetune_fail_cnt++;
	}
	else if(phb_err_integral > ERR_INTEGRAL_MAX)
	{
		phb_err_integral = ERR_INTEGRAL_MAX;
		phb_finetune_fail_cnt++;
	}
	else
		phb_finetune_fail_cnt = 0;


	if(phb_finetune_fail_cnt > 7)
	{
		SAFETY_SHUTDOWN();
		uart_print_string_blocking("\r\nCharger PID integral windup 2\r\n");
		DBG_PR_VAR_I32(phb_err_integral);
		charger_safety_errhandler();
	}

	int phb_finetune = pid_p*err + pid_i*phb_err_integral;


	int vratio_offtime = calc_check_offtime(phb_finetune>>8);

	SET_OFFTIME_PHB(vratio_offtime);


	PHB_SKIP_CTRL:

	#ifdef ENABLE_TRACE
		traces[3][trace_at] = phb_finetune;
		traces[1][trace_at] = cur;
		if(trace_at < TRACE_LEN-1)
			trace_at++;
	#endif

	LED_OFF();
}


void charger_test()
{
	static int cnt;

	uart_print_string_blocking("\r\n\r\n");

	int32_t ca = latest_cur_pha;
	int32_t cb = latest_cur_phb;

	uart_print_string_blocking("PHA curr = ");
	o_utoa16_fixed(ADC_TO_MA(ca), printbuf); uart_print_string_blocking(printbuf);
	uart_print_string_blocking("  raw  ");
	o_utoa16_fixed(ca, printbuf); uart_print_string_blocking(printbuf);
	uart_print_string_blocking("\r\n");

	uart_print_string_blocking("PHB curr = ");
	o_utoa16_fixed(ADC_TO_MA(cb), printbuf); uart_print_string_blocking(printbuf);
	uart_print_string_blocking("  raw  ");
	o_utoa16_fixed(cb, printbuf); uart_print_string_blocking(printbuf);
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

	uart_print_string_blocking("current_setpoint = ");
	o_utoa32(current_setpoint, printbuf); uart_print_string_blocking(printbuf);
	uart_print_string_blocking("\r\n");

	uart_print_string_blocking("CALC_DUTY() = ");
	o_itoa32(CALC_DUTY(), printbuf); uart_print_string_blocking(printbuf);
	uart_print_string_blocking("\r\n");
	
	uart_print_string_blocking("int_count = ");
	o_utoa32(interrupt_count, printbuf); uart_print_string_blocking(printbuf);
	uart_print_string_blocking("\r\n");

	DBG_PR_VAR_I32(pid_p);
	DBG_PR_VAR_I32(pid_i);


	uint8_t cmd = uart_input();

	if(cmd == 'q')
		pid_p++;
	else if(cmd=='a')
	{
		if(pid_p>0) pid_p--;
	}
	else if(cmd=='w')
		pid_i++;
	else if(cmd=='s')
	{
		if(pid_i>0) pid_i--;
	}
/*	if(cmd == 'a')
	{
		start_pha();
	}
	if(cmd == 's')
	{
		stop_pha();
	}
	if(cmd == '0')
	{
		current_setpoint = 500;
	}
	if(cmd == '1')
	{
		current_setpoint = 1000;
	}
	if(cmd == '2')
	{
		current_setpoint = 2000;
	}
	if(cmd == '3')
	{
		current_setpoint = 3000;
	}
	if(cmd == '4')
	{
		current_setpoint = 4000;
	}
	if(cmd == '5')
	{
		current_setpoint = 5000;
	}
	if(cmd == '6')
	{
		current_setpoint = 6000;
	}
	if(cmd == '7')
	{
		current_setpoint = 7000;
	}

*/


	cnt++;

	if(cnt==10)
	{
		start_phab(0);
		delay_us(20);
//		set_current(4000);
//		delay_us(60);
		stop_phab();

		#ifdef ENABLE_TRACE
			uart_print_string_blocking("\r\n\r\ncur_a_ma,cur_b_ma,fine_a,fine_b\r\n");
			int end = trace_at;
			for(int i=0; i<end; i++)
			{
				o_itoa32(ADC_TO_MA(traces[0][i]), printbuf); uart_print_string_blocking(printbuf);
				uart_print_string_blocking(",");
				o_itoa32(ADC_TO_MA(traces[1][i]), printbuf); uart_print_string_blocking(printbuf);
				uart_print_string_blocking(",");
				o_itoa32(traces[2][i], printbuf); uart_print_string_blocking(printbuf);
				uart_print_string_blocking(",");
				o_itoa32(traces[3][i], printbuf); uart_print_string_blocking(printbuf);
				uart_print_string_blocking("\r\n");
			}
			uart_print_string_blocking("\r\n");
		#endif
		cnt = 0;
	}

	delay_ms(99);
}


// cur setpoint=1200mA
// in 30.5V    0.97A
// out 15.17V  1.40A
// Eff = 71%
// losses = 8.347W

// cur setpoint=3000mA
// in 30.4V    2.23A
// out 15.3V   3.83A
// Eff = 86%
// losses = 9.193W

// cur setpoint=3600mA
// in 30.35V    2.66A
// out 15.38V   4.63A
// Eff = 88%
// losses = 9.52W

// tr = 7ns, tf = 7ns

// switching losses alone must be around 7.2W!!
// Excel estimate for the MOSFETs was 2.3W/FET, but the actual tr/tf is shorter -> should be less
// dialing in measured tr, tf, results in 1.4W/FET
// Let's assume 2W/fet, total 4W -> 3.2W in caps and inductors.

// tpeak at MOSFET = 91 degC on lab, without heatsink
// inductor 54 degC
// elcap (the hottest) 48 degC



void charger_test2()
{
	init_cpu_profiler();
	
	profile_cpu_blocking_40ms();
	profile_cpu_blocking_40ms();
	profile_cpu_blocking_40ms();
	profile_cpu_blocking_40ms();

	start_phab(0);
	delay_us(100);
	set_current(3600);
	while(1)
	{
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

		profile_cpu_blocking_40ms();
		delay_ms(500);
	}
}

void init_charger()
{
	dis_gate_pha();
	dis_gate_phb();
	IO_TO_GPO(GPIOG, 0);
	IO_TO_GPO(GPIOG, 1);

	infet_chargepump_0();
	IO_TO_GPO(GPIOE, 10);

	pulseout_0();
	IO_TO_GPO(GPIOE, 11);

	while(1)
	{

		charger_test2();
	}

}

int started = 0;
void start_phab(int start_b) __attribute__((section(".text_itcm")));
void start_phab(int start_b)
{
	if(started)
	{
		error(13);
	}
	started = 1;
	en_gate_pha();

	phb_disabled = !start_b;

	phase = 0;

	if(start_b)
		en_gate_phb();


	start_pump();

	SET_ADC12_VECTOR(charger_adc2_pha_inthandler);
	pha_lowcurr_fail_cnt = 0;
	phb_lowcurr_fail_cnt = 0;
	pha_finetune_fail_cnt = 0;
	phb_finetune_fail_cnt = 0;
	pha_err_integral = INITIAL_ERR_INTEGRAL;
	phb_err_integral = INITIAL_ERR_INTEGRAL;


	RCC->CFGR |= 1UL<<14; // Use CPU clock (400MHz) as HRTIM clock.
	__DSB();

	RCC->APB2ENR  |= 1UL<<29; // Enable HRTIM clock
	RCC->APB4ENR  |= 1UL<<14; // Enable COMP1,2 clock
	RCC->APB1LENR |= 1UL<<29; // Enable DAC1,2 clock. Only DAC1 is used by us; DAC2 may be used by others (audio 
				  // when writing this comment)
	

	// DAC1: Current setpoint (don't touch channel 2, beware: configuration registers are shared)
	DAC1->MCR |= 0b011UL<<0 /*Channel 1: Connected to on-chip peripherals only, with buffer disabled*/;
//	CURRLIM_DAC = MA_TO_DAC(6000);
	set_current(2000);
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

#define RETRIG (1UL<<4)
#define CONT   (1UL<<3)

	// Before the preload is enabled:
	HRTIM.ADC1R = 1UL<<28 /*ADC Trigger 1 on CHE CMP2*/ | 1UL<<24 /*..and also on CHD CMP2*/;

	HRTIM_CHE.DTxR = DEADTIME_FALLING_REG<<16 | DEADTIME_RISING_REG<<0 | 0b011<<10 /*prescaler = 1*/;
	HRTIM_CHD.DTxR = DEADTIME_FALLING_REG<<16 | DEADTIME_RISING_REG<<0 | 0b011<<10 /*prescaler = 1*/;

	HRTIM_MASTER.MCR = 1UL<<27 /*preload*/ | 0b101 /*prescaler = 1*/ | CONT | 1UL<<18 /*reset or PER triggers update*/;
	HRTIM_CHE.TIMxCR = 1UL<<27 /*preload*/ | 0b101 /*prescaler = 1*/ | 1UL<<18 /*reset or PER triggers update*/;
	HRTIM_CHD.TIMxCR = 1UL<<27 /*preload*/ | 0b101 /*prescaler = 1*/ | 1UL<<18 /*reset or PER triggers update*/;

	/*
		The event mapping documentation is completely fucked up by random
		name changes, incoherent, and partially non-documented. By reverse-engineering
		by trial and error, we found out the following:
		COMP1: EEV6, source regval=1
		
	*/
	HRTIM.EECR2 =
		1UL<<0 /*EEV6 source*/ | EV_ACTIVEHI<<2 | EV_LEVEL<<3 |
		1UL<<6 /*EEV7 source*/ | EV_ACTIVEHI<<8 | EV_LEVEL<<9;

// 0: no filter
// 1: 2 samples - 5ns extra latency
// 2: 4 samples - 10ns extra latency
// 3: 8 samples - 20ns extra latency
#define COMP_DIGITAL_FILTER 3
#if COMP_DIGITAL_FILTER < 0 || COMP_DIGITAL_FILTER > 3
#error Invalid COMP_DIGITAL_FILTER
#endif

	HRTIM.EECR3 = COMP_DIGITAL_FILTER<<6 /*EEV7*/ | COMP_DIGITAL_FILTER /*EEV6*/;
//	HRTIM.EEFxR = 0b0001UL<<1 /*EEV6 filter: blanking from reset to CMP1 (of our own unit)*/;

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


	// Confusion warning: RSTx1R is _output_ turn-off register. RSTxR is _counter_ reset register

	HRTIM_MASTER.MPER   = PERIOD;
	HRTIM_MASTER.MCMP1R = PERIOD/2;

	HRTIM_CHE.PERxR  = PERIOD;
	HRTIM_CHE.SETx1R = 1UL<<26 /*EEV6 (overcurr)*/ | 1UL<<7 /*MASTER PER*/;  // turns on bottom FET -> current starts decresing
	HRTIM_CHE.RSTxR  = 1UL<<4 /*MASTER PER*/;
	HRTIM_CHE.RSTx1R = 1UL<<3 /*CMP1*/;  // CMP1 turns off bottom FET -> current starts increasing

	
	HRTIM_CHD.PERxR  = PERIOD;
	HRTIM_CHD.SETx1R = 1UL<<27 /*EEV7 (overcurr)*/ | 1UL<<8 /*MASTER CMP1*/;  // turns on bottom FET -> current starts decresing
	HRTIM_CHD.RSTxR  = 1UL<<5 /*MASTER CMP1*/;
	HRTIM_CHD.RSTx1R = 1UL<<3 /*CMP1*/;  // CMP1 turns off bottom FET -> current starts increasing




//	HRTIM_CHD.SETx1R = 1UL<<27 /*EEV7*/; // Overcurrent turns on bottom FET
//	HRTIM_CHD.RSTx1R = 1UL<<26 /*EEV6*/; // Overcurrent of the master phase turns off the bottom FET, starting current-increasing cycle on the slave phase
//	HRTIM_CHD.RSTxR  = 1UL<<15 /*EEV7*/; // Our own overcurrent also starts a backup safety off-time counter
//	HRTIM_CHD.PERxR  = SAFETY_MAX_OFFTIME_REG;
//	HRTIM_CHD.CMP1xR = SAFETY_MAX_OFFTIME_REG-1;
//	HRTIM_CHD.TIMxDIER = 1UL; // Compare 1 interrupt

	IO_ALTFUNC(GPIOG,  6,  2); // CHE1: PHA LOFET
	IO_ALTFUNC(GPIOG,  7,  2); // CHE2: PHA HIFET
	IO_ALTFUNC(GPIOA, 11,  2); // CHD1: PHB LOFET
	IO_ALTFUNC(GPIOA, 12,  2); // CHD2: PHB HIFET

	/*

		Tested no-gap end-to-end inits and deinits of ADC2 just to be sure it wakes up after a deinit without extra delay needed:

		init_adc2();
		deinit_adc2();
		init_adc2();
		deinit_adc2();
		init_adc2();

		Test OK.
	*/

	init_adc2();


//	NVIC_SetPriority(HRTIM1_TIME_IRQn, 2);
//	NVIC_EnableIRQ(HRTIM1_TIME_IRQn);

	/*
		"How long it takes from the start_pump(), until Vinbus rises to Vin?"
		Test results:
		cha_vinbus_meas0 = 18968 (before)
		cha_vinbus_meas1 = 19484 (after full init, incl. ADC2 calibration)
		cha_vinbus_meas2 = 19466 (after 500us extra)
		cha_vinbus_meas3 = 19470 (500 us more)
		cha_vinbus_meas4 = 19470 (500 us more)
		cha_vinbus_meas5 = 19470 (500 us more)
		cha_vinbus_meas6 = 19448 (5000 us more)

		-> the ADC init itself is enough
		-> there is no massive harm done even if it's too quick - the initial duty is
		   calculated slightly wrong.

	*/

	// Based on input/output voltages, calculate the duty (offtime actually) we want to have:
	int desired_offtime = calc_check_offtime(INITIAL_FINETUNE);
	int actual_first_offtime, actual_second_offtime;

	// Now, the chances are, we need a longer offtime for the very first cycle to charge the bootstrap capacitor properly:
	if(desired_offtime >= INITIAL_DUTY_OFF)
	{
		// The desired offtime is acceptable as is:
		actual_first_offtime = actual_second_offtime = desired_offtime;
	}
	else
	{
		// Use the longer-than-desired offtime.
		actual_first_offtime = INITIAL_DUTY_OFF;
		// To compensate for the stored energy in the inductor, i.e., to get a net neutral magnetic field after two cycles,
		// the next cycle would have shorter-than-desired offtime.
		// Example: Desired 700, must have 800 first --> actual_first = 800. Actual second = 600. Average 700.
		actual_second_offtime = desired_offtime - (INITIAL_DUTY_OFF - desired_offtime);

		// This can easily go below the minimum offtime allowed, or even below zero. In this case, we can't compensate for it in
		// one cycle. Saturate it to the acceptable minimum, and let the ISR do its feedback magic.
		if(actual_second_offtime < MAX_DUTY_OFF)
			actual_second_offtime = MAX_DUTY_OFF;
	}

	SET_OFFTIME_PHA(actual_first_offtime);
	SET_OFFTIME_PHB(actual_first_offtime);



	HRTIM_CHE.SETx1R |= 1UL; // Software preposition the lofet on.
	HRTIM_CHD.SETx1R |= 1UL; // Software preposition the lofet on.
	DIS_IRQ();
	HRTIM_MASTER.MCNTR = PERIOD-1; // Make the master wrap around almost instantly.

//	HRTIM.CR2 = 1UL<<5 /*SW update CHE for the new CMPs*/ | 1UL<<4 /*same for CHD*/ |
//	            0UL<<8 /* SW reset MASTER*/ | 1UL<<13 /*SW reset CHE*/ | 1UL<<12 /*SW reset CHD*/;

	HRTIM_OUTEN_PHA();
	HRTIM.CR2 = 0b11111100111111UL; // Force software update on all timers & reset counters.
	HRTIM_MASTER.MCR |= 1UL<<21 /*Enable CHE*/ | 1UL<<20 /*Enable CHD*/ | 1UL<<16 /*Enable MASTER*/;
	delay_tenth_us(27); //finetuned for exact 1.5us delay for the first cycle

	if(start_b)
		HRTIM_OUTEN_PHB();

	// To the shadow registers, applies to the next cycle:
	SET_OFFTIME_PHA(actual_second_offtime);
	delay_tenth_us(8); //finetuned so that register update doesn't happen at CHD reset, which happens during this wait. Tested: "1" not enough, "2" is enough.
	SET_OFFTIME_PHB(actual_second_offtime);
	ENA_IRQ();
	#ifdef ENABLE_TRACE
		trace_at = 0;
	#endif

}


void stop_phab() __attribute__((section(".text_itcm")));
void stop_phab()
{
	started = 0;
	HRTIM_OUTDIS_PHA();
	HRTIM_OUTDIS_PHB();
	dis_gate_pha();
	dis_gate_phb();
	stop_pump();
	infet_chargepump_0();
	pulseout_0();

	// disconnect gate signals from HRTIM:
	IO_TO_GPI(GPIOG, 6);
	IO_TO_GPI(GPIOG, 7);
	IO_TO_GPI(GPIOA, 11);
	IO_TO_GPI(GPIOA, 12);

	deinit_adc2();

	COMP1->CFGR = 0;
	COMP2->CFGR = 0;

	DAC1->CR &= ~(1UL<<0); // Disable DAC channel 1, without touching channel 2

	// Reset HRTIM:
	RCC->APB2RSTR = (1UL<<29);
	__DSB();
	RCC->APB2RSTR = 0;

	// Reset COMPs:
	RCC->APB4RSTR = (1UL<<14);
	__DSB();
	RCC->APB4RSTR = 0;

	RCC->APB2ENR &= ~(1UL<<29); // HRTIM clock
	RCC->APB4ENR &= ~(1UL<<14); // COMP1,2 clock
	// We won't turn the DACs off due to other uses for DAC2.

}

void stop_b()  __attribute__((section(".text_itcm")));
void stop_b()
{

	HRTIM_OUTDIS_PHB();
	dis_gate_phb();
	phb_disabled = 1;
}



