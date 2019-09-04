/*
	RobotBoard firmware project
	
	ADC module: configures the three on-chip ADCs and relevant DMA channels

	(c) 2017-2019 Pulu Robotics and other contributors
	Maintainer: Antti Alhonen <antti.alhonen@iki.fi>

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License version 2, as 
	published by the Free Software Foundation.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	GNU General Public License version 2 is supplied in file LICENSING.
*/

#pragma once
#include <stdint.h>


/*
	All analog values are converted by ADC1, ADC2 and ADC3, all by using DMA.
	Some analog things are timing-critical, some are not - the required timing
	defines the order the values are converted in.

	Since DMA is used, data is stored in the memory in the same order.

	ADC sequence configuration, and the data structures need to be kept in sync.

	Some signals are connected to multiple ADCs, which is mentioned in comments.

	Note that channel numbers on separate ADCs are unique by ADC: for example,
	ADC1 1+ is a different channel to ADC3 1+. On the other hand, ADC12 7+ is the
	same channel 7 on both ADC1 and ADC2.

	While ADC1 channels need to reside in adc1_group, and so on for ADC2 and ADC3,
	there is some freedom in reorganizing channels shared by multiple ADCs (e.g.,
	there are quite a few ADC12 channels, and even some ADC123.)


	ADC1: MOTOR CONTROLLERS


	ADC1 is "dedicated" for motor controllers - i.e., synchronized (triggered) by advanced control timers
	(TIM1&TIM8). ADC1 is chosen for this purpose because one of the current sense signals (mc0_imeasb) is
	available to ADC1 only.

	To reduce DC link ripple, MC0 and MC1 run their PWMs 180 degrees out of phase. Hence, mc0_imeas* and
	mc1_imeas* are measured with a half a PWM cycle timing offset.

	ADC1 is triggered in the middle of the PWM cycle, once for MC0, then once for MC1.
	Due to this, ADC1 is run in discontinuous-grouped mode. Each trigger runs 5 conversions out of 10.



	ADC2: CHARGER

	Similarly, ADC2 is dedicated to the charger. Charger is a dual-phase synchronous buck converter.
	The phases are 180 degrees offset. ADC2 is triggered by the charger HRTIM, twice per full PWM cycle, 
	in two discountinuous groups.

	Charger fsw = 350kHz
	Charger period = around 3 us
	Min duty cycle is 18V/50V = 36%
	Increasing current for at least around 1 us
	Max duty cycle is around 25.2V/34V = 74%, let's say 80%
	-> minimum off-time will be 20%*3us = 600ns
	-> minimum off-time is around 35%*3us = 1000ns

	With 14-bit resolution, and 75 ns sampling, total time = 300ns
	-> two samples fit during off-time.
	-> three samples fit during minimum on-time.

	-> 5 samples fit max.



	ADC sampling times at 33.33MHz

	set	clk	ns
	0	1.5	45
	1	2.5	75
	2	8.5	255
	3	16.5	495
	4	32.5	975
			us
	5	64.5	1.935
	6	387.5	11.626
	7	810.5	24.317

	Conversion times
	reso	clk	ns
	16-bit	8.5	255
	14-bit	7.5	225
	12-bit	6.5	195
	10-bit	5.5	165
	8-bit	4.5	135
*/


#define VREF_MV 3300
#define ADC_BITS 14
#define DAC_BITS 14
#define ADCRANGE 16384
#define DACRANGE 4096

#define ADC_RDIV_LSB_TO_MV(val_, rhi_, rlo_) ( ( ((rhi_)+(rlo_))*((val_*VREF_MV) / (rlo_)) ) >> ADC_BITS)
#define ADC_RDIV_MV_TO_LSB(val_, rhi_, rlo_)  ( (  (rlo_)  *  (((val_)*ADCRANGE)/VREF_MV)  )/((rhi_)+(rlo_)) )
#define DAC_RDIV_LSB_TO_MV(val_, rhi_, rlo_) ( ( ((rhi_)+(rlo_))*((val_*VREF_MV) / (rlo_)) ) >> DAC_BITS)
#define DAC_RDIV_MV_TO_LSB(val_, rhi_, rlo_)  ( (  (rlo_)  *  (((val_)*DACRANGE)/VREF_MV)  )/((rhi_)+(rlo_)) )

// Massively slow. Needs calibration anyway. Replaced with a simple multiplication and shift
// ADC val is 14 bits. Shifting the result by 13 bits leaves max multiplier (with signed numbers just in case) at 2^4 = 16.

extern uint32_t cha_vinbus_mult;
extern uint32_t cha_vin_mult;
extern uint32_t vbat_mult;
extern uint32_t vapp_mult;
extern uint32_t vgapp_mult;
extern uint32_t vgplat_mult;

extern uint32_t vbat_per_vinbus_mult;



//#define CHA_VINBUS_MEAS_TO_MV(x_) (ADC_RDIV_LSB_TO_MV((x_), 464, 22))
#define CHA_VINBUS_MEAS_TO_MV(x_) (((x_)*cha_vinbus_mult)>>13)

//#define CHA_VIN_MEAS_TO_MV(x_)    (ADC_RDIV_LSB_TO_MV((x_), 474, 22))
#define CHA_VIN_MEAS_TO_MV(x_)    (((x_)*cha_vin_mult)>>13)

// #define VBAT_MEAS_TO_MV(x_)       (ADC_RDIV_LSB_TO_MV((x_), 468, 68))
#define VBAT_MEAS_TO_MV(x_)       (((x_)*vbat_mult)>>13)

#define VAPP_MEAS_TO_MV(x_)       (((x_)*vapp_mult)>>13)
#define VGAPP_MEAS_TO_MV(x_)      (((x_)*vgapp_mult)>>13)
#define VGPLAT_MEAS_TO_MV(x_)     (((x_)*vgplat_mult)>>13)

#define MV_TO_CHA_VINBUS_MEAS(x_) (ADC_RDIV_MV_TO_LSB((x_), 469, 22))
#define MV_TO_CHA_VIN_MEAS(x_)    (ADC_RDIV_MV_TO_LSB((x_), 469, 22))
#define MV_TO_VBAT_MEAS(x_)       (ADC_RDIV_MV_TO_LSB((x_), 469, 68))

#define AWD_VBAT_LO    MV_TO_VBAT_MEAS(15900) // 15.9V = 2.65 V/cell
#define AWD_VBAT_HI    MV_TO_VBAT_MEAS(26000) // 26.0V = 4.33 V/cell

#define AWD_CHA_VINBUS_LO   MV_TO_CHA_VINBUS_MEAS(14800) // 14.8V -> more than diode drop less from VBAT low limit

// Abs max charger voltage is 50.0V
// Due to noise, the quick-reacting watchdog needs to a bit over - 53V.
// Vdsmax for MOSFETs is 80V, but there is SW node overshoot due to fast switching.
// Prototype was tested with this limit set at 54V - SW node overshoot was measured at
// 70.4V max (over hundreds of thousands of cycles, running at 2.4A/phase) when the 54V limit triggered.
// So, there is around 9V of margin to the MOSFET rating. The MOSFETs are avalanche rated, so this should be enough.
// But DO NOT increase this limit over 54V.
#define AWD_CHA_VINBUS_HI   MV_TO_CHA_VINBUS_MEAS(53000) // Do not increase unless you know very well what you are doing.

// Hard-coded sanity limit checks, so that the AWDs have a chance of working at all.
#if (AWD_VBAT_LO < 100 || AWD_VBAT_HI > 16363)
	#error Please recheck AWD_VBAT settings.
#endif

#if (AWD_CHA_VINBUS_LO < 100 || AWD_CHA_VINBUS_HI > 16364)
	#error Please recheck AWD_CHA_VINBUS settings.
#endif

#define ADC1_SEQ_LEN 8
#define ADC1_DISCONTINUOUS_GROUP_LEN 4


#ifdef REV2A
	#define ADC1_SEQ 15,14,17, 4,2,7,18, 8, 0, 0, 0, 0, 0, 0, 0, 0
	// Which channels are used? LSb = ch 0
	#define ADC1_CHANNELS_IN_USE ((1<<15)|(1<<14)|(1<<17)|(1<<4)|(1<<2)|(1<<7)|(1<<18)|(1<<8))
#endif
#ifdef REV2B
	#define ADC1_SEQ 17,16,15, 4,2,7,18, 8, 0, 0, 0, 0, 0, 0, 0, 0
	#define ADC1_CHANNELS_IN_USE ((1<<17)|(1<<16)|(1<<15)|(1<<4)|(1<<2)|(1<<7)|(1<<18)|(1<<8))
#endif


#define ADC2_SEQ_LEN 2
#define ADC2_SEQ  9, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
#define ADC2_DISCONTINUOUS_GROUP_LEN 1
#define ADC2_CHANNELS_IN_USE ((1<<9)|(1<<5))


#define ADC3_SEQ_LEN 7

#ifdef REV2A
	#define ADC3_SEQ  5, 6,10, 1,13,12,18,0,0,0,0,0,0,0,0,0
	#define ADC3_CHANNELS_IN_USE ((1<<5)|(1<<6)|(1<<10)|(1<<1)|(1<<13)|(1<<12)|(1<<18)|(1<<14)|(1<<16))
#endif

#ifdef REV2B
	#define ADC3_SEQ  5, 6,10, 1,13,14,18,0,0,0,0,0,0,0,0,0
	#define ADC3_CHANNELS_IN_USE ((1<<5)|(1<<6)|(1<<10)|(1<<1)|(1<<13)|(1<<14)|(1<<18)|(1<<16)|(1<<9))
#endif


typedef union
{
	struct __attribute__((packed))
	{
		// CONVERSION GROUP 1: 4 items

		// Motor controller 1 phase B current measurement
		// REV2A: ADC12  15+ PA3
		// REV2B: ADC1   17+ PA1  (DIFFERENT!)
		uint16_t mc1_imeasb;

		// Motor controller 1 phase C current measurement
		// REV2A: ADC12  14+ PA2
		// REV2B: ADC1   16+ PA0  (DIFFERENT!)
		uint16_t mc1_imeasc;

		// Platform power switch MOSFET&fuse temperature NTC
		// REV2A: ADC1   17+ PA1
		// REV2B: ADC12  15+ PA3  (DIFFERENT!)
		uint16_t bms_temp_plat_mosfets; 

		// Charger input voltage
		// REV2A: ADC12  4+  PC4
		// REV2B: ADC12  4+  PC4  (same)
		uint16_t cha_vin_meas;



		// CONVERSION GROUP 2: 4 items

		// Motor controller 0 phase B current measurement
		// REV2A: ADC1   2+  PF11
		// REV2B: ADC1   2+  PF11  (same)
		uint16_t mc0_imeasb;

		// Motor controller 0 phase C current measurement
		// REV2A: ADC12  7+  PA7
		// REV2B: ADC12  7+  PA7  (same)
		uint16_t mc0_imeasc;

		// Super important:
		// vbat_meas and cha_vinbus_meas need to be next to each other, vbat first, and aligned(4), meaning there
		// MUST BE AN EVEN NUMBER of entries before these two guys here.
		// These two are used in a very timing-critical part in charger to calculate their ratio
		// (vbat/cha_vinbus), and being able to load them using one instruction is critical.
		// See the "quick" union field below.

		// Battery voltage
		// REV2A: ADC12  18+ PA4
		// REV2B: ADC12  18+ PA4  (same)
		uint16_t vbat_meas;

		// Charger input voltage after the input ideal diode MOSFET
		// REV2A: ADC12  8+  PC5
		// REV2B: ADC12  8+  PC5  (same)
		uint16_t cha_vinbus_meas;
	} s;

	uint16_t b[ADC1_SEQ_LEN];

	struct __attribute__((packed))
	{
		uint16_t dummy[6];
		uint32_t vbat_and_vinbus;
	} quick;	
} adc1_group_t;
extern volatile adc1_group_t adc1;

// Sample times from channel 0 to channel 19
// Longer time for vbat_meas, same channel (18) for REV2A, REV2B
#define ADC1_SMPTIMES 2,2,2,2,2, \
                      2,2,2,2,2, \
                      2,2,2,2,2, \
                      2,2,2,3,2



// ADC2 is configured separately, in a different way, utilizing HW oversampling mode.
// DMA is not used, data registers are read directly in charger control ISR, so there is
// no data structure.
// Sample times from channel 0 to channel 19
#define ADC2_SMPTIMES 1,1,1,1,1, \
                      1,1,1,1,1, \
                      1,1,1,1,1, \
                      1,1,1,1,1



// ADC3 freeruns, some channels use a long sampling time. Nothing timing-critical here.
// bms_appfet_g_meas and vapp_meas are missing on purpose - they, as the only timing-critical entities, 
// are in the injected sequence

typedef union
{
	struct __attribute__((packed))
	{
		// 3DTOF sensor stray light estimate measurement (phototransistor), muxed from active sensor
		// REV2A: ADC3   5+  PF3
		// REV2B: ADC3   5+  PF3  (same)
		uint16_t epc_stray_estimate;
    
		// Extension B analog in 1
		// REV2A: ADC3   6+  PF10
		// REV2B: ADC3   6+  PF10  (same)
		uint16_t eb_analog1;

		// Extension B analog in 2
		// REV2A: ADC123 10+ PC0
		// REV2B: ADC123 10+ PC0  (same)
		uint16_t eb_analog2;

		// Main power switch MOSFET gate voltage
		// REV2A: ADC3   1+  PC3
		// REV2B: ADC3   1+  PC3  (same)
		uint16_t bms_mainfet_g_meas;

		// Application power switch MOSFET&fuse temperature NTC
		// REV2A: ADC3   13+ PH2
		// REV2B: ADC3   13+ PH2  (same)
		uint16_t bms_temp_app_mosfets;

		// Battery temperature NTC
		// REV2A: ADC123 12+ PC2
		// REV2B: ADC123 14+ PH3  (DIFFERENT!)
		uint16_t bms_temp_battery;

		// CPU temperature (on-chip sensor)
		// REV2A: ADC3   18+ internal
		// REV2A: ADC3   18+ internal  (same)
		uint16_t cpu_temp; 
	} s;
	uint16_t b[ADC3_SEQ_LEN];

} adc3_group_t;
extern volatile adc3_group_t adc3;


// Injected channels:
// Application power switch MOSFET gate voltage         <--- IN INJECTED SEQUENCE
// REV2A: ADC3   14+ PH3   
// REV2B: ADC3   16+ PH5  (DIFFERENT!)

// App voltage meas   <--- IN INJECTED SEQUENCE
// REV2A: ADC3   16+ PH5
// REV2B: ADC3   9+  PF4  (DIFFERENT!)


#define ADC3_VAPP_DATAREG  (ADC3->JDR1)
#define ADC3_VGAPP_DATAREG (ADC3->JDR2)


// Sample times from channel 0 to channel 19
// CPU_TEMP absolutely needs the long sample time, it's a high-impedance source with no
// amplifier nor a capacitor.

// All channels use a very long sample time, to purposedly slow down the
// ADC. The channels (except internal CPU_TEMP) do have 100nF caps to provide
// low AC impedance, so they could be sampled quickly, but this cap diminishes if sampled
// too frequently, so need to limit sample rate anyway. Easiest way is to use long sampling
// and let the ADC free run. It could be possible to later cost-optimize the 100n caps away.
// Injected channels (vapp, vgapp) must use shorter sample times: setting 3 is 16.5 ADC clk's,
// so total conversion time is 2 * (16.5 + 7.5) * 1/33.3MHz  = 1.44 us.
#ifdef REV2A

	#define ADC3_SMPTIMES 6,6,6,6,6, \
		              6,6,6,6,6, \
		              6,6,6,6,3, \
		              6,3,6,6,6

#endif

#ifdef REV2B

	#define ADC3_SMPTIMES 6,6,6,6,6, \
		              6,6,6,6,3, \
		              6,6,6,6,6, \
		              6,3,6,6,6

#endif

#if 0
	// Convenient in testing, if everything fails:
	#ifdef DEFINE_VARS
	const char* const adc1_names[ADC1_SEQ_LEN] =
	{
		"mc1_imeasb",
		"mc1_imeasc",
		"bms_temp_plat_mosfets",
		"cha_vin_meas",
		"mc0_imeasb",
		"mc0_imeasc",
		"vbat_meas",
		"cha_vinbus_meas"
	};

	const char* const adc3_names[ADC3_SEQ_LEN] =
	{
		"epc_stray_estimate",
		"eb_analog1",
		"eb_analog2",
		"bms_mainfet_g_meas",
		"bms_temp_app_mosfets",
		"bms_temp_battery",
		"cpu_temp"
	};

	#else
	extern const char* const adc1_names[ADC1_SEQ_LEN];
	extern const char* const adc3_names[ADC3_SEQ_LEN];

	#endif
#endif

#define ADC1_DMA DMA2
#define ADC1_DMA_STREAM DMA2_Stream1
#define ADC1_DMA_STREAM_NUM 1
#define ADC1_DMA_STREAM_IRQ DMA2_Stream1_IRQn
#define ADC1_DMAMUX() do{DMAMUX1_Channel9->CCR = 9;}while(0)

#define ADC2_DMA DMA2
#define ADC2_DMA_STREAM DMA2_Stream2
#define ADC2_DMA_STREAM_NUM 2
#define ADC2_DMA_STREAM_IRQ DMA2_Stream2_IRQn
#define ADC2_DMAMUX() do{DMAMUX1_Channel10->CCR = 10;}while(0)

#define ADC3_DMA DMA2
#define ADC3_DMA_STREAM DMA2_Stream3
#define ADC3_DMA_STREAM_NUM 3
#define ADC3_DMA_STREAM_IRQ DMA2_Stream3_IRQn
#define ADC3_DMAMUX() do{DMAMUX1_Channel11->CCR = 115;}while(0)


void init_adcs();
void init_adc2();
void deinit_adc2();

#define ADC3_CONV_INJECTED() do{ADC3->CR |= 1UL<<3;}while(0)

