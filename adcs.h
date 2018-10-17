/*
	RobotBoard firmware project
	
	ADC module: configures the three on-chip ADCs and relevant DMA channels

	(c) 2017-2018 Pulu Robotics and other contributors
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


#define CHA_VINBUS_MEAS_TO_MV(x_) (ADC_RDIV_LSB_TO_MV((x_), 464, 22))
#define CHA_VIN_MEAS_TO_MV(x_)    (ADC_RDIV_LSB_TO_MV((x_), 474, 22))
#define VBAT_MEAS_TO_MV(x_)       (ADC_RDIV_LSB_TO_MV((x_), 475, 68))

#define MV_TO_CHA_VINBUS_MEAS(x_) (ADC_RDIV_MV_TO_LSB((x_), 464, 22))
#define MV_TO_CHA_VIN_MEAS(x_)    (ADC_RDIV_MV_TO_LSB((x_), 474, 22))
#define MV_TO_VBAT_MEAS(x_)       (ADC_RDIV_MV_TO_LSB((x_), 475, 68))

#define AWD_VBAT_LO    MV_TO_VBAT_MEAS(14000) // 15.0V = 2.5 V/cell
#define AWD_VBAT_HI    MV_TO_VBAT_MEAS(18000) // 26.0V = 4.33 V/cell

#define AWD_CHA_VINBUS_LO   MV_TO_CHA_VINBUS_MEAS(14000) // 14.0V -> more than diode drop less from VBAT low limit
#define AWD_CHA_VINBUS_HI   MV_TO_CHA_VINBUS_MEAS(35000) // 55V quick-reacting absolute maximum on Vinbus (Vdsmax for MOSFETs = 80V)

// Hard-coded sanity limit checks, so that the AWDs have a chance of working at all.
#if (AWD_VBAT_LO < 100 || AWD_VBAT_HI > 16364)
	#error Please recheck AWD_VBAT settings.
#endif

#if (AWD_CHA_VINBUS_LO < 100 || AWD_CHA_VINBUS_HI > 16364)
	#error Please recheck AWD_CHA_VINBUS settings.
#endif

#define ADC1_SEQ_LEN 10
#define ADC1_SEQ  2, 7,16,17, 4,15,14,18,12, 8, 0, 0, 0, 0, 0, 0
#define ADC1_DISCONTINUOUS_GROUP_LEN 5

// Which channels are used? LSb = ch 0
#define ADC1_CHANNELS_IN_USE ((1<<2)|(1<<7)|(1<<16)|(1<<17)|(1<<4)|(1<<15)|(1<<14)|(1<<18)|(1<<12)|(1<<8))

#define ADC2_SEQ_LEN 2
#define ADC2_SEQ  9, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
#define ADC2_DISCONTINUOUS_GROUP_LEN 1
#define ADC2_CHANNELS_IN_USE ((1<<9)|(1<<5))

#define ADC3_SEQ_LEN 9
#define ADC3_SEQ  5, 6,10, 1,14,15,16,13,18,0,0,0,0,0,0,0
#define ADC3_CHANNELS_IN_USE ((1<<5)|(1<<6)|(1<<10)|(1<<1)|(1<<14)|(1<<15)|(1<<16)|(1<<13)|(1<<18))

typedef union
{
	struct __attribute__((packed))
	{
		// CONVERSION GROUP 1: 5 items

		uint16_t mc0_imeasb;            // ADC1   2+  PF11  Motor controller 0 phase B current measurement
		uint16_t mc0_imeasc;            // ADC12  7+  PA7   Motor controller 0 phase C current measurement

		uint16_t bms_temp_contacts;     // ADC1   16+ PA0   Charger contact temperature NTC
		uint16_t bms_temp_plat_mosfets; // ADC1   17+ PA1   Platform power switch MOSFET&fuse temperature NTC
		uint16_t cha_vin_meas;          // ADC12  4+  PC4   Charger input voltage

		// CONVERSION GROUP 2: 5 items

		uint16_t mc1_imeasb;            // ADC12  15+ PA3   Motor controller 1 phase B current measurement
		uint16_t mc1_imeasc;            // ADC12  14+ PA2   Motor controller 1 phase C current measurement

		uint16_t vbat_meas;             // ADC12  18+ PA4   Battery voltage
		uint16_t bms_temp_battery;      // ADC123 12+ PC2   Battery temperature NTC
		uint16_t cha_vinbus_meas;       // ADC12  8+  PC5   Charger input voltage after the input ideal diode MOSFET
	} s;
	uint16_t b[ADC1_SEQ_LEN];
} adc1_group_t;
extern volatile adc1_group_t adc1;

// Sample times from channel 0 to channel 19
#define ADC1_SMPTIMES 2,2,2,2,2, \
                      2,2,2,2,2, \
                      2,2,2,2,2, \
                      2,2,2,3,2

#if 0
typedef union
{
	struct __attribute__((packed))
	{
		// CONVERSION GROUP 1:
		uint16_t cha_currmeasa[3];      // ADC12  9+  PB0   Charger sync buck phase A inductor current
		// free time to convert other things

		// CONVERSION GROUP 2:
		uint16_t cha_currmeasb[3];      // ADC12  5+  PB1   Charger sync buck phase B inductor current
		// free time to convert other things

	} s;
	uint16_t b[ADC2_SEQ_LEN];
} adc2_group_t;
extern volatile adc2_group_t adc2;
#endif

// Sample times from channel 0 to channel 19
#define ADC2_SMPTIMES 1,1,1,1,1, \
                      1,1,1,1,1, \
                      1,1,1,1,1, \
                      1,1,1,1,1

typedef union
{
	struct __attribute__((packed))
	{
		uint16_t epc_stray_estimate;    // ADC3   5+  PF3   3DTOF sensor stray light estimate measurement (phototransistor), muxed from active sensor
		uint16_t eb_analog1;            // ADC3   6+  PF10  Extension B analog in 1
		uint16_t eb_analog2;            // ADC123 10+ PC0   Extension B analog in 2
		uint16_t bms_mainfet_g_meas;    // ADC3   1+  PC3   Main power switch MOSFET gate voltage
		uint16_t bms_appfet_g_meas;     // ADC3   14+ PH3   Application power switch MOSFET gate voltage
		uint16_t bms_vref3;             // ADC3   15+ PH4   TI BMS chip 3.0V reference voltage
		uint16_t bms_vmeas;             // ADC3   16+ PH5   TI BMS chip cell measurement voltage
		uint16_t bms_temp_app_mosfets;  // ADC3   13+ PH2   Application power switch MOSFET&fuse temperature NTC
		uint16_t cpu_temp;              // ADC3   18+ internal
	} s;
	uint16_t b[ADC3_SEQ_LEN];

} adc3_group_t;
extern volatile adc3_group_t adc3;





// Sample times from channel 0 to channel 19
#define ADC3_SMPTIMES 4,4,4,4,4, \
                      4,4,4,4,4, \
                      4,4,4,4,4, \
                      4,4,4,5,4

#ifdef DEFINE_VARS
const char* const adc1_names[ADC1_SEQ_LEN] =
{
	"mc0_imeasb",
	"mc0_imeasc",
	"bms_temp_contacts",
	"bms_temp_plat_mosfets",
	"cha_vin_meas",
	"mc1_imeasb",
	"mc1_imeasc",
	"vbat_meas",
	"bms_temp_battery",
	"cha_vinbus_meas"
};

#if 0
const char* const adc2_names[ADC2_SEQ_LEN] =
{
	"cha_currmeasa[0]",
	"cha_currmeasa[1]",
	"cha_currmeasb[0]",
	"cha_currmeasb[1]"
};
#endif

const char* const adc3_names[ADC3_SEQ_LEN] =
{
	"epc_stray_estimate",
	"eb_analog1",
	"eb_analog2",
	"bms_mainfet_g_meas",
	"bms_appfet_g_meas",
	"bms_vref3",
	"bms_vmeas",
	"bms_temp_app_mosfets",
	"cpu_temp"
};

#else
extern const char* const adc1_names[ADC1_SEQ_LEN];
//extern const char* const adc2_names[ADC2_SEQ_LEN];
extern const char* const adc3_names[ADC3_SEQ_LEN];

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


