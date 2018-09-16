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

*/

typedef struct __attribute__((packed))
{
	uint16_t bms_temp_contacts;     // ADC1   16+ PA0   Charger contact temperature NTC
	uint16_t bms_temp_plat_mosfets; // ADC1   17+ PA1   Platform power switch MOSFET&fuse temperature NTC
	uint16_t mc0_imeasb;            // ADC1   2+  PF11  Motor controller 0 phase B current measurement
	uint16_t mc0_imeasc;            // ADC12  7+  PA7   Motor controller 0 phase C current measurement
} adc1_group_t;

typedef struct __attribute__((packed))
{
	uint16_t mc1_imeasb;            // ADC12  15+ PA3   Motor controller 1 phase B current measurement
	uint16_t mc1_imeasc;            // ADC12  14+ PA2   Motor controller 1 phase C current measurement
	uint16_t vbat_meas;             // ADC12  18+ PA4   Battery voltage
	uint16_t cha_vin_meas;          // ADC12  4+  PC4   Charger input voltage
	uint16_t cha_vinbus_meas;       // ADC12  8+  PC5   Charger input voltage after the input ideal diode MOSFET
	uint16_t cha_currmeasa;         // ADC12  9+  PB0   Charger sync buck phase A inductor current
	uint16_t cha_currmeasb;         // ADC12  5+  PB1   Charger sync buck phase B inductor current
} adc2_group_t;

typedef struct __attribute__((packed))
{
	uint16_t epc_stray_estimate;    // ADC3   5+  PF3   3DTOF sensor stray light estimate measurement (phototransistor), muxed from active sensor
	uint16_t eb_analog1;            // ADC3   6+  PF10  Extension B analog in 1
	uint16_t eb_analog2;            // ADC123 10+ PC0   Extension B analog in 2
	uint16_t bms_mainfet_g_meas;    // ADC3   1+  PC3   Main power switch MOSFET gate voltage
	uint16_t bms_appfet_g_meas;     // ADC3   14+ PH3   Application power switch MOSFET gate voltage
	uint16_t bms_vref3;             // ADC3   15+ PH4   TI BMS chip 3.0V reference voltage
	uint16_t bms_vmeas;             // ADC3   16+ PH5   TI BMS chip cell measurement voltage
	uint16_t bms_temp_app_mosfets;  // ADC3   13+ PH2   Application power switch MOSFET&fuse temperature NTC
	uint16_t bms_temp_battery;      // ADC123 12+ PC2   Battery temperature NTC
	uint16_t cpu_temp;              // ADC3   18+ internal
} adc3_group_t;

#define ADC1_SEQ_LEN 4
#define ADC1_SEQ 16,17,2,7,0,0,0,0,0,0,0,0,0,0,0,0

#define ADC2_SEQ_LEN 7
#define ADC2_SEQ 15,14,18,4,8,9,5,0,0,0,0,0,0,0,0,0

#define ADC3_SEQ_LEN 10
#define ADC3_SEQ 5,6,10,1,14,15,16,13,12,18,0,0,0,0,0,0


