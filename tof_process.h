/*
	RobotBoard firmware project

	3D Time-of-Flight sensor data processing. Datatypes to store sensor calibration, image data,
	and functions to process the raw data into accurate distance measurements.


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

#define TOF_XS 160
#define TOF_YS 60

/*
	TOF calibration data is the dominating source of flash storage - if we followed the Espros'
	recommendations for storing the calibration data, we would run out of flash with just a few sensors.

	To store 10 sensors, we need to implement some trickery to reduce storaging redundant informaton. Generic data compression
	would be inefficient, but realizing that most of the error in any type of error source is mostly common mode
	error for all pixels, with only minor differences between the pixels. Separating the errors to global and per-pixel
	errors causes a slight runtime calculation penalty, but not necessarily much, since we now need to read less RAM
	during the processing. Also, with a smaller calibration structure, it fits completely inside fast RAM.
	Using SIMD instructions, processing will be efficient even when extra operations are needed to combine global
	and per-pixel calibration.

	From 16 flash sectors, 12 can be dedicated to TOF calibration data, leaving enough margin for all code, and 
	for other modules (e.g., IMU calibration tables, audio data).

	Thus, 153.6*1024 bytes is available per sensor. When divided for two different modulation frequencies, 76.8*1024
	bytes is left. This fits nicely in core-coupled 128*1024 DTCM RAM, if necessary, or in SRAM1, or in SRAM2.


	
*/

typedef struct __attribute__((packed))
{
	// Base offset error for each pixel. This error combines total offset error from all modulation and demodulation paths,
	// for each pixel. In mm.
	int16_t  perpix_offsets[TOF_XS*TOF_YS];             // 9600 bytes

	// 
	int8_t   global_linearity[256];                     // 256 bytes
	uint8_t  perpix_linearity[32][TOF_XS*TOF_YS/4];     // 76800 bytes

	uint16_t global_amblight_corr[256];                 // 256 bytes
	uint8_t  perpix_amblight_corr[8][TOF_XS*TOF_YS/4];  // 19200 bytes
	

} tof_sensor_calib_t;
