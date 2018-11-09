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

#define TOF_XS (160)
#define TOF_YS (60)

#define TOF_XS_NARROW (32)
#define TOF_YS_NARROW (44)

#define TOF_NARROW_Y_START (8)
#define TOF_NARROW_X_START (64)

typedef struct __attribute__((packed))
{
	uint16_t start_pad[2];
	uint16_t img[TOF_XS*TOF_YS];
} epc_img_t;

typedef struct __attribute__((packed))
{
//	uint16_t start_pad[2];
	uint16_t img[TOF_XS_NARROW*TOF_YS_NARROW];
} epc_img_narrow_t;

typedef struct __attribute__((packed))
{
	epc_img_t dcs[4];
} epc_4dcs_t;

typedef struct __attribute__((packed))
{
	epc_img_narrow_t dcs[4];
} epc_4dcs_narrow_t;

typedef struct __attribute__((packed))
{
	epc_img_t dcs[2];
} epc_2dcs_t;

typedef struct __attribute__((packed))
{
	epc_img_narrow_t dcs[2];
} epc_2dcs_narrow_t;


#define SIZEOF_MONO (sizeof(epc_img_t))
#define SIZEOF_2DCS (sizeof(epc_2dcs_t))
#define SIZEOF_2DCS_NARROW (sizeof(epc_2dcs_narrow_t))
#define SIZEOF_4DCS (sizeof(epc_4dcs_t))
#define SIZEOF_4DCS_NARROW (sizeof(epc_4dcs_narrow_t))


void tof_calc_dist_ampl(uint8_t *ampl_out, uint16_t *dist_out, epc_4dcs_t *in, int offset_mm, int clk_div);
void tof_calc_dist_ampl_narrow(uint8_t *ampl_out, uint16_t *dist_out, epc_4dcs_narrow_t *in, int offset_mm, int clk_div);
void tof_calc_ampl_hdr(uint8_t *ampl_out, uint8_t* long_in, uint8_t* short_in);
void calc_toofar_ignore_from_2dcs(uint8_t *ignore_out, epc_4dcs_t *in, int threshold /*mm*/, int offset_mm, int clk_div);
void calc_interference_ignore_from_2dcs(uint8_t *ignore_out, epc_4dcs_t *in, int threshold);
void tof_calc_dist_3hdr_with_ignore(uint16_t* dist_out, uint8_t* ampl, uint16_t* dist, uint8_t* ignore_in);
void tof_calc_dist_3hdr_with_ignore_with_straycomp(uint16_t* dist_out, uint8_t* ampl, uint16_t* dist, uint8_t* ignore_in, uint16_t stray_ampl, uint16_t stray_dist);
void tof_remove_midliers(uint16_t* out, uint16_t* in);
void process_bw(uint8_t *out, epc_img_t *in);
void process_dcs(int16_t *out, epc_img_t *in);
void process_dcs_narrow(int16_t *out, epc_img_narrow_t *in);





#define LINEARITY_NGROUPS 256  // Max 256 to hold the index in 8 bits
#define LINEARITY_NSTEPS 64

#define AMBLIGHT_NGROUPS 16    // Max 16 to hold the index in 4 bits
#define AMBLIGHT_NSTEPS 32

#define TEMPERATURE_NSTEPS 3

/*
	TOF calibration data is the dominating source of flash storage - if we followed the Espros'
	recommendations for storing the calibration data, we would run out of flash with just a few sensors.

	To store 10 sensors, we need to implement some trickery to reduce redundant informaton. Generic data compression
	would be inefficient, but realizing that most of the error in any type of error source is mostly common mode
	error for all pixels, with only minor differences between the pixels. 

	Separating the errors to global and per-pixel
	errors causes a slight runtime calculation penalty, but not necessarily much, since we now need to read less RAM
	during the processing. Also, with a smaller calibration structure, it fits completely inside fast RAM.
	Using SIMD instructions, processing will be efficient even when extra operations are needed to combine global
	and per-pixel calibration.

	It also seems that the errors fall into an uneven distribution: a group of pixels perform close to each other,
	then another group of pixels perform again similarly. This is because some error sources cause clear repeating patterns
	(such as even/odd lines performing differently), and some error sources cause spatially slowly-changing patterns
	(such as the spatial distance from the midpoint of the sensor causing different amount of distortion). Additionally,
	there will be clear outlier pixels, which are almost "dead" pixel but perform well enough to be accepted, but with
	non-typical calibration. It is expected the number of such pixels are low enough so that they'll fall within their
	own groups (one group per such a pixel).

	This means, instead of storing full calibration for every 9600 pixels, we can model a smaller subset, and round
	every pixel to fit the nearest group.

	From 16 flash sectors, 12 can be dedicated to TOF calibration data, leaving enough margin for all code, and 
	for other modules (e.g., IMU calibration tables, audio data).

	Thus, 153.6*1024 bytes is available per sensor. When divided for two different modulation frequencies, 76.8*1024
	bytes is left. This fits nicely in core-coupled 128*1024 DTCM RAM, if necessary, or in SRAM1, or in SRAM2.


	
*/

typedef struct __attribute__((packed))
{

	/*
		Per-pixel index that tells us in which ambient light correction group the pixel belongs to.
		4 bits per pixel: max 16 groups. (Ambient light correction is less critical, and the proposed
		solutions elsewhere never bother to do any per-pixel calibration, so 16 groups will be much
		better than the established baseline of single correction, anyway.)

		For each group of pixels, for each level of ambient light, there is a correction for both DCS2-DCS0 ([0]), and
		DCS3-DCS1 ([1]).

		Linear interpolation between the two steps is recommended but may not be necessary.

		Unit: Espros ADC LSB (corrections are applied to DCS3-DCS1, and DCS2-DCS0 raw values before distance
		calculation)
	*/
	uint8_t  perpix_amblight_group_idx[TOF_XS*TOF_YS/2];                 // 4800 bytes
	int16_t  pergroup_amblight[AMBLIGHT_NGROUPS][AMBLIGHT_NSTEPS][2];    // 2048 bytes


	/*
		Base offset error for the complete sensor. This error combines total offset error from both modulation (LED)
		and demodulation (pixel) paths, so that per-pixel corrections fit in smaller range. Unit: 2mm.
	*/
	int16_t  global_offset;                             // 2 bytes


	/*
		Base offset error for each pixel. This error combines total offset error from both modulation (LED)
		and demodulation (pixel) paths, separately for each pixel. To be summed with global_offset. Unit: 2mm.
	*/
	int8_t  perpix_offsets[TOF_XS*TOF_YS];              // 9600 bytes


	/*
		Per-pixel index that tells us in which linearity correction group the pixel belongs to.
	*/
	uint8_t  perpix_linearity_group_idx[TOF_XS*TOF_YS]; // 9600 bytes


	/*
		Distance nonlinearity correction table, combining all linearity and gain errors in calculated distance,
		regardless of their original source.

		Since we don't have memory to store linearity table for all TOF_XS*TOF_YS pixels separately, similarly behaving
		pixels are grouped and pointed to by perpix_linearity_group_idx[].

		Each linearity table is indexed by the value, which is already corrected for global_offset and perpix_offset,
		and runs from 0 mm to unambiguity range. This range is compressed to indexes from 0 to TOF_CALIB_LINEARITY_NSTEPS.
		Linear interpolation between the two steps is strongly recommended.

		Unit: 2mm

	*/
	int8_t   pergroup_linearity[LINEARITY_NGROUPS][LINEARITY_NSTEPS];    // 16384 bytes


	/*
		Temperature correction table.

		Temperature causes distance shift due to changed electron mobility in silicon. The drift happens
		in modulation logic, FET drivers, and LEDs (common to all pixels), and in demodulation logic and
		pixel demodulators (partially differing for each pixel).

		Industry baseline is to compensate all pixels with a single temperature coefficient - pixels are
		close enough. It's OK, but we can do better.

		Fixed point math will be used:

		dist_corrected = dist - (global_temp_coeff (int32) + perpix_temp_coeff[pixel] (uint4)) * (actual_temp - calibration_temp)

		Because the temperature correction may be slightly nonlinear, the coefficient is not exactly a constant.

		This is why several steps with different constants are used. Interpolate linearly between the constants.
	*/

	// Calibration temperature
	int16_t temp_coeffs_measured_at[TEMPERATURE_NSTEPS];


	int32_t global_temp_coeff[TEMPERATURE_NSTEPS];    // 12 bytes

	uint8_t perpix_temp_coeff[TEMPERATURE_NSTEPS][TOF_XS*TOF_YS/2]; // 14400 bytes

	
} tof_chip_calib_t;


/*
	Optical calibration for a sensor.

	Calibration is fixed for a single rotation of the sensor. For different rotations, the data itself needs to be rotated.
	This way, a lot of runtime calculation is saved. The obvious limitation is that the sensors cannot be installed in
	rotating arms, etc. Rotating a sensor requires calibration data update (which can be made a quick and simple operation,
	but still comparable to firmware update.)

*/

typedef struct __attribute__((packed))
{
	/*
		Angles the pixels point at.

		The final point is calculated by:
		x = d * cos(pix_ver_ang + sensor_ver_ang) * cos(pix_hor_ang + sensor_hor_ang) + sensor_x;
		y = d * cos(pix_ver_ang + sensor_ver_ang) * sin(pix_hor_ang + sensor_hor_ang) + sensor_y;
		z = d * sin(pix_ver_ang + sensor_ver_ang) + sensor_z;
		
		Horizontal angle is positive CCW. Zero looking right on the standard UI screen - East for the recommended standard notation.
		Vertical angle is positive upwards. Zero looking forward.

		Angle unit is, like everywhere, so that full data type range corresponds for 360 degrees.
		These angles are, naturally, limited between -90...+90 degrees, so we could add one bit of extra precision by shifting, but no need
		for it right now - with 16-bit range, the resolution is about 0.005 degrees.
	*/

	int16_t perpix_ver_angs[TOF_XS*TOF_YS];
	int16_t perpix_hor_angs[TOF_XS*TOF_YS];

	/*
		TODO:

		Parameters for lens blur & stray light model. Brute-forcing/lookup-tabling these would get huge; so
		they will be simple, small parameters generating a "good enough" model. Not expecting much memory
		footprint for this.
	*/


} tof_optical_calib_t;


typedef struct __attribute__((packed))
{
	tof_chip_calib_t    chip_calib_at_freq[2];
	tof_optical_calib_t optical_calib;
} tof_calib_t;



