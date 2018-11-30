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

#include "tof_ctrl.h" // for N_SENSORS

typedef struct __attribute__((packed))
{
	uint16_t start_pad[2];
	uint16_t img[TOF_XS*TOF_YS];
} epc_img_t;

typedef struct __attribute__((packed))
{
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


void copy_cal_to_shadow(int sid, int f);
void compensated_tof_calc_dist_ampl(uint8_t *max_ampl_out, uint8_t *ampl_out, uint16_t *dist_out, epc_4dcs_t *in, epc_img_t *bw);
void compensated_tof_calc_dist_ampl_narrow(uint8_t *max_ampl_out, uint8_t *ampl_out, uint16_t *dist_out, epc_4dcs_narrow_t *in, epc_img_t *bwimg);

int32_t calc_widnar_correction(int32_t* corr, uint8_t *wid_ampl, uint16_t *wid_dist, uint8_t *nar_ampl, uint16_t *nar_dist);

void calc_stray_estimate(uint8_t *ampl_in, uint16_t *dist_in, uint16_t *stray_ampl, uint16_t *stray_dist);


/*
	TOF calibration data is the dominating source of flash storage - if we followed the Espros'
	recommendations for storing the calibration data, we would run out of flash with just one sensor.

	To store 10 sensors, we need to implement some trickery to reduce redundant informaton. Generic data compression
	would be inefficient,
	From 16 flash sectors, 12 can be dedicated to TOF calibration data, leaving enough margin for all code, and 
	for other modules (e.g., IMU calibration tables, audio data).
	


	All 4-bit fields follow little endianness: the first pixel index is stored in &0x0f, the second in &0xf0.
*/


#define WID_N_PIXGROUPS 12
#define NAR_N_PIXGROUPS 8

#define TOF_TBL_SEG_LEN 96

#define COMP_AMBIENT_INTLEN_US 100 // it's important to follow this in both calibration and runtime

// Bigger calibration dataset for important, accurate ranges
typedef struct __attribute__((packed))
{
	/*
		Full distance LUTs for two frequencies (20MHz and 10MHz).
		16 pixel groups
		8 segments of the compensated atan2 function (atan2(a,b) + correction), so that a>0, b>0, b>a, hence 8 segments with swapped polarities / order
		The table reads the distance in mm directly.

		lut_group_id (4 bit per pixel) tells which group the pixel belongs to.
	
	*/
	uint8_t  wid_lut_group_ids[TOF_XS*TOF_YS/2];
	uint16_t wid_luts[WID_N_PIXGROUPS][8][TOF_TBL_SEG_LEN];

	uint8_t  nar_lut_group_ids[TOF_XS_NARROW*TOF_YS_NARROW/2];
	uint16_t nar_luts[NAR_N_PIXGROUPS][8][TOF_TBL_SEG_LEN];


	/*
		Ambient light correction.

		The B/W pixel intensity is multiplied by this value, and the result is subtracted from
		both raw (dcs3-dcs1), and raw (dcs2-dcs0), before calculating distance.

		It would be optimum to have separate correction coeffs (dcs31 and dcs20), but they are close
		enough.

		Resolution is 4 bits:  0~0.0 .. 15~0.9375 (16~1.0)

		B/W image is taken with a fixed integration time specifically designed for this use only.

		dcs31_fix = (dcs3-dcs1) - (bw*corr)>>4
		dcs20_fix = (dcs2-dcs0) - (bw*corr)>>4
	*/

	uint8_t amb_corr[TOF_XS*TOF_YS/2];
} chipcal_hifreq_t;

// Smaller calibration dataset for secondary, far-seeing ranges (that use 2dcs)
typedef struct __attribute__((packed))
{
	/*
		6.66MHz and 5MHz are 2dcs aqcuisitions and compensated for offset only - they are imprecise anyway, and just need to be fast, and see far.

		4 bits, unit: 32mm.
		Base offset is reduced by (15/2)*32mm from what it actually is, so no need
		to sign extend anything, just sum the 4-bit value <<5
	*/

	int16_t  wid_base_offset_2dcs;
	uint8_t  wid_offsets_2dcs[TOF_XS*TOF_YS/2]; // 4 bits, unit 32mm

	int16_t nar_base_offset_2dcs;
	uint8_t  nar_offsets_2dcs[TOF_XS_NARROW*TOF_YS_NARROW/2]; // 4 bits, unit 32mm

	// Ambient correction works the same way as in hifreq_t
	uint8_t amb_corr[TOF_XS*TOF_YS/2];

} chipcal_lofreq_t;


typedef struct __attribute__((packed))
{
	uint32_t magic;
	uint32_t chip_id;
	uint32_t calib_timestamp;
	uint32_t calib_info;
	uint32_t reserved;

	chipcal_hifreq_t hif[2];
	chipcal_lofreq_t lof[2];

	/*
		No temperature correction table at this time.
		To do. Let's see if we need any! The fine dll scheme works really well, and needs no
		processing. We may add some per-pixel thingies for finetuning later, to support extra-accuracy
		sensors individually characterized for temperature shifts during calibration.
	*/

	// Calibration temperature
	int16_t ref_temp; // in 0.1 degC
	int16_t zerofine_temp; // in 0.1 degC
	int32_t fine_steps_per_temp[4]; // Required fine-DLL shift is ((zerofine_temp-latest_temp)*fine_steps_per_temp[freq])>>8

	/*
		Optical calibration for a sensor.

		Calibration is fixed for a single rotation of the sensor. For different rotations, the data itself needs to be rotated.
		This way, a lot of runtime calculation is saved. The obvious limitation is that the sensors cannot be installed in
		rotating arms, etc. Rotating a sensor requires calibration data update (which can be made a quick and simple operation,
		but still comparable to firmware update.)

		If really needed, this can be worked around later by writing a runtime conversion on the calib data. It's not impossible
		to make it fast enough.

	*/

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

	int16_t perpix_ver_angs[TOF_XS*TOF_YS/4];
	int16_t perpix_hor_angs[TOF_XS*TOF_YS/4];

	/*
		TODO:

		Parameters for lens blur & stray light model. Brute-forcing/lookup-tabling these would get huge; so
		they will be simple, small parameters generating a "good enough" model. Not expecting much memory
		footprint for this.
	*/


} tof_calib_t;

#define TOFCAL_SIZE 157284
extern const tof_calib_t * const tof_calibs[N_SENSORS];


