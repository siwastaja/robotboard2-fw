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


#define DIST_OVEREXP 1
#define DIST_UNDEREXP 0

#ifndef DIST_MASK
	#define DIST_SHIFT 3  // shift from/to millimeters: 8mm resolution
	#define DIST_MASK 0x0fff
#endif

#define MAX_DISTVAL (4095)

#define SIZEOF_MONO (sizeof(epc_img_t))
#define SIZEOF_MONO_NARROW (sizeof(epc_img_narrow_t))
#define SIZEOF_2DCS (sizeof(epc_2dcs_t))
#define SIZEOF_2DCS_NARROW (sizeof(epc_2dcs_narrow_t))
#define SIZEOF_4DCS (sizeof(epc_4dcs_t))
#define SIZEOF_4DCS_NARROW (sizeof(epc_4dcs_narrow_t))



void copy_cal_to_shadow(int sid, int f);
void copy_cal_to_shadow_narrow(int sid, int f);


// bwimg is in raw EPC format (shifted and masked inside the function)
void conv_4dcs_to_2dcs_wide     (int16_t* restrict dcs20_out, int16_t* restrict dcs31_out, epc_4dcs_t* restrict in, uint16_t* restrict bwimg);
void conv_4dcs_to_2dcs_hdr0_wide(int16_t* restrict dcs20_out, int16_t* restrict dcs31_out, epc_4dcs_t* restrict in, uint16_t* restrict bwimg);
void conv_4dcs_to_2dcs_hdr1_wide(int16_t* restrict dcs20_out, int16_t* restrict dcs31_out, epc_4dcs_t* restrict in, uint16_t* restrict bwimg);

void conv_4dcs_to_2dcs_narrow     (int16_t* restrict dcs20_out, int16_t* restrict dcs31_out, epc_4dcs_narrow_t* restrict in, uint16_t* restrict bwimg);
void conv_4dcs_to_2dcs_hdr0_narrow(int16_t* restrict dcs20_out, int16_t* restrict dcs31_out, epc_4dcs_narrow_t* restrict in, uint16_t* restrict bwimg);
void conv_4dcs_to_2dcs_hdr1_narrow(int16_t* restrict dcs20_out, int16_t* restrict dcs31_out, epc_4dcs_narrow_t* restrict in, uint16_t* restrict bwimg);


int calc_avg_ampl_x256(int16_t* restrict dcs20_in, int16_t* restrict dcs31_in);
int calc_avg_ampl_x256_narrow(int16_t* restrict dcs20_in, int16_t* restrict dcs31_in);
int calc_avg_ampl_x256_nar_region_on_wide(int16_t* restrict dcs20_in, int16_t* restrict dcs31_in);


void compensated_2dcs_6mhz_dist_masked(uint8_t *dist_out, epc_2dcs_t* restrict in, uint16_t* restrict bwimg);
void compensated_2dcs_6mhz_dist_masked_narrow(uint8_t *dist_out, epc_2dcs_narrow_t* restrict in, uint16_t* restrict bwimg);


// The latest thing as of now, dealing with 16-bit images:
void compensated_tof_calc_ampldist(int is_narrow, uint16_t* restrict ampldist_out, int16_t* restrict dcs20_in, int16_t* restrict dcs31_in, uint8_t* restrict dealias_dist, int freq, int inttime_norm_number);
// Optimized version for obst. avoid use (quicker, fewer features):
void compensated_tof_calc_ampldist_nodealias_noampl_nonarrow(uint16_t* restrict ampldist_out, int16_t* restrict dcs20_in, int16_t* restrict dcs31_in);

void remove_narrow_edgevals(int16_t* restrict img20, int16_t* restrict img31);


void mask_highly_corrected(int is_narrow, int16_t* restrict img20_lenscorr, int16_t* restrict img31_lenscorr, int16_t* restrict img20, int16_t* restrict img31);

/*
	TOF calibration data is the dominating source of flash storage - if we followed the Espros'
	recommendations for storing the calibration data, we would run out of flash with just one sensor.

	To store 10 sensors, we need to implement some trickery to reduce redundant informaton. Generic data compression
	would be inefficient,
	From 16 flash sectors, 12 can be dedicated to TOF calibration data, leaving enough margin for all code, and 
	for other modules (e.g., IMU calibration tables, audio data).
	


	All 4-bit fields follow little endianness: the first pixel index is stored in &0x0f, the second in &0xf0.
*/

// For compatibility with the first vacuum proto (only one unit exists):
#ifdef REV2A
	#define WID_N_PIXGROUPS 12
	#define NAR_N_PIXGROUPS 8
	#define TOF_TBL_SEG_LEN 96
	#define TOF_TBL_SEG_LEN_MULT (TOF_TBL_SEG_LEN-1)
#else
	#define WID_N_PIXGROUPS 14
	#define NAR_N_PIXGROUPS 12
	#define TOF_TBL_SEG_LEN 125 // actual table is one longer, for optimization
	#define TOF_TBL_SEG_LEN_MULT (TOF_TBL_SEG_LEN)
#endif

#define COMP_AMBIENT_INTLEN_US 100 // it's important to follow this in both calibration and runtime

// Bigger calibration dataset for important, accurate ranges
typedef struct __attribute__((packed))
{
	/*
		Full distance LUTs for two frequencies (20MHz and 10MHz).
		Max 16 pixel groups (exact number WID_N_PIXGROUPS, NAR_N_PIXGROUPS)
		8 segments of the compensated atan2 function (atan2(a,b) + correction), so that a>0, b>0, b>a, hence 8 segments with swapped polarities / order
		The table reads the distance in mm directly.

		lut_group_id (4 bit per pixel) tells which group the pixel belongs to.
	
	*/
	uint8_t  wid_lut_group_ids[TOF_XS*TOF_YS/2];

	#ifdef REV2A // For compatibility with the first vacuum proto
		uint16_t wid_luts[WID_N_PIXGROUPS][8][TOF_TBL_SEG_LEN];
	#else
		uint16_t wid_luts[WID_N_PIXGROUPS][8][TOF_TBL_SEG_LEN+1]; // Table is one longer, to handle the fairly rare case of equality dcs31==dcs20 without extra code.
	#endif

	uint8_t  nar_lut_group_ids[TOF_XS_NARROW*TOF_YS_NARROW/2];

	#ifdef REV2A
		uint16_t nar_luts[NAR_N_PIXGROUPS][8][TOF_TBL_SEG_LEN];
	#else
		uint16_t nar_luts[NAR_N_PIXGROUPS][8][TOF_TBL_SEG_LEN+1];
	#endif

	/*
		Ambient light correction.

		The B/W pixel intensity is multiplied by this value, and the result is subtracted from
		both raw (dcs3-dcs1), and raw (dcs2-dcs0), before calculating distance.

		It would be optimum to have separate correction coeffs (dcs31 and dcs20), but they are close
		enough.

		Resolution is 4 bits:  0~0.0 .. 15~0.9375 (16~1.0)

		B/W image is taken with a fixed integration time specifically designed for this use only.

		dcs31_fix = (dcs3-dcs1) + (bw*corr)>>4
		dcs20_fix = (dcs2-dcs0) + (bw*corr)>>4

		WARNING: copy_cal_to_shadow_narrow() assumes this is the last element on this struct.
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
	// WARNING: copy_cal_to_shadow_narrow() assumes this is the last element on this struct.
	uint8_t amb_corr[TOF_XS*TOF_YS/2];

} chipcal_lofreq_t;

/*
	Sensor mount position 1:
	 _ _
	| | |
	| |L|
	|O|L|
	| |L|
	|_|_|  (front view)

	Sensor mount position 2:
	 _ _
	| | |
	|L| |
	|L|O|
	|L| |
	|_|_|  (front view)

	Sensor mount position 3:

	-------------
	|  L  L  L  |
	-------------
	|     O     |
	-------------

	Sensor mount position 4:

	-------------
	|     O     |
	-------------
	|  L  L  L  |
	-------------
*/

typedef struct __attribute__((packed))
{
	int16_t mount_mode;             // mount position 1,2,3 or 4
	int16_t x_rel_robot;          // zero = robot origin. Positive = robot front (forward)
	int16_t y_rel_robot;          // zero = robot origin. Positive = to the right of the robot
	uint16_t ang_rel_robot;        // zero = robot forward direction. positive = ccw
	uint16_t vert_ang_rel_ground;  // zero = looks directly forward. positive = looks up. negative = looks down
	int16_t z_rel_ground;         // sensor height from the ground	
} sensor_mount_t;

#define BLUR_PARAMS_RATIO_X 5
#define BLUR_PARAMS_RATIO_Y 5
#define BLUR_PARAMS_XS (TOF_XS/BLUR_PARAMS_RATIO_X)
#define BLUR_PARAMS_YS (TOF_YS/BLUR_PARAMS_RATIO_Y)

/*
	d: box blur mix ratio, 6 bits

	convolution params:
	a: 10 bits, unsigned
	b: int8
	c: int8
*/

#define BLUR_PARAM_A(x_) ((x_).d_a & 0x3ff)
#define BLUR_PARAM_B(x_) ((x_).b)
#define BLUR_PARAM_C(x_) ((x_).c)
#define BLUR_PARAM_D(x_) ((x_).d_a >> 10)

typedef struct __attribute__((packed))
{
	uint16_t d_a; // Access macros above
	int8_t b;
	int8_t c;
} mcu_blur_params_t;

#define TC(x_, y_) ((y_)*TOF_XS+(x_))

// Give index to narrow image, from full image x,y coordinates
// Beware of overindexing. Must be within the narrow image area.
#define TNC(x_, y_) (((y_)-TOF_NARROW_Y_START)*TOF_XS_NARROW+((x_)-TOF_NARROW_X_START))

typedef struct __attribute__((packed))
{
	uint32_t magic;
	uint32_t chip_id;
	uint32_t calib_timestamp;
	uint32_t calib_info;
	uint32_t reserved;

	chipcal_hifreq_t hif[2]; // 20 MHz, 10 MHz, in 4DCS mode
	chipcal_lofreq_t lof[1]; // 6.66MHz in 2DCS mode

	/*
		No temperature correction table at this time.
		To do. Let's see if we need any! The fine dll scheme works really well, and needs no
		processing. We may add some per-pixel thingies for finetuning later, to support extra-accuracy
		sensors individually characterized for temperature shifts during calibration.
	*/

	// Calibration temperature
	int16_t ref_temp; // in 0.1 degC
	int16_t zerofine_temp; // in 0.1 degC
	int32_t fine_steps_per_temp[3]; // Required fine-DLL shift is ((zerofine_temp-latest_temp)*fine_steps_per_temp[freq])>>8

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
		Lens geometry model:
		The angles the pixels point at.

		The final point is calculated by:
		x = d * cos(pix_ver_ang + sensor_ver_ang) * cos(pix_hor_ang + sensor_hor_ang) + sensor_x;
		y = d * cos(pix_ver_ang + sensor_ver_ang) * sin(pix_hor_ang + sensor_hor_ang) + sensor_y;
		z = d * sin(pix_ver_ang + sensor_ver_ang) + sensor_z;
		
		Horizontal angle is positive CCW. Zero looking right on the standard UI screen - East for the recommended standard notation.
		Vertical angle is positive upwards. Zero looking forward.

		Angle unit is, like everywhere, so that full data type range corresponds for 360 degrees.
		These angles are, naturally, limited between -90...+90 degrees, so we could add one bit of extra precision by shifting, but no need
		for it right now - with 16-bit range, the resolution is about 0.005 degrees.

		Datapoints are reduced in half, both vertically and horizontally (4x reduction)
		[pix 0] = [0]
		[pix 1] = ([0] + [1])/2
		[pix 2] = [1]
		...
	*/

	sensor_mount_t mount;

	uint16_t hor_angs[TOF_XS*TOF_YS/4];
	uint16_t ver_angs[TOF_XS*TOF_YS/4];

	/*
		Lens blur/flare model

		Blur / close-range flare model uses a combination of a small convolution kernel, and a larger box blur,
		parameterized by mcu_blur_params_t (4 bytes). Different areas of image have their own parameters.
	*/

	mcu_blur_params_t blur_params[BLUR_PARAMS_XS*BLUR_PARAMS_YS];

	/*
		After compensating close range error, remaining far-away blur/flare is compensated with a brute-force
		table: each input section is averaged and correlated against each output section with a coefficient of error
		(8 bits per coeff)

		Input area coordinates are variable and stored in resical_bounds.
		[0] is always 0 and [last] is always the last pixel index
	*/

#define RESICAL_IN_XS 15
#define RESICAL_IN_YS 7
#define RESICAL_OUT_RATIO 10
#define RESICAL_OUT_XS 16
#define RESICAL_OUT_YS 6
#define RESICAL_DIVIDER 32768

#define GET_RESICAL_COEFF(cal_, ix_, iy_, ox_, oy_) ((cal_).resical_coeffs[(iy_)*RESICAL_IN_XS+(ix_)][(oy_)*RESICAL_OUT_XS+(ox_)])


	uint8_t resical_bounds_x[RESICAL_IN_XS+1];
	uint8_t resical_bounds_y[RESICAL_IN_YS+1];
	uint8_t resical_coeffs[RESICAL_IN_XS*RESICAL_IN_YS][RESICAL_OUT_XS*RESICAL_OUT_YS];

} tof_calib_t;

#define TOFCAL_SIZE 157284
extern const tof_calib_t * const tof_calibs[N_SENSORS];


//#define M_PI 3.141592653
#define RADTODEG(x) ((x)*(360.0/(2.0*M_PI)))
#define DEGTORAD(x) ((x)*((2.0*M_PI)/360.0))
#define DEGTOANG16(x)  ((uint16_t)((float)(x)/(360.0)*65536.0))


#include "../robotsoft/api_board_to_soft.h"

void tof_to_obstacle_avoidance(uint16_t* ampldist, int sidx);
void total_sensor_obstacle(int sidx);

void verify_calibration();

void tof_enable_chafind_datapoints();
void tof_disable_chafind_datapoints();

// 7 is maximum possible, so that the 13-bit (dcs2-dcs0) or (dcs3-dcs1) data range (approx -4096..+4096), multiplied
// by this factor, still fits int16, and leaves margin for image processing.
#define HDR_FACTOR 7

void run_lens_model(int16_t* in, int16_t* out, tof_calib_t* p_tof_calib);
void run_lens_model_narrow(int16_t* in, int16_t* out, tof_calib_t* p_tof_calib);


#define OBSTACLE_SHORT_US 25
#define OBSTACLE_LONG_US  (OBSTACLE_SHORT_US*HDR_FACTOR)

