#include <stdint.h>
#include <string.h>
#include "sbc_comm.h"
#include <math.h>

#include "misc.h"
#include "flash.h"
#include "tof_muxing.h"
#include "tof_process.h"
#include "tof_ctrl.h"
#include "micronavi.h"
#include "adcs.h"
#include "pwrswitch.h"
#include "charger.h"
#include "bldc.h"
#include "imu.h"
#include "audio.h"
#include "sbc_comm.h"
#include "timebase.h"
#include "backup_ram.h"
#include "run.h"
#include "own_std.h"
#include "drive.h"
#include "micronavi.h"

static const int intlen_mults[4] = {12, 6, 4, 3}; // with these, unit is always 0.6us.

// Temporary bodge to increase exposure times on two hand-soldered prototype sensors, which lack half of the LEDs. Only applies to the sole prototype of REV2A.
static int bubblegum = 1;

static int gen_data;

// HDR accumulation images:
static int16_t img20[TOF_XS*TOF_YS];
static int16_t img31[TOF_XS*TOF_YS];

static int16_t img20_lenscorr[TOF_XS*TOF_YS];
static int16_t img31_lenscorr[TOF_XS*TOF_YS];

static uint8_t lofreq_dist[TOF_XS*TOF_YS];

static uint16_t ampldist[TOF_XS*TOF_YS];




epc_img_t mono_comp __attribute__((aligned(4)));
epc_img_narrow_t mono_comp_narrow __attribute__((aligned(4)));
epc_4dcs_t dcsa __attribute__((aligned(4)));
epc_2dcs_t dcs2 __attribute__((aligned(4)));
epc_4dcs_narrow_t dcsa_narrow __attribute__((aligned(4)));
epc_2dcs_narrow_t dcs2_narrow __attribute__((aligned(4)));


#define ENABLE_TOF_TS
#ifdef ENABLE_TOF_TS
	uint32_t timestamp_initial;
	#define INIT_TOF_TS() do{timestamp_initial = cnt_100us;}while(0)
	#define TOF_TS(id_) do{if(gen_data && tof_diagnostics) { tof_diagnostics->timestamps[(id_)] = cnt_100us - timestamp_initial;} }while(0)
#else
	#define INIT_TOF_TS()
	#define TOF_TS(id_)
#endif



int err_cnt;
static void log_err(int sidx)
{
//	err_cnt += 10;
//	if(err_cnt > 30)
	{
		error(100+sidx);
	}
}


uint32_t led_colors[N_SENSORS];
int led_modes[N_SENSORS];
int led_prev_blink[N_SENSORS];

void led_status(int sid, uint32_t val, int mode)
{
	if(sid<0 || sid>=N_SENSORS) error(150);
	led_colors[sid] = val;
	led_modes[sid] = mode;
}

void update_led(int sidx)
{
	if(led_modes[sidx] == LED_MODE_BLINK)
	{
		if(led_prev_blink[sidx])
			rgb_update(0);
		else
			rgb_update(led_colors[sidx]);
		led_prev_blink[sidx] = ~led_prev_blink[sidx];
	}
	else
	{
		rgb_update(led_colors[sidx]);
		if(led_modes[sidx] == LED_MODE_FADE)
		{
			int bright = (led_colors[sidx]&0xff000000)>>24;
			int r = (led_colors[sidx]&0xff0000)>>16;
			int g = (led_colors[sidx]&0x00ff00)>>8;
			int b = (led_colors[sidx]&0x0000ff)>>0;
	
			//r = (r*7)>>3;
			//g = (g*7)>>3;
			//b = (b*7)>>3;
			r = (r*3)>>2;
			g = (g*3)>>2;
			b = (b*3)>>2;

			if(r < 2 && g < 2 && b < 2)
				led_colors[sidx] = 0;
			else
				led_colors[sidx] = (bright<<24) | (r<<16) | (g<<8) | (b<<0);

		}
	}
}

void restart_led(int sidx) // like update_led, but no state changes, just turn the LEDs on/off
{
	if(led_modes[sidx] == LED_MODE_BLINK)
	{
		if(led_prev_blink[sidx])
			rgb_update(0);
		else
			rgb_update(led_colors[sidx]);
	}
	else
	{
		rgb_update(led_colors[sidx]);
	}
}

// Return stray from 1 .. 16383
int measure_stray()
{
	int32_t min = INT32_MAX;
	int32_t max = INT32_MIN;
	for(int i=0; i<600; i++)
	{
		delay_us(12);
		int32_t now = adc3.s.epc_stray_estimate;
//		DBG_PR_VAR_I16(now);
		if(now > max)
			max = now;
		if(now < min)
			min = now;
	}

//	DBG_PR_VAR_I16(stray);

	int stray = max-min;
	stray -= 300; // TODO: better light shielding to properly read low levels
	if(stray < 1) stray = 1;
	return stray;
}

#define IFDBG if(sidx == 99)

void adjust()
{
	//uint8_t cmd = uart_input();

#if 0
	extern int corr_mask_ratio;
	if(cmd >= '0' && cmd <= '9')
	{
		corr_mask_ratio = (int)(cmd-'0')*10;
	}
#endif

}

/*
	These two functions:
	Take in the latest chip temperature measurement.
	Average it (per sensor, keeps track of sensors internally) using an exponentially decaying running average
	Return the number of fine dll steps required, based on that average and sensor calibration parameters
*/
static int32_t chiptemp_flt_x256[N_SENSORS] = {INT32_MIN, INT32_MIN, INT32_MIN, INT32_MIN, INT32_MIN, INT32_MIN, INT32_MIN, INT32_MIN, INT32_MIN, INT32_MIN};

static void filt_temperature(int sidx, int32_t chiptemp)
{
	if(sidx < 0 || sidx >= N_SENSORS)
		error(1234);

	if(chiptemp_flt_x256[sidx] == INT32_MIN)
		chiptemp_flt_x256[sidx] = (chiptemp<<8);
	else
		chiptemp_flt_x256[sidx] = (7*chiptemp_flt_x256[sidx] + (chiptemp<<8))>>3;
}

static int calc_dll_steps(int sidx, int freq)
{
	int32_t chiptemp_flt = chiptemp_flt_x256[sidx]>>8;

	int fine_steps = ((tof_calibs[sidx]->zerofine_temp-chiptemp_flt)*tof_calibs[sidx]->fine_steps_per_temp[freq])>>8;
	if(fine_steps < 0) fine_steps = 0;
	else if(fine_steps > 799) fine_steps = 799;

	return fine_steps;
}

static int cnta = 0;

// A delay was needed here once, for unknown reasons. Don't know why it's OK now without.
#define EPC_MODECHANGE_DELAY()
//delay_ms(1)


static void basic_set(int sidx, int* ssval_out, int* narrow_avg_ampl_out)
{
INIT_TOF_TS();

	// Compensation BW + chiptemp
	dcmi_crop_wide();
	epc_greyscale(); block_epc_i2c(4);
	EPC_MODECHANGE_DELAY();
	epc_dis_leds(); block_epc_i2c(4);
	epc_clk_div(1); block_epc_i2c(4);
	epc_intlen(intlen_mults[1], INTUS(COMP_AMBIENT_INTLEN_US)); block_epc_i2c(4);
	epc_temperature_magic_mode(sidx);
	dcmi_start_dma(&mono_comp, SIZEOF_MONO);
	epc_trig();

	// Do something here

	if(poll_capt_with_timeout_complete()) log_err(sidx);

	int32_t chiptemp = epc_read_temperature(sidx);
	epc_temperature_magic_mode_off(sidx);
	filt_temperature(sidx, chiptemp);

/*
	if(sidx==5)
	{
		uart_print_string_blocking("BASIC\r\n");
		DBG_PR_VAR_I32(chiptemp);
	}
*/

	#define SUPERSHORT_US 12

	// SUPER SHORT WIDE
	// Works as autoexposure basis - may work as a HDR short in the case where the autoexp would result in the shortest possible time.

	epc_fine_dll_steps(calc_dll_steps(sidx, 0)); block_epc_i2c(4);
	epc_4dcs(); block_epc_i2c(4);
	EPC_MODECHANGE_DELAY();
	epc_ena_wide_leds(); block_epc_i2c(4);

	epc_clk_div(0); block_epc_i2c(4);
	epc_intlen(intlen_mults[0], INTUS(SUPERSHORT_US*bubblegum)); block_epc_i2c(4);

	dcmi_start_dma(&dcsa, SIZEOF_4DCS);
	epc_trig();

	// 16000-15000 = 1000. 16000-14000 = 2000
	int stray = measure_stray();
	// TODO: temperature compensation and calibration for stray phototransistors

	// Do something here:
		copy_cal_to_shadow(sidx, 0);

	if(poll_capt_with_timeout_complete()) log_err(sidx);


	if(stray > 13000)
	{
		total_sensor_obstacle(sidx);
		if(ssval_out) *ssval_out = -1;
		if(narrow_avg_ampl_out) *narrow_avg_ampl_out = -1;
		return;
	}

TOF_TS(0); // 13.5ms

	conv_4dcs_to_2dcs_wide(img20, img31, &dcsa, mono_comp.img);

TOF_TS(1); // 1.8ms

	int avg_ampl = calc_avg_ampl_x256(img20, img31);

TOF_TS(2); // 0.7ms

	if(avg_ampl < 1) avg_ampl = 1;
	int mid_exp = 10000 / avg_ampl; // 333us at amplitude 30

	/*
		"Stray estimate" sees beyond the active image area, directly at the wide LEDs.
	*/
	int mid_exp_by_stray = 150000 / stray;

	if(mid_exp_by_stray < SUPERSHORT_US) mid_exp_by_stray = SUPERSHORT_US; 

	if(mid_exp < SUPERSHORT_US) mid_exp = SUPERSHORT_US;

	// TODO: In addition to just lowering int.time based on stray estimate,
	// inject it to the lens compensation model to improve the data.

//	uart_print_string_blocking("\r\n");
//	DBG_PR_VAR_I32(sidx);
//	DBG_PR_VAR_I32(avg_ampl);
//	DBG_PR_VAR_I32(mid_exp);
//	DBG_PR_VAR_I32(stray);
//	DBG_PR_VAR_I32(mid_exp_by_stray);

//
	if(mid_exp_by_stray < mid_exp)
	{
		mid_exp = mid_exp_by_stray;

//		uart_print_string_blocking(" STRAY LIMITED ---> ");
//		DBG_PR_VAR_I32(mid_exp_by_stray);
	}

	// Before clipping the max exposure value, deliver it to the long_wide.
	// Long_wide still wants to limit the exposure time if there is a reason (close obstacle),
	// but the maximum in good conditions can be higher.
	if(ssval_out) *ssval_out = mid_exp;

	if(mid_exp > 500) mid_exp = 500;  // long inttime max 3500us

	// TODO: skip everything, report obstacle, if expval is much below SUPERSHORT_US





//	int ss_saturated = supershort_avg_ampl_x256;
//	if(ss_saturated > 400) ss_saturated = 400;
	
	// min exposures: 12, 84
	// max exposures: 332, 2324
	// 7230mm of range measured - medium-light wood-color (approx. 40-50%?) door can be seen at 2324us int.time
	// This is more than enough for the basic set. In a typical middle-sized apartment, sees all rooms physically possible
	// at the same time.

//	int mid_exp = SUPERSHORT_US + sq(400 - ss_saturated)/500; 




	int long_exp = mid_exp*HDR_FACTOR;

	DBG_PR_VAR_I32(sidx);
	DBG_PR_VAR_I32(mid_exp);
	DBG_PR_VAR_I32(long_exp);

	// 6.66MHz 2dcs for dealiasing - same inttime as the long HDR exp
	// (Going from 20MHz to 6.66MHz gives amplitude gain of at least 2, enough to compensate
	// for halving the amplitude from using 2DCS mode)

	epc_fine_dll_steps(calc_dll_steps(sidx, 2)); block_epc_i2c(4);

	epc_2dcs(); block_epc_i2c(4);
	epc_clk_div(2); block_epc_i2c(4);
	epc_intlen(intlen_mults[2], INTUS(long_exp*bubblegum)); block_epc_i2c(4);

	dcmi_start_dma(&dcs2, SIZEOF_2DCS);
	epc_trig();

	// Do something here


		// Run obstacle avoidance for the supershort data - react to overexposed parts, as well, treating them as
		// zero-distance objects. Supershort exposure time is so short that even very reflective things won't overexpose
		// too easily. Testing with worst-case polished mirror-like surfaces revealed that they overexpose at about 20-25cm
		// apart.
		compensated_tof_calc_ampldist_nodealias_noampl_nonarrow(ampldist, img20, img31);
		tof_to_obstacle_avoidance(ampldist, sidx, 1);


		copy_cal_to_shadow(sidx, 2);


	if(poll_capt_with_timeout_complete()) log_err(sidx);

TOF_TS(3); //8.6ms

	adjust();


	// HDR mid exp

	epc_fine_dll_steps(calc_dll_steps(sidx, 0)); block_epc_i2c(4);

	epc_clk_div(0); block_epc_i2c(4);
	epc_intlen(intlen_mults[0], INTUS(mid_exp*bubblegum)); block_epc_i2c(4);
	epc_4dcs(); block_epc_i2c(4);

	dcmi_start_dma(&dcsa, SIZEOF_4DCS);
	epc_trig();
	adjust();

	// Do something here - shadow cal set = 2
		compensated_2dcs_6mhz_dist_masked(lofreq_dist, &dcs2, mono_comp.img);

		copy_cal_to_shadow(sidx, 0);


		// Take the pose here
		if(tof_slam_set)
		{
			hires_pos_to_hw_pose(&tof_slam_set->sets[0].pose, &cur_pos);
		}

	if(poll_capt_with_timeout_complete()) log_err(sidx);

TOF_TS(4); //9.5ms

	// Clears img20 and img31, collects the first (shorter) HDR exposure there
	conv_4dcs_to_2dcs_hdr0_wide(img20, img31, &dcsa, mono_comp.img);

TOF_TS(5); //2.6ms

	// HDR long exp

	epc_intlen(intlen_mults[0], INTUS(long_exp*bubblegum)); block_epc_i2c(4);

	dcmi_start_dma(&dcsa, SIZEOF_4DCS);
	epc_trig();
	adjust();

	// Do something here

		// Average amplitude on narrow region on the wide mid-exp data is useful for 
		// autoexposing the narrow long set later.
		// May not be needed - do not calculate if NULL.
		// Normalize so that is independent of exp. time
		if(narrow_avg_ampl_out)
			*narrow_avg_ampl_out = (256*calc_avg_ampl_x256_nar_region_on_wide(img20, img31))/mid_exp; // 0.0ms

	if(poll_capt_with_timeout_complete()) log_err(sidx);

TOF_TS(6); //16.3ms

	// Adds the second (longer) HDR exposure on the top of the existing exposure, replacing data whenever possible (not overexposed):
	conv_4dcs_to_2dcs_hdr1_wide(img20, img31, &dcsa, mono_comp.img);

TOF_TS(7); //2.3ms

//	if(cnta < 5)
	{
		// Deblur/deflare based on uniquely calibrated lens models:
		run_lens_model(img20, img20_lenscorr, tof_calibs[sidx]);
		run_lens_model(img31, img31_lenscorr, tof_calibs[sidx]);

TOF_TS(8); //20.0ms --> 15.5ms after some minor optimizations (and putting in ITCM) --> 14.6ms after a local stack copy of blur_params

		// Remove data that needed high amounts of correction - it's likely too wrong, despite correction:
		mask_highly_corrected(0, img20_lenscorr, img31_lenscorr, img20, img31);


TOF_TS(9); //1.5ms

	}

//		memcpy(img20_lenscorr, img20, sizeof img20_lenscorr);
//		memcpy(img31_lenscorr, img31, sizeof img31_lenscorr);

/*
	else
	{
		memcpy(img20_lenscorr, img20, sizeof img20_lenscorr);
		memcpy(img31_lenscorr, img31, sizeof img31_lenscorr);
	}
*/
	// Dealias and calculate final amplitude&distance map:
	compensated_tof_calc_ampldist(0, ampldist, img20_lenscorr, img31_lenscorr, lofreq_dist, 0, long_exp);

TOF_TS(10); //8.3ms --> 5.6ms after putting in ITCM

	tof_to_obstacle_avoidance(ampldist, sidx, 0);

TOF_TS(11); //0.8ms

	if(tof_slam_set)
	{
		memcpy(tof_slam_set->sets[0].ampldist, ampldist, sizeof ampldist);
	}

	adjust();
}



static void chamount_set(int sidx)
{
	// Compensation BW + chiptemp
	dcmi_crop_wide();
	epc_greyscale(); block_epc_i2c(4);
	EPC_MODECHANGE_DELAY();
	epc_dis_leds(); block_epc_i2c(4);
	epc_clk_div(1); block_epc_i2c(4);
	epc_intlen(intlen_mults[1], INTUS(COMP_AMBIENT_INTLEN_US)); block_epc_i2c(4);
	epc_temperature_magic_mode(sidx);
	dcmi_start_dma(&mono_comp, SIZEOF_MONO);
	epc_trig();

	// Do something here

	if(poll_capt_with_timeout_complete()) log_err(sidx);

	int32_t chiptemp = epc_read_temperature(sidx);
	epc_temperature_magic_mode_off(sidx);
	filt_temperature(sidx, chiptemp);

	// SUPER SHORT WIDE
	// Works as autoexposure basis - may work as a HDR short in the case where the autoexp would result in the shortest possible time.

	epc_fine_dll_steps(calc_dll_steps(sidx, 0)); block_epc_i2c(4);
	epc_4dcs(); block_epc_i2c(4);
	EPC_MODECHANGE_DELAY();
	epc_ena_wide_leds(); block_epc_i2c(4);

	epc_clk_div(0); block_epc_i2c(4);
	epc_intlen(intlen_mults[0], INTUS(SUPERSHORT_US*bubblegum)); block_epc_i2c(4);

	dcmi_start_dma(&dcsa, SIZEOF_4DCS);
	epc_trig();

	// 16000-15000 = 1000. 16000-14000 = 2000
	int stray = measure_stray();
	// TODO: temperature compensation and calibration for stray phototransistors

	// Do something here:
		copy_cal_to_shadow(sidx, 0);

	if(poll_capt_with_timeout_complete()) log_err(sidx);


	if(stray > 13000)
	{
		total_sensor_obstacle(sidx);
		return;
	}

	conv_4dcs_to_2dcs_wide(img20, img31, &dcsa, mono_comp.img);

	int avg_ampl = calc_avg_ampl_x256(img20, img31);

	if(avg_ampl < 1) avg_ampl = 1;
	int mid_exp = 10000 / avg_ampl; // 333us at amplitude 30

	/*
		"Stray estimate" sees beyond the active image area, directly at the wide LEDs.
	*/
	int mid_exp_by_stray = 150000 / stray;

	if(mid_exp_by_stray < SUPERSHORT_US) mid_exp_by_stray = SUPERSHORT_US; 

	if(mid_exp < SUPERSHORT_US) mid_exp = SUPERSHORT_US;

	// TODO: In addition to just lowering int.time based on stray estimate,
	// inject it to the lens compensation model to improve the data.

//	DBG_PR_VAR_I32(sidx);
//	DBG_PR_VAR_I32(avg_ampl);
//	DBG_PR_VAR_I32(mid_exp);
//	DBG_PR_VAR_I32(stray);
//	DBG_PR_VAR_I32(mid_exp_by_stray);
//
	if(mid_exp_by_stray < mid_exp)
	{
		mid_exp = mid_exp_by_stray;

//		uart_print_string_blocking(" STRAY LIMITED ---> ");
//		DBG_PR_VAR_I32(mid_exp_by_stray);
	}

	if(mid_exp > 100) mid_exp = 100;  // long inttime max 700us

	int long_exp = mid_exp*HDR_FACTOR;

	// 6.66MHz 2dcs for dealiasing - same inttime as the long HDR exp
	// (Going from 20MHz to 6.66MHz gives amplitude gain of at least 2, enough to compensate
	// for halving the amplitude from using 2DCS mode)

	epc_fine_dll_steps(calc_dll_steps(sidx, 2)); block_epc_i2c(4);

	epc_2dcs(); block_epc_i2c(4);
	epc_clk_div(2); block_epc_i2c(4);
	epc_intlen(intlen_mults[2], INTUS(long_exp*bubblegum)); block_epc_i2c(4);

	dcmi_start_dma(&dcs2, SIZEOF_2DCS);
	epc_trig();

	// Do something here


		copy_cal_to_shadow(sidx, 2);


	if(poll_capt_with_timeout_complete()) log_err(sidx);

	// HDR mid exp

	epc_fine_dll_steps(calc_dll_steps(sidx, 0)); block_epc_i2c(4);

	epc_clk_div(0); block_epc_i2c(4);
	epc_intlen(intlen_mults[0], INTUS(mid_exp*bubblegum)); block_epc_i2c(4);
	epc_4dcs(); block_epc_i2c(4);

	dcmi_start_dma(&dcsa, SIZEOF_4DCS);
	epc_trig();
	adjust();

	// Do something here - shadow cal set = 2
		compensated_2dcs_6mhz_dist_masked(lofreq_dist, &dcs2, mono_comp.img);

		copy_cal_to_shadow(sidx, 0);


	if(poll_capt_with_timeout_complete()) log_err(sidx);

	// Clears img20 and img31, collects the first (shorter) HDR exposure there
	conv_4dcs_to_2dcs_hdr0_wide(img20, img31, &dcsa, mono_comp.img);

	// HDR long exp

	epc_intlen(intlen_mults[0], INTUS(long_exp*bubblegum)); block_epc_i2c(4);

	dcmi_start_dma(&dcsa, SIZEOF_4DCS);
	epc_trig();
	adjust();

	// Do something here

	if(poll_capt_with_timeout_complete()) log_err(sidx);

	// Adds the second (longer) HDR exposure on the top of the existing exposure, replacing data whenever possible (not overexposed):
	conv_4dcs_to_2dcs_hdr1_wide(img20, img31, &dcsa, mono_comp.img);

	// Deblur/deflare based on uniquely calibrated lens models:
	run_lens_model(img20, img20_lenscorr, tof_calibs[sidx]);
	run_lens_model(img31, img31_lenscorr, tof_calibs[sidx]);

	// Remove data that needed high amounts of correction - it's likely too wrong, despite correction:
	mask_highly_corrected(0, img20_lenscorr, img31_lenscorr, img20, img31);

	// Dealias and calculate final amplitude&distance map:
	compensated_tof_calc_ampldist(0, ampldist, img20_lenscorr, img31_lenscorr, lofreq_dist, 0, long_exp);

	tof_to_chamount(ampldist, sidx);
}

void request_chamount_mid_image()
{
	#ifdef CONTACTS_ON_BACK
		tof_mux_select(5);
		chamount_set(5);
	#else
		tof_mux_select(0);
		chamount_set(0);
	#endif
}

void request_chamount_full_images()
{
	#ifdef CONTACTS_ON_BACK
		tof_mux_select(4);
		chamount_set(4);
		tof_mux_select(5);
		chamount_set(5);
		tof_mux_select(6);
		chamount_set(6);
	#else
		tof_mux_select(9);
		chamount_set(9);
		tof_mux_select(0);
		chamount_set(0);
		tof_mux_select(1);
		chamount_set(1);
	#endif
}


static void long_wide_set(int sidx, int ss_val)
{
	if(ss_val < 1)
		return;

	// Compensation BW + chiptemp
	// We could reuse the BW from basic set, but that would be too old now (false compensation due to motion)
	dcmi_crop_wide();
	epc_greyscale(); block_epc_i2c(4);
	EPC_MODECHANGE_DELAY();
	epc_dis_leds(); block_epc_i2c(4);
	epc_clk_div(1); block_epc_i2c(4);
	epc_intlen(intlen_mults[1], INTUS(COMP_AMBIENT_INTLEN_US)); block_epc_i2c(4);
	epc_temperature_magic_mode(sidx);
	dcmi_start_dma(&mono_comp, SIZEOF_MONO);
	epc_trig();

	// Do something here

	if(poll_capt_with_timeout_complete()) log_err(sidx);

	int32_t chiptemp = epc_read_temperature(sidx);
	epc_temperature_magic_mode_off(sidx);
	filt_temperature(sidx, chiptemp);


	// Do the longer-range wide. Use the autoexposure from before, but allow
	// *1.9 amplitude (gained from using lower frequency at the same inttime!), even if this
	// causes increased error due to stray / multipath.
	// And when we actually can, use a longer time (a higher saturation value), to see much further.

	int short_exp = ss_val;
	if(short_exp > 600) short_exp = 600; // 4200us max.

	int long_exp = short_exp * HDR_FACTOR;

	int dealias_exp = (long_exp*19)/10;

	// 6.66MHz 2dcs dealias wide

	epc_fine_dll_steps(calc_dll_steps(sidx, 2)); block_epc_i2c(4);

	epc_2dcs(); block_epc_i2c(4);
	EPC_MODECHANGE_DELAY();
	epc_ena_wide_leds(); block_epc_i2c(4);

	epc_clk_div(2); block_epc_i2c(4);
	epc_intlen(intlen_mults[2], INTUS(dealias_exp*bubblegum)); block_epc_i2c(4);

	dcmi_start_dma(&dcs2, SIZEOF_2DCS);
	epc_trig();

	// Do something here
		adjust();
		copy_cal_to_shadow(sidx, 2);


	if(poll_capt_with_timeout_complete()) log_err(sidx);


	// 10MHz wide short exposure

	epc_fine_dll_steps(calc_dll_steps(sidx, 1)); block_epc_i2c(4);

	epc_4dcs(); block_epc_i2c(4);
	epc_clk_div(1); block_epc_i2c(4);
	epc_intlen(intlen_mults[1], INTUS(short_exp*bubblegum)); block_epc_i2c(4);

	dcmi_start_dma(&dcsa, SIZEOF_4DCS);
	epc_trig();

	// Do something here:
		compensated_2dcs_6mhz_dist_masked(lofreq_dist, &dcs2, mono_comp.img);

		copy_cal_to_shadow(sidx, 1);

		adjust();

		// Take the pose here
		if(tof_slam_set)
		{
			hires_pos_to_hw_pose(&tof_slam_set->sets[1].pose, &cur_pos);
		}


	if(poll_capt_with_timeout_complete()) log_err(sidx);

	// Clears img20 and img31, collects the first (shorter) HDR exposure there
	conv_4dcs_to_2dcs_hdr0_wide(img20, img31, &dcsa, mono_comp.img);


	// 10MHz wide long exposure

	epc_intlen(intlen_mults[1], INTUS(long_exp*bubblegum)); block_epc_i2c(4);

	dcmi_start_dma(&dcsa, SIZEOF_4DCS);
	epc_trig();

	// Do something here:
		adjust();

	if(poll_capt_with_timeout_complete()) log_err(sidx);

	// Adds the second (longer) HDR exposure on the top of the existing exposure, replacing data whenever possible (not overexposed):
	conv_4dcs_to_2dcs_hdr1_wide(img20, img31, &dcsa, mono_comp.img);

	#if 1
		// Deblur/deflare based on uniquely calibrated lens models:
		run_lens_model(img20, img20_lenscorr, tof_calibs[sidx]);
		run_lens_model(img31, img31_lenscorr, tof_calibs[sidx]);

		// Remove data that needed high amounts of correction - it's likely too wrong, despite correction:
		mask_highly_corrected(0, img20_lenscorr, img31_lenscorr, img20, img31);
	#else
		memcpy(img20_lenscorr, img20, sizeof img20_lenscorr);
		memcpy(img31_lenscorr, img31, sizeof img31_lenscorr);
	#endif
	// Dealias and calculate final amplitude&distance map:
	compensated_tof_calc_ampldist(0, ampldist, img20_lenscorr, img31_lenscorr, lofreq_dist, 1, long_exp*2);

	if(tof_slam_set)
	{
		tof_slam_set->flags |= TOF_SLAM_SET_FLAG_SET1_WIDE;
		memcpy(tof_slam_set->sets[1].ampldist, ampldist, sizeof ampldist);
	}

	adjust();
}


static void long_narrow_set(int sidx, int avg_ampl)
{
	if(avg_ampl < 0) // -1: invalid (sensor blockage)
		return;
	// Decide the exposure time based on the average amplitude
	// calculated for us by the basic_set. This is calculated
	// from the narrow region, even though the data source is the supershort wide.

	if(avg_ampl < 1) avg_ampl = 1;
	int short_exp = 24000 / avg_ampl; // 600us at amplitude 40

	if(short_exp < SUPERSHORT_US) short_exp = SUPERSHORT_US;
	else if(short_exp > 700) short_exp = 700;  // long inttime max 4900us, dealias max 9310us, very close to the max. inttime regval, don't increase.

	if(short_exp < 100) // TODO: take a 20MHz accuracy-increasing set instead.
		return;

	int long_exp = short_exp * HDR_FACTOR;

	//DBG_PR_VAR_I32(sidx);
	//DBG_PR_VAR_I32(avg_ampl);
	//DBG_PR_VAR_I32(short_exp);

	int dealias_exp = (long_exp*19)/10;
/*
	uart_print_string_blocking("NARROW SET:\r\n");
	DBG_PR_VAR_I32(sidx);
	DBG_PR_VAR_I32(short_exp);
	DBG_PR_VAR_I32(long_exp);
	DBG_PR_VAR_I32(dealias_exp);
*/
	// Compensation BW + chiptemp
	// We could reuse the BW from basic set, but that would be too old now (false compensation due to motion)
	// Even better, we can take the compensation BW with the same narrow cropping, so indexing it is simplified.
	dcmi_crop_narrow();
	epc_greyscale(); block_epc_i2c(4);
	EPC_MODECHANGE_DELAY();
	epc_dis_leds(); block_epc_i2c(4);
	epc_clk_div(1); block_epc_i2c(4);
	epc_intlen(intlen_mults[1], INTUS(COMP_AMBIENT_INTLEN_US)); block_epc_i2c(4);
	epc_temperature_magic_mode(sidx);
	dcmi_start_dma(&mono_comp_narrow, SIZEOF_MONO_NARROW);
	epc_trig();

	// Do something here:


	if(poll_capt_with_timeout_complete()) log_err(sidx);

	int32_t chiptemp = epc_read_temperature(sidx);
	epc_temperature_magic_mode_off(sidx);
	filt_temperature(sidx, chiptemp);


	// 6.66MHz 2dcs dealias narrow

	epc_fine_dll_steps(calc_dll_steps(sidx, 2)); block_epc_i2c(4);

	epc_2dcs(); block_epc_i2c(4);
	EPC_MODECHANGE_DELAY();
	epc_ena_narrow_leds(); block_epc_i2c(4);

	epc_clk_div(2); block_epc_i2c(4);
	epc_intlen(intlen_mults[2], INTUS(dealias_exp*bubblegum)); block_epc_i2c(4);

	dcmi_start_dma(&dcs2_narrow, SIZEOF_2DCS_NARROW);
	epc_trig();

	// Do something here:

		copy_cal_to_shadow_narrow(sidx, 2);

	if(poll_capt_with_timeout_complete()) log_err(sidx);


	// 10MHz narrow HDR short

	epc_fine_dll_steps(calc_dll_steps(sidx, 1)); block_epc_i2c(4);

	epc_4dcs(); block_epc_i2c(4);
	epc_clk_div(1); block_epc_i2c(4);
	epc_intlen(intlen_mults[1], INTUS(short_exp)); block_epc_i2c(4);

	dcmi_start_dma(&dcsa_narrow, SIZEOF_4DCS_NARROW);
	epc_trig();

	// Do something here:

		compensated_2dcs_6mhz_dist_masked_narrow(lofreq_dist, &dcs2_narrow, mono_comp_narrow.img);

		copy_cal_to_shadow_narrow(sidx, 1);

		// Take the pose here
		if(tof_slam_set)
		{
			hires_pos_to_hw_pose(&tof_slam_set->sets[1].pose, &cur_pos);
		}


	if(poll_capt_with_timeout_complete()) log_err(sidx);


	// Clears img20 and img31, collects the first (shorter) HDR exposure there
	conv_4dcs_to_2dcs_hdr0_narrow(img20, img31, &dcsa_narrow, mono_comp_narrow.img);


	// 10MHz narrow HDR long

	epc_intlen(intlen_mults[1], INTUS(long_exp)); block_epc_i2c(4);

	dcmi_start_dma(&dcsa_narrow, SIZEOF_4DCS_NARROW);
	epc_trig();

	// Do something here:



	if(poll_capt_with_timeout_complete()) log_err(sidx);


	// Adds the second (longer) HDR exposure on the top of the existing exposure, replacing data whenever possible (not overexposed):
	conv_4dcs_to_2dcs_hdr1_narrow(img20, img31, &dcsa_narrow, mono_comp_narrow.img);

	if(1) // cnta>7)
	{
		// Deblur/deflare based on uniquely calibrated lens models:
		run_lens_model_narrow(img20, img20_lenscorr, tof_calibs[sidx]);
		run_lens_model_narrow(img31, img31_lenscorr, tof_calibs[sidx]);

		// Remove data that needed high amounts of correction - it's likely too wrong, despite correction:
		mask_highly_corrected(1, img20_lenscorr, img31_lenscorr, img20, img31);
	}
	else
	{

		memcpy(img20_lenscorr, img20, sizeof img20_lenscorr);
		memcpy(img31_lenscorr, img31, sizeof img31_lenscorr);
	}

	remove_narrow_edgevals(img20_lenscorr, img31_lenscorr);

	// Dealias and calculate final amplitude&distance map:
	compensated_tof_calc_ampldist(1, ampldist, img20_lenscorr, img31_lenscorr, lofreq_dist, 1, 8*long_exp);


	if(tof_slam_set)
	{
		tof_slam_set->flags |= TOF_SLAM_SET_FLAG_SET1_NARROW;
		memcpy(tof_slam_set->sets[1].ampldist, ampldist, sizeof ampldist);
	}

}

static void obstacle_set(int sidx)
{
	// Compensation BW + chiptemp
	dcmi_crop_wide();
	epc_greyscale(); block_epc_i2c(4);
	EPC_MODECHANGE_DELAY();
	epc_dis_leds(); block_epc_i2c(4);
	epc_clk_div(1); block_epc_i2c(4);
	epc_intlen(intlen_mults[1], INTUS(COMP_AMBIENT_INTLEN_US)); block_epc_i2c(4);
	dcmi_start_dma(&mono_comp, SIZEOF_MONO);
	epc_trig();

	// Do something here

	if(poll_capt_with_timeout_complete()) log_err(sidx);

	// Don't read the temperature

	// HDR short exp

	epc_fine_dll_steps(calc_dll_steps(sidx, 1)); block_epc_i2c(4);

	epc_4dcs(); block_epc_i2c(4);
	EPC_MODECHANGE_DELAY();
	epc_ena_wide_leds(); block_epc_i2c(4);

	epc_intlen(intlen_mults[1], INTUS(OBSTACLE_SHORT_US*bubblegum)); block_epc_i2c(4);

	dcmi_start_dma(&dcsa, SIZEOF_4DCS);
	epc_trig();

	int stray = measure_stray();

	//DBG_PR_VAR_I32(stray);

	// Do something here:
	copy_cal_to_shadow(sidx, 1);

	if(poll_capt_with_timeout_complete()) log_err(sidx);

	if(stray > 13000)
	{
		total_sensor_obstacle(sidx);
		return;
	}

	conv_4dcs_to_2dcs_hdr0_wide(img20, img31, &dcsa, mono_comp.img);

	// HDR long exp

	epc_intlen(intlen_mults[1], INTUS(OBSTACLE_LONG_US*bubblegum)); block_epc_i2c(4);

	dcmi_start_dma(&dcsa, SIZEOF_4DCS);
	epc_trig();
	adjust();

	// Do something here

	if(poll_capt_with_timeout_complete()) log_err(sidx);

	conv_4dcs_to_2dcs_hdr1_wide(img20, img31, &dcsa, mono_comp.img);

	run_lens_model(img20, img20_lenscorr, tof_calibs[sidx]);
	run_lens_model(img31, img31_lenscorr, tof_calibs[sidx]);

	mask_highly_corrected(0, img20_lenscorr, img31_lenscorr, img20, img31);
//	memcpy(img20_lenscorr, img20, sizeof(img20));
//	memcpy(img31_lenscorr, img31, sizeof(img20));
	compensated_tof_calc_ampldist_nodealias_noampl_nonarrow(ampldist, img20_lenscorr, img31_lenscorr);

	#if 0
		// Normally won't do this, but copy obstacle set for diagnosis output.
		if(tof_slam_set)
		{
			tof_slam_set->sidx = sidx;
			tof_slam_set->sensor_orientation = tof_calibs[sidx]->mount.mount_mode;

			memcpy(tof_slam_set->sets[0].ampldist, ampldist, sizeof ampldist);
		}
	#endif

	tof_to_obstacle_avoidance(ampldist, sidx, 1);
}




static int lowbat_die_cnt = 0;

static uint32_t latest_timestamps[N_SENSORS];

#define MS(x) ((x)*10)
int32_t latency_targets[N_SENSORS] =
{
	MS(2000),  // 0
	MS(2000),  // 1
	MS(2000),  // 2
	MS(2000), // 3
	MS(2000), // 4
	MS(2000), // 5
	MS(2000), // 6
	MS(2000), // 7
	MS(2000),  // 8
	MS(2000)   // 9
};


void run_cycle()
{
	// Checks if charger is mounted, starts charger (blocking, a few milliseconds)
	charger_freerunning_fsm();
	// Checks if application power on is requested (by app_power_on()), runs the turn-on procedure (which is blocking, a few tens of milliseconds)
	app_power_freerunning_fsm();

	drive_freerunning_fsm(); // For now, only manages gyro calibration (rotating the robot, when requested)

	#ifdef VACUUM_APP
		extern volatile int drive_is_rotating; // true whenever the robot is correcting a big angular error, enough to prevent linear motion, i.e., rotating in its place
		ext_vacuum_freerunning_fsm(drive_is_rotating /*temporary rise*/);
	#endif


	for(int i=0; i<10; i++)
	{
		if(!sensors_in_use[i])
			continue;

		tof_mux_select(i);
		update_led(i);
	}




	adjust();




	int bat_mv = VBAT_MEAS_TO_MV(adc1.s.vbat_meas);

	if(
		(bat_mv < 3100*6 && !charger_is_running()) ||
		(bat_mv < 2900*6) )
	{
		lowbat_die_cnt++;
		beep_blocking(10, 4000, 2000);
		if(lowbat_die_cnt > 20)
		{
			SAFETY_SHUTDOWN();
			beep_blocking(10, 4000, 2000);
			delay_ms(200);
			beep_blocking(10, 4000, 2000);
			delay_ms(200);
			beep_blocking(50, 4000, 2000);
			delay_ms(200);
			shutdown();
			while(1);
		}
	}
	else
		lowbat_die_cnt = 0;


	if(is_tx_overrun())
	{
		//uart_print_string_blocking("\r\nTX buffer overrun! Skipping data generation.\r\n"); 
		gen_data = 0;
	}
	else
		gen_data = 1;


	if(gen_data && tof_slam_set)
	{
		tof_slam_set->flags = 0;
	}


	

	/*
	All the sensors run in circular round robin, providing "basic" measurements, sometimes
	joined with a long measurement.

	Quick "obstacle" measurements are stuffed in-between, based on the timing requirements set
	based on the robot travel direction and speed.

	Starvation of round-robin measurements is prevented by limiting maximum number of back-to-back
	"obstacle" measurements
	*/

	static int basic_sidx = 0;


	static int skip;

	if(!drive_is_robot_moving())
	{

		int first_sidx = 0;
		while(!sensors_in_use[first_sidx]) first_sidx++;

		if(basic_sidx == first_sidx)
			skip = 1;
	}

	if(skip)
	{
		if(drive_is_robot_moving())
		{
			skip = 0;
			basic_sidx = 0;
		}
		else
		{
			delay_ms(60);
			goto SKIP_TOF;
		}
	}


	#if 0
		#define N_LONG_SLOTS 3
		static int wid_long_sidxs[N_LONG_SLOTS] = {0, 3, 6};
		static int nar_long_sidxs[N_LONG_SLOTS] = {1, 4, 7};
	#else
		#define N_LONG_SLOTS 5
		static int wid_long_sidxs[N_LONG_SLOTS] = {0, 2, 4, 6, 8};
		static int nar_long_sidxs[N_LONG_SLOTS] = {1, 3, 5, 7, 9};
	#endif

	// 0 3 6,   1 4 7,   2 5 8,   3 6 9,   4 7 0,   5 8 1,   6 9 2,   7 0 3,   8 1 4,   9 2 5, ...
	// 1 4 7,   2 5 8,   3 6 9,   4 7 0,   5 8 1 ,  6 9 2,   7 0 3,   8 1 4,   9 2 5, ...
	do 
	{
		basic_sidx++;
		if(basic_sidx >= N_SENSORS)
		{
			basic_sidx = 0;

			for(int i=0; i<N_LONG_SLOTS; i++)
			{
				wid_long_sidxs[i]++;
				if(wid_long_sidxs[i] >= N_SENSORS)
					wid_long_sidxs[i] = 0;

				nar_long_sidxs[i]++;
				if(nar_long_sidxs[i] >= N_SENSORS)
					nar_long_sidxs[i] = 0;
			}

			extern int obstacle_front_near, obstacle_back_near, obstacle_left_near, obstacle_right_near;
			extern int obstacle_left_very_near, obstacle_right_very_near;
			extern int obstacle_front_far, obstacle_back_far, obstacle_left_far, obstacle_right_far;
			obstacle_front_near = 0;
			obstacle_back_near = 0;
			obstacle_left_near = 0;
			obstacle_right_near = 0;
			obstacle_front_far = 0;
			obstacle_back_far = 0;
			obstacle_left_far = 0;
			obstacle_right_far = 0;
			obstacle_left_very_near = 0;
			obstacle_right_very_near = 0;
		}
	}
	while(!sensors_in_use[basic_sidx]);



	int take_wid_long = 0;

	int take_nar_long = 1; // For now, just take narrow shots every time

	#if 0
		for(int i=0; i<N_LONG_SLOTS; i++)
		{
//			if(basic_sidx == wid_long_sidxs[i])
//				take_wid_long = 1;
			if(basic_sidx == nar_long_sidxs[i])
				take_nar_long = 1;
		}
	#endif

	if(take_wid_long && take_nar_long)
		error(573);

/*	extern int chafind_enabled;
	if(chafind_enabled)
	{
		// Don't waste time doing the longs when aligning to charger
		take_wid_long = 0;
		take_nar_long = 0;
	}
*/

	// On hand-made prototype, sensors 6 and 9 have broken LED strings and need longer exposure
	#ifdef REV2A
		if(basic_sidx == 6 || basic_sidx==9)
			bubblegum = 2;
		else
			bubblegum = 1;
	#endif

	tof_mux_select(basic_sidx);
	adjust();


	if(gen_data && tof_slam_set)
	{
		tof_slam_set->flags = 0 | TOF_SLAM_SET_FLAG_VALID;
		tof_slam_set->sidx = basic_sidx;
		tof_slam_set->sensor_orientation = tof_calibs[basic_sidx]->mount.mount_mode;
	}


	int ssval = 0;
	int narrow_avg_ampl;


	if(basic_sidx == 9)
	{
		cnta++;

		if(cnta == 10)
		{
			cnta = 0;
		}
	}

	
	basic_set(basic_sidx, &ssval, take_nar_long?(&narrow_avg_ampl):NULL);
	latest_timestamps[basic_sidx] = cnt_100us;

	if(take_wid_long)
	{
		long_wide_set(basic_sidx, ssval);

	}

	if(take_nar_long 
		#ifdef REV2A
			&& basic_sidx != 1) // prototype issue: sensor 1 has one of the narrow leds shorted, don't use it.
		#else
			)
		#endif
	{

		#ifdef REV2A
			// Temporary fix: turn off RGB leds during narrow beam imaging - the power supply is undersized to run both
			// (fixed on production PCB)
			
			for(int i=0; i<10; i++)
			{
				if(!sensors_in_use[i])
					continue;

				tof_mux_select(i);
				rgb_update(0);
			}
		#endif

		tof_mux_select(basic_sidx);

		long_narrow_set(basic_sidx, narrow_avg_ampl);


		#ifdef REV2A

			// Turn LEDs back on:
			for(int i=0; i<10; i++)
			{
				if(!sensors_in_use[i])
					continue;

				tof_mux_select(i);
				restart_led(i);
			}
		#endif
	}



	// Take quick obstacle avoidance shots - don't do it if we are mounting to charger
	if(1)// !chafind_enabled)
	{

		// Allow max three back-to-back obstacle images, to prevent starvation of round-robin measurements
		for(int n_obst = 0; n_obst < 3; n_obst++)
		{
			int obst_sidx = -1;
			int32_t lowest_margin = 99999999;
			for(int i=0; i<N_SENSORS; i++)
			{
				int32_t time_from_prev_img = cnt_100us - latest_timestamps[i];
				int32_t margin = latency_targets[i] - time_from_prev_img; // negative margin = we are already late

				//DBG_PR_VAR_I32(i);
				//DBG_PR_VAR_I32(time_from_prev_img);
				//DBG_PR_VAR_I32(margin);

				if(sensors_in_use[i] && margin < lowest_margin)
				{
					lowest_margin = margin;
					obst_sidx = i;
				}
			}

			if(n_obst > 0)
			{
				// We already took one obst avoidance measurement - only take more
				// if they are really getting late
				if(lowest_margin > MS(20))
					break;
			}
			else
			{
				// By all means take at least one obst avoidance image, even if it still isn't acutely urgent
				if(lowest_margin > MS(100))
					break;
			}

			if(obst_sidx < 0 || obst_sidx >= N_SENSORS)
				error(555);

			tof_mux_select(obst_sidx);


			// On hand-made prototype, sensors 6 and 9 have broken LED strings and need longer exposure
			#ifdef REV2A
				if(basic_sidx == 6 || basic_sidx==9)
					bubblegum = 2;
				else
					bubblegum = 1;
			#endif
			obstacle_set(obst_sidx);
			latest_timestamps[obst_sidx] = cnt_100us;


		}

	}


	// 3.2ms memcpy's raw dist, raw dist narrow, ampl8, ampl8 narrow, ambient8.

	adjust();

	SKIP_TOF:;


	if(uart_input() == 'l')
	{
		find_charger();
	}

	extern void chamount_freerun_fsm();

	chamount_freerun_fsm();


	if(gen_data && pwr_status)
	{
		pwr_status->flags = 0;

		if(charger_is_running())
			pwr_status->flags |= PWR_STATUS_FLAG_CHARGING;

		if(charger_is_full())
			pwr_status->flags |= PWR_STATUS_FLAG_FULL;

		extern volatile int main_power_enabled;

		if(main_power_enabled == 1)
			pwr_status->flags |= PWR_STATUS_FLAG_TURNOFF;

		pwr_status->bat_mv = bat_mv; //filtered_bat_mv_x256>>8;
		pwr_status->bat_percent = conv_bat_percent(bat_mv);
		pwr_status->charger_input_mv = CHA_VIN_MEAS_TO_MV(adc1.s.cha_vin_meas);
		pwr_status->pha_charging_current_ma = charger_get_latest_cur_pha();
		pwr_status->phb_charging_current_ma = charger_get_latest_cur_phb();
	}


	if(gen_data)
	{
//		uart_print_string_blocking("\r\nPush!\r\n"); 
		tx_fifo_push();
	}


	for(int i=0; i<10; i++)
	{
		check_rx();
//		delay_ms(10);
		adjust();
	}	


	//profile_cpu_blocking_20ms();

	err_cnt -= 1;

}
