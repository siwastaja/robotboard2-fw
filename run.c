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
static int bubblegum = 1; // Temporary bodge to increase exposure times on two hand-soldered prototype sensors, which lack half of the LEDs
static int gen_data;


static int16_t img20[3][TOF_XS*TOF_YS]; // __attribute__((section(".dtcm_bss")));
static int16_t img31[3][TOF_XS*TOF_YS];

static uint8_t lofreq_dist[TOF_XS*TOF_YS];

static uint16_t ampldist[TOF_XS*TOF_YS];



uint8_t conv_bat_percent(int mv)
{
	int bat_percentage = (100*(mv-(3200*6)))/((4200-3200)*6);
	if(bat_percentage < 0) bat_percentage = 0;
	if(bat_percentage > 127) bat_percentage = 127;
	return bat_percentage;

}

epc_img_t mono_comp __attribute__((aligned(4)));
epc_4dcs_t dcsa __attribute__((aligned(4)));
epc_2dcs_t dcs2 __attribute__((aligned(4)));
epc_4dcs_narrow_t dcsa_narrow __attribute__((aligned(4)));
epc_2dcs_narrow_t dcs2_narrow __attribute__((aligned(4)));

void soft_err()
{
	while(1)
	{
		beep_blocking(15, 1000, 100);
		delay_ms(500);
	}
}

uint32_t timestamp_initial;
#define INIT_TOF_TS() do{timestamp_initial = cnt_100us;}while(0)
#define TOF_TS(id_) do{if(gen_data && tof_diagnostics) { tof_diagnostics->timestamps[(id_)] = cnt_100us - timestamp_initial;} }while(0)




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

uint16_t measure_stray()
{
	int32_t stray = 16384;
	for(int i=0; i<40; i++)
	{
//		delay_us(12);
		uint16_t now = adc3.s.epc_stray_estimate;
		DBG_PR_VAR_I16(now);

		if(now < stray) stray = now;
	}

	DBG_PR_VAR_I16(stray);

//	stray = 16384-stray-1000;
//	if(stray < 0) stray = 0;
//	int64_t s = stray;
//	stray = (s*s*s)/1500000000LL;
	return stray;
}

#define IFDBG if(sidx == 99)


void adjust()
{
	return;
}

/*
	Takes in the latest chip temperature measurement.
	Averages it (per sensor, keeps track of sensors internally) using an exponentially decaying running average
	Returns the number of fine dll steps required, based on that average and sensor calibration parameters
*/
static int filt_calc_dll_steps(int sidx, int32_t chiptemp)
{
	static int32_t chiptemp_flt_x256[N_SENSORS];

	if(sidx < 0 || sidx >= N_SENSORS)
		error(1234);

	chiptemp_flt_x256[sidx] = (7*chiptemp_flt_x256[sidx] + (chiptemp<<8))>>3;

	int32_t chiptemp_flt = chiptemp_flt_x256[sidx]>>8;

	int fine_steps = ((tof_calibs[sidx]->zerofine_temp-chiptemp_flt)*tof_calibs[sidx]->fine_steps_per_temp[0])>>8;
	if(fine_steps < 0) fine_steps = 0;
	else if(fine_steps > 799) fine_steps = 799;

	return fine_steps;
}


static void basic_set(int sidx, int* mid_exp_out, int* long_exp_out, int* narrow_avg_ampl_out)
{
	// Compensation BW + chiptemp
	dcmi_crop_wide();
	epc_greyscale(); block_epc_i2c(4);
	delay_ms(1);
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

	int fine_steps = filt_calc_dll_steps(sidx, chiptemp);
	epc_fine_dll_steps(fine_steps); block_epc_i2c(4);


	#define SUPERSHORT_US 12

	// SUPER SHORT WIDE
	// Works as autoexposure basis & HDR short

	epc_4dcs(); block_epc_i2c(4);
	delay_ms(1);
	epc_ena_wide_leds(); block_epc_i2c(4);

	epc_clk_div(0); block_epc_i2c(4);
	epc_intlen(intlen_mults[0], INTUS(SUPERSHORT_US*bubblegum)); block_epc_i2c(4);

	dcmi_start_dma(&dcsa, SIZEOF_4DCS);
	epc_trig();

	// Do something here:
		copy_cal_to_shadow(sidx, 0);

	if(poll_capt_with_timeout_complete()) log_err(sidx);

	conv_4dcs_to_2dcs(img20[0], img31[0], &dcsa, &mono_comp);

	int supershort_avg_ampl_x256 = calc_avg_ampl_x256(img20[0], img31[0]);

	IFDBG
	{
//		DBG_PR_VAR_I32(supershort_avg_ampl_x256);
//		DBG_PR_VAR_I32(base_exp);
	}

	int ss_saturated = (int)supershort_avg_ampl_x256;
	if(ss_saturated > 400) ss_saturated = 400;
	
	int mid_exp = SUPERSHORT_US + sq(400 - ss_saturated)/888; // max exposures: 192, 3072

	if(mid_exp < 12) mid_exp = 12;

	// mid exp 24..192
	// long exp 384..3072
	// first hdr factor 2..16
	// second hdr factor 16

	// Make mid_exp integer multiple of SUPERSHORT
	mid_exp /= SUPERSHORT_US;
	int first_hdr_factor = mid_exp;
	mid_exp *= SUPERSHORT_US;

	int long_exp = mid_exp*16;
	if(mid_exp_out) *mid_exp_out = mid_exp;
	if(long_exp_out) *long_exp_out = long_exp;

	IFDBG
	{
		DBG_PR_VAR_I32(sidx);
		DBG_PR_VAR_I32(supershort_avg_ampl_x256);
		DBG_PR_VAR_I32(mid_exp);
		DBG_PR_VAR_I32(long_exp);
	}

	// 6.66MHz 2dcs for dealiasing - same inttime as the long HDR exp
	// (Going from 20MHz to 6.66MHz gives amplitude gain of at least 2, enough to compensate
	// for halving the amplitude from using 2DCS mode)

	epc_2dcs(); block_epc_i2c(4);
	epc_clk_div(2); block_epc_i2c(4);
	epc_intlen(intlen_mults[2], INTUS(long_exp*bubblegum)); block_epc_i2c(4);

	dcmi_start_dma(&dcs2, SIZEOF_2DCS);
	epc_trig();

	// Do something here


		// Average amplitude on narrow region on the wide mid-exp data is useful for 
		// autoexposing the narrow long set later.
		// May not be needed - do not calculate if NULL.
		if(narrow_avg_ampl_out)
			*narrow_avg_ampl_out = calc_avg_ampl_x256_nar_region_on_wide(img20[0], img31[0]);

		copy_cal_to_shadow(sidx, 2);


	if(poll_capt_with_timeout_complete()) log_err(sidx);

	adjust();


	// HDR mid exp

	epc_clk_div(0); block_epc_i2c(4);
	epc_intlen(intlen_mults[0], INTUS(mid_exp*bubblegum)); block_epc_i2c(4);
	epc_4dcs(); block_epc_i2c(4);

	dcmi_start_dma(&dcsa, SIZEOF_4DCS);
	epc_trig();
	adjust();

	// Do something here - shadow cal set = 2
		compensated_2dcs_6mhz_dist_masked(lofreq_dist, &dcs2, &mono_comp);

		copy_cal_to_shadow(sidx, 0);


		// Take the pose here
		if(tof_slam_set)
		{
			hires_pos_to_hw_pose(&tof_slam_set->sets[0].pose, &cur_pos);
		}

	if(poll_capt_with_timeout_complete()) log_err(sidx);

	conv_4dcs_to_2dcs(img20[1], img31[1], &dcsa, NULL);

	// HDR long exp

	epc_intlen(intlen_mults[0], INTUS(long_exp*bubblegum)); block_epc_i2c(4);

	dcmi_start_dma(&dcsa, SIZEOF_4DCS);
	epc_trig();
	adjust();

	// Do something here

	if(poll_capt_with_timeout_complete()) log_err(sidx);


	conv_4dcs_to_2dcs(img20[2], img31[2], &dcsa, NULL);

	compensated_3hdr_tof_calc_ampldist_flarecomp(0, ampldist, 
		img20[0], img31[0], img20[1], img31[1], img20[2], img31[2], first_hdr_factor, 16, lofreq_dist, 0);


	tof_to_obstacle_avoidance(ampldist, sidx);

	if(tof_slam_set)
	{
		memcpy(tof_slam_set->sets[0].ampldist, ampldist, sizeof ampldist);
	}

	adjust();


}


static void long_wide_set(int sidx, int basic_long_exp)
{
	// Compensation BW + chiptemp
	// We could reuse the BW from basic set, but that would be too old now (false compensation due to motion)
	dcmi_crop_wide();
	epc_greyscale(); block_epc_i2c(4);
	delay_ms(1);
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

	int fine_steps = filt_calc_dll_steps(sidx, chiptemp);
	epc_fine_dll_steps(fine_steps); block_epc_i2c(4);


	// Do the longer-range wide. Use the autoexposure from before, but allow
	// *4 exposure (*2 from inttime, *2 from using lower frequency), even if this
	// causes increased error due to stray / multipath. Using a fixed long value would
	// go totally overboard, but *4 (*2 the distance) seems a good compromise.
	// Limit the maximum to tint=4.4ms, however; we have limited budget for time and motion blur.


	int short_exp = (basic_long_exp * 2) / 16;
	if(short_exp > 4400/16) short_exp = 4400/16;

	int long_exp = short_exp * 16;

	int dealias_exp = (long_exp*19)/10;


	// 6.66MHz 2dcs dealias wide

	epc_2dcs(); block_epc_i2c(4);
	delay_ms(1);
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

	epc_4dcs(); block_epc_i2c(4);
	epc_clk_div(1); block_epc_i2c(4);
	epc_intlen(intlen_mults[1], INTUS(short_exp)); block_epc_i2c(4);

	dcmi_start_dma(&dcsa, SIZEOF_4DCS);
	epc_trig();

	// Do something here:
		compensated_2dcs_6mhz_dist_masked(lofreq_dist, &dcs2, &mono_comp);

		copy_cal_to_shadow(sidx, 1);

		adjust();

		// Take the pose here
		if(tof_slam_set)
		{
			hires_pos_to_hw_pose(&tof_slam_set->sets[1].pose, &cur_pos);
		}


	if(poll_capt_with_timeout_complete()) log_err(sidx);

	conv_4dcs_to_2dcs(img20[0], img31[0], &dcsa, NULL);


	// 10MHz wide long exposure

	epc_intlen(intlen_mults[1], INTUS(long_exp)); block_epc_i2c(4);

	dcmi_start_dma(&dcsa, SIZEOF_4DCS);
	epc_trig();

	// Do something here:
		adjust();

	if(poll_capt_with_timeout_complete()) log_err(sidx);

	conv_4dcs_to_2dcs(img20[1], img31[1], &dcsa, NULL);

	compensated_2hdr_tof_calc_ampldist_flarecomp(0, ampldist, 
		img20[0], img31[0], img20[1], img31[1], 16, lofreq_dist, 1);

	if(tof_slam_set)
	{
		tof_slam_set->flags |= TOF_SLAM_SET_FLAG_SET1_WIDE;
		memcpy(tof_slam_set->sets[1].ampldist, ampldist, sizeof ampldist);
	}

	adjust();
}


static void long_narrow_set(int sidx, int avg_ampl)
{
	// Compensation BW + chiptemp
	// We could reuse the BW from basic set, but that would be too old now (false compensation due to motion)
	dcmi_crop_wide();
	epc_greyscale(); block_epc_i2c(4);
	delay_ms(1);
	epc_dis_leds(); block_epc_i2c(4);
	epc_clk_div(1); block_epc_i2c(4);
	epc_intlen(intlen_mults[1], INTUS(COMP_AMBIENT_INTLEN_US)); block_epc_i2c(4);
	epc_temperature_magic_mode(sidx);
	dcmi_start_dma(&mono_comp, SIZEOF_MONO);
	epc_trig();

	// Do something here:


	if(poll_capt_with_timeout_complete()) log_err(sidx);

	int32_t chiptemp = epc_read_temperature(sidx);
	epc_temperature_magic_mode_off(sidx);

	int fine_steps = filt_calc_dll_steps(sidx, chiptemp);
	epc_fine_dll_steps(fine_steps); block_epc_i2c(4);


	int avg_ampl_saturated = avg_ampl;
	if(avg_ampl_saturated > 800) avg_ampl_saturated = 800;
	
	int long_exp = 200 + sq(800 - avg_ampl_saturated)/168; // max exposure: 4009

	int short_exp = long_exp/16; // [200..4009] -> [12..250]

	long_exp = short_exp*16; // [200..4009] -> [192..4000]

	int dealias_exp = (long_exp*19)/10; // [364..7600]


	// 6.66MHz 2dcs dealias narrow

	epc_2dcs(); block_epc_i2c(4);
	delay_ms(1);
	epc_ena_narrow_leds(); dcmi_crop_narrow(); block_epc_i2c(4);

	epc_clk_div(2); block_epc_i2c(4);
	epc_intlen(intlen_mults[2], INTUS(dealias_exp*bubblegum)); block_epc_i2c(4);

	dcmi_start_dma(&dcs2_narrow, SIZEOF_2DCS_NARROW);
	epc_trig();

	// Do something here:

		copy_cal_to_shadow(sidx, 2);

	if(poll_capt_with_timeout_complete()) log_err(sidx);


	// 10MHz narrow HDR short

	epc_4dcs(); block_epc_i2c(4);
	epc_clk_div(1); block_epc_i2c(4);
	epc_intlen(intlen_mults[1], INTUS(short_exp)); block_epc_i2c(4);

	dcmi_start_dma(&dcsa_narrow, SIZEOF_4DCS_NARROW);
	epc_trig();

	// Do something here:

		compensated_2dcs_6mhz_dist_masked_narrow(lofreq_dist, &dcs2_narrow, &mono_comp);

		copy_cal_to_shadow(sidx, 1);

		// Take the pose here
		if(tof_slam_set)
		{
			hires_pos_to_hw_pose(&tof_slam_set->sets[1].pose, &cur_pos);
		}


	if(poll_capt_with_timeout_complete()) log_err(sidx);


	conv_4dcs_to_2dcs_narrow(img20[0], img31[0], &dcsa_narrow, &mono_comp);


	// 10MHz narrow HDR long

	epc_intlen(intlen_mults[1], INTUS(long_exp)); block_epc_i2c(4);

	dcmi_start_dma(&dcsa_narrow, SIZEOF_4DCS_NARROW);
	epc_trig();

	// Do something here:



	if(poll_capt_with_timeout_complete()) log_err(sidx);


	conv_4dcs_to_2dcs_narrow(img20[1], img31[1], &dcsa_narrow, &mono_comp);



	compensated_2hdr_tof_calc_ampldist_flarecomp(1, ampldist, 
		img20[0], img31[0], img20[1], img31[1], 16, lofreq_dist, 1);

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
	delay_ms(1);
	epc_dis_leds(); block_epc_i2c(4);
	epc_clk_div(1); block_epc_i2c(4);
	epc_intlen(intlen_mults[1], INTUS(COMP_AMBIENT_INTLEN_US)); block_epc_i2c(4);
	epc_temperature_magic_mode(sidx);
	dcmi_start_dma(&mono_comp, SIZEOF_MONO);
	epc_trig();

	// Do something here

	if(poll_capt_with_timeout_complete()) log_err(sidx);

	// Don't read the temperature


	#define OBSTACLE_SHORT_US 25
	#define OBSTACLE_LONG_US  (OBSTACLE_SHORT_US*16)

	// HDR short exp

	epc_4dcs(); block_epc_i2c(4);
	delay_ms(1);
	epc_ena_wide_leds(); block_epc_i2c(4);

	epc_intlen(intlen_mults[1], INTUS(OBSTACLE_SHORT_US*bubblegum)); block_epc_i2c(4);

	dcmi_start_dma(&dcsa, SIZEOF_4DCS);
	epc_trig();

	// Do something here:
	copy_cal_to_shadow(sidx, 1);

	if(poll_capt_with_timeout_complete()) log_err(sidx);

	conv_4dcs_to_2dcs(img20[0], img31[0], &dcsa, NULL);

	// HDR long exp

	epc_intlen(intlen_mults[1], INTUS(OBSTACLE_LONG_US*bubblegum)); block_epc_i2c(4);

	dcmi_start_dma(&dcsa, SIZEOF_4DCS);
	epc_trig();
	adjust();

	// Do something here

	if(poll_capt_with_timeout_complete()) log_err(sidx);

	conv_4dcs_to_2dcs(img20[1], img31[1], &dcsa, NULL);

	compensated_2hdr_tof_calc_ampldist_flarecomp(0, ampldist, 
		img20[0], img31[0], img20[1], img31[1], 16, NULL, 1);

	adjust();


	tof_to_obstacle_avoidance(ampldist, sidx);
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


void run_cycle()  __attribute__((section(".text_itcm")));
void run_cycle()
{
	charger_freerunning_fsm();

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


	if(!drive_is_robot_moving())
	{
		delay_ms(40);
		goto SKIP_TOF;
	}

//	static int filtered_bat_mv_x256; 
//	filtered_bat_mv_x256 = ((bat_mv<<8) + 15*filtered_bat_mv_x256)>>4;


//	sidx = 7;

//		goto SKIP_TOF;



	/*
	All the sensors run in circular round robin, providing "basic" measurements, sometimes
	joined with a long measurement.

	Quick "obstacle" measurements are stuffed in-between, based on the timing requirements set
	based on the robot travel direction and speed.

	Starvation of round-robin measurements is prevented by limiting maximum number of back-to-back
	"obstacle" measurements
	*/

	static int basic_sidx = 0;

	#define N_LONG_SLOTS 3
	static int wid_long_sidxs[N_LONG_SLOTS] = {0, 3, 6};
	static int nar_long_sidxs[N_LONG_SLOTS] = {1, 4, 7};

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
			extern int obstacle_front_far, obstacle_back_far, obstacle_left_far, obstacle_right_far;
			obstacle_front_near = 0;
			obstacle_back_near = 0;
			obstacle_left_near = 0;
			obstacle_right_near = 0;
			obstacle_front_far = 0;
			obstacle_back_far = 0;
			obstacle_left_far = 0;
			obstacle_right_far = 0;
		}
	}
	while(!sensors_in_use[basic_sidx]);



	int take_wid_long = 0;
	int take_nar_long = 0;
	for(int i=0; i<N_LONG_SLOTS; i++)
	{
		if(basic_sidx == wid_long_sidxs[i])
			take_wid_long = 1;
		if(basic_sidx == nar_long_sidxs[i])
			take_nar_long = 1;
	}

	if(take_wid_long && take_nar_long)
		error(573);

	extern int chafind_enabled;
	if(chafind_enabled)
	{
		// Don't waste time doing the longs when aligning to charger
		take_wid_long = 0;
		take_nar_long = 0;
	}

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


	int long_exp;
	int narrow_avg_ampl;


	basic_set(basic_sidx, NULL, &long_exp, take_nar_long?(&narrow_avg_ampl):NULL);
	latest_timestamps[basic_sidx] = cnt_100us;

	if(take_wid_long)
	{
		long_wide_set(basic_sidx, long_exp);

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
	if(!chafind_enabled)
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

	micronavi_fsm();

	SKIP_TOF:;

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
