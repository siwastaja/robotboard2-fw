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


static int16_t img20[2][TOF_XS*TOF_YS] __attribute__((section(".dtcm_bss")));
static int16_t img31[2][TOF_XS*TOF_YS];

static uint16_t lofreq_wid_dist[TOF_XS*TOF_YS];
static uint16_t lofreq_nar_dist[TOF_XS*TOF_YS];

static uint8_t  wid_ampl[TOF_XS*TOF_YS];
static uint16_t wid_dist[TOF_XS*TOF_YS];

static uint8_t  nar_ampl[TOF_XS_NARROW*TOF_YS_NARROW];
static uint16_t nar_dist[TOF_XS_NARROW*TOF_YS_NARROW];


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
	
			r = (r*7)>>3;
			g = (g*7)>>3;
			b = (b*7)>>3;
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

#define IFDBG if(sidx == 7)


uint32_t vox_cnt;
int32_t vox_ref_x = 0;
int32_t vox_ref_y = 0;

void restart_voxmap()
{
//	static int tmp_y = 50;
//	static int cnt = 0;
//	cnt++;
//	if(cnt > 5)
//	{
//		cnt = 0;
//		tmp_y++;
//	}
	memset(&voxmap, 0, sizeof(voxmap));
//	voxmap.segs[0][tmp_y*100+80] = 0b0111011001111111;
	vox_cnt++;
	vox_ref_x = cur_pos.x>>16;
	vox_ref_y = cur_pos.y>>16;

}

#define VOXMAP_SEND_INTERVAL 3

extern void adjust();

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


/*

BASIC SET	HDR_FACTOR	16						
								
Beam	mode	freq	Int	Tot.int	t.conv	int+con	range	descr
Both	BW	(10MHz)	150	150	1900	2050		BW compensation
Wide	2DCS	6.66MHz	3072	6144	3800	9944	5499	Dealias
Wide	4DCS	20MHz	12	48	7600	7648	335	Short exp / autoexp basis
Wide	4DCS	20MHz	192	768	7600	8368	1342	HDR short
Wide	4DCS	20MHz	3072	12288	7600	19888	5367	HDR long
								
						Total time		
						47.898	ms
*/

/*
LONG SET	HDR_FACTOR nar	8						
								
Beam	mode	freq	Int	Tot.int	t.conv	int+con	range	descr
Both	BW	(10MHz)	150	150	1900	2050		BW compensation
Wide	2DCS	6.66MHz	2000	4000	3800	7800	4437	Dealias
Wide	4DCS	10MHz	3500	14000	7600	21600	8101	
Narrow	2DCS	6.66MHz	2000	4000	3800	7800	7984	Dealias
Narrow	4DCS	10MHz	600	2400	7600	10000	6185	HDR short
Narrow	4DCS	10MHz	4800	19200	7600	26800	17493	HDR long
								
																
						Total time		
						76.05	ms	

*/

/*
OBSTACLE SET								
								
Beam	mode	freq	Int	Tot.int	t.conv	int+con	range	descr
Both	BW	(10MHz)	150	150	1900	2050		BW compensation
Wide	4DCS	10MHz	50	200	7600	7800	968	HDR short
Wide	4DCS	10MHz	400	1600	3800	5400	2739	HDR long
								
						Total time		
						15.25	ms	
*/

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
	int long_exp = mid_exp*16;
	if(mid_exp_out) *mid_exp_out = mid_exp;
	if(long_exp_out) *long_exp_out = long_exp;

//	DBG_PR_VAR_I32(sidx);
//	DBG_PR_VAR_I32(supershort_avg_ampl_x256);
//	DBG_PR_VAR_I32(mid_exp);
//	DBG_PR_VAR_I32(long_exp);


	// 6.66MHz 2dcs for dealiasing - same inttime as the long HDR exp

	epc_2dcs(); block_epc_i2c(4);
	epc_clk_div(2); block_epc_i2c(4);
	epc_intlen(intlen_mults[2], INTUS(long_exp*bubblegum)); block_epc_i2c(4);

	dcmi_start_dma(&dcs2, SIZEOF_2DCS);
	epc_trig();

	// Do something here


	// Average amplitude on narrow region on the wide mid-exp data is useful for autoexposing the narrow long set later.
	// May not be needed - do not calculate if NULL.
	if(narrow_avg_ampl_out)
		*narrow_avg_ampl_out = calc_avg_ampl_x256_nar_region_on_wide(img20[0], img31[0]);



	// Process the supershort - shadow cal set = 0
	// Supershort gives ~130mm too long results
	compensated_nonhdr_tof_calc_dist_ampl_flarecomp(wid_ampl, wid_dist, img20[0], img31[0], -70);
	tof_to_voxmap(wid_ampl, wid_dist, sidx, 5, 255, vox_ref_x, vox_ref_y);

	copy_cal_to_shadow(sidx, 2);


	if(poll_capt_with_timeout_complete()) log_err(sidx);



	adjust();


	// HDR short exp

	epc_clk_div(0); block_epc_i2c(4);
	epc_intlen(intlen_mults[0], INTUS(mid_exp*bubblegum)); block_epc_i2c(4);
	epc_4dcs(); block_epc_i2c(4);

	dcmi_start_dma(&dcsa, SIZEOF_4DCS);
	epc_trig();
	adjust();

	// Do something here - shadow cal set = 2
	compensated_2dcs_6mhz_dist_masked(lofreq_wid_dist, &dcs2, &mono_comp);



	copy_cal_to_shadow(sidx, 0);



	if(poll_capt_with_timeout_complete()) log_err(sidx);


	conv_4dcs_to_2dcs(img20[0], img31[0], &dcsa, NULL);

	// HDR long exp

	epc_intlen(intlen_mults[0], INTUS(long_exp*bubblegum)); block_epc_i2c(4);

	dcmi_start_dma(&dcsa, SIZEOF_4DCS);
	epc_trig();
	adjust();

	// Do something here



	if(poll_capt_with_timeout_complete()) log_err(sidx);




	conv_4dcs_to_2dcs(img20[1], img31[1], &dcsa, NULL);

	compensated_hdr_tof_calc_dist_ampl_flarecomp(wid_ampl, wid_dist, img20[0], img31[0], img20[1], img31[1]);


#if 1
	if(gen_data && tof_raw_dist)
	{
		tof_raw_dist->sensor_idx = sidx;
		tof_raw_dist->sensor_orientation = sensor_mounts[sidx].mount_mode;
		memcpy(tof_raw_dist->dist, wid_dist, sizeof tof_raw_dist->dist);
		memset(tof_raw_dist->dist_narrow, 0, sizeof tof_raw_dist->dist_narrow);
	}

	if(tof_raw_ampl8)
	{
		tof_raw_ampl8->sensor_idx = sidx;
		memcpy(tof_raw_ampl8->ampl, wid_ampl, sizeof tof_raw_ampl8->ampl);
		memset(tof_raw_ampl8->ampl_narrow, 0, sizeof tof_raw_ampl8->ampl_narrow);
	}
#endif

	adjust();

	dealias_20mhz(wid_dist, lofreq_wid_dist);


	tof_to_voxmap(wid_ampl, wid_dist, sidx, 5, 254, vox_ref_x, vox_ref_y);


#if 0
	if(gen_data && tof_raw_dist)
	{
		tof_raw_dist->sensor_idx = sidx;
		tof_raw_dist->sensor_orientation = sensor_mounts[sidx].mount_mode;
		memcpy(tof_raw_dist->dist, wid_dist, sizeof tof_raw_dist->dist);
		memset(tof_raw_dist->dist_narrow, 0, sizeof tof_raw_dist->dist_narrow);
//		tof_raw_dist->wide_stray_estimate_adc = wide_stray;
//		tof_raw_dist->narrow_stray_estimate_adc = narrow_stray;
	}

	if(gen_data && tof_raw_ampl8)
	{
		tof_raw_ampl8->sensor_idx = sidx;
		memcpy(tof_raw_ampl8->ampl, wid_ampl, sizeof tof_raw_ampl8->ampl);
		memset(tof_raw_ampl8->ampl_narrow, 0, sizeof tof_raw_ampl8->ampl_narrow);
	}
#endif


}


/*
QP1 wide leds, left string
QP2 wide leds, right string
QP3 narrow leds
QP4: yellow: wide supply on robotboard, blue: wide supply on pulutof2 elcap
QP7: same for narrow, supply drops by 400mV!

*/
#if 0
static void test_set(int sidx)
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

#define FREQ 0
#define INTTIME_WIDE 1000
#define INTTIME_NAR 1000

	// WIDE:

	epc_4dcs(); block_epc_i2c(4);
	delay_ms(1);
	epc_ena_wide_leds(); block_epc_i2c(4);

	epc_clk_div(FREQ); block_epc_i2c(4);
	epc_intlen(intlen_mults[FREQ], INTUS(INTTIME_WIDE)); block_epc_i2c(4);

	dcmi_start_dma(&dcsa, SIZEOF_4DCS);
	epc_trig();
	uint16_t wide_stray = measure_stray();

	// Do something here:
	copy_cal_to_shadow(sidx, FREQ);

	if(poll_capt_with_timeout_complete()) log_err(sidx);

	conv_4dcs_to_2dcs(img20[0], img31[0], &dcsa, &mono_comp);

	int wide_avg_ampl_x256 = calc_avg_ampl_x256(img20[0], img31[0]);

	DBG_PR_VAR_I32(wide_avg_ampl_x256);


	delay_ms(1);
	epc_ena_narrow_leds(); dcmi_crop_narrow(); block_epc_i2c(4);
	delay_ms(1);

	epc_intlen(intlen_mults[FREQ], INTUS(INTTIME_NAR)); block_epc_i2c(4);

	dcmi_start_dma(&dcsa_narrow, SIZEOF_4DCS_NARROW);
	epc_trig();
	uint16_t narrow_stray = 0; //measure_stray();


	if(poll_capt_with_timeout_complete()) log_err(sidx);

	conv_4dcs_to_2dcs_narrow(img20[0], img31[0], &dcsa_narrow, &mono_comp);

	int narrow_avg_ampl_x256 = calc_avg_ampl_x256_narrow(img20[0], img31[0]);

	DBG_PR_VAR_I32(narrow_avg_ampl_x256);

	compensated_nonhdr_tof_calc_dist_ampl_flarecomp_narrow(nar_ampl, nar_dist, img20[0], img31[0]);

	if(gen_data && tof_raw_dist)
	{
		tof_raw_dist->sensor_idx = sidx;
		tof_raw_dist->sensor_orientation = sensor_mounts[sidx].mount_mode;
		memcpy(tof_raw_dist->dist, wid_dist, sizeof tof_raw_dist->dist);
		memcpy(tof_raw_dist->dist_narrow, nar_dist, sizeof tof_raw_dist->dist_narrow);
		tof_raw_dist->wide_stray_estimate_adc = wide_stray;
		tof_raw_dist->narrow_stray_estimate_adc = narrow_stray;
	}

	if(gen_data && tof_raw_ampl8)
	{
		tof_raw_ampl8->sensor_idx = sidx;
		memcpy(tof_raw_ampl8->ampl, wid_ampl, sizeof tof_raw_ampl8->ampl);
		memcpy(tof_raw_ampl8->ampl_narrow, nar_ampl, sizeof tof_raw_ampl8->ampl_narrow);
	}

}
#endif

static void long_set(int sidx, int basic_long_exp, int nar_avg_ampl)
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
	// Limit the maximum to tint=4ms, however; we have limited budget for time and motion blur.


	int wide_exp = basic_long_exp * 2;
	if(wide_exp > 4000) wide_exp = 4000;

	int dealias_wide_exp = (wide_exp*19)/10;


	int nar_avg_ampl_saturated = nar_avg_ampl;
	if(nar_avg_ampl_saturated > 800) nar_avg_ampl_saturated = 800;
	
	int nar_exp = 200 + sq(800 - nar_avg_ampl_saturated)/168; // max exposure: 4009

	int dealias_nar_exp = (nar_exp*19)/10;



	// 6.66MHz 2dcs dealias wide

	epc_2dcs(); block_epc_i2c(4);
	delay_ms(1);
	epc_ena_wide_leds(); block_epc_i2c(4);

	epc_clk_div(2); block_epc_i2c(4);
	epc_intlen(intlen_mults[2], INTUS(dealias_wide_exp*bubblegum)); block_epc_i2c(4);

	dcmi_start_dma(&dcs2, SIZEOF_2DCS);
	epc_trig();

	// Do something here

	copy_cal_to_shadow(sidx, 2);


	if(poll_capt_with_timeout_complete()) log_err(sidx);



	// 10MHz wide long exposure

	epc_4dcs(); block_epc_i2c(4);
	epc_clk_div(1); block_epc_i2c(4);
	epc_intlen(intlen_mults[1], INTUS(wide_exp)); block_epc_i2c(4);

	dcmi_start_dma(&dcsa, SIZEOF_4DCS);
	epc_trig();

	// Do something here:
	compensated_2dcs_6mhz_dist_masked(lofreq_wid_dist, &dcs2, &mono_comp);

	copy_cal_to_shadow(sidx, 1);

	if(poll_capt_with_timeout_complete()) log_err(sidx);










	// Basically the same set for narrow:


	// 6.66MHz 2dcs dealias narrow

	epc_2dcs(); block_epc_i2c(4);
	delay_ms(1);
	epc_ena_narrow_leds(); dcmi_crop_narrow(); block_epc_i2c(4);

	epc_clk_div(2); block_epc_i2c(4);
	epc_intlen(intlen_mults[2], INTUS(dealias_nar_exp*bubblegum)); block_epc_i2c(4);

	dcmi_start_dma(&dcs2, SIZEOF_2DCS_NARROW);
	epc_trig();

	// Do something here

	// Process the previous wide stuff.
	conv_4dcs_to_2dcs(img20[0], img31[0], &dcsa, &mono_comp);

	compensated_nonhdr_tof_calc_dist_ampl_flarecomp(wid_ampl, wid_dist, img20[0], img31[0], 0);

	dealias_10mhz(wid_dist, lofreq_wid_dist);

	// remove close readings - they are better captured by the basic set
	for(int i = 0; i<TOF_XS*TOF_YS; i++)
	{
		if(wid_dist[i] < 4000)
			wid_ampl[i] = 0;
	}

	// Similarly, ignore high-amplitude readings - they should be in the basic set.
	// If something is in the basic set at ampl=40, it's here at ampl=160
	tof_to_voxmap(wid_ampl, wid_dist, sidx, 5, 160, vox_ref_x, vox_ref_y);

	copy_cal_to_shadow(sidx, 2);


	if(poll_capt_with_timeout_complete()) log_err(sidx);






	// 10MHz narrow long exposure

	epc_4dcs(); block_epc_i2c(4);
	epc_clk_div(1); block_epc_i2c(4);
	epc_intlen(intlen_mults[1], INTUS(nar_exp)); block_epc_i2c(4);

	dcmi_start_dma(&dcsa_narrow, SIZEOF_4DCS_NARROW);
	epc_trig();

	// Do something here:
	compensated_2dcs_6mhz_dist_masked(lofreq_wid_dist, &dcs2, &mono_comp);

	copy_cal_to_shadow(sidx, 1);

	if(poll_capt_with_timeout_complete()) log_err(sidx);


	conv_4dcs_to_2dcs_narrow(img20[0], img31[0], &dcsa_narrow, &mono_comp);

	compensated_nonhdr_tof_calc_dist_ampl_flarecomp_narrow(nar_ampl, nar_dist, img20[0], img31[0], 0);

	dealias_10mhz_narrow(nar_dist, lofreq_nar_dist);

	// remove close readings - they are better captured by the basic set
	for(int i = 0; i<TOF_XS_NARROW*TOF_YS_NARROW; i++)
	{
		if(nar_dist[i] < 4000)
			nar_ampl[i] = 0;
	}

	// Similarly, ignore high-amplitude readings - they should be in the basic set.
	tof_to_voxmap_narrow(nar_ampl, nar_dist, sidx, 4, 160, vox_ref_x, vox_ref_y);


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
	#define OBSTACLE_LONG_US  (OBSTACLE_SHORT_US*HDR_FACTOR)

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

	compensated_hdr_tof_calc_dist_ampl_flarecomp(wid_ampl, wid_dist, img20[0], img31[0], img20[1], img31[1]);

	adjust();


	tof_to_obstacle_avoidance(wid_ampl, wid_dist, sidx, 5, 255);
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
	for(int i=0; i<10; i++)
	{
		if(!sensors_in_use[i])
			continue;

		tof_mux_select(i);
		update_led(i);
	}




	charger_freerunning_fsm();



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
	static int long_sidx_a = 0;
	static int long_sidx_b = 3;
	static int long_sidx_c = 6;

	// 0 3 6,   1 4 7,   2 5 8,   3 6 9,   4 7 0,   5 8 1,   6 9 2,   7 0 3,   8 1 4,   9 2 5, ...
	do 
	{
		basic_sidx++;
		if(basic_sidx >= N_SENSORS)
		{
			basic_sidx = 0;

			long_sidx_a++;
			long_sidx_b++;
			long_sidx_c++;

			if(long_sidx_a >= N_SENSORS)
				long_sidx_a = 0;
			if(long_sidx_b >= N_SENSORS)
				long_sidx_b = 0;
			if(long_sidx_c >= N_SENSORS)
				long_sidx_c = 0;


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



	if(is_tx_overrun())
	{
		//uart_print_string_blocking("\r\nTX buffer overrun! Skipping data generation.\r\n"); 
		gen_data = 0;
	}
	else
		gen_data = 1;


	int take_long = 0;
	if(basic_sidx == long_sidx_a || basic_sidx == long_sidx_b || basic_sidx == long_sidx_c)
		take_long = 1;

	#define VOXMAP_SEND_MASK 0b1111111110
	static int voxmap_send_mask;
	if(take_long)
	{
		voxmap_send_mask |= 1<<basic_sidx;
	}


	extern int chafind_enabled;
	if(chafind_enabled)
		take_long = 0; // Don't waste time doing the longs when aligning to charger


	int send_voxmap = 0;
	if((voxmap_send_mask & VOXMAP_SEND_MASK) == VOXMAP_SEND_MASK)
	{
		send_voxmap = 1;
		add_sub(1); // Force this subscription for now... Others may be temporarily dropped if they won't fit (this has lowest ID)
	}


	if(gen_data && pwr_status)
	{
		pwr_status->flags = 0;

		if(charger_is_running())
			pwr_status->flags |= PWR_STATUS_FLAG_CHARGING;

		if(charger_is_full())
			pwr_status->flags |= PWR_STATUS_FLAG_FULL;

		pwr_status->bat_mv = bat_mv; //filtered_bat_mv_x256>>8;
		pwr_status->bat_percent = conv_bat_percent(bat_mv);
		pwr_status->charger_input_mv = CHA_VIN_MEAS_TO_MV(adc1.s.cha_vin_meas);
		pwr_status->pha_charging_current_ma = charger_get_latest_cur_pha();
		pwr_status->phb_charging_current_ma = charger_get_latest_cur_phb();
	}



	// On hand-made prototype, sensors 6 and 9 have broken LED strings and need longer exposure
	if(basic_sidx == 6 || basic_sidx==9)
		bubblegum = 2;
	else
		bubblegum = 1;


	int long_exp;
	int narrow_avg_ampl;


	//tof_mux_select(5);
	//test_set(5);
	//delay_ms(950);


	tof_mux_select(basic_sidx);
	adjust();


	basic_set(basic_sidx, NULL, &long_exp, take_long?(&narrow_avg_ampl):NULL);
	latest_timestamps[basic_sidx] = cnt_100us;

	if(take_long && basic_sidx != 1) // prototype issue: sensor 1 has one of the narrow leds shorted, don't use it.
	{

		// Temporary fix: turn off RGB leds during narrow beam imaging - the power supply is undersized to run both
		// (fixed on production PCB)
		
		for(int i=0; i<10; i++)
		{
			if(!sensors_in_use[i])
				continue;

			tof_mux_select(i);
			rgb_update(0);
		}
		tof_mux_select(basic_sidx);


		long_set(basic_sidx, long_exp, narrow_avg_ampl);


		// Turn LEDs back on:
		for(int i=0; i<10; i++)
		{
			if(!sensors_in_use[i])
				continue;

			tof_mux_select(i);
			restart_led(i);
		}


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
			if(basic_sidx == 6 || basic_sidx==9)
				bubblegum = 2;
			else
				bubblegum = 1;

			obstacle_set(obst_sidx);
			latest_timestamps[obst_sidx] = cnt_100us;


		}

	}


/*	if(gen_data && tof_diagnostics)
	{
		tof_diagnostics->temperature = chiptemp;
	}
*/

	adjust();

	int do_restore_subs;
	if(send_voxmap)
	{
		// Subscription is forced above.
		if(!mcu_multi_voxel_map)
		{
			error(155); // multi_voxel_map must fit by design; if it doesn't, make it smaller, or increase TX FIFO max size.
		}
		if(gen_data)
		{
			static int bl;

			mcu_multi_voxel_map->running_cnt = vox_cnt;
			mcu_multi_voxel_map->ref_x = vox_ref_x;
			mcu_multi_voxel_map->ref_y = vox_ref_y;
			mcu_multi_voxel_map->first_block_id = bl;
			mcu_multi_voxel_map->z_step = Z_STEP;
			mcu_multi_voxel_map->base_z = BASE_Z;
			//DBG_PR_VAR_I32(bl);
			memcpy(mcu_multi_voxel_map->maps[0], voxmap.segs[bl+0], sizeof(mcu_multi_voxel_map->maps[0]));
			memcpy(mcu_multi_voxel_map->maps[1], voxmap.segs[bl+1], sizeof(mcu_multi_voxel_map->maps[0]));
			memcpy(mcu_multi_voxel_map->maps[2], voxmap.segs[bl+2], sizeof(mcu_multi_voxel_map->maps[0]));
			bl+=3;
			if(bl >= 12)
			{
				//uart_print_string_blocking("restart\r\n"); 
				bl = 0;
				execute_corr_pos();
				restart_voxmap();
				voxmap_send_mask = 0;
				do_restore_subs = 1;
			}
		}
		else
		{
			//uart_print_string_blocking("\r\nVOXMAP: TX buffer overrun! Skipping data generation.\r\n"); 

		}
		
	}



/*
	if(gen_data && tof_raw_ambient8)
	{
		tof_raw_ambient8->sensor_idx = sidx;
		memcpy(tof_raw_ambient8->ambient, ambient, sizeof tof_raw_ambient8->ambient);
		tof_raw_ambient8->temperature = chiptemp;
	}
*/

	// 3.2ms memcpy's raw dist, raw dist narrow, ampl8, ampl8 narrow, ambient8.

	adjust();

	micronavi_fsm();



//	SKIP_TOF:;

	if(gen_data)
	{
//		uart_print_string_blocking("\r\nPush!\r\n"); 
		tx_fifo_push();
	}

	if(do_restore_subs)
	{
		restore_subs();
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
