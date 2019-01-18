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

#ifdef USE_NARROW
epc_4dcs_narrow_t dcsa_narrow __attribute__((aligned(4)));
#endif

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
	err_cnt += 10;
	if(err_cnt > 30)
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

uint16_t measure_stray()
{
	int32_t stray = 16384;
	for(int i=0; i<40; i++)
	{
		delay_us(12);
		uint16_t now = adc3.s.epc_stray_estimate;
		if(now < stray) stray = now;
	}

	stray = 16384-stray-1000;
	if(stray < 0) stray = 0;
	int64_t s = stray;
	stray = (s*s*s)/1500000000LL;
	return stray;
}

#define IFDBG if(sidx == 7)


//static int round_of_longer_exposure;

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

static int lowbat_die_cnt = 0;
void run_cycle()  __attribute__((section(".text_itcm")));
void run_cycle()
{

	static int16_t wid_hdr20[2][TOF_XS*TOF_YS] __attribute__((section(".dtcm_bss")));
	static int16_t wid_hdr31[2][TOF_XS*TOF_YS];

#ifdef USE_NARROW
	static int16_t nar_hdr20[2][TOF_XS_NARROW*TOF_YS_NARROW] __attribute__((section(".dtcm_bss")));
	static int16_t nar_hdr31[2][TOF_XS_NARROW*TOF_YS_NARROW] __attribute__((section(".dtcm_bss")));
#endif

	static uint16_t lofreq_wid_dist[TOF_XS*TOF_YS];

	static uint8_t  wid_ampl[TOF_XS*TOF_YS];
	static uint16_t wid_dist[TOF_XS*TOF_YS];

#ifdef USE_NARROW
	static uint8_t  nar_ampl[TOF_XS_NARROW*TOF_YS_NARROW];
	static uint16_t nar_dist[TOF_XS_NARROW*TOF_YS_NARROW];
#endif


	for(int i=0; i<10; i++)
	{
		if(!sensors_in_use[i])
			continue;

		tof_mux_select(i);
		update_led(i);
	}



	static int sidx = 0;
	sidx++;
//	sidx = 5;
//	static int test_cnt = -1;

//	test_cnt++;
//	if(test_cnt > 2)
//	{
//		test_cnt = 0;
//	}

	if(sidx >= N_SENSORS)
	{
		sidx = 0;
//		round_of_longer_exposure = ~round_of_longer_exposure;
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
	if(!sensors_in_use[sidx])
		return;

	charger_freerunning_fsm();

	// On hand-made prototype, sensors 6 and 9 have broken LED strings and need longer exposure
	int bubblegum = 1;
	if(sidx == 6 || sidx==9)
		bubblegum = 2;


	adjust();

	int gen_data = 1;

//	if(sidx&1)
//		gen_data = 1;

	if(is_tx_overrun())
	{
		uart_print_string_blocking("\r\nTX buffer overrun! Skipping data generation.\r\n"); 
		gen_data = 0;
	}

	static int voxmap_send_cnt;

	if(sidx == 9) // && round_of_longer_exposure)
	{
		voxmap_send_cnt++;
	}
	
	if(voxmap_send_cnt >= VOXMAP_SEND_INTERVAL)
	{
		add_sub(1); // Force this subscription for now... Others may be temporarily dropped if they won't fit (this has lowest ID)
	}


	int bat_mv = VBAT_MEAS_TO_MV(adc1.s.vbat_meas);

	if(bat_mv < 3100*6)
	{
		lowbat_die_cnt++;
		if(lowbat_die_cnt > 20)
		{
			beep_blocking(10, 4000, 1500);
			delay_ms(100);
			beep_blocking(10, 4000, 1500);
			delay_ms(100);
			beep_blocking(50, 4000, 1500);
			delay_ms(100);
			shutdown();

		}
	}
	else
		lowbat_die_cnt = 0;

//	static int filtered_bat_mv_x256; 
//	filtered_bat_mv_x256 = ((bat_mv<<8) + 15*filtered_bat_mv_x256)>>4;

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



//	sidx = 7;

//		goto SKIP_TOF;


	const int intlen_mults[4] = {12, 6, 4, 3}; // with these, unit is always 0.6us.


	tof_mux_select(sidx);
	adjust();


	INIT_TOF_TS();

	TOF_TS(0);

	// Acquire compensation B/W with fixed intlen - chip temperature at the same time
	dcmi_crop_wide();
	epc_greyscale(); block_epc_i2c(4);
	delay_ms(1);
	epc_dis_leds(); block_epc_i2c(4);
	epc_clk_div(1); block_epc_i2c(4);
	epc_intlen(intlen_mults[1], INTUS(COMP_AMBIENT_INTLEN_US)); block_epc_i2c(4);
	epc_temperature_magic_mode(sidx);
	dcmi_start_dma(&mono_comp, SIZEOF_MONO);
	epc_trig();
	if(poll_capt_with_timeout_complete()) log_err(sidx);

	int32_t chiptemp = epc_read_temperature(sidx);
	epc_temperature_magic_mode_off(sidx);

	int fine_steps = ((tof_calibs[sidx]->zerofine_temp-chiptemp)*tof_calibs[sidx]->fine_steps_per_temp[0])>>8;
	if(fine_steps < 0) fine_steps = 0;
	else if(fine_steps > 799) fine_steps = 799;


/*
	IFDBG
	{
		DBG_PR_VAR_I32(chiptemp);
		DBG_PR_VAR_I32(fine_steps);
	}
*/

	epc_fine_dll_steps(fine_steps); block_epc_i2c(4);


	TOF_TS(1);


	// 6.66MHz 2dcs

	epc_2dcs(); block_epc_i2c(4);
	delay_ms(1);
	epc_ena_wide_leds(); dcmi_crop_wide(); block_epc_i2c(4);

	epc_clk_div(2); block_epc_i2c(4);
	epc_intlen(intlen_mults[2], INTUS(4000*bubblegum)); block_epc_i2c(4);

	dcmi_start_dma(&dcs2, SIZEOF_2DCS);
	epc_trig();

	copy_cal_to_shadow(sidx, 2);

	if(poll_capt_with_timeout_complete()) log_err(sidx);

	TOF_TS(2);


	#ifndef HDR_FACTOR
		#define HDR_FACTOR 8
	#endif

	#define SUPERSHORT1_US 12
	#define SUPERSHORT2_US (SUPERSHORT1_US*HDR_FACTOR)

	// SUPER SHORT WIDE

	epc_4dcs(); block_epc_i2c(4);
	delay_ms(1);
	epc_ena_wide_leds(); dcmi_crop_wide(); block_epc_i2c(4);

	epc_clk_div(0); block_epc_i2c(4);
	epc_intlen(intlen_mults[0], INTUS(SUPERSHORT1_US*bubblegum)); block_epc_i2c(4);

	dcmi_start_dma(&dcsa, SIZEOF_4DCS);
	epc_trig();

	compensated_2dcs_6mhz_dist_masked(lofreq_wid_dist, &dcs2, &mono_comp); // Calc the prev

	copy_cal_to_shadow(sidx, 0);

	if(poll_capt_with_timeout_complete()) log_err(sidx);

	TOF_TS(3);

	conv_4dcs_to_2dcs(wid_hdr20[0], wid_hdr31[0], &dcsa, &mono_comp);

	epc_intlen(intlen_mults[0], INTUS(SUPERSHORT2_US*bubblegum)); block_epc_i2c(4);

	dcmi_start_dma(&dcsa, SIZEOF_4DCS);
	epc_trig();


	int supershort_avg_ampl_x256 = calc_avg_ampl_x256(wid_hdr20[0], wid_hdr31[0]);

	IFDBG
	{
//		DBG_PR_VAR_I32(supershort_avg_ampl_x256);
//		DBG_PR_VAR_I32(base_exp);
	}


	if(poll_capt_with_timeout_complete()) log_err(sidx);

	conv_4dcs_to_2dcs(wid_hdr20[1], wid_hdr31[1], &dcsa, &mono_comp);


	TOF_TS(4);


	int ss_saturated = (int)supershort_avg_ampl_x256;
	ss_saturated *= 4;
	if(ss_saturated > 6*256) ss_saturated = 6*256;
	
	int base_exp = SUPERSHORT1_US + sq(6*256 - ss_saturated)/4000;







	TOF_TS(5);


	adjust();



	// HDR test, wide




	int base_exp_nar = base_exp/3;

	epc_clk_div(0); block_epc_i2c(4);
	epc_intlen(intlen_mults[0], INTUS(base_exp*bubblegum)); block_epc_i2c(4);
	epc_4dcs(); block_epc_i2c(4);

	delay_ms(1);
	epc_ena_wide_leds(); dcmi_crop_wide(); block_epc_i2c(4);


	dcmi_start_dma(&dcsa, SIZEOF_4DCS);
	epc_trig();
	adjust();

	// Do the previous calc:
//	compensated_nonhdr_tof_calc_dist_ampl_flarecomp(wid_ampl, wid_dist, wid_hdr20[0], wid_hdr31[0]);
	compensated_hdr_tof_calc_dist_ampl_flarecomp(wid_ampl, wid_dist, wid_hdr20[0], wid_hdr31[0], wid_hdr20[1], wid_hdr31[1]);

//	if(sidx==5)
		tof_to_voxmap(wid_ampl, wid_dist, 0, sidx, 5, 255, vox_ref_x, vox_ref_y);

#if 0
	if(gen_data && tof_raw_dist)
	{
		tof_raw_dist->sensor_idx = sidx;
		tof_raw_dist->sensor_orientation = sensor_mounts[sidx].mount_mode;
		memcpy(tof_raw_dist->dist, wid_dist, sizeof tof_raw_dist->dist);
		#ifdef USE_NARROW
		memcpy(tof_raw_dist->dist_narrow, nar_dist, sizeof tof_raw_dist->dist_narrow);
		#else
		memset(tof_raw_dist->dist_narrow, 0, sizeof tof_raw_dist->dist_narrow);
		#endif
//		tof_raw_dist->wide_stray_estimate_adc = wide_stray;
//		tof_raw_dist->narrow_stray_estimate_adc = narrow_stray;
	}

	if(gen_data && tof_raw_ampl8)
	{
		tof_raw_ampl8->sensor_idx = sidx;
		memcpy(tof_raw_ampl8->ampl, wid_ampl, sizeof tof_raw_ampl8->ampl);
		#ifdef USE_NARROW
		memcpy(tof_raw_ampl8->ampl_narrow, nar_ampl, sizeof tof_raw_ampl8->ampl_narrow);
		#else
		memset(tof_raw_ampl8->ampl_narrow, 0, sizeof tof_raw_ampl8->ampl_narrow);
		#endif
	}
#endif

	if(poll_capt_with_timeout_complete()) log_err(sidx);

	TOF_TS(6);


	conv_4dcs_to_2dcs(wid_hdr20[0], wid_hdr31[0], &dcsa, NULL);

	TOF_TS(7);

	epc_intlen(intlen_mults[0], INTUS(base_exp*HDR_FACTOR*bubblegum)); block_epc_i2c(4);

	dcmi_start_dma(&dcsa, SIZEOF_4DCS);
	epc_trig();
	adjust();

	if(poll_capt_with_timeout_complete()) log_err(sidx);

	TOF_TS(8);

	conv_4dcs_to_2dcs(wid_hdr20[1], wid_hdr31[1], &dcsa, NULL);

	TOF_TS(9);

	compensated_hdr_tof_calc_dist_ampl_flarecomp(wid_ampl, wid_dist, wid_hdr20[0], wid_hdr31[0], wid_hdr20[1], wid_hdr31[1]);

	TOF_TS(10);


#ifdef USE_NARROW

	// HDR test, narrow

	delay_ms(1);
	epc_ena_narrow_leds(); dcmi_crop_narrow(); block_epc_i2c(4);

	epc_intlen(intlen_mults[0], INTUS(base_exp_nar)); block_epc_i2c(4);

	dcmi_start_dma(&dcsa_narrow, SIZEOF_4DCS_NARROW);
	epc_trig();
	adjust();

	if(poll_capt_with_timeout_complete()) log_err(sidx);

	TOF_TS(11);


	conv_4dcs_to_2dcs_narrow(nar_hdr20[0], nar_hdr31[0], &dcsa_narrow, NULL);

	TOF_TS(12);

	epc_intlen(intlen_mults[0], INTUS(base_exp_nar*HDR_FACTOR)); block_epc_i2c(4);

	dcmi_start_dma(&dcsa_narrow, SIZEOF_4DCS_NARROW);
	epc_trig();
	adjust();

	if(poll_capt_with_timeout_complete()) log_err(sidx);

	TOF_TS(13);

	conv_4dcs_to_2dcs_narrow(nar_hdr20[1], nar_hdr31[1], &dcsa_narrow, NULL);

	TOF_TS(14);

	compensated_hdr_tof_calc_dist_ampl_flarecomp_narrow(nar_ampl, nar_dist, nar_hdr20[0], nar_hdr31[0], nar_hdr20[1], nar_hdr31[1]);

	TOF_TS(15);

#endif

	adjust();


	dealias_20mhz(wid_dist, lofreq_wid_dist);

#ifdef USE_NARROW
	dealias_20mhz_narrow(nar_dist, lofreq_wid_dist);
#endif

	TOF_TS(16);


/*
	if(test_cnt == 2)
	{
		if(gen_data && tof_raw_dist)
		{
			tof_raw_dist->sensor_idx = test_cnt;
			tof_raw_dist->sensor_orientation = sensor_mounts[sidx].mount_mode;
			memcpy(tof_raw_dist->dist, wid_dist, sizeof tof_raw_dist->dist);
			memcpy(tof_raw_dist->dist_narrow, nar_dist, sizeof tof_raw_dist->dist_narrow);
		}

		if(gen_data && tof_raw_ampl8)
		{
			tof_raw_ampl8->sensor_idx = test_cnt;
			memcpy(tof_raw_ampl8->ampl, wid_ampl, sizeof tof_raw_ampl8->ampl);
			memcpy(tof_raw_ampl8->ampl_narrow, nar_ampl, sizeof tof_raw_ampl8->ampl_narrow);
		}
	}
*/


	int32_t widnar_corr = 0;

#ifdef USE_NARROW

	int widnar_ret;
	widnar_ret = calc_widnar_correction(&widnar_corr, wid_ampl, wid_dist, nar_ampl, nar_dist);

	if(widnar_ret > 20)
	{
		if(widnar_ret < 30)
			widnar_corr /= 4;
		else if(widnar_ret < 40)
			widnar_corr /= 3;
		else if(widnar_ret < 50)
			widnar_corr /= 2;
		else if(widnar_ret < 60)
			widnar_corr = (widnar_corr*3)/4;
	}

	IFDBG
	{
		DBG_PR_VAR_I32(widnar_ret);
		DBG_PR_VAR_I32(widnar_corr);
	}

#endif


//	if(widnar_corr < -100 || widnar_corr > 100)
//	{
//		DBG_PR_VAR_I32(widnar_ret);
//		DBG_PR_VAR_I32(widnar_corr);
//	}

	TOF_TS(17);


	adjust();

//	if(sidx == 5)
		tof_to_voxmap(wid_ampl, wid_dist, widnar_corr, sidx, 3, 254, vox_ref_x, vox_ref_y);

	TOF_TS(18);


#if 1
	if(gen_data && tof_raw_dist)
	{
		tof_raw_dist->sensor_idx = sidx;
		tof_raw_dist->sensor_orientation = sensor_mounts[sidx].mount_mode;
		memcpy(tof_raw_dist->dist, wid_dist, sizeof tof_raw_dist->dist);
		#ifdef USE_NARROW
		memcpy(tof_raw_dist->dist_narrow, nar_dist, sizeof tof_raw_dist->dist_narrow);
		#else
		memset(tof_raw_dist->dist_narrow, 0, sizeof tof_raw_dist->dist_narrow);
		#endif
//		tof_raw_dist->wide_stray_estimate_adc = wide_stray;
//		tof_raw_dist->narrow_stray_estimate_adc = narrow_stray;
	}

	if(gen_data && tof_raw_ampl8)
	{
		tof_raw_ampl8->sensor_idx = sidx;
		memcpy(tof_raw_ampl8->ampl, wid_ampl, sizeof tof_raw_ampl8->ampl);
		#ifdef USE_NARROW
		memcpy(tof_raw_ampl8->ampl_narrow, nar_ampl, sizeof tof_raw_ampl8->ampl_narrow);
		#else
		memset(tof_raw_ampl8->ampl_narrow, 0, sizeof tof_raw_ampl8->ampl_narrow);
		#endif
	}
#endif

	if(gen_data && tof_diagnostics)
	{
		tof_diagnostics->temperature = chiptemp;
	}

	SKIP:;

	adjust();

	int do_restore_subs;
	if(voxmap_send_cnt >= VOXMAP_SEND_INTERVAL)
	{
		// Subscription is forced above.
		if(!mcu_multi_voxel_map)
		{
			error(155); // multi_voxel_map must fit by design; if it doesn't, make it smaller, or increase TX FIFO max size.
		}
		if(!is_tx_overrun()) // could look at gen_data, but let's recheck in case there is room now even if there wasn't before.
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
				voxmap_send_cnt = 0;
				do_restore_subs = 1;
			}
		}
		else
		{
			uart_print_string_blocking("\r\nVOXMAP: TX buffer overrun! Skipping data generation.\r\n"); 

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

//	static int test_cnt = 0;

//	DBG_PR_VAR_U16(test_cnt);
//	tof_calibrator_ambient_lvl(test_cnt);

//	test_cnt++;
//	if(test_cnt >= 16) test_cnt = 0;

	for(int i=0; i<10; i++)
	{
		check_rx();
//		delay_ms(10);
		adjust();
	}	
/*	
	if(sidx==9)
	{
		extern void bldc_print_debug();
		bldc_print_debug();
	}
*/
	//profile_cpu_blocking_20ms();

	err_cnt -= 1;

}
