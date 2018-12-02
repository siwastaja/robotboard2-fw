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

static char printbuf[128];

uint8_t conv_bat_percent(int mv)
{
	int bat_percentage = (100*(mv-(3200*6)))/((4200-3200)*6);
	if(bat_percentage < 0) bat_percentage = 0;
	if(bat_percentage > 127) bat_percentage = 127;
	return bat_percentage;

}

epc_img_t mono_comp __attribute__((aligned(4)));
epc_4dcs_t dcsa __attribute__((aligned(4)));
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
static void log_err()
{
	err_cnt += 10;
	if(err_cnt > 30)
	{
		error(101);
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

#define IFDBG if(sidx == 999)


static int round_of_longer_exposure;

uint32_t vox_cnt;
int32_t vox_ref_x = 0;
int32_t vox_ref_y = 0;

void restart_voxmap()
{
	memset(&voxmap, 0, sizeof(voxmap));
	vox_cnt++;
	vox_ref_x = cur_pos.x>>16;
	vox_ref_y = cur_pos.y>>16;
}

#define VOXMAP_SEND_INTERVAL 3

static int lowbat_die_cnt = 0;
void run_cycle()  __attribute__((section(".text_itcm")));
void run_cycle()
{
	static int sidx = 0;
	sidx++;
	if(sidx >= N_SENSORS)
	{
		sidx = 0;
		round_of_longer_exposure = ~round_of_longer_exposure;
	}
	if(!sensors_in_use[sidx])
		return;


	int gen_data = 0;

	gen_data = 1;

	if(is_tx_overrun())
	{
		uart_print_string_blocking("\r\nTX buffer overrun! Skipping data generation.\r\n"); 
		gen_data = 0;
	}

	static int voxmap_send_cnt;

	if(sidx == 9 && round_of_longer_exposure)
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


//	epc_enable_dll();  block_epc_i2c(4);
//	epc_coarse_dll_steps(0); block_epc_i2c(4);
//	epc_pll_steps(0); block_epc_i2c(4);

	INIT_TOF_TS();

	// Acquire compensation B/W with fixed intlen - temperature at the same time
	dcmi_crop_wide();
	epc_greyscale(); block_epc_i2c(4);
	epc_dis_leds(); block_epc_i2c(4);
	epc_clk_div(1); block_epc_i2c(4);
	epc_intlen(intlen_mults[1], INTUS(COMP_AMBIENT_INTLEN_US)); block_epc_i2c(4);
	epc_temperature_magic_mode(sidx);
	dcmi_start_dma(&mono_comp, SIZEOF_MONO);
	epc_trig();
	if(poll_capt_with_timeout_complete()) log_err();

	int16_t chiptemp = epc_read_temperature(sidx);
	epc_temperature_magic_mode_off(sidx);


	int fine_steps = ((tof_calibs[sidx]->zerofine_temp-chiptemp)*tof_calibs[sidx]->fine_steps_per_temp[0])>>8;
	if(fine_steps < 0) fine_steps = 0;
	else if(fine_steps > 799) fine_steps = 799;
	//DBG_PR_VAR_U32(fine_steps);
	epc_fine_dll_steps(fine_steps); block_epc_i2c(4);



	static uint8_t  wid_ampl[TOF_XS*TOF_YS];
	static uint16_t wid_dist[TOF_XS*TOF_YS];
	static uint8_t  nar_ampl[TOF_XS_NARROW*TOF_YS_NARROW];
	static uint16_t nar_dist[TOF_XS_NARROW*TOF_YS_NARROW];


	// SUPER SHORT WIDE

	epc_4dcs(); block_epc_i2c(4);
	delay_ms(1);
	epc_ena_wide_leds(); dcmi_crop_wide(); block_epc_i2c(4);
	epc_clk_div(0); block_epc_i2c(4);
	epc_intlen(intlen_mults[0], INTUS(25)); block_epc_i2c(4);

	dcmi_start_dma(&dcsa, SIZEOF_4DCS);
	epc_trig();

	copy_cal_to_shadow(sidx, 0);

	// Next will be longer:
	epc_intlen(intlen_mults[0], INTUS(150)); block_epc_i2c(4);

	if(poll_capt_with_timeout_complete()) log_err();

	uint8_t wid_ampl_max = 0;
	uint8_t nar_ampl_max = 0;

	compensated_tof_calc_dist_ampl(&wid_ampl_max, wid_ampl, wid_dist, &dcsa, &mono_comp);

	// QUITE SHORT WIDE

	dcmi_start_dma(&dcsa, SIZEOF_4DCS);
	epc_trig();
	uint16_t wide_stray = measure_stray(); // blocks for 500us



	if(poll_capt_with_timeout_complete()) log_err();

	if(wid_ampl_max == 255)
	{
		delay_ms(20);
		goto SKIP;
	}


	// QUITE SHORT NARROW
	delay_ms(1);
	epc_ena_narrow_leds(); dcmi_crop_narrow(); block_epc_i2c(4);
	epc_intlen(intlen_mults[0], INTUS(50)); block_epc_i2c(4);

	dcmi_start_dma(&dcsa_narrow, SIZEOF_4DCS_NARROW);
	rgb_update(0); // red led slightly distorts the stray meas
	epc_trig();
	uint16_t narrow_stray = measure_stray(); // blocks for 500us
	update_led(sidx);

	// calc quite short wide:
	compensated_tof_calc_dist_ampl(&wid_ampl_max, wid_ampl, wid_dist, &dcsa, &mono_comp);
	uint16_t inimage_stray_ampl_est;
	uint16_t inimage_stray_dist_est;
	calc_stray_estimate(wid_ampl, wid_dist, &inimage_stray_ampl_est, &inimage_stray_dist_est);
//	tof_to_voxmap(wid_ampl, wid_dist, 0, sidx, 20);

	IFDBG
	{
		DBG_PR_VAR_U16(inimage_stray_ampl_est);
		DBG_PR_VAR_U16(inimage_stray_dist_est);
		DBG_PR_VAR_U16(wide_stray);
		DBG_PR_VAR_U16(narrow_stray);
	}

	if(poll_capt_with_timeout_complete()) log_err();


	if(wid_ampl_max == 255)
	{
		delay_ms(15);

		goto SKIP;
	}


	// Actual wide
	// Previous inttime: 150 us
	// Aim for max amplitude of 150

//	int inttime_us = 4000.0 -   sqrt(sqrt((double)wid_ampl_avg[1])) * 15.9*(4000.0-200.0)/40.0;
//	int inttime_us = 3000.0 -   sqrt(sqrt((double)wid_ampl_avg[1])) * 15.9*(4000.0-200.0)/40.0;
	int inttime_us = 150.0/(double)wid_ampl_max * 150.0;
	if(inttime_us < 150) inttime_us = 150;
	else if(inttime_us > 3000) inttime_us = 3000;
//	int inttime_us = 400;

	if(round_of_longer_exposure)
	{
		inttime_us *= 4;
		if(inttime_us > 8000) inttime_us = 8000;
	}

	IFDBG
	{
		DBG_PR_VAR_I32(wid_ampl_max);
		DBG_PR_VAR_I32(inttime_us);
	}


	// AUTOEXPOSED MIDLONG WIDE

	epc_intlen(intlen_mults[0], INTUS(inttime_us)); block_epc_i2c(4);
	epc_4dcs(); block_epc_i2c(4);
	delay_ms(1);
	epc_ena_wide_leds(); dcmi_crop_wide(); block_epc_i2c(4);

	dcmi_start_dma(&dcsa, SIZEOF_4DCS);
	epc_trig();

	compensated_tof_calc_dist_ampl_narrow(NULL, nar_ampl, nar_dist, &dcsa_narrow, &mono_comp);

	if(poll_capt_with_timeout_complete()) log_err();




	// AUTOEXPOSED MIDLONG NARROW

	delay_ms(1);
	epc_ena_narrow_leds(); dcmi_crop_narrow(); block_epc_i2c(4);
	epc_intlen(intlen_mults[0], INTUS(inttime_us/3)); block_epc_i2c(4);

	dcmi_start_dma(&dcsa_narrow, SIZEOF_4DCS_NARROW);
	epc_trig();
//	delay_us(350);
//	uint16_t narrow_stray = adc3.s.epc_stray_estimate;


	// calc autoexposed midlong wide: reuse 0
	TOF_TS(0);

	compensated_tof_calc_dist_ampl(&wid_ampl_max, wid_ampl, wid_dist, &dcsa, &mono_comp);
	TOF_TS(1);


	if(poll_capt_with_timeout_complete()) log_err();

	compensated_tof_calc_dist_ampl_narrow(NULL, nar_ampl, nar_dist, &dcsa_narrow, &mono_comp);

	int32_t widnar_corr = 0;
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

	TOF_TS(2);
	uint8_t min_ampl;
	uint8_t max_ampl;
	if(round_of_longer_exposure)
	{
		min_ampl = 6;
		max_ampl = 128;
	}
	else
	{
		min_ampl = 12;
		max_ampl = 254;
	}

	tof_to_voxmap(wid_ampl, wid_dist, widnar_corr, sidx, min_ampl, max_ampl, vox_ref_x, vox_ref_y);
	TOF_TS(3);

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

	if(gen_data && tof_diagnostics)
	{
		tof_diagnostics->temperature = chiptemp;
	}


	SKIP:;


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

	for(int i=0; i<5; i++)
	{
		check_rx();
		delay_ms(1);
	}	
	
	//profile_cpu_blocking_20ms();

	err_cnt -= 1;

}
