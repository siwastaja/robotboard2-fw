#include <stdint.h>
#include <string.h>
#include "sbc_comm.h"

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

static int lowbat_die_cnt = 0;
void run_cycle()
{
	int gen_data = 1;
	if(is_tx_overrun())
	{
		uart_print_string_blocking("\r\nTX buffer overrun! Skipping data generation.\r\n"); 
		gen_data = 0;
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

/*	static int sidx = 0;
	sidx++;
	if(sidx >= N_SENSORS)
		sidx = 0;
	if(!sensors_in_use[sidx])
		return;*/

	static int sidx=0;
//		goto SKIP_TOF;


	const int intlen_mults[4] = {12, 6, 4, 3}; // with these, unit is always 0.6us.

	tof_mux_select(sidx);

	rgb_update(1, 255, 0, 0);
	delay_ms(5);
	rgb_update(0, 0, 0, 0);

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
	if(poll_capt_with_timeout()) soft_err();
	TOF_TS(0);

	int16_t chiptemp = epc_read_temperature(sidx);
	epc_temperature_magic_mode_off(sidx);


	int fine_steps = ((tof_calibs[sidx]->zerofine_temp-chiptemp)*tof_calibs[sidx]->fine_steps_per_temp[0])>>8;
	if(fine_steps < 0) fine_steps = 0;
	else if(fine_steps > 799) fine_steps = 799;
	DBG_PR_VAR_U32(fine_steps);
	epc_fine_dll_steps(fine_steps); block_epc_i2c(4);


	// Acquire the wide image

	epc_4dcs(); block_epc_i2c(4);
	epc_ena_wide_leds(); dcmi_crop_wide(); block_epc_i2c(4);
	epc_clk_div(0); block_epc_i2c(4);
	epc_intlen(intlen_mults[1], INTUS(300)); block_epc_i2c(4);

	dcmi_start_dma(&dcsa, SIZEOF_4DCS);
	epc_trig();
	delay_us(350);
	uint16_t wide_stray = adc3.s.epc_stray_estimate;


	if(poll_capt_with_timeout()) soft_err();

	TOF_TS(1);

	// Acquire the narrow image

	epc_ena_narrow_leds(); dcmi_crop_narrow(); block_epc_i2c(4);
	epc_intlen(intlen_mults[0], INTUS(150)); block_epc_i2c(4);

	dcmi_start_dma(&dcsa_narrow, SIZEOF_4DCS_NARROW);
	epc_trig();
	delay_us(350);
	uint16_t narrow_stray = adc3.s.epc_stray_estimate;


	if(poll_capt_with_timeout()) soft_err();



	static uint8_t  old_ampl[TOF_XS*TOF_YS];
	static uint8_t  new_ampl[TOF_XS*TOF_YS];
//	static uint8_t  ambient[TOF_XS*TOF_YS];
	static uint16_t old_dist[TOF_XS*TOF_YS];
	static uint16_t new_dist[TOF_XS*TOF_YS];

	static uint8_t  old_ampl_narrow[TOF_XS_NARROW*TOF_YS_NARROW];
	static uint16_t old_dist_narrow[TOF_XS_NARROW*TOF_YS_NARROW];
	static uint8_t  new_ampl_narrow[TOF_XS_NARROW*TOF_YS_NARROW];
	static uint16_t new_dist_narrow[TOF_XS_NARROW*TOF_YS_NARROW];


	TOF_TS(2);

	DBG_PR_VAR_U32_HEX(tof_calibs[sidx]->magic);
	DBG_PR_VAR_U32_HEX(tof_calibs[sidx]->chip_id);

	copy_cal_to_shadow(sidx, 0);

	TOF_TS(3);
	compensated_tof_calc_dist_ampl(new_ampl, new_dist, &dcsa, &mono_comp);
	TOF_TS(4);
	tof_calc_dist_ampl(old_ampl, old_dist, &dcsa, 2000, 1); // 5.0 ms
	TOF_TS(5);
	tof_calc_dist_ampl_narrow(old_ampl_narrow, old_dist_narrow, &dcsa_narrow, 2600, 1); // 1.0 ms


	static int hommel;
	hommel++;
	if(gen_data && tof_raw_dist)
	{
		if(hommel&1)
		{
			tof_raw_dist->sensor_idx = sidx+1;
			memcpy(tof_raw_dist->dist, old_dist, sizeof tof_raw_dist->dist);
			memcpy(tof_raw_dist->dist_narrow, old_dist_narrow, sizeof tof_raw_dist->dist_narrow);
			tof_raw_dist->wide_stray_estimate_adc = wide_stray;
			tof_raw_dist->narrow_stray_estimate_adc = narrow_stray;
		}
		else
		{
			tof_raw_dist->sensor_idx = sidx;
			memcpy(tof_raw_dist->dist, new_dist, sizeof tof_raw_dist->dist);
			memcpy(tof_raw_dist->dist_narrow, new_dist_narrow, sizeof tof_raw_dist->dist_narrow);
			tof_raw_dist->wide_stray_estimate_adc = wide_stray;
			tof_raw_dist->narrow_stray_estimate_adc = narrow_stray;
		}
	}

	if(gen_data && tof_raw_ampl8)
	{
		tof_raw_ampl8->sensor_idx = sidx;
//		memcpy(tof_raw_ampl8->ampl, new_ampl, sizeof tof_raw_ampl8->ampl);
//		memcpy(tof_raw_ampl8->ampl_narrow, new_ampl_narrow, sizeof tof_raw_ampl8->ampl_narrow);
	}

	if(gen_data && tof_raw_ambient8)
	{
		tof_raw_ambient8->sensor_idx = sidx;
//		memcpy(tof_raw_ambient8->ambient, ambient, sizeof tof_raw_ambient8->ambient);
		tof_raw_ambient8->temperature = chiptemp;
	}

	// 3.2ms memcpy's raw dist, raw dist narrow, ampl8, ampl8 narrow, ambient8.

	if(gen_data && tof_diagnostics)
	{
		tof_diagnostics->temperature = chiptemp;
	}

//	SKIP_TOF:;

	if(gen_data)
	{
		uart_print_string_blocking("\r\nPush!\r\n"); 
		tx_fifo_push();
	}

//	static int test_cnt = 0;

//	DBG_PR_VAR_U16(test_cnt);
//	tof_calibrator_ambient_lvl(test_cnt);

//	test_cnt++;
//	if(test_cnt >= 16) test_cnt = 0;

	for(int i=0; i<100; i++)
	{
		check_rx();
		delay_ms(10);
	}	
	
	profile_cpu_blocking_20ms();

}
