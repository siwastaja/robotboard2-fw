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
	//tof_idx = 9;


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

	static int tof_idx = 0;
	tof_idx++;
	if(tof_idx >= N_SENSORS)
		tof_idx = 0;
	if(!sensors_in_use[tof_idx])
		goto SKIP_TOF;


	/*
	Integration lengths:
	If the multiplier is set to led freq in MHz, the time parameter is us directly (which looks nice and intuitive).

	4 ms per DCS will be the longest integration ever used - 16ms total 4DCS.

	clk_div	freq	unamb	mult	unit	max
	1	20MHz	7.5m	10	0.5us	8192us
	2	10MHz	15m	5	0.5us	8192us
	3	6.667M	22.5m	3	0.45us	7372.8us
	4	5MHz	30m	3	0.6us	9830.4us
	5	4MHz	37.5m	2	0.5us	8192us
	6	3.333M	45m	2	0.6us	9830.4us
	
	*/

#define MULT_20M  10
#define MULT_10M  5
#define MULT_6M6  3
#define MULT_5M   3
#define MULT_4M   2
#define MULT_3M3  2

	tof_mux_select(tof_idx);


	/*
		TOF heat-up test:
		Frame rate = 5.2FPS
		Integration time = 4000us per DCS
		10 MHz
		Both wide and narrow image, both at the same 4000us int time
		Standard heatsink attached on back. No lens hood plastic thing installed.

		Ambient temp = 20 degC

		After running 15 minutes:
		EPC chip reported: 42
		Black heatsinks nearest the chip: 37
		Black heatsinks nearest the LEDs: 39
		PCB frontside directly over LEDs: 48
		PCB frontside between the wide and narrow LEDs (at FET gate driver): 40

		After running 32 minutes:
		EPC chip reported: 44
		Black heatsinks nearest the chip: 41
		Black heatsinks nearest the LEDs: 42
		PCB frontside directly over LEDs: 50
		PCB frontside between the wide and narrow LEDs (at FET gate driver): 42


		Note: 4000us int.time is quite extreme. In normal conditions (where sunlight rejection, and not having motion blur artefacts)
		matters, the int.time is limited to around 2000us, which means halved power dissipation!



	*/

	// 150us at 20 MHz
	// 120us at 10 MHz
	// 100us at 5 MHz


	rgb_update(4, 255, 0, 0);
	delay_ms(100);
	rgb_update(4, 0, 255, 0);
	delay_ms(100);
	rgb_update(4, 0, 0, 255);
	delay_ms(100);
	rgb_update(4, 255, 255, 255);
	delay_ms(100);
	rgb_update(0, 0, 0, 0);

	INIT_TOF_TS();
	dcmi_crop_wide();
	epc_greyscale(); while(epc_i2c_is_busy());
	epc_dis_leds(); while(epc_i2c_is_busy());
	epc_clk_div(1); while(epc_i2c_is_busy());

	epc_intlen(MULT_20M, 2*20);

	while(epc_i2c_is_busy());

	epc_temperature_magic_mode(tof_idx);
	dcmi_start_dma(&mono_comp, SIZEOF_MONO);
	epc_trig();
	TOF_TS(0);
	if(poll_capt_with_timeout()) soft_err();
	TOF_TS(1);

	int32_t temperature = epc_read_temperature(tof_idx);

	epc_temperature_magic_mode_off(tof_idx);

	epc_4dcs(); while(epc_i2c_is_busy());
	epc_ena_wide_leds(); dcmi_crop_wide(); while(epc_i2c_is_busy());

	dcmi_start_dma(&dcsa, SIZEOF_4DCS);
	epc_trig();
	TOF_TS(2);
	delay_us(350);
	uint16_t wide_stray = adc3.s.epc_stray_estimate;
	TOF_TS(3);
	if(poll_capt_with_timeout()) soft_err();
	TOF_TS(4);

	epc_ena_narrow_leds(); dcmi_crop_narrow(); while(epc_i2c_is_busy());
	dcmi_start_dma(&dcsa_narrow, SIZEOF_4DCS_NARROW);
	epc_trig();
	TOF_TS(5);
	// 130us delay before capturing stray_estimate is hit-or-miss
	// 155us still occasionally misses it
	// 210us still has a bit noise
	// 300 us is enough.
	delay_us(350);
	uint16_t narrow_stray = adc3.s.epc_stray_estimate;
	TOF_TS(6);
	if(poll_capt_with_timeout()) soft_err();
	TOF_TS(7);

	static uint8_t  ampl[TOF_XS*TOF_YS];
	static uint8_t  ambient[TOF_XS*TOF_YS];
	static uint16_t dist[TOF_XS*TOF_YS];

	static uint8_t  ampl_narrow[TOF_XS_NARROW*TOF_YS_NARROW];
	static uint16_t dist_narrow[TOF_XS_NARROW*TOF_YS_NARROW];

	tof_calc_dist_ampl(ampl, dist, &dcsa, 4000, 1); // 5.0 ms
	TOF_TS(8);
	tof_calc_dist_ampl_narrow(ampl_narrow, dist_narrow, &dcsa_narrow, 4600, 1); // 1.0 ms
	TOF_TS(9);
	process_bw(ambient, &mono_comp); // 0.7ms
	TOF_TS(10);


	if(gen_data && tof_raw_dist)
	{
		tof_raw_dist->sensor_idx = tof_idx;
		memcpy(tof_raw_dist->dist, dist, sizeof tof_raw_dist->dist);
		memcpy(tof_raw_dist->dist_narrow, dist_narrow, sizeof tof_raw_dist->dist_narrow);
		tof_raw_dist->wide_stray_estimate_adc = wide_stray;
		tof_raw_dist->narrow_stray_estimate_adc = narrow_stray;
	}

	if(gen_data && tof_raw_ampl8)
	{
		tof_raw_ampl8->sensor_idx = tof_idx;
		memcpy(tof_raw_ampl8->ampl, ampl, sizeof tof_raw_ampl8->ampl);
		memcpy(tof_raw_ampl8->ampl_narrow, ampl_narrow, sizeof tof_raw_ampl8->ampl_narrow);
	}

	if(gen_data && tof_raw_ambient8)
	{
		tof_raw_ambient8->sensor_idx = tof_idx;
		memcpy(tof_raw_ambient8->ambient, ambient, sizeof tof_raw_ambient8->ambient);
		tof_raw_ambient8->temperature = temperature;
	}

	// 3.2ms memcpy's raw dist, raw dist narrow, ampl8, ampl8 narrow, ambient8.
	TOF_TS(11);

	if(gen_data && tof_diagnostics)
	{
		tof_diagnostics->temperature = temperature;
	}

	SKIP_TOF:;

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

	for(int i=0; i<80; i++)
	{
		check_rx();
		delay_ms(10);
	}	
	
	profile_cpu_blocking_20ms();

}
