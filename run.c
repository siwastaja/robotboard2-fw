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
epc_4dcs_t dcsb __attribute__((aligned(4)));
epc_4dcs_narrow_t dcsa_narrow __attribute__((aligned(4)));
epc_4dcs_narrow_t dcsb_narrow __attribute__((aligned(4)));

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


typedef struct
{
	int mount_mode;             // mount position 1,2,3 or 4
	float x_rel_robot;          // zero = robot origin. Positive = robot front (forward)
	float y_rel_robot;          // zero = robot origin. Positive = to the right of the robot
	float ang_rel_robot;        // zero = robot forward direction. positive = ccw
	float vert_ang_rel_ground;  // zero = looks directly forward. positive = looks up. negative = looks down
	float z_rel_ground;         // sensor height from the ground	
} sensor_mount_t;

//#define M_PI 3.141592653
#define RADTODEG(x) ((x)*(360.0/(2.0*M_PI)))
#define DEGTORAD(x) ((x)*((2.0*M_PI)/360.0))

static const sensor_mount_t sensor_mounts[N_SENSORS] =
{          //      mountmode    x     y       hor ang           ver ang      height    
 /*0:                */ { 0,     0,     0, DEGTORAD(       0), DEGTORAD(  2),   0 },
 /*1:                */ { 1,   160,   140, DEGTORAD(      23), DEGTORAD(  2), 320 },
 /*2:                */ { 2,  -200,   230, DEGTORAD(   90-23), DEGTORAD(  2), 320 },
 /*3:                */ { 1,  -410,   230, DEGTORAD(      90), DEGTORAD(  2), 320 },
 /*4:                */ { 2,  -490,   140, DEGTORAD(  180-23), DEGTORAD(  2), 320 },
 /*5:                */ { 2,  -490,     0, DEGTORAD(    180 ), DEGTORAD(  2), 320 },
 /*6:                */ { 1,  -490,  -140, DEGTORAD(  180+23), DEGTORAD(  2), 320 },
 /*7:                */ { 2,  -410,  -230, DEGTORAD(   270  ), DEGTORAD(  2), 320 },
 /*8:                */ { 1,  -200,  -230, DEGTORAD(  270+23), DEGTORAD(  2), 320 },
 /*9:                */ { 2,   160,  -140, DEGTORAD(  360-23), DEGTORAD(  2), 320 }
};


static int lowbat_die_cnt = 0;
static int cycle;
void run_cycle()
{
	int gen_data = 0;
	cycle++;
	if(cycle == 4*10)
	{
		cycle = 0;
	}

	if(cycle <= 9)
		gen_data = 1;

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

	static int sidx = 0;
	sidx++;
	if(sidx >= N_SENSORS)
		sidx = 0;
	if(!sensors_in_use[sidx])
		return;

//		goto SKIP_TOF;


	const int intlen_mults[4] = {12, 6, 4, 3}; // with these, unit is always 0.6us.

	tof_mux_select(sidx);


//	epc_enable_dll();  block_epc_i2c(4);
//	epc_coarse_dll_steps(0); block_epc_i2c(4);
//	epc_pll_steps(0); block_epc_i2c(4);

	INIT_TOF_TS();

	rgb_update(1, 128, 128, 128);

	// Acquire compensation B/W with fixed intlen - temperature at the same time
	dcmi_crop_wide();
	epc_greyscale(); block_epc_i2c(4);
	epc_dis_leds(); block_epc_i2c(4);
	epc_clk_div(1); block_epc_i2c(4);
	epc_intlen(intlen_mults[1], INTUS(COMP_AMBIENT_INTLEN_US)); block_epc_i2c(4);
	epc_temperature_magic_mode(sidx);
	dcmi_start_dma(&mono_comp, SIZEOF_MONO);
	epc_trig();
	if(poll_capt_with_timeout_complete()) error(101);
	TOF_TS(0);

	int16_t chiptemp = epc_read_temperature(sidx);
	epc_temperature_magic_mode_off(sidx);


	int fine_steps = ((tof_calibs[sidx]->zerofine_temp-chiptemp)*tof_calibs[sidx]->fine_steps_per_temp[0])>>8;
	if(fine_steps < 0) fine_steps = 0;
	else if(fine_steps > 799) fine_steps = 799;
	//DBG_PR_VAR_U32(fine_steps);
	epc_fine_dll_steps(fine_steps); block_epc_i2c(4);

	rgb_update(1, 64, 64, 64);




	static uint8_t  wid_ampl[2][TOF_XS*TOF_YS];
	static uint16_t wid_dist[2][TOF_XS*TOF_YS];
	static uint8_t  nar_ampl[2][TOF_XS_NARROW*TOF_YS_NARROW];
	static uint16_t nar_dist[2][TOF_XS_NARROW*TOF_YS_NARROW];


	int wid_raw_send = 0;
	int nar_raw_send = 0;

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
	epc_intlen(intlen_mults[0], INTUS(100)); block_epc_i2c(4);

	if(poll_capt_with_timeout_complete()) error(102);

	rgb_update(0, 0, 0, 0);


	// QUITE SHORT WIDE

	dcmi_start_dma(&dcsb, SIZEOF_4DCS);
	epc_trig();
	delay_us(350);
	uint16_t wide_stray = adc3.s.epc_stray_estimate;

	uint8_t wid_ampl_avg[2] = {0};
	uint8_t nar_ampl_avg[2] = {0};

	compensated_tof_calc_dist_ampl(&wid_ampl_avg[0], wid_ampl[0], wid_dist[0], &dcsa, &mono_comp);

	if(poll_capt_with_timeout_complete()) error(103);

	if(wid_ampl_avg[0] > 22)
	{
		rgb_update(4, 255, 0, 0);
		delay_ms(30);
		rgb_update(0, 0, 0, 0);
		wid_raw_send = 0;
		goto SKIP;
	}


	// QUITE SHORT NARROW
	delay_ms(1);
	epc_ena_narrow_leds(); dcmi_crop_narrow(); block_epc_i2c(4);
	epc_intlen(intlen_mults[0], INTUS(100)); block_epc_i2c(4);

	dcmi_start_dma(&dcsa_narrow, SIZEOF_4DCS_NARROW);
	epc_trig();
	delay_us(350);
	uint16_t narrow_stray = adc3.s.epc_stray_estimate;

	compensated_tof_calc_dist_ampl(&wid_ampl_avg[1], wid_ampl[1], wid_dist[1], &dcsb, &mono_comp);



	if(poll_capt_with_timeout_complete()) error(104);


	if(wid_ampl_avg[1] > 40)
	{
		rgb_update(4, 150, 80, 0);
		delay_ms(20);
		rgb_update(0, 0, 0, 0);
		wid_raw_send = 1;

		goto SKIP;
	}


	// Actual wide
	// Previous inttime: 200 us
	// avg	new
	// pre
	// 40	200
	//  0	4000
	int inttime_us = 4000.0 -   sqrt(sqrt((double)wid_ampl_avg[1])) * 15.9*(4000.0-200.0)/40.0;
	if(sidx==1)
		DBG_PR_VAR_I32(inttime_us);

	epc_intlen(intlen_mults[0], INTUS(inttime_us)); block_epc_i2c(4);
	epc_4dcs(); block_epc_i2c(4);
	delay_ms(1);
	epc_ena_wide_leds(); dcmi_crop_wide(); block_epc_i2c(4);

	dcmi_start_dma(&dcsa, SIZEOF_4DCS);
	epc_trig();

	compensated_tof_calc_dist_ampl_narrow(NULL, nar_ampl[0], nar_dist[0], &dcsa_narrow, &mono_comp);

	nar_raw_send = 0;

	if(poll_capt_with_timeout_complete()) error(105);

	// reuse 0
	compensated_tof_calc_dist_ampl(&wid_ampl_avg[0], wid_ampl[0], wid_dist[0], &dcsa, &mono_comp);

	wid_raw_send = 0;

	SKIP:


	if(gen_data && tof_raw_dist)
	{
		tof_raw_dist->sensor_idx = sidx;
		tof_raw_dist->sensor_orientation = sensor_mounts[sidx].mount_mode;
		memcpy(tof_raw_dist->dist, wid_dist[wid_raw_send], sizeof tof_raw_dist->dist);
		memcpy(tof_raw_dist->dist_narrow, nar_dist[nar_raw_send], sizeof tof_raw_dist->dist_narrow);
		memcpy(tof_raw_dist->dist, wid_dist[0], sizeof tof_raw_dist->dist);
		memcpy(tof_raw_dist->dist_narrow, nar_dist[0], sizeof tof_raw_dist->dist_narrow);
		tof_raw_dist->wide_stray_estimate_adc = wide_stray;
		tof_raw_dist->narrow_stray_estimate_adc = narrow_stray;
	}

	if(gen_data && tof_raw_ampl8)
	{
		tof_raw_ampl8->sensor_idx = sidx;
		memcpy(tof_raw_ampl8->ampl, wid_ampl[0], sizeof tof_raw_ampl8->ampl);
		memcpy(tof_raw_ampl8->ampl, wid_ampl[wid_raw_send], sizeof tof_raw_ampl8->ampl);
		memcpy(tof_raw_ampl8->ampl_narrow, nar_ampl[0], sizeof tof_raw_ampl8->ampl_narrow);
		memcpy(tof_raw_ampl8->ampl_narrow, nar_ampl[nar_raw_send], sizeof tof_raw_ampl8->ampl_narrow);
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

	for(int i=0; i<5; i++)
	{
		check_rx();
		delay_ms(1);
	}	
	
	//profile_cpu_blocking_20ms();

}
