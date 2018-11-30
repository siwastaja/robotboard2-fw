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
 /*1:                */ { 2,   160,   140, DEGTORAD(      23), DEGTORAD(  2), 320 },
 /*2:                */ { 1,  -200,   230, DEGTORAD(   90-23), DEGTORAD(  2), 320 },
 /*3:                */ { 2,  -410,   230, DEGTORAD(      90), DEGTORAD(  2), 320 },
 /*4:                */ { 1,  -490,   140, DEGTORAD(  180-23), DEGTORAD(  2), 320 },
 /*5:                */ { 2,  -490,     0, DEGTORAD(    180 ), DEGTORAD(  2), 320 },
 /*6:                */ { 2,  -490,  -140, DEGTORAD(  180+23), DEGTORAD(  2), 320 },
 /*7:                */ { 1,  -410,  -230, DEGTORAD(   270  ), DEGTORAD(  2), 320 },
 /*8:                */ { 2,  -200,  -230, DEGTORAD(  270+23), DEGTORAD(  2), 320 },
 /*9:                */ { 1,   160,  -140, DEGTORAD(  360-23), DEGTORAD(  2), 320 }
};

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

#define IFDBG if(sidx == 7)

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

//	if(cycle <= 9)
	gen_data = 1;

	if(is_tx_overrun())
	{
//		uart_print_string_blocking("\r\nTX buffer overrun! Skipping data generation.\r\n"); 
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
	TOF_TS(0);

	int16_t chiptemp = epc_read_temperature(sidx);
	epc_temperature_magic_mode_off(sidx);


	int fine_steps = ((tof_calibs[sidx]->zerofine_temp-chiptemp)*tof_calibs[sidx]->fine_steps_per_temp[0])>>8;
	if(fine_steps < 0) fine_steps = 0;
	else if(fine_steps > 799) fine_steps = 799;
	//DBG_PR_VAR_U32(fine_steps);
	epc_fine_dll_steps(fine_steps); block_epc_i2c(4);



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
	epc_intlen(intlen_mults[0], INTUS(150)); block_epc_i2c(4);

	if(poll_capt_with_timeout_complete()) log_err();



	// QUITE SHORT WIDE

	dcmi_start_dma(&dcsb, SIZEOF_4DCS);
	epc_trig();
	uint16_t wide_stray = measure_stray(); // blocks for 500us

	uint8_t wid_ampl_max[2] = {0};
	uint8_t nar_ampl_max[2] = {0};

	compensated_tof_calc_dist_ampl(&wid_ampl_max[0], wid_ampl[0], wid_dist[0], &dcsa, &mono_comp);

	if(poll_capt_with_timeout_complete()) log_err();

	if(wid_ampl_max[0] == 255)
	{
		delay_ms(20);
		wid_raw_send = 0;
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
	compensated_tof_calc_dist_ampl(&wid_ampl_max[1], wid_ampl[1], wid_dist[1], &dcsb, &mono_comp);
	uint16_t inimage_stray_ampl_est;
	uint16_t inimage_stray_dist_est;
	calc_stray_estimate(wid_ampl[1], wid_dist[1], &inimage_stray_ampl_est, &inimage_stray_dist_est);

	IFDBG
	{
		DBG_PR_VAR_U16(inimage_stray_ampl_est);
		DBG_PR_VAR_U16(inimage_stray_dist_est);
		DBG_PR_VAR_U16(wide_stray);
		DBG_PR_VAR_U16(narrow_stray);
	}

	if(poll_capt_with_timeout_complete()) log_err();

	wid_raw_send = 1;

	if(wid_ampl_max[1] == 255)
	{
		delay_ms(15);
		wid_raw_send = 1;

		goto SKIP;
	}


	// Actual wide
	// Previous inttime: 150 us
	// Aim for max amplitude of 150

//	int inttime_us = 4000.0 -   sqrt(sqrt((double)wid_ampl_avg[1])) * 15.9*(4000.0-200.0)/40.0;
//	int inttime_us = 3000.0 -   sqrt(sqrt((double)wid_ampl_avg[1])) * 15.9*(4000.0-200.0)/40.0;
	int inttime_us = 150.0/(double)wid_ampl_max[1] * 150.0;
	if(inttime_us < 150) inttime_us = 150;
	else if(inttime_us > 4000) inttime_us = 4000;
//	int inttime_us = 400;

	IFDBG
	{
		DBG_PR_VAR_I32(wid_ampl_max[1]);
		DBG_PR_VAR_I32(inttime_us);
	}


	// AUTOEXPOSED MIDLONG WIDE

	epc_intlen(intlen_mults[0], INTUS(inttime_us)); block_epc_i2c(4);
	epc_4dcs(); block_epc_i2c(4);
	delay_ms(1);
	epc_ena_wide_leds(); dcmi_crop_wide(); block_epc_i2c(4);

	dcmi_start_dma(&dcsa, SIZEOF_4DCS);
	epc_trig();

	compensated_tof_calc_dist_ampl_narrow(NULL, nar_ampl[0], nar_dist[0], &dcsa_narrow, &mono_comp);

	nar_raw_send = 0;

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
	compensated_tof_calc_dist_ampl(&wid_ampl_max[0], wid_ampl[0], wid_dist[0], &dcsa, &mono_comp);
	wid_raw_send = 0;


	if(poll_capt_with_timeout_complete()) log_err();

	compensated_tof_calc_dist_ampl_narrow(NULL, nar_ampl[1], nar_dist[1], &dcsa_narrow, &mono_comp);
	nar_raw_send = 1;

	int32_t widnar_corr = 0;
	int widnar_ret;
	widnar_ret = calc_widnar_correction(&widnar_corr, wid_ampl[0], wid_dist[0], nar_ampl[1], nar_dist[1]);

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

//		for(int i=0; i<TOF_XS*TOF_YS; i++)
//		{
//			wid_dist[0][i] += widnar_corr;
//		}
	}


	IFDBG
	{
		DBG_PR_VAR_I32(widnar_ret);
		DBG_PR_VAR_I32(widnar_corr);
	}

	//if(test_msg3)
	{
		//memset(test_msg3->buf, 0, 200*200);
		memset(wid_dist[1], 0, sizeof wid_dist[1]);
		memcpy(wid_ampl[1], wid_ampl[0], sizeof wid_ampl[1]);
		uint8_t ampl_accept = 4;
		for(int yy=1; yy<TOF_YS-1; yy++)
		{
			for(int xx=1; xx<TOF_XS-1; xx++)
			{
				int32_t dists[5];

				dists[0] = wid_dist[0][(yy+0)*TOF_XS+(xx+0)] + widnar_corr;
				dists[1] = wid_dist[0][(yy-1)*TOF_XS+(xx+0)] + widnar_corr;
				dists[2] = wid_dist[0][(yy+1)*TOF_XS+(xx+0)] + widnar_corr;
				dists[3] = wid_dist[0][(yy+0)*TOF_XS+(xx+1)] + widnar_corr;
				dists[4] = wid_dist[0][(yy+0)*TOF_XS+(xx-1)] + widnar_corr;

				uint8_t ampls[5];
				ampls[0] = wid_ampl[0][(yy+0)*TOF_XS+(xx+0)];
				ampls[1] = wid_ampl[0][(yy-1)*TOF_XS+(xx+0)];
				ampls[2] = wid_ampl[0][(yy+1)*TOF_XS+(xx+0)];
				ampls[3] = wid_ampl[0][(yy+0)*TOF_XS+(xx+1)];
				ampls[4] = wid_ampl[0][(yy+0)*TOF_XS+(xx-1)];

				int32_t avg = (dists[0]+dists[1]+dists[2]+dists[3]+dists[4])/5;

				int n_conform = 0;
				int32_t conform_avg = 0;
				for(int i=0; i<5; i++)
				{
					if(ampls[i] < 255 && ampls[i] > ampl_accept && dists[i] > avg-100 && dists[i] < avg+100)
					{
						n_conform++;
						conform_avg += dists[i];
					}
				}

				if(n_conform >= 3)
				{
					conform_avg /= n_conform;
				}
				else
					conform_avg = 65534;

				wid_dist[1][yy*TOF_XS+xx] = conform_avg;
			}
		}
		wid_raw_send = 1;

	}

	SKIP:


	if(gen_data && tof_raw_dist)
	{
		tof_raw_dist->sensor_idx = sidx;
		tof_raw_dist->sensor_orientation = sensor_mounts[sidx].mount_mode;
		memcpy(tof_raw_dist->dist, wid_dist[wid_raw_send], sizeof tof_raw_dist->dist);
		memcpy(tof_raw_dist->dist_narrow, nar_dist[nar_raw_send], sizeof tof_raw_dist->dist_narrow);
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
//		uart_print_string_blocking("\r\nPush!\r\n"); 
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

	err_cnt -= 1;

}
