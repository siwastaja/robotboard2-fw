#include <stdint.h>
#include <string.h>
#include "drive.h"
#include "misc.h"
#include "imu.h"
#include "bldc.h"
#include "own_std.h"
#include "sin_lut.h"
#include "math.h"
#include "../robotsoft/api_board_to_soft.h"
#include "../robotsoft/api_soft_to_board.h"

//#define LEDS_ON

#define G_DC_MOTDET_TH 250 // 100 generates false noise detections about every 2-3 seconds
#define G_AC_MOTDET_TH (150*256) // threshold after DC offset correction

#define GYRO_X_BLANKING_TH (100*256)
#define GYRO_Y_BLANKING_TH (100*256)
#define GYRO_Z_BLANKING_TH (100*256)

#define G_DC_FILT 10 // Quadratic effect, 15 maximum.

static int moving = 1000;
static inline void robot_moves()
{
	moving = 1000;
}

static inline void robot_doesnt_move()
{
	if(moving > 0)
		moving--;
}

static inline int is_robot_moving()
{
	return moving;
}

hires_pos_t cur_pos;

int obstacle_front, obstacle_back, obstacle_right, obstacle_left; // Written to in tof_process.c

int motors_enabled = 0;

int32_t cur_id;
uint32_t micronavi_status;

int new_direction;
int backmode;

static int run;
static int do_start;

int is_driving()
{
	return run;
}

static double max_ang_speed = 12.0; // steps per cycle // was 15.0 -> 10.0
static double min_ang_speed = 2.0;
static double max_lin_speed = 20.0; // was 30.0 -> 15.0
static double min_lin_speed = 5.0;

void set_top_speed_max(int old_style_value)
{
	max_ang_speed = (double)old_style_value / 5.0;
	max_lin_speed = (double)old_style_value / 2.0;

	if(max_ang_speed < min_ang_speed) max_ang_speed = min_ang_speed;
	if(max_lin_speed < min_lin_speed) max_lin_speed = min_lin_speed;
	if(max_ang_speed > 13.0) max_ang_speed = 13.0;
	if(max_lin_speed > 25.0) max_lin_speed = 25.0;
}

static int mode_xy;
static uint32_t ang_to_target;


void cmd_go_to(s2b_move_abs_t* m)
{
	target_pos.x = (int64_t)m->x<<16;
	target_pos.y = (int64_t)m->y<<16;
	backmode = m->backmode;
	new_direction = 1;
	cur_id = m->id;

	mode_xy = 1;
	do_start = 1;
}

#if 0
// Started implementing function taking straight distance and calculating x,y once
void straight_rel(int32_t mm)
{ 
	uint32_t ang = cur_pos.ang;

	if(mm < 0)
	{
		ang += (uint32_t)(ANG_180_DEG);
		mm *= -1;
	}

	int64_t x = (((int64_t)lut_cos_from_u32(ang) * (int64_t)mm)>>SIN_LUT_RESULT_SHIFT)<<16;
	int64_t y = (((int64_t)lut_sin_from_u32(ang) * (int64_t)mm)>>SIN_LUT_RESULT_SHIFT)<<16;

}
#endif

static int64_t store_lin_err;

void straight_rel(int32_t mm)
{
	ang_to_target = cur_pos.ang;
	store_lin_err = (int64_t)mm<<16;
	mode_xy = 0;
	do_start = 1;
}

void rotate_rel(int32_t ang32)
{
	ang_to_target = cur_pos.ang + (uint32_t)ang32;
	store_lin_err = 0;
	mode_xy = 0;
	do_start = 1;
}

void rotate_and_straight_rel(int32_t ang32, int32_t mm)
{
	ang_to_target = cur_pos.ang + (uint32_t)ang32;
	store_lin_err = (int64_t)mm<<16;
	mode_xy = 0;
	do_start = 1;
}

void cmd_motors(int enabled)
{
	motors_enabled = enabled;
}

static s2b_corr_pos_t stored_corrpos;
static int corrpos_in_queue;

void cmd_corr_pos(s2b_corr_pos_t* cmd)
{
	memcpy(&stored_corrpos, cmd, sizeof(s2b_corr_pos_t));
	corrpos_in_queue = 1;
}

void execute_corr_pos()
{
	if(!corrpos_in_queue)
		return;

	int32_t da = stored_corrpos.da;
	int64_t dx = stored_corrpos.dx;
	int64_t dy = stored_corrpos.dy;

	if(dx > 2000LL || dx < -2000LL || dy > 2000LL || dy < -2000LL)
		error(167);

	// Divide by 2 by only shifting 15 places instead of 16.
	dx<<=15;
	dy<<=15;
	da/=2;

	cur_pos.x += dx;
	cur_pos.y += dy;
	cur_pos.ang += (uint32_t)da;
//	target_pos.x += dx;
//	target_pos.y += dy;

	corrpos_in_queue = 0;
}




static uint32_t ang_to_target;

static void stop()
{
	act_fifo_discard_all();
	acting = 0;
	motor_stop_now(0);
	motor_stop_now(1);
}

void cmd_stop_movement()
{
	stop();
	micronavi_status = 0;
}

static int err_cnt;
static void log_err()  __attribute__((section(".text_itcm")));
static void log_err()
{
	err_cnt+=1000;
	if(err_cnt > 3000)
		error(130);
}


typedef enum
{
	DRIVE_FREE, // Free motors, destination is kept the same as current pose
	DRIVE_XY_AUTO, // Go to the next XY, autoselect drive direction to minimize turning
	DRIVE_XY_FWD,  // Only go forward; allow very short backward motions to eliminate strange unnecessary turning
	DRIVE_XY_BWD,  // Only go backwards, allow very short forward motions
	DRIVE_ROTATE_CW,
	DRIVE_ROTATE_CCW,
	DRIVE_FWD,
	DRIVE_BWD
} drive_state_t;

typedef struct
{
	drive_state_t state;
	int64_t target_x;
	int64_t target_y;
} drive_action_t;

#define ACT_FIFO_LEN 6
drive_action_t act_fifo[ACT_FIFO_LEN]
int act_fifo_wr, act_fifo_rd;

void act_fifo_push(drive_action_t *act) __attribute__((section(".text_itcm")));
void act_fifo_push(drive_action_t *act)
{
	int next = act_fifo_wr + 1;
	if(next >= ACT_FIFO_LEN) next = 0;

	if(next == act_fifo_rd)
		error(139); // Drive action fifo overrun

	memcpy(&act_fifo[act_fifo_wr], act, sizeof drive_action_t);
	act_fifo_wr = next;
}

drive_action_t* act_fifo_peek() __attribute__((section(".text_itcm")));
drive_action_t* act_fifo_peek()
{
	if(act_fifo_wr == act_fifo_rd)
		return NULL;
	return &act_fifo[act_fifo_rd];
}

drive_action_t* act_fifo_peekpeek() __attribute__((section(".text_itcm")));
drive_action_t* act_fifo_peekpeek()
{
	if(act_fifo_rd == act_fifo_wr)
		return NULL;
	int next = act_fifo_rd+1; if(next >= ACT_FIFO_LEN) next = 0;
	if(next == act_fifo_wr)
		return NULL;
	return &act_fifo[next];
}

void act_fifo_discard() __attribute__((section(".text_itcm")));
void act_fifo_discard() // Callable after peek
{
	if(act_fifo_wr == act_fifo_rd)
		error(138); // act_fifo_discard() called without readable data - call _peek() first, only call _discard() if peek() is succesful.
	act_fifo_rd++;
	if(act_fifo_rd >= ACT_FIFO_LEN) act_fifo_rd = 0;
}

void act_fifo_discard_all() __attribute__((section(".text_itcm")));
void act_fifo_discard_all()
{
	act_fifo_rd = act_fifo_wr;
}

#define HALL_STEPS_PER_REV (90)
#define WHEEL_DIAM_MM 790LL


int acting = 0;

static double cur_ang_speed;
static double cur_lin_speed;

static void control_motor_setpoints(int32_t ang_err, int64_t lin_err)
{
	uint32_t mpos[2];
	mpos[0] = bldc_pos[0];
	mpos[1] = bldc_pos[1];


	double max_ang_speed_by_ang_err = 1.5 + 0.16*abso(ang_err)/ANG_1_DEG;
	double max_ang_speed_by_lin_err = 999.9; // do not use such limitation

	double max_lin_speed_by_lin_err = 5.0 + 0.05*(double)abso((lin_err>>16)); // 400mm error -> speed unit 20
	double max_lin_speed_by_ang_err = 100.0 / (abso(ang_err)/ANG_1_DEG); // 10 deg error -> max lin speed 10 units.


	// Example 1: 10 degree error = 119304650 units  -> squared = 14233599511622500  -> max lin speed 10 units
	// Example 2: 1 degree error    -> max lin speed 1000 units (limited by everything else)
	// Example 3: 20 degree error   -> max lin speed 2.5 units
//	double ang_err_squared = (double)(sq((int64_t)ang_err));
//	if(ang_err_squared < 1000.0)
//		ang_err_squared = 1000.0; // Prevent division by zero, and massive results.
//	double max_lin_speed_by_ang_err = (double)142335995116225000.0 / ang_err_squared;


	// Calculate the target wheel positions to correct the measured angular error
	// In theory, this movement produces the "correct" end result automatically
	// However, only the _remaining_ error (by gyro!) is used on each cycle, so the target
	// wheel position gets better and better.
	// 180 degree positive turn is -18000 units on both wheels
	// 18 degree is -1800
	// 1.8 degree is -180 units
	// 1 degree is -100 units
	// 0.1 degrees is -10 units
	// 0.01 degrees is -1 unit
	
	int32_t delta_mpos_ang[2];
	int32_t delta_mpos_lin[2];

	delta_mpos_ang[0] = (ang_err/ANG_0_01_DEG);
	delta_mpos_ang[1] = (ang_err/ANG_0_01_DEG);

	delta_mpos_lin[0] = lin_err / ((WHEEL_DIAM_MM<<16)/((int64_t)HALL_STEPS_PER_REV*256LL));
	delta_mpos_lin[1] = -1*lin_err / ((WHEEL_DIAM_MM<<16)/((int64_t)HALL_STEPS_PER_REV*256LL));


	// Find the slowest allowed max speed (pick the lowest constraint):

	double lowest_max_ang_speed = max_ang_speed;
	if(max_ang_speed_by_ang_err < lowest_max_ang_speed) 
		lowest_max_ang_speed = max_ang_speed_by_ang_err;
	if(max_ang_speed_by_lin_err < lowest_max_ang_speed)
		lowest_max_ang_speed = max_ang_speed_by_lin_err;

	double lowest_max_lin_speed = max_lin_speed;
	if(max_lin_speed_by_ang_err < lowest_max_lin_speed);
		lowest_max_lin_speed = max_lin_speed_by_ang_err;
	if(max_lin_speed_by_lin_err < lowest_max_lin_speed)
		lowest_max_lin_speed = max_lin_speed_by_lin_err;


	// Decelerate, if going too fast - else, accelerate
	// current speed has a meaninglessly small oscillation due to this.

	if(cur_ang_speed > lowest_max_ang_speed)
		cur_ang_speed *= 1.0/1.004;
	else
		cur_ang_speed *= 1.004;

	if(cur_lin_speed > lowest_max_lin_speed)
		cur_lin_speed *= 1.0/1.005;
	else
		cur_lin_speed *= 1.005;


	int max_mpos_err = 5*256;
//	static int max_pos_err_cnt;
	// We know the target wheel positions, but limit the rate of change
	for(int m=0; m<2; m++)
	{
		int ang_speed_i = (int)cur_ang_speed;
		int lin_speed_i = (int)cur_lin_speed;

		int d_ang = delta_mpos_ang[m];
		int d_lin = delta_mpos_lin[m];

		if(d_ang > ang_speed_i) d_ang = ang_speed_i;
		else if(d_ang < -1*ang_speed_i) d_ang = -1*ang_speed_i;

		if(d_lin > lin_speed_i) d_lin = lin_speed_i;
		else if(d_lin < -1*lin_speed_i) d_lin = -1*lin_speed_i;

		int increment = d_ang + d_lin;
		uint32_t new_pos = bldc_pos_set[m] + (uint32_t)increment;
		int new_pos_err = mpos[m] - new_pos;
		if(new_pos_err > max_mpos_err || new_pos_err < -1*max_mpos_err)
		{
//			max_pos_err_cnt++;
		}
		else
		{
			bldc_pos_set[m] = new_pos;
		}
	}
}

// Always positive
static inline int64_t calc_lin_err(int64_t target_x, int64_t target_y)
{
	int64_t dx = cur_pos.x - target_x;
	int64_t dy = cur_pos.y - target_y;
	return sqrt(sq((double)dx) + sq((double)dy));
}

static inline uint32_t calc_ang_to_target(int64_t target_x, int64_t target_y)
{
	int64_t dx = cur_pos.x - target_x;
	int64_t dy = cur_pos.y - target_y;
	return (uint32_t)(((double)ANG_180_DEG*2.0*(M_PI+atan2((double)dy, (double)dx)))/(2*M_PI));
}

static uint32_t cur_ang_to_target;
static int64_t cur_lin_to_target;
static int32_t cur_id;

static void select_dir(uint8_t dir)
{
	int32_t ang_err = cur_pos.ang - cur_ang_to_target;

	if(dir == DIR_AUTO || cur_lin_to_target < 100*65536)
	{
		if(ang_err < -1*ANG_90_DEG || ang_err > ANG_90_DEG)
		{
			cur_ang_to_target += (uint32_t)ANG_180_DEG;
			cur_lin_to_target *= -1;
		}
	}
	else if(dir == DIR_FWD)
	{
	}
	else if(dir == DIR_BWD)
	{
		cur_ang_to_target += (uint32_t)ANG_180_DEG;
		cur_lin_to_target *= -1;
	}
}

static int check_obstacles()
{
	int32_t ang_err = cur_pos.ang - cur_ang_to_target;


	if(cur_lin_to_target > 100 && obstacle_front > 20)
	{
		micronavi_status |= 1UL<<0;
#ifdef LEDS_ON
		led_status(9, RED, LED_MODE_FADE);
		led_status(0, RED, LED_MODE_FADE);
		led_status(1, RED, LED_MODE_FADE);
#endif
		return 1;
	}
	else if(cur_lin_to_target < -100 && obstacle_back > 20)
	{
		micronavi_status |= 1UL<<0;
#ifdef LEDS_ON
		led_status(4, RED, LED_MODE_FADE);
		led_status(5, RED, LED_MODE_FADE);
		led_status(6, RED, LED_MODE_FADE);
#endif
		return 1;
	}
	else if(ang_err > 10*ANG_1_DEG && obstacle_left > 20)
	{
		micronavi_status |= 1UL<<2;
#ifdef LEDS_ON
		led_status(2, RED, LED_MODE_FADE);
		led_status(3, RED, LED_MODE_FADE);
#endif
		return 1;
	}
	else if(ang_err < -10*ANG_1_DEG && obstacle_right > 20)
	{
		micronavi_status |= 1UL<<2;
#ifdef LEDS_ON
		led_status(7, RED, LED_MODE_FADE);
		led_status(8, RED, LED_MODE_FADE);
#endif
		return 1;
	}
	return 0;
}

void reset_obstacles()
{
	micronavi_status = 0;
	obstacle_front = 0;
	obstacle_back = 0;
	obstacle_left = 0;
	obstacle_right = 0;
}


static void drive_action_fsm()
{
	if(drive_diag)
	{
		drive_diag->enabled = 0;
		drive_diag->micronavi_stop_flags = micronavi_status;
		drive_diag->cur_id = cur_id;
	}

	drive_action_t *cur_act = act_fifo_peek();
	drive_action_t *next_act = act_fifo_peekpeek();
	if(!acting) // Idle, waiting for instruction. Note: between instructions, acting goes low at the end of function
	{
		if(cur_act) // Got an instruction
		{
			reset_obstacles();
			acting = 1;
			// Calculate angle to the target immediately. If we are too close, further recalculations are suppressed, hence we need this here.
			cur_ang_to_target = calc_ang_to_target(cur_act->target_x, cur_act->target_y);
			cur_lin_to_target = calc_lin_err(cur_act->target_x, cur_act->target_y);

			select_dir(cur_act.dir); // Modifies cur_ang_to_target, cur_lin_to_target

			motor_torque_lim(0, 50);
			motor_torque_lim(1, 50);
			motor_run(0);
			motor_run(1);

			cur_id = cur_act->id;

//			cur_ang_speed = min_ang_speed;
//			cur_lin_speed = min_lin_speed;
		}
		else
		{
			if(idle_cnt > 0)
			{
				idle_cnt--;
			}
			else
			{
				motor_release(0);
				motor_release(1);
			}
		}
	}
	else // Acting on cur_act
	{
		idle_cnt = 1000; // Reset here; let downcount after stopping acting

		#ifdef LEDS_ON
			if(cur_lin_to_target < 0)
			{
				led_status(4, WHITE, LED_MODE_FADE);
				led_status(5, WHITE, LED_MODE_FADE);
				led_status(6, WHITE, LED_MODE_FADE);
				led_status(9, BLACK, LED_MODE_FADE);
				led_status(0, BLACK, LED_MODE_FADE);
				led_status(1, BLACK, LED_MODE_FADE);
			}
			else
			{
				led_status(4, BLACK, LED_MODE_FADE);
				led_status(5, BLACK, LED_MODE_FADE);
				led_status(6, BLACK, LED_MODE_FADE);
				led_status(9, WHITE, LED_MODE_FADE);
				led_status(0, WHITE, LED_MODE_FADE);
				led_status(1, WHITE, LED_MODE_FADE);
			}
		#endif

		if(check_obstacles())
		{
			stop();
		}
		else if(abso(cur_lin_to_target) < 30*65536)
		{
			// Finished
			act_fifo_discard();
			acting = 0;
			motor_let_stop(0);
			motor_let_stop(1);
		}
		else
		{
			// cur_lin_to_target is updated by the wheel odometry automatically;
			// If we do nothing, we end up at the initially calculated destination.
			// Recalculate it the vector to the target here, but stop doing recalculation
			// near the end, because even a small drift would cause excessive turning.
#if 0
			int32_t new_lin_to_target = calc_lin_err(cur_act->target_x, cur_act->target_y);

			if(new_lin_to_target > 200)
			{
				cur_lin_to_target = new_lin_to_target;
				cur_ang_to_target = calc_ang_to_target(cur_act->target_x, cur_act->target_y);
				select_dir(cur_act.dir);
			}

			if(next_act)
			{
				uint32_t cur_ang_to_next_target = calc_ang_to_target(next_act->target_x, next_act->target_y);

				int32_t ang_err_to_cur_target  = cur_pos.ang - cur_ang_to_target;
				int32_t ang_err_to_next_target = cur_pos.ang - cur_ang_to_next_target;

				int32_t errdif = (uint32_t)ang_err_to_cur_target - (uint32_t)ang_err_to_next_target;
				int32_t abs_errdif = abso(errdif);

				if(abs_errdif < 10*ANG_1_DEG)
					abs_errdif = 10*ANG_1_DEG;

				/*
					If we would take the next point early, our current angular error would change by
					abs_errdif. Say, abs_errdif is:
					10 degrees  -> max linear error dismissed 500mm (saturated to this)
					20 degrees  -> max linear error dismissed 250mm
					40 degrees  -> max linear error dismissed 125mm
				*/

				int64_t early_allowed = (5000*65536)/(abs_errdif/ANG_1_DEG);

				if(abso(cur_lin_to_target) < early_allowed)
				{
					act_fifo_discard(); // Finish the current move early
					acting = 0;
				}
			}
#endif

			int32_t ang_err = cur_pos.ang - cur_ang_to_target;
			control_motor_setpoints(ang_err, cur_lin_to_target);

			if(drive_diag)
			{
				drive_diag->enabled = 1;
				drive_diag->cur_x = cur_pos.x>>16;
				drive_diag->cur_y = cur_pos.y>>16;
				drive_diag->target_x = cur_act->target_x>>16;
				drive_diag->target_y = cur_act->target_y>>16;
				drive_diag->cur_ang_to_target = cur_ang_to_target;
				drive_diag->cur_lin_to_target = cur_lin_to_target>>16;
				drive_diag->ang_err = ang_err;
				drive_diag->cur_id = cur_id;
			}


		}
	}


}



void drive_handler() __attribute__((section(".text_itcm")));
void drive_handler()
{
	static int gyro_dc_x[6];
	static int gyro_dc_y[6];
	static int gyro_dc_z[6];

	int32_t g_pitch=0, g_roll=0, g_yaw=0;
	int32_t a_x=0, a_y=0, a_z=0; // in robot coordinate frame


	static int ac_th_det_on = 0;

	static const uint8_t x_is_pitch_y_is_roll[6] = {1,1,0,  0,0,0}; // Otherwise, x is roll, y is pitch
	static const int8_t pitch_mult[6] = {-1, 1, 1,   -1, 1,-1};
	static const int8_t roll_mult[6] =  {-1, 1,-1,    1,-1, 1};

	static const int8_t x_mult[6] =  {-1, 1,-1,    1,-1, 1};
	static const int8_t y_mult[6] =  { 1,-1,-1,    1,-1, 1};


	/*

	Roll:
	"Left wing" rising: positive g_roll, positive a_y

	Pitch:
	Nose rising: positive g_pitch, positive a_x

	Yaw:
	Counter-clockwise = positive

	a_z is -g when oriented normally

	*/


	for(int imu=0; imu<6; imu++)
	{
		if(imu_a[imu]->n < 4 || imu_a[imu]->n > 7 || imu_g[imu]->n < 4 || imu_g[imu]->n > 7)
		{
			log_err();
			// Just use the old values!
		}

		int32_t cur_gx=0, cur_gy=0, cur_gz=0;
		//int32_t cur_ax=0, cur_ay=0, cur_az=0;

		for(int n=0; n<imu_g[imu]->n; n++)
		{
			cur_gx += imu_g[imu]->xyz[n].x;
			cur_gy += imu_g[imu]->xyz[n].y;
			cur_gz += imu_g[imu]->xyz[n].z;
		}

		cur_gx /= imu_g[imu]->n;
		cur_gy /= imu_g[imu]->n;
		cur_gz /= imu_g[imu]->n;

		// With x256 resolution:
		int cur_gx_dccor = (cur_gx<<8) - (gyro_dc_x[imu]>>7);
		int cur_gy_dccor = (cur_gy<<8) - (gyro_dc_y[imu]>>7);
		int cur_gz_dccor = (cur_gz<<8) - (gyro_dc_z[imu]>>7);

		if(cur_gz < -G_DC_MOTDET_TH || cur_gz > G_DC_MOTDET_TH || cur_gx < -G_DC_MOTDET_TH || cur_gx > G_DC_MOTDET_TH || cur_gy < -G_DC_MOTDET_TH || cur_gy > G_DC_MOTDET_TH ||
		   ((ac_th_det_on>20000) && (cur_gz_dccor < -G_AC_MOTDET_TH || cur_gz_dccor > G_AC_MOTDET_TH || cur_gx_dccor < -G_AC_MOTDET_TH || cur_gx_dccor > G_AC_MOTDET_TH || cur_gy_dccor < -G_AC_MOTDET_TH || cur_gy_dccor > G_AC_MOTDET_TH)))
		{
			robot_moves();
		}
		else
		{
			robot_doesnt_move();
		}

		if(is_robot_moving())
		{
//			led_status(9, WHITE, LED_MODE_FADE);
//			led_status(0, WHITE, LED_MODE_FADE);
//			led_status(1, WHITE, LED_MODE_FADE);
		}
		else
		{
//			led_status(9, BLACK, LED_MODE_KEEP);
//			led_status(0, BLACK, LED_MODE_KEEP);
//			led_status(1, BLACK, LED_MODE_KEEP);
			if(ac_th_det_on < 100000) ac_th_det_on++;
			gyro_dc_x[imu] = ((cur_gx<<15) + ((1<<G_DC_FILT)-1)*gyro_dc_x[0])>>G_DC_FILT;
			gyro_dc_y[imu] = ((cur_gy<<15) + ((1<<G_DC_FILT)-1)*gyro_dc_y[0])>>G_DC_FILT;
			gyro_dc_z[imu] = ((cur_gz<<15) + ((1<<G_DC_FILT)-1)*gyro_dc_z[0])>>G_DC_FILT;
		}

		if(cur_gx_dccor < -GYRO_X_BLANKING_TH || cur_gx_dccor > GYRO_X_BLANKING_TH)
		{
			if(x_is_pitch_y_is_roll[imu])
			{
				g_pitch += (int)pitch_mult[imu] * cur_gx_dccor;
			}
			else
			{
				g_roll += (int)roll_mult[imu] * cur_gx_dccor;
			}
		}

		if(cur_gy_dccor < -GYRO_Y_BLANKING_TH || cur_gy_dccor > GYRO_Y_BLANKING_TH)
		{
			if(x_is_pitch_y_is_roll[imu])
			{
				g_roll += (int)roll_mult[imu] * cur_gy_dccor;
			}
			else
			{
				g_pitch += (int)pitch_mult[imu] * cur_gy_dccor;
			}
		}

		// Yaw is easy, all IMUs are mounted on the PCB top layer.
		if(cur_gz_dccor < -GYRO_Z_BLANKING_TH || cur_gz_dccor > GYRO_Z_BLANKING_TH)
		{
			g_yaw += cur_gz_dccor;
		}


		int32_t cur_ax=0, cur_ay=0, cur_az=0;

		for(int n=0; n<imu_g[imu]->n; n++)
		{
			cur_ax += imu_a[imu]->xyz[n].x;
			cur_ay += imu_a[imu]->xyz[n].y;
			cur_az += imu_a[imu]->xyz[n].z;
		}

		cur_ax /= imu_g[imu]->n;
		cur_ay /= imu_g[imu]->n;
		cur_az /= imu_g[imu]->n;

		if(x_is_pitch_y_is_roll[imu])
		{
			a_y += (int)y_mult[imu] * cur_ax;
		}
		else
		{
			a_x += (int)x_mult[imu] * cur_ax;
		}

		if(x_is_pitch_y_is_roll[imu])
		{
			a_x += (int)x_mult[imu] * cur_ay;
		}
		else
		{
			a_y += (int)y_mult[imu] * cur_ay;
		}

		// Z is easy, all IMUs point the same way.
		a_z += -1*cur_az;
	}

	g_pitch /= 6;
	g_roll /= 6;
	g_yaw /= 6;

	a_x /= 6;
	a_y /= 6;
	a_z /= 6;

	if(ac_th_det_on>1000)
	{
		cur_pos.ang += (97495LL*(int64_t)g_yaw)>>16;
		cur_pos.pitch += (97495LL*(int64_t)g_pitch)>>16;
		cur_pos.roll += (97495LL*(int64_t)g_roll)>>16;

		int32_t pitch_by_acc = (uint32_t) (4294967296.0 * (M_PI + -1*atan2(a_x, a_z)) / (2.0*M_PI));
		int32_t roll_by_acc  = (uint32_t) (4294967296.0 * (M_PI + -1*atan2(a_y, a_z)) / (2.0*M_PI));

		int32_t pitch_err = pitch_by_acc - cur_pos.pitch;
		int32_t roll_err = roll_by_acc - cur_pos.roll;

		cur_pos.pitch += pitch_err/2048;
		cur_pos.roll += roll_err/2048;
	}
	else
	{
		if(is_robot_moving())
		{
			for(int i=0; i<10; i++)
				led_status(i, RED, LED_MODE_FADE);

			ac_th_det_on = 0;
		}
	}


	uint32_t mpos[2];
	mpos[0] = bldc_pos[0];
	mpos[1] = bldc_pos[1];
	
	static uint32_t prevpos[2];
	static int initialized;
	int32_t deltapos[2];
	if(!initialized)
	{
		prevpos[0] = mpos[0];
		prevpos[1] = mpos[1];
		initialized=1;
	}
	deltapos[0] = mpos[0] - prevpos[0];
	deltapos[1] = mpos[1] - prevpos[1];
	prevpos[0] = mpos[0];
	prevpos[1] = mpos[1];

	if(deltapos[0] < -1000 || deltapos[0] > 1000 || deltapos[1] < -1000 || deltapos[1] > 1000) error(132);

	deltapos[0] *= (WHEEL_DIAM_MM<<16)/(90*256);
	deltapos[1] *= -1*(WHEEL_DIAM_MM<<16)/(90*256);

	int32_t fwd = (deltapos[0] + deltapos[1])>>1;

	cur_lin_to_target -= fwd;

	cur_pos.x += ((int64_t)lut_cos_from_u32(cur_pos.ang) * (int64_t)fwd)>>SIN_LUT_RESULT_SHIFT;
	cur_pos.y += ((int64_t)lut_sin_from_u32(cur_pos.ang) * (int64_t)fwd)>>SIN_LUT_RESULT_SHIFT;


	if(hw_pose)
	{
		hw_pose->ang = cur_pos.ang;
		hw_pose->pitch = cur_pos.pitch;
		hw_pose->roll = cur_pos.roll;
		hw_pose->x = cur_pos.x>>16;
		hw_pose->y = cur_pos.y>>16;
	}



	// Decrement watchdog
	if(motors_enabled > 0)
	{
		motors_enabled--;
		drive_action_fsm();
	}
	else
	{
		micronavi_status |= 1UL<<5;
		stop();
		if(drive_diag)
		{
			drive_diag->enabled = 0;
			drive_diag->micronavi_stop_flags = micronavi_status;
			drive_diag->cur_id = cur_id;
		}
	}



	if(err_cnt > 0) err_cnt--;


}

