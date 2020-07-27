/*

Drive module:

Functions that process drive-related commands (go to, move/turn relative, etc.)

Drive handler, triggered by software interrupt by the imu.c module, after the required IMU data
packet has been fully acquired. Drive handler runtime-compensates IMU data, keeps track of odometry
and pose estimate, and commands the bldc.c module to produce wheel motion.

*/

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
#include "audio.h"
#include "tof_ctrl.h" // for N_SENSORS
#include "timebase.h" // cnt_100us

//#define STOP_LEDS_ON
#define LEDS_ON

// Motion detection thresholds:
// When exceeded, robot is considered moving, which has multitude of consequences, such as:
// - TOF sensors may activate (see run.c)
// - If still active, the initial gyro DC offset calibration is restarted (boop boop!)
// - Runtime gyro DC offset calibration is disabled for some time

#define G_DC_MOTDET_TH 800 // Up to 300-500 generates false noise detections, especially if there is any vibration nearby
#define G_AC_MOTDET_TH (200*256) // threshold after DC offset correction

// Gyro values below blanking are considered 0. This is after DC offset correction.
// If the DC correction was conceptually perfect, and if the noise was normally distributed,
// this wouldn't be necessary. In reality, the response is slightly nonlinear and noise distribution
// not exactly gaussian, so blanking the noise out prevents very slow drifts.
#define GYRO_X_BLANKING_TH (100*256)
#define GYRO_Y_BLANKING_TH (100*256)
#define GYRO_Z_BLANKING_TH (100*256)

#define G_DC_FILT 14 // Quadratic effect, 15 maximum.
#define G_DC_FILT_INIT 11 // During gyro init settle

#define MOVING_RESET_VALUE 500

#define GYRO_INIT_PERIOD -1 //3000

static int moving = MOVING_RESET_VALUE/2;
static inline void robot_moves()
{
	moving = MOVING_RESET_VALUE; // 2 seconds
}

static inline int is_robot_moving()
{
	return moving;
}

int drive_is_robot_moving()
{
	return moving;
}

static int ignore_commands = 1;

void hires_pos_to_hw_pose(hw_pose_t* out, hires_pos_t* in)
{
	out->ang = in->ang;
	out->pitch = in->pitch;
	out->roll = in->roll;
	out->x = in->x>>16;
	out->y = in->y>>16;
}

void hw_pose_to_hires_pos(hires_pos_t* out, hw_pose_t* in)
{
	out->ang = in->ang;
	out->pitch = in->pitch;
	out->roll = in->roll;
	out->x = (int64_t)(in->x)<<16;
	out->y = (int64_t)(in->y)<<16;
}


hires_pos_t cur_pos;
hires_pos_t target_pos;


void new_target(hires_pos_t pos)
{
	target_pos = pos;
}

int obstacle_front_near, obstacle_back_near, obstacle_right_near, obstacle_left_near; // Written to in tof_process.c
int obstacle_right_very_near, obstacle_left_very_near; // Written to in tof_process.c
int obstacle_front_far, obstacle_back_far, obstacle_right_far, obstacle_left_far; // Written to in tof_process.c

int motors_enabled = 0;

int32_t cur_id;
uint32_t micronavi_status;

uint32_t get_micronavi_status()
{
	return micronavi_status;
}

int new_direction;
int backmode;

static int run;

int is_driving()
{
	return run;
}

static double max_ang_speed = 50.0;
static double max_lin_speed = 200.0;

void set_top_speed_max(int old_style_value)
{
	max_ang_speed = (double)old_style_value * 0.75;// / 5.0;
	max_lin_speed = (double)old_style_value * 2.5;


	if(max_ang_speed < 20.0) max_ang_speed = 20.0;
	if(max_lin_speed < 50.0) max_lin_speed = 50.0;
	if(max_ang_speed > 100.0) max_ang_speed = 100.0;
	if(max_lin_speed > 250.0) max_lin_speed = 250.0;
}

static uint32_t target_ang;
static int reverse;

volatile int lock_processing;

static int stop_indicators;

static int accurot;

static int64_t lin_left; 

int32_t get_remaining_lin()
{
	return lin_left>>16;
}


void cmd_go_to(s2b_move_abs_t* m)
{
	if(ignore_commands)
		return;

	robot_moves();
	stop_chafind();
	lock_processing = 1;
	set_top_speed_max(60);
	target_pos.x = (int64_t)m->x<<16;
	target_pos.y = (int64_t)m->y<<16;
	backmode = m->backmode;
	new_direction = 1;
	cur_id = m->id;
	micronavi_status = 0;

	accurot = 0;

	reverse = 0;
	int64_t dx = cur_pos.x - target_pos.x;
	int64_t dy = cur_pos.y - target_pos.y;
	lin_left = sqrt(sq((double)dx) + sq((double)dy));

	target_ang  = (uint32_t)(((double)ANG_180_DEG*2.0*(M_PI+atan2((double)dy, (double)dx)))/(2.0*M_PI));


	int auto_decision = 0;
	if((lin_left < 300*65536) || backmode==2)
	{
		auto_decision = 1;
	}

	int32_t ang_err = cur_pos.ang - target_ang;

	if((auto_decision && (ang_err > 90*ANG_1_DEG || ang_err < -90*ANG_1_DEG)) || (backmode==1 && !auto_decision))
	{
		reverse = 1;
		//ang_err = (uint32_t)ang_err + (uint32_t)(ANG_180_DEG);
		target_ang = (uint32_t)target_ang + (uint32_t)(ANG_180_DEG);
	}


//	do_start = 1;

	lock_processing = 0;

	if(stop_indicators == 0)
		beep(75, 800, -600, 30);

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

void straight_rel(int32_t mm)
{
	if(ignore_commands)
		return;

	robot_moves();

//	uart_print_string_blocking("straight_rel\r\n");
//	DBG_PR_VAR_I32(mm);
	target_ang = cur_pos.ang;
	lin_left = abso((int64_t)mm<<16);
	reverse = (mm<0)?1:0;

	if(stop_indicators == 0)
		beep(75, 800, -600, 30);

	accurot = 1;

	micronavi_status = 0;

//	do_start = 1;
}

void rotate_rel(int32_t ang32, int accurate)
{
	if(ignore_commands)
		return;

	robot_moves();

//	uart_print_string_blocking("rotate_rel\r\n");
//	DBG_PR_VAR_I32(ang32);
//	DBG_PR_VAR_I32(target_ang);
	target_ang = cur_pos.ang + (uint32_t)ang32;
//	DBG_PR_VAR_I32(target_ang);
//	store_lin_err = 0;

	if(stop_indicators == 0)
		beep(75, 800, -600, 30);

	accurot = accurate;

	micronavi_status = 0;

//	do_start = 1;
}

void rotate_rel_on_fly(int32_t ang32, int accurate)
{
	if(ignore_commands)
		return;

	robot_moves();

//	uart_print_string_blocking("rotate_rel\r\n");
//	DBG_PR_VAR_I32(ang32);
//	DBG_PR_VAR_I32(target_ang);
	target_ang = target_ang + (uint32_t)ang32;
//	DBG_PR_VAR_I32(target_ang);

	if(stop_indicators == 0)
		beep(75, 800, -600, 30);

	accurot = accurate;

//	do_start = 1;
}

void rotate_and_straight_rel(int32_t ang32, int32_t mm, int accurate_rotation_first)
{
	if(ignore_commands)
		return;

	robot_moves();

//	uart_print_string_blocking("rotate_and_straight_rel\r\n");
//	DBG_PR_VAR_I32(ang32);
//	DBG_PR_VAR_I32(mm);
	target_ang = cur_pos.ang + (uint32_t)ang32;
	lin_left = abso((int64_t)mm<<16);
	reverse = (mm<0)?1:0;

	if(stop_indicators == 0)
		beep(75, 800, -600, 30);

	accurot = accurate_rotation_first;

	micronavi_status = 0;

//	do_start = 1;
}

void cmd_motors(int enabled_ms)
{
	motors_enabled = enabled_ms/4;
}

static s2b_corr_pos_t stored_corrpos;
static int corrpos_in_queue;

void cmd_corr_pos(s2b_corr_pos_t* cmd)
{
	memcpy(&stored_corrpos, cmd, sizeof(s2b_corr_pos_t));
	corrpos_in_queue = 1;
}

/*
void cmd_set_pose(s2b_set_pose_t* cmd)
{
	hw_pose_to_hires_pos(&cur_pos, &cmd->new_pose);
}
*/

void execute_corr_pos()
{
	if(!corrpos_in_queue)
		return;

	int32_t da = stored_corrpos.da;
	int64_t dx = stored_corrpos.dx;
	int64_t dy = stored_corrpos.dy;

	if(dx > 2000LL || dx < -2000LL || dy > 2000LL || dy < -2000LL)
		error(130);

	// Divide by 2 by only shifting 15 places instead of 16.
//	dx<<=15;
//	dy<<=15;
//	da/=2;

	dx<<=16;
	dy<<=16;

	cur_pos.x += dx;
	cur_pos.y += dy;
	cur_pos.ang += (uint32_t)da;
	target_pos.x += dx;
	target_pos.y += dy;

	corrpos_in_queue = 0;
}


static void stop(int reason)
{
	target_pos = cur_pos;
	new_direction = 0;
	run = 0;
	lin_left = 0;
//	do_start = 0;
	target_ang = cur_pos.ang;
	backmode = 2; // auto decision
//	DBG_PR_VAR_I32(reason);
}

void cmd_stop_movement()
{
	stop_chafind();
	lock_processing = 1;
	stop(1);
	micronavi_status = 0;
	lock_processing = 0;
}

static int start_self_calib;
static int self_calib_dir;
static int self_calib_turns;
static int self_calib_speed;
static int gyro_calib_state;

void self_calib(s2b_self_calib_request_t* cmd)
{
	//uart_print_string_blocking("self_calib()\r\n");

	gyro_calib_state = 1;
	self_calib_turns = abso(cmd->n_turns);
	self_calib_dir = (cmd->n_turns<0)?1:0;
	self_calib_speed = cmd->speed;

	if(self_calib_turns < 1 || self_calib_turns > 100 || self_calib_speed < 2 || self_calib_speed > 100)
		error(111);
}

static int ignore_front, ignore_left, ignore_back, ignore_right;

void obstacle_avoidance_ignore(int dir, int ms)
{
	ms *= 4;
	if(dir==0)
		ignore_front = ms;
	else if(dir==1)
		ignore_left = ms;
	else if(dir==2)
		ignore_back = ms;
	else if(dir==3)
		ignore_right = ms;
}

static int err_cnt;
static void log_err()  __attribute__((section(".text_itcm")));
static void log_err()
{
	err_cnt+=1000;
	if(err_cnt > 3000)
		error(131);
}


extern int32_t latency_targets[N_SENSORS];
#define MS(x) ((x)*10)

void sensors_fwd(double speed)
{
	// Latencies: the criticals; the secondaries (sides), the rest (nonrelated)
	// Speed   0 -> 500ms, 1000ms, 2000ms
	// Speed  50 -> 400ms, 800ms,  2200ms
	// Speed 100 -> 300ms, 600ms,  2400ms
	// Speed 150 and over -> 200ms, 400ms,  2600ms
//	int target = 500.0 - speed*2.0;
//	if(target < 200) target = 200;
	int target = 600.0 - speed*2.0;
	if(target < 300) target = 300;

	int secondaries = target * 2;
	int nonrelated = 3000 - 2*target;

	latency_targets[9] = MS(target);
	latency_targets[0] = MS(target);
	latency_targets[1] = MS(target);

	latency_targets[2] = MS(secondaries);
	latency_targets[8] = MS(secondaries);

	latency_targets[3] = MS(nonrelated);
	latency_targets[4] = MS(nonrelated);
	latency_targets[5] = MS(nonrelated);
	latency_targets[6] = MS(nonrelated);
	latency_targets[7] = MS(nonrelated);
}

void sensors_bwd(double speed)
{
	// Latencies: the criticals; the secondaries (sides), the rest (nonrelated)
	// Speed   0 -> 500ms, 1000ms, 2000ms
	// Speed  50 -> 400ms, 800ms,  2200ms
	// Speed 100 -> 300ms, 600ms,  2400ms
	// Speed 150 and over -> 200ms, 400ms,  2600ms
//	int target = 500.0 - speed*2.0;
//	if(target < 200) target = 200;
	int target = 600.0 - speed*2.0;
	if(target < 300) target = 300;

	int secondaries = target * 2;
	int nonrelated = 3000 - 2*target;

	latency_targets[4] = MS(target);
	latency_targets[5] = MS(target);
	latency_targets[6] = MS(target);

	latency_targets[3] = MS(secondaries);
	latency_targets[7] = MS(secondaries);

	latency_targets[0] = MS(nonrelated);
	latency_targets[1] = MS(nonrelated);
	latency_targets[2] = MS(nonrelated);
	latency_targets[8] = MS(nonrelated);
	latency_targets[9] = MS(nonrelated);
}


void sensors_posang(double speed)
{
	// Latencies: the criticals; the secondaries (sides), the rest (nonrelated)
	// Speed   0 -> 500ms, 1000ms, 2000ms
	// Speed  10 -> 400ms, 800ms,  2200ms
	// Speed  20 -> 300ms, 600ms,  2400ms
	// Speed  30 and over -> 200ms, 400ms,  2600ms
//	int target = 500.0 - speed*10.0;
//	if(target < 200) target = 200;
	int target = 600.0 - speed*10.0;
	if(target < 300) target = 300;

	int secondaries = target * 2;
	int nonrelated = 3000 - 2*target;

	latency_targets[7] = MS(target);
	latency_targets[8] = MS(target);

	latency_targets[9] = MS(secondaries);
	latency_targets[4] = MS(secondaries);
	latency_targets[5] = MS(secondaries);

	latency_targets[0] = MS(nonrelated);
	latency_targets[1] = MS(nonrelated);
	latency_targets[2] = MS(nonrelated);
	latency_targets[3] = MS(nonrelated);
	latency_targets[6] = MS(nonrelated);
}

void sensors_negang(double speed)
{
	// Latencies: the criticals; the secondaries (sides), the rest (nonrelated)
	// Speed   0 -> 500ms, 1000ms, 2000ms
	// Speed  10 -> 400ms, 800ms,  2200ms
	// Speed  20 -> 300ms, 600ms,  2400ms
	// Speed  30 and over -> 200ms, 400ms,  2600ms
//	int target = 500.0 - speed*10.0;
//	if(target < 200) target = 200;
	int target = 600.0 - speed*10.0;
	if(target < 300) target = 300;

	int secondaries = target * 2;
	int nonrelated = 3000 - 2*target;

	latency_targets[2] = MS(target);
	latency_targets[3] = MS(target);

	latency_targets[1] = MS(secondaries);
	latency_targets[5] = MS(secondaries);
	latency_targets[6] = MS(secondaries);

	latency_targets[0] = MS(nonrelated);
	latency_targets[4] = MS(nonrelated);
	latency_targets[7] = MS(nonrelated);
	latency_targets[8] = MS(nonrelated);
	latency_targets[9] = MS(nonrelated);
}

void sensors_idle()
{
	for(int i=0; i<N_SENSORS; i++)
		latency_targets[i] = MS(3000);
}

// Magnetometer code
// Magnetometers 2,3,4,5 are used.
// 0,1 are not used because they are located near the high-current wiring; these IMUs are in optimum placement
// for gyro&accelerometer and used for these purposes only.
// Code is written to support using all magnetometers later if that works out anyway.



/*
	Run rotation_fsm (about at least ~10Hz or more):
	rotation_fsm(3) - stop rotating
	rotation_fsm(1) - start rotating CW, reset rotation counter
	rotation_fsm(2) - start rotating CCW, reset rotation counter
	rotation_fsm(0) - return rotation counter
*/

#define ROTA_CMD_POLL 0
#define ROTA_CMD_START_POSANG 1
#define ROTA_CMD_START_NEGANG 2
#define ROTA_CMD_STOP 3
#define ROTA_CMD_STOP_ABRUPT 4

static int rotation_fsm(int cmd)
{
	static int state;
	static uint32_t start_ang;
	static int n_rounds;
	static int expecting_full_turn;

	if(cmd == ROTA_CMD_STOP_ABRUPT)
	{
		cmd_motors(1000);
		state = 0;
		stop(2);
	}
	else if(cmd == ROTA_CMD_STOP)
	{
		cmd_motors(1000);
		state = 0;
		target_ang = start_ang;
	}
	else if(cmd == ROTA_CMD_START_POSANG)
	{
		cmd_motors(1000);
		start_ang = cur_pos.ang;
		robot_moves();
		accurot = 1;
		new_direction = 1;
		//set_top_speed_max(30);
		n_rounds = 0;
		expecting_full_turn = 0;
		state = 1;
	}
	else if(cmd == ROTA_CMD_START_NEGANG)
	{
		cmd_motors(1000);
		robot_moves();
		accurot = 1;
		new_direction = 1;
		//set_top_speed_max(30);
		start_ang = cur_pos.ang;
		n_rounds = 0;
		expecting_full_turn = 0;
		state = 2;
	}

	switch(state)
	{
		case 0:
		break;

		case 1:
		{
			int32_t diff_from_start = cur_pos.ang - start_ang;
			if(diff_from_start > -30*ANG_1_DEG && diff_from_start < -20*ANG_1_DEG)
			{
				// Almost a full turn - set a sub-state
				expecting_full_turn = 1;
			}
			else if(diff_from_start > 0 && expecting_full_turn)
			{
				// Got a full turn.
				n_rounds++;
				expecting_full_turn = 0;
			}

			cmd_motors(1000);

			target_ang = cur_pos.ang + 45*ANG_1_DEG;

		} break;

		case 2:
		{
			int32_t diff_from_start = cur_pos.ang - start_ang;
			if(diff_from_start > 20*ANG_1_DEG && diff_from_start < 30*ANG_1_DEG)
			{
				// Almost a full turn - set a sub-state
				expecting_full_turn = 1;
			}
			else if(diff_from_start < 0 && expecting_full_turn)
			{
				// Got a full turn.
				n_rounds++;
				expecting_full_turn = 0;
			}

			cmd_motors(1000);

			target_ang = cur_pos.ang - 45*ANG_1_DEG;

		} break;

		default: error(132); while(1);
	}

	return n_rounds;
}


// IMU magnetometer orientations on the PCB, 0 degrees = East, 90 degrees = North
static const uint32_t m_orientations[6] = {90*UANG_1_DEG, 270*UANG_1_DEG, 0*UANG_1_DEG, 180*UANG_1_DEG, 0*UANG_1_DEG, 180*UANG_1_DEG};

static const int     m_num    = 6;
static const uint8_t m_use[6] = {1,1,1,1,1,1};


static int compass_state;


static uint32_t latest_compass_heading;

typedef struct
{

	// Input data for calibration
	// Collect during all possible angular poses
	int min_x;
	int max_x;
	int min_y;
	int max_y;

	// Hard-iron removal
	int offs_x;
	int offs_y;

	// Simple soft-iron removal
	int scale_x;
	int scale_y;
} m_iron_calib_t;

m_iron_calib_t m_iron_calib[6];

void reset_compass_calib()
{
	for(int imu=0; imu<6; imu++)
	{
		m_iron_calib[imu].min_x = 10000;
		m_iron_calib[imu].min_y = 10000;
		m_iron_calib[imu].max_x = -10000;
		m_iron_calib[imu].max_y = -10000;
	}
}


static void calc_compass_calib(int imu)
{
	// Hard iron offsets
	m_iron_calib[imu].offs_x = (m_iron_calib[imu].max_x + m_iron_calib[imu].min_x)/2;
	m_iron_calib[imu].offs_y = (m_iron_calib[imu].max_y + m_iron_calib[imu].min_y)/2;

	// Simple "good enough for now" soft iron correction: scale the ellipse so that its x width, y width are the same.

	int avg_dx = (m_iron_calib[imu].max_x - m_iron_calib[imu].min_x)/2;
	int avg_dy = (m_iron_calib[imu].max_y - m_iron_calib[imu].min_y)/2;
	int avg_d = (avg_dx + avg_dy)/2;

	// The absolute scale of the numbers is irrelevant. Here we use x16384 fixed point math.
	// Could use floating point instead, but limiting the FP usage now when there is no reason to.

	m_iron_calib[imu].scale_x = (16384*avg_d) / avg_dx;
	m_iron_calib[imu].scale_y = (16384*avg_d) / avg_dy;
}

static inline int compass_x(int imu, int val)
{
	return (val - m_iron_calib[imu].offs_x) * m_iron_calib[imu].scale_x;
}

static inline int compass_y(int imu, int val)
{
	return (val - m_iron_calib[imu].offs_y) * m_iron_calib[imu].scale_y;
}

static void compass_fsm()
{
	switch(compass_state)
	{
		case 0:
		{

		} break;

		case 1:
		{
			max_ang_speed = 7.0;
			reset_compass_calib();
			rotation_fsm(ROTA_CMD_START_POSANG);
			compass_state++;
		} break;

		case 2:
		{
			if(rotation_fsm(ROTA_CMD_POLL) >= 4)
			{
				rotation_fsm(ROTA_CMD_START_NEGANG);
				compass_state++;
			}
		} break;

		case 3:
		{
			if(rotation_fsm(ROTA_CMD_POLL) >= 4)
			{
				rotation_fsm(ROTA_CMD_STOP);
				max_ang_speed = 12.0;
				for(int i=0; i<6; i++)
				{
					if(m_use[i])
						calc_compass_calib(i);
				}
				compass_state++;
			}
		} break;

		case 4:
		{
			// Calibrated
		} break;

		default: error(133); while(1);
	}
}

static void gyro_calib_fsm();

void compass_handler() __attribute__((section(".text_itcm")));
void compass_handler()
{
	uint32_t headings[6];

	for(int imu=0; imu<6; imu++)
	{
		if(!m_use[imu])
			continue;

		int16_t x = m_compensate_x(imu_m[imu]->coords.x, imu_m[imu]->coords.rhall, imu);
		int16_t y = m_compensate_y(imu_m[imu]->coords.y, imu_m[imu]->coords.rhall, imu);
		//int16_t z = m_compensate_z(imu_m[imu]->coords.z, imu_m[imu]->coords.rhall, imu);

		if(compass_state >= 2 && compass_state <= 3)
		{
			if(x < m_iron_calib[imu].min_x)
				m_iron_calib[imu].min_x = x;
			if(y < m_iron_calib[imu].min_y)
				m_iron_calib[imu].min_y = y;
			if(x > m_iron_calib[imu].max_x)
				m_iron_calib[imu].max_x = x;
			if(y > m_iron_calib[imu].max_y)
				m_iron_calib[imu].max_y = y;
		}
		else if(compass_state == 4)
		{
			int corr_x = compass_x(imu, x);
			int corr_y = compass_y(imu, y);

/*
			DBG_PR_VAR_I32(imu);
			DBG_PR_VAR_I32(m_iron_calib[imu].min_x);
			DBG_PR_VAR_I32(m_iron_calib[imu].max_x);
			DBG_PR_VAR_I32(m_iron_calib[imu].min_y);
			DBG_PR_VAR_I32(m_iron_calib[imu].max_y);
			DBG_PR_VAR_I32(m_iron_calib[imu].offs_x);
			DBG_PR_VAR_I32(m_iron_calib[imu].offs_y);
			DBG_PR_VAR_I32(m_iron_calib[imu].scale_x);
			DBG_PR_VAR_I32(m_iron_calib[imu].scale_y);
			DBG_PR_VAR_I32(x);
			DBG_PR_VAR_I32(y);
			DBG_PR_VAR_I32(corr_x);
			DBG_PR_VAR_I32(corr_y);
*/
			uint32_t heading = (uint32_t) (4294967296.0 * (M_PI + -1*atan2(corr_y, corr_x)) / (2.0*M_PI));
			heading += m_orientations[imu];
			headings[imu] = heading;

			if(compass_heading)
			{
				compass_heading->heading_per_imu[imu] = heading;
				compass_heading->imus_valid |= 1<<imu;
			}
		}
	}

	// Average the headings.
	// For proper wrap-around handling (i.e., think about averaging 359 and 1 together, which should result 0),
	// all values are, if necessary, wrapped to mid-range (now think about averaging 179 and 181), then wrapped back.

	if(compass_state == 4)
	{
		int wrap = 0;
		int first = 0;
		while(!m_use[first]) first++;

		if(headings[first] < 90*UANG_1_DEG || headings[first] > 270*UANG_1_DEG)
			wrap = 1;

		if(wrap)
		{
			for(int i=0; i<6; i++)
				headings[i] += ANG_180_DEG;
		}

		uint64_t acc = 0;
		for(int i=0; i<6; i++)
		{
			if(!m_use[i])
				continue;
			acc += headings[i];
		}

		uint32_t avg = acc/m_num;

		// Any measurement deviating a lot from others should be treated as failure
		// If nothing else, averaging through wrapping fails if the differences are too big.
/*		for(int i=0; i<6; i++)
		{
			if(!m_use[i])
				continue;
			if(abso(headings[i]-avg) > 45*ANG_1_DEG)
			{
//				error(134); // TODO: bail out softly
			}
		}
*/

		if(wrap)
			avg += ANG_180_DEG;

		latest_compass_heading = avg;

		if(compass_heading)
			compass_heading->combined_heading = avg;
	}

/*	if(start_self_calib)
	{
		//DBG_PR_VAR_I16(start_self_calib);
		compass_state = 1;
		//gyro_calib_state = 1;
		start_self_calib = 0;
	}
*/
	//gyro_calib_fsm();
//	compass_fsm();

	static int cnt = 0;
	if((gyro_calib_state == 2) || (compass_state > 0 && compass_state < 4))
	{
		cnt++;
		if(cnt >= 20)
		{
			cnt = 0;
			beep(75, 800, -600, 30);
		}
	}

}



static int32_t gyro_mult_pos = 2437;
static int32_t gyro_mult_neg = 2437;

// Linear measurements are (mm*65536). Hall steps are (actual steps *256).
// Both numenator and denominator are reduced by x256,
// which is why wheel_diam[] is only x256 and HALL_STEPS_PER_TURN is directly number of actual hall steps

static int32_t wheel_diam[2] = {765*256, 765*256};
#define HALL_STEPS_PER_TURN (90)


/*
	Gyro DC offset is automatically cancelled, so it doesn't drift while the robot stays still.
	What's left:
	* Gain error
	* Nonlinearity errors
	* Acceleration-induced offset (caused by quick shaking, i.e., when travelling on uneven surface)

	As a first order approximation, we do nonlinearity correction by dividing to two ranges: positive and
	negative, having different gains for them. In the future, might add more steps, i.e., different gains
	for different angular rate ranges.

	Gain multipliers (positive and negative) need to be tuned automatically to compensate for gain drift
	over time, temperature, etc. Basically, there are two ways to do this:
	* Calibrate using visual clues
	* Calibrate using compass

	Compass is easier to implement on low level.


	Compass has large absolute inaccuracy, but it's noncumulative.
	Gyro has small relative inaccuracy, which cumulates to become large over time.

	Rotating large enough number of full circles causes the compass absolute inaccuracy (which stays constant, say +/- 5 deg)
	to become small compared to cumulated gyro inaccuracy (which cumulates, so say 1 deg/full circle -> 20 deg per 20 full circles).
	At this point, we can approximate the compass reading as the "ground truth", and calculate the
	correction factor for the gyro.
*/

static void calc_gyro_corr(uint32_t m_start, uint32_t m_end, uint32_t g_start, uint32_t g_end, int n_rounds)
{
	if(n_rounds > -5 && n_rounds < 5)
		error(138);

	// Ex.1: g_start = 90 deg, g_end = 92 deg, rounds=10  -> turned_by_g = 3600 + 2 deg
	// Ex.2: g_start = 359 deg, g_end = 1 deg, rounds=10  -> turned_by_g = 3600 + 2 deg

	int64_t turned_by_g = n_rounds*ANG_360_DEG_LL + (int32_t)(g_end - g_start);
	int64_t turned_by_m = n_rounds*ANG_360_DEG_LL + (int32_t)(m_end - m_start);

	//DBG_PR_VAR_I32(turned_by_g/ANG_0_1_DEG);
	//DBG_PR_VAR_I32(turned_by_m/ANG_0_1_DEG);

	if(n_rounds >= 0)
	{
		int32_t new_mult = ((int64_t)gyro_mult_pos*turned_by_m)/turned_by_g;
		//DBG_PR_VAR_I32(gyro_mult_pos);
		//DBG_PR_VAR_I32(new_mult);
		gyro_mult_pos = new_mult;
	}
	else
	{
		int32_t new_mult = ((int64_t)gyro_mult_neg*turned_by_m)/turned_by_g;
		//DBG_PR_VAR_I32(gyro_mult_neg);
		//DBG_PR_VAR_I32(new_mult);
		gyro_mult_neg = new_mult;
	}

}

static int64_t gyrocal_rate_accum;
static int32_t gyrocal_rate_accum_n;
#define N_GYRO_CALIB_TURNS 30
// Calibrates gyro with compass
static void gyro_calib_fsm()
{
	switch(gyro_calib_state)
	{
		case 0:
		{
			// Idle
		} break;

		case 1:
		{
			max_ang_speed = self_calib_speed;
			DBG_PR_VAR_I32(self_calib_speed);
			rotation_fsm(self_calib_dir?ROTA_CMD_START_NEGANG:ROTA_CMD_START_POSANG);
			gyrocal_rate_accum = 0;
			gyrocal_rate_accum_n = 0;

			gyro_calib_state++;
		} break;

		case 2:
		{
			int cur_rounds = rotation_fsm(ROTA_CMD_POLL);

			if(gyrocal_results)
			{
				gyrocal_results->n_rounds = cur_rounds;
			}

			if(cur_rounds == self_calib_turns)
			{
				rotation_fsm(ROTA_CMD_STOP);
				max_ang_speed = 12.0;

				if(gyrocal_results)
				{
					gyrocal_results->avg_rate = gyrocal_rate_accum/gyrocal_rate_accum_n;
				}

				gyro_calib_state++;

			}
		} break;

		case 3:
		{
			// Success
		} break;

		case 4:
		{
			// Failure
		} break;


		default: error(136); while(1);
	}

	if(gyrocal_results)
	{
		gyrocal_results->state = gyro_calib_state;
	}

}

void drive_freerunning_fsm()
{
	gyro_calib_fsm();
}

#define MANUAL_DRIVE_TIMER_SETVAL  400 // manual drive message received -> this timer value set
#define MANUAL_DRIVE_SLOWDOWN_TIME 300 // timer has ran down to this value -> start ignoring FAST button, acting slow instead
#define MANUAL_DRIVE_STOP_TIME     200 // timer has ran down to this value -> ignore all buttons, start slowing down towards stop
// timer has ran down to 0 -> manual drive is turned off, normal commands are used again


static int manual_drive;
static uint_fast16_t manual_drive_buttons;
static uint_fast16_t manual_drive_flags;

void drive_manual_drive(s2b_manual_drive_t* msg)
{
	manual_drive = MANUAL_DRIVE_TIMER_SETVAL;
	manual_drive_buttons = msg->buttons;
	manual_drive_flags = msg->flags;
}


#define DEFAULT_GYROMULT 153322782
#define MIN_GYROMULT ((2*DEFAULT_GYROMULT)/3)
#define MAX_GYROMULT ((4*DEFAULT_GYROMULT)/3)
static gyro_cal_t gyrocal = {1, DEFAULT_GYROMULT, 2, DEFAULT_GYROMULT, -1, DEFAULT_GYROMULT, -2, DEFAULT_GYROMULT};

void drive_inject_gyrocal(gyro_cal_t* gc)
{
	memcpy(&gyrocal, gc, sizeof(gyro_cal_t));

	DBG_PR_VAR_I32(gyrocal.pos_lospeed_mult);
	DBG_PR_VAR_I32(gyrocal.pos_lospeed_at);
	DBG_PR_VAR_I32(gyrocal.pos_hispeed_mult);
	DBG_PR_VAR_I32(gyrocal.pos_hispeed_at);

	DBG_PR_VAR_I32(gyrocal.neg_lospeed_mult);
	DBG_PR_VAR_I32(gyrocal.neg_lospeed_at);
	DBG_PR_VAR_I32(gyrocal.neg_hispeed_mult);
	DBG_PR_VAR_I32(gyrocal.neg_hispeed_at);

	if(gyrocal.pos_lospeed_at < 1 || gyrocal.pos_hispeed_at < 2 ||
	   gyrocal.neg_lospeed_at > -1 || gyrocal.neg_hispeed_at > -2 ||
	   gyrocal.pos_lospeed_mult < MIN_GYROMULT || gyrocal.pos_hispeed_mult > MAX_GYROMULT ||
	   gyrocal.neg_lospeed_mult < MIN_GYROMULT || gyrocal.neg_hispeed_mult > MAX_GYROMULT)
		error(137);

}

volatile int drive_is_rotating;
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

	static uint32_t prev_cnt_100us;

	static int imu_dt_err_cnt;

	// prev_cnt initializes 0; cnt_100us is already running because timebase initializes before IMUs.
	// first dt after bootup is excessive. Let's accept that as a single error condition.
	// At 250Hz expected interrupt rate, expected dt is 40, give or take the IMU RC oscillator inaccuracies.
	int32_t dt = cnt_100us-prev_cnt_100us;
	prev_cnt_100us = cnt_100us;

	//DBG_PR_VAR_I32(dt);

	if(dt < 25 || dt > 60)
	{
		// Still use the data - assume optimal dt. Log error.
		dt = 40; 
		imu_dt_err_cnt+=10000;

		// Initialization always results in one error. Beyond that, we have real errors.
		if(imu_dt_err_cnt > 30000)
		{
			error(139);
		}
	}
	else
	{
		if(imu_dt_err_cnt > 0)
			imu_dt_err_cnt--;
	}

	for(int imu=0; imu<6; imu++)
	{
		if(imu_a[imu]->n < 4 || imu_a[imu]->n > 7 || imu_g[imu]->n < 4 || imu_g[imu]->n > 7)
		{
			log_err();
			// Just use the old values!
		}

		int32_t cur_gx=0, cur_gy=0, cur_gz=0;

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

		// Detect any movement with all gyro axes.
		// Use larger limits to directly evaluate uncorrected raw values
		// After the DC correction filter is up and running, use DC-offset-corrected
		// values with tighter motion detection thresholds.
		if(cur_gz < -G_DC_MOTDET_TH || cur_gz > G_DC_MOTDET_TH || 
		   cur_gx < -G_DC_MOTDET_TH || cur_gx > G_DC_MOTDET_TH ||
		   cur_gy < -G_DC_MOTDET_TH || cur_gy > G_DC_MOTDET_TH ||
		   ((ac_th_det_on>GYRO_INIT_PERIOD) && (
		   cur_gz_dccor < -G_AC_MOTDET_TH || cur_gz_dccor > G_AC_MOTDET_TH ||
		   cur_gx_dccor < -G_AC_MOTDET_TH || cur_gx_dccor > G_AC_MOTDET_TH ||
		   cur_gy_dccor < -G_AC_MOTDET_TH || cur_gy_dccor > G_AC_MOTDET_TH)))
		{
			robot_moves();
		}

		if(is_robot_moving())
		{
		}
		else
		{
			if(ac_th_det_on>GYRO_INIT_PERIOD)
			{
				gyro_dc_x[imu] = ((cur_gx<<15) + ((1<<G_DC_FILT)-1)*gyro_dc_x[imu])>>G_DC_FILT;
				gyro_dc_y[imu] = ((cur_gy<<15) + ((1<<G_DC_FILT)-1)*gyro_dc_y[imu])>>G_DC_FILT;
				gyro_dc_z[imu] = ((cur_gz<<15) + ((1<<G_DC_FILT)-1)*gyro_dc_z[imu])>>G_DC_FILT;
			}
			else
			{
				gyro_dc_x[imu] = ((cur_gx<<15) + ((1<<G_DC_FILT_INIT)-1)*gyro_dc_x[imu])>>G_DC_FILT_INIT;
				gyro_dc_y[imu] = ((cur_gy<<15) + ((1<<G_DC_FILT_INIT)-1)*gyro_dc_y[imu])>>G_DC_FILT_INIT;
				gyro_dc_z[imu] = ((cur_gz<<15) + ((1<<G_DC_FILT_INIT)-1)*gyro_dc_z[imu])>>G_DC_FILT_INIT;
			}
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

		#if 0
			if(imu==0)
			{
				o_itoa16_fixed((gyro_dc_z[imu]>>7), printbuf); uart_print_string_blocking(printbuf); 

				uart_print_string_blocking("  ");

				o_itoa16_fixed((cur_gz), printbuf); uart_print_string_blocking(printbuf); 

				uart_print_string_blocking("  ");

				o_itoa16_fixed((cur_gz_dccor>>8), printbuf); uart_print_string_blocking(printbuf); 

				uart_print_string_blocking("\r\n");
			}
		#endif
	
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


	if(moving > 0)
		moving--;

	g_pitch *= dt;
	g_roll *= dt;
	g_yaw *= dt;

	g_pitch /= 6;
	g_roll /= 6;
	g_yaw /= 6;

	a_x /= 6;
	a_y /= 6;
	a_z /= 6;

	if(gyro_calib_state == 2)
	{
		gyrocal_rate_accum += g_yaw;
		gyrocal_rate_accum_n++;
		if(gyrocal_rate_accum_n > 250*60*30) // 30 minutes
		{
			error(199);
		}
	}

	/*
		setting: 97495:
		8 rounds in positive direction:
		358.1 - too little
		1.9 deg -> 0.2375 per 360 too little
		New setting: 97559


		8 rounds in negative direction:
		353.5 - too much
		6.5 deg -> 0.8125 per 360 too much
		New setting: 97276

	*/


	if(ac_th_det_on>GYRO_INIT_PERIOD)
	{
		ignore_commands = 0;

		// Calculate yaw difference using gyro calibration data
		// DC offset is already automatically tracked and removed
		// Only gain nonlinearity is left.
		// Use measured gain coeffs, at four different angular speed points:
		// Positive hi and lo angular speed, negative hi and lo angular speed.
		// Between zero and lo speed, use the lo coeff.
		// Over hi speed, use the hi coeff
		// Between the lo and hi rates, interpolate the coeff.

		int64_t dyaw = 0; // Remains zero if g_yaw == 0

		if(g_yaw >= gyrocal.pos_hispeed_at)
			dyaw = (int64_t)gyrocal.pos_hispeed_mult*(int64_t)g_yaw;
		else if(g_yaw > 0 && g_yaw <= gyrocal.pos_lospeed_at)
			dyaw = (int64_t)gyrocal.pos_lospeed_mult*(int64_t)g_yaw;
		else if(g_yaw > 0) // g_yaw between pos_lospeed_at and pos_hispeed_at - interpolate the multiplier
		{
			int32_t interp_range = gyrocal.pos_hispeed_at - gyrocal.pos_lospeed_at;
			int32_t interp_loc_x256 = (16*(g_yaw-gyrocal.pos_lospeed_at))/(interp_range/16);

//			o_itoa32(g_yaw, printbuf); uart_print_string_blocking(printbuf); 
//			uart_print_string_blocking("  ");
//			o_itoa32(interp_loc_x256, printbuf); uart_print_string_blocking(printbuf); 
//			uart_print_string_blocking("\r\n");


			int32_t wlo = 256-interp_loc_x256;
			int32_t whi = interp_loc_x256;
			int64_t mult = (wlo*(int64_t)gyrocal.pos_lospeed_mult + whi*(int64_t)gyrocal.pos_hispeed_mult)>>8;

//			uart_print_string_blocking("P ");
//			o_itoa32(mult, printbuf); uart_print_string_blocking(printbuf); 
//			uart_print_string_blocking("\r\n");

			dyaw = (int64_t)mult*(int64_t)g_yaw;
		}
		else if(g_yaw <= gyrocal.neg_hispeed_at)
			dyaw = (int64_t)gyrocal.neg_hispeed_mult*(int64_t)g_yaw;
		else if(g_yaw < 0 && g_yaw >= gyrocal.neg_lospeed_at)
			dyaw = (int64_t)gyrocal.neg_lospeed_mult*(int64_t)g_yaw;
		else if(g_yaw < 0) // g_yaw between neg_lospeed_at and neg_hispeed_at - interpolate the multiplier
		{
			int32_t interp_range = gyrocal.neg_hispeed_at - gyrocal.neg_lospeed_at;
			int32_t interp_loc_x256 = (16*(g_yaw-gyrocal.neg_lospeed_at))/(interp_range/16);

			int32_t wlo = 256-interp_loc_x256;
			int32_t whi = interp_loc_x256;
			int64_t mult = (wlo*(int64_t)gyrocal.neg_lospeed_mult + whi*(int64_t)gyrocal.neg_hispeed_mult)>>8;

//			uart_print_string_blocking("N ");
//			o_itoa32(mult, printbuf); uart_print_string_blocking(printbuf); 
//			uart_print_string_blocking("\r\n");

			dyaw = (int64_t)mult*(int64_t)g_yaw;
		}

		cur_pos.ang += dyaw>>32;


		// Update pitch and roll from gyro, but slowly correct them with accelerometer vector (which averages as gravity)

		cur_pos.pitch += (159711232LL*(int64_t)g_pitch)>>32;
		cur_pos.roll += (159711232LL*(int64_t)g_roll)>>32;

		int32_t pitch_by_acc = (uint32_t) (4294967296.0 * (M_PI + -1*atan2(a_x, a_z)) / (2.0*M_PI));
		int32_t roll_by_acc  = (uint32_t) (4294967296.0 * (M_PI + -1*atan2(a_y, a_z)) / (2.0*M_PI));

		int32_t pitch_err = pitch_by_acc - cur_pos.pitch;
		int32_t roll_err = roll_by_acc - cur_pos.roll;

		cur_pos.pitch += pitch_err/2048;
		cur_pos.roll += roll_err/2048;
	}
	else
	{
		ignore_commands = 1;
		// Give beep warnings is someone moves the robot during initial DC offset calibration
		for(int i=0; i<10; i++)
			led_status(i, RED, LED_MODE_FADE);

		if(moving > MOVING_RESET_VALUE/2+1)
		{
			if(stop_indicators < 1)
			{
				beep(500, 100, 0, 70);
				stop_indicators = 125;
			}
		}


		if(moving > 0)
		{
			ac_th_det_on = 0;

			for(int i=0; i<6;i++)
			{
				gyro_dc_x[i] = 0;
				gyro_dc_y[i] = 0;
				gyro_dc_z[i] = 0;
			}
		}
		else
		{
			ac_th_det_on++;
		}

	}

	uint32_t mpos[2];
	mpos[0] = bldc_pos[0];
	mpos[1] = bldc_pos[1];

	static uint32_t prev_mpos[2];

	static int initialized;

	if(initialized)
	{
		if(prev_mpos[0] != mpos[0] || prev_mpos[1] != mpos[1])
			robot_moves();
	}

	prev_mpos[0] = mpos[0];
	prev_mpos[1] = mpos[1];

//	int32_t mpos_err[2];
//	mpos_err[0] = bldc_pos[0] - bldc_pos_set[0];
//	mpos_err[1] = bldc_pos[1] - bldc_pos_set[1];

	
	static uint32_t prevpos[2];
	int32_t deltapos[2];
	if(!initialized)
	{
		prevpos[0] = mpos[0];
		prevpos[1] = mpos[1];
		initialized=1;
		target_pos = cur_pos;
	}
	deltapos[0] = mpos[0] - prevpos[0];
	deltapos[1] = mpos[1] - prevpos[1];
	prevpos[0] = mpos[0];
	prevpos[1] = mpos[1];

	if(deltapos[0] < -1000 || deltapos[0] > 1000 || deltapos[1] < -1000 || deltapos[1] > 1000) error(135);

	deltapos[0] *= wheel_diam[0]/HALL_STEPS_PER_TURN;
	deltapos[1] *= -1*wheel_diam[1]/HALL_STEPS_PER_TURN;

	int32_t fwd = (deltapos[0] + deltapos[1])>>1;

	cur_pos.x += ((int64_t)lut_cos_from_u32(cur_pos.ang) * (int64_t)fwd)>>SIN_LUT_RESULT_SHIFT;
	cur_pos.y += ((int64_t)lut_sin_from_u32(cur_pos.ang) * (int64_t)fwd)>>SIN_LUT_RESULT_SHIFT;


	if(hw_pose)
	{
		hires_pos_to_hw_pose(hw_pose, &cur_pos);
	}

	// The above is just pose calculation

	if(lock_processing)
		return;

	
	// The rest is movement control, processing when to start and commanding motors


//	int clockwise = 0;



	if(reverse)
		lin_left += fwd;
	else
		lin_left -= fwd;



	new_direction = 0;

	int32_t ang_err = cur_pos.ang - target_ang;
	// if rotation auto decision:	
//	clockwise = (ang_err > 0);

#ifdef LEDS_ON
	if(stop_indicators == 0 && !manual_drive)
	{
		if(ang_err > 20*ANG_1_DEG && run)
		{
			led_status(9, YELLOW, LED_MODE_FADE);
			led_status(4, YELLOW, LED_MODE_FADE);
			led_status(3, YELLOW, LED_MODE_FADE);
			led_status(2, YELLOW, LED_MODE_FADE);
		}
		else if(ang_err < -20*ANG_1_DEG && run)
		{
			led_status(1, YELLOW, LED_MODE_FADE);
			led_status(6, YELLOW, LED_MODE_FADE);
			led_status(7, YELLOW, LED_MODE_FADE);
			led_status(8, YELLOW, LED_MODE_FADE);
		}
		else if(reverse)
		{
			if(run)
			{
				led_status(4, WHITE, LED_MODE_FADE);
				led_status(5, WHITE, LED_MODE_FADE);
				led_status(6, WHITE, LED_MODE_FADE);
			}
//			led_status(9, BLACK, LED_MODE_FADE);
//			led_status(0, BLACK, LED_MODE_FADE);
//			led_status(1, BLACK, LED_MODE_FADE);
		}
		else
		{
//			led_status(4, BLACK, LED_MODE_FADE);
//			led_status(5, BLACK, LED_MODE_FADE);
//			led_status(6, BLACK, LED_MODE_FADE);
			if(run)
			{
				led_status(9, WHITE, LED_MODE_FADE);
				led_status(0, WHITE, LED_MODE_FADE);
				led_status(1, WHITE, LED_MODE_FADE);
			}
		}
	}
#endif

	// Significant wrong direction, or over 100 meters is an error.
	if(lin_left < -200LL*65536LL || lin_left > 100000LL*65536LL) error(136);


	int dbg1, dbg2, dbg3, dbg4, dbg5, dbg6;


	static int correcting_angle = 1;
	static int correcting_linear = 0;

	int rotation_happening = 0;
	if(accurot)
	{
		if(abso(ang_err) > 5*ANG_1_DEG)
		{
			rotation_happening = 1;
			correcting_linear = 0;
			dbg6 = 1;
		}
		else if((abso(ang_err) < 3*ANG_1_DEG) && lin_left > 50*65535)
		{
			correcting_linear = 1;
			dbg6 = 2;
		}
		else
			dbg6 = 3;
	}
	else
	{
		if(abso(ang_err) > 10*ANG_1_DEG)
		{
			rotation_happening = 1;
			correcting_linear = 0;
			dbg6 = 4;
		}
		else if((abso(ang_err) < 6*ANG_1_DEG) && lin_left > 50*65536)
		{
			correcting_linear = 1;
			dbg6 = 5;
		}
		else
			dbg6 = 6;
	}

	drive_is_rotating = rotation_happening;


	static double ang_speed;
	double targ_ang_speed;

	if(!correcting_angle)
		targ_ang_speed = 0.0;
	else
	{
//		if(correcting_linear) // && lin_left > 100*65536)
//			targ_ang_speed =  -0.000001000*(double)ang_err;
//		else

		targ_ang_speed =  -0.000000500*(double)ang_err;
	}


	static double lin_speed;
	double max_lin_speed_by_lin_err = 0.80*(double)(lin_left>>16);
	if(lin_left > 0)
		max_lin_speed_by_lin_err += 25;
	double max_lin_speed_by_ang_err;

	if(accurot)
	{
		if(abso(ang_err) > 5*ANG_1_DEG)
			max_lin_speed_by_ang_err = 0.0;
		else
			max_lin_speed_by_ang_err = ((double)ANG_1_DEG*100.0) / ((double)abso(ang_err)); // 10 deg error -> max lin speed 10 units.
	}
	else
	{
		if(abso(ang_err) > 15*ANG_1_DEG)
			max_lin_speed_by_ang_err = 0.0;
		else
			max_lin_speed_by_ang_err = 999.9; // ((double)ANG_1_DEG*200.0) / ((double)abso(ang_err)); // 10 deg error -> max lin speed 20 units.
	}


	if(targ_ang_speed > max_ang_speed)
		targ_ang_speed = max_ang_speed;
	else if(targ_ang_speed < -1.0*max_ang_speed)
		targ_ang_speed = -1.0*max_ang_speed;


	// If overspeeding, decelerate. If underspeeding, accelerate.
	// ang_speed = signed
	if(ang_speed > targ_ang_speed+0.10)
		ang_speed -= 0.5;
	else if(ang_speed < targ_ang_speed-0.10)
		ang_speed += 0.5;


	double decided_max_lin_speed = max_lin_speed;
	if(!correcting_linear) decided_max_lin_speed = 0.0;
	if(decided_max_lin_speed > max_lin_speed_by_ang_err) decided_max_lin_speed = max_lin_speed_by_ang_err;
	if(decided_max_lin_speed > max_lin_speed_by_lin_err) decided_max_lin_speed = max_lin_speed_by_lin_err;

	if(lin_speed > decided_max_lin_speed)
		lin_speed -= 0.40;
	else if(lin_speed < decided_max_lin_speed - 0.50)
		lin_speed += 0.40;

	if(lin_speed < 0.0) lin_speed = 0.0;


	dbg1 = targ_ang_speed;
	dbg2 = ang_speed;
	dbg3 = 0;
	dbg4 = lin_speed;
	dbg5 = correcting_linear;
	//dbg6 = 0;

	if(correcting_linear)
	{
		if(reverse)
			sensors_bwd(lin_speed);
		else
			sensors_fwd(lin_speed);
	}
	else // Rotating only
	{
		if(ang_err < -6*ANG_1_DEG)
			sensors_posang(ang_speed);
		else if(ang_err > +6*ANG_1_DEG)
			sensors_negang(ang_speed);
		else
			sensors_idle();
	}


	int ang_speed_i = (int)(ang_speed*256.0); //*(clockwise?-1:1);
	int lin_speed_i = (int)(lin_speed*256.0)*(reverse?-1:1);


	// joystick-like mode: override calculated ang_speed_i and lin_speed_i

	static int manual_drive_lin_speed;
	static int manual_drive_ang_speed;

	#define MANUAL_ACCEL 100
	#define MANUAL_FAST_LIN_SPEED 35000
	#define MANUAL_SLOW_LIN_SPEED 15000
	#define MANUAL_FAST_ANG_SPEED 15000
	#define MANUAL_SLOW_ANG_SPEED 7000


#define OBST_THRESHOLD 4
#define OBST_THRESHOLD_FAR 99999

	if(manual_drive > 0)
	{
		lin_left = 0;
		target_ang = cur_pos.ang;
		manual_drive--;

		int right = manual_drive_buttons & (1<<0);
		int left  = manual_drive_buttons & (1<<1);
		int down  = manual_drive_buttons & (1<<2);
		int up    = manual_drive_buttons & (1<<3);
		int fast  = manual_drive_buttons & (1<<4);


		if(manual_drive < MANUAL_DRIVE_SLOWDOWN_TIME)
			fast = 0;

		if(manual_drive < MANUAL_DRIVE_STOP_TIME)
			right=left=down=up=fast = 0;


		// pressing opposite directions simultaneously equals pressing neither
		if(up && down) up = down = 0;
		if(left && right) left = right = 0;


		// Give some indication about obstacles, and prevent going FAST if going to hit

		static int beep_cnt;
		beep_cnt++;
		if((!fast && (beep_cnt&255) == 255) || (fast && (beep_cnt&127) == 127))
		{
			if(up && obstacle_front_near > OBST_THRESHOLD)
			{
				fast = 0;
				beep(500, 100, 0, 70);
				stop_indicators = 125;
				#ifdef STOP_LEDS_ON
				led_status(9, RED, LED_MODE_FADE);
				led_status(0, RED, LED_MODE_FADE);
				led_status(1, RED, LED_MODE_FADE);
				#endif
			}
			else if(down && obstacle_back_near > OBST_THRESHOLD)
			{
				fast = 0;
				beep(500, 100, 0, 70);
				stop_indicators = 125;
				#ifdef STOP_LEDS_ON
				led_status(4, RED, LED_MODE_FADE);
				led_status(5, RED, LED_MODE_FADE);
				led_status(6, RED, LED_MODE_FADE);
				#endif
			}
			else if(right && obstacle_left_near > OBST_THRESHOLD)
			{
				fast = 0;
				beep(500, 100, 0, 70);
				stop_indicators = 125;
				#ifdef STOP_LEDS_ON
				led_status(2, RED, LED_MODE_FADE);
				led_status(3, RED, LED_MODE_FADE);
				#endif
			}
			else if(left && obstacle_right_near > OBST_THRESHOLD) 
			{
				fast = 0;
				beep(500, 100, 0, 70);
				stop_indicators = 125;
				#ifdef STOP_LEDS_ON
				led_status(7, RED, LED_MODE_FADE);
				led_status(8, RED, LED_MODE_FADE);
				#endif
			}
			else
			{
				beep(100, 2000, 0, 50);

				for(int i=0; i<N_SENSORS; i++)
					led_status(i, fast?MAGENTA:BLUE, LED_MODE_FADE);

			}
		}



		// Increment speeds depending on button pressed; decrement if no press
		if(up)
			manual_drive_lin_speed += MANUAL_ACCEL;
		else if(down)
			manual_drive_lin_speed -= MANUAL_ACCEL;
		else // neither is pressed - slow down towards 0
			if(manual_drive_lin_speed != 0) manual_drive_lin_speed += (manual_drive_lin_speed>0)?-MANUAL_ACCEL:MANUAL_ACCEL;

		if(left)
			manual_drive_ang_speed += MANUAL_ACCEL;
		else if(right)
			manual_drive_ang_speed -= MANUAL_ACCEL;
		else // neither is pressed - slow down towards 0
			if(manual_drive_ang_speed != 0) manual_drive_ang_speed += (manual_drive_ang_speed>0)?-MANUAL_ACCEL:MANUAL_ACCEL;

		// Ramp speeds down if overspeeding. Speedlimit depends on whether FAST is pressed
		if(manual_drive_lin_speed > (fast?MANUAL_FAST_LIN_SPEED:MANUAL_SLOW_LIN_SPEED))
			manual_drive_lin_speed -= MANUAL_ACCEL;
		else if(manual_drive_lin_speed < (fast?-MANUAL_FAST_LIN_SPEED:-MANUAL_SLOW_LIN_SPEED))
			manual_drive_lin_speed += MANUAL_ACCEL;

		if(manual_drive_ang_speed > (fast?MANUAL_FAST_ANG_SPEED:MANUAL_SLOW_ANG_SPEED))
			manual_drive_ang_speed -= MANUAL_ACCEL;
		else if(manual_drive_ang_speed < (fast?-MANUAL_FAST_ANG_SPEED:-MANUAL_SLOW_ANG_SPEED))
			manual_drive_ang_speed += MANUAL_ACCEL;

		lin_speed_i = manual_drive_lin_speed;
		ang_speed_i = manual_drive_ang_speed;
		motors_enabled = manual_drive;
	}


	if(run && !manual_drive)
	{
		int32_t ang_err_limited = ang_err;
		if(ang_err_limited > 7*ANG_1_DEG)
			ang_err_limited = 7*ANG_1_DEG;
		else if(ang_err_limited < -7*ANG_1_DEG)
			ang_err_limited = -7*ANG_1_DEG;

		gyro_to_motor_feedforward = 2*ang_err_limited/ANG_0_01_DEG;
	}


	if(run || manual_drive)
	{
		set_motor_velocities(lin_speed_i - ang_speed_i , -lin_speed_i - ang_speed_i);
	}
	else
	{
		set_motor_velocities(0,0);
	}

	
	if(abso(lin_speed_i) > 0 || abso(ang_speed_i) > 0)
		robot_moves();



	// All stopping conditions first:
	if(!motors_enabled)
	{
		stop(3);
	}
	else if(run && !manual_drive)
	{
		if((accurot && (ang_err > -3*ANG_0_5_DEG && ang_err < 3*ANG_0_5_DEG && lin_left < 20LL*65536LL)) ||
		  (!accurot && (ang_err > -4*ANG_1_DEG && ang_err < 4*ANG_1_DEG && lin_left < 20LL*65536LL)) )
		{
			if(stop_indicators == 0)
				beep(100, 2000, 0, 50);
			run = 0;

		}
	}

	int jerk0 = get_jerk_status(0);
	int jerk1 = get_jerk_status(1);

	if(jerk0 > 70 || jerk1 > 70)
	{
		beep(500, 100, 0, 70);
		stop_indicators = 125;
		#ifdef STOP_LEDS_ON
		for(int i=0; i<N_SENSORS; i++)
			led_status(i, RED, LED_MODE_FADE);
		#endif
		micronavi_status |= 1UL<<5;
		manual_drive = 0;
		stop(8);

	}

	if(!manual_drive)
	{
		if(ignore_front==0 && run && correcting_linear && !reverse && ((lin_left > 10*65536 && obstacle_front_near > OBST_THRESHOLD) || (lin_left > 300*65536 && obstacle_front_far > OBST_THRESHOLD_FAR)))
		{
			{
				beep(500, 100, 0, 70);
				stop_indicators = 125;
				#ifdef STOP_LEDS_ON
				led_status(9, RED, LED_MODE_FADE);
				led_status(0, RED, LED_MODE_FADE);
				led_status(1, RED, LED_MODE_FADE);
				#endif
			}
			micronavi_status |= 1UL<<0;
			stop(4);

		}
		else if(ignore_back==0 && run && correcting_linear && reverse && ((lin_left > 10*65536 && obstacle_back_near > OBST_THRESHOLD) || (lin_left > 300*65536 && obstacle_back_far > OBST_THRESHOLD_FAR)))
		{
			{
				beep(500, 100, 0, 70);
				stop_indicators = 125;
				#ifdef STOP_LEDS_ON
				led_status(4, RED, LED_MODE_FADE);
				led_status(5, RED, LED_MODE_FADE);
				led_status(6, RED, LED_MODE_FADE);
				#endif
			}

			micronavi_status |= 1UL<<0;
			stop(5);
		}
		else if(ignore_left==0 && run && correcting_angle && ((ang_err > 0*ANG_1_DEG && obstacle_left_very_near > OBST_THRESHOLD) || (ang_err > 10*ANG_1_DEG && obstacle_left_near > OBST_THRESHOLD) || (ang_err > 25*ANG_1_DEG && obstacle_left_far > OBST_THRESHOLD_FAR)))
		{
			{
				beep(500, 100, 0, 70);
				stop_indicators = 125;
				#ifdef STOP_LEDS_ON
				led_status(2, RED, LED_MODE_FADE);
				led_status(3, RED, LED_MODE_FADE);
				#endif
			}

			micronavi_status |= 1UL<<2;
			stop(6);
		}
		else if(ignore_right==0 && run && correcting_angle && ((ang_err < -0*ANG_1_DEG && obstacle_right_very_near > OBST_THRESHOLD) || (ang_err < -10*ANG_1_DEG && obstacle_right_near > OBST_THRESHOLD) || (ang_err < -25*ANG_1_DEG && obstacle_right_far > OBST_THRESHOLD_FAR)))
		{
			{
				beep(500, 100, 0, 70);
				stop_indicators = 125;
				#ifdef STOP_LEDS_ON
				led_status(7, RED, LED_MODE_FADE);
				led_status(8, RED, LED_MODE_FADE);
				#endif
			}
			micronavi_status |= 1UL<<2;

			stop(7);
		}

		if(ignore_left > 0) ignore_left--;
		if(ignore_right > 0) ignore_right--;
		if(ignore_front > 0) ignore_front--;
		if(ignore_back > 0) ignore_back--;
	}

	int do_start = 0;

	// Then, starting conditions:

	static int prev_manual_drive;

	if(prev_manual_drive == 0 && manual_drive > 0)
		do_start = 1;
	prev_manual_drive = manual_drive;
	if(accurot)
	{
		if(ang_err < -2*ANG_1_DEG || ang_err > 2*ANG_1_DEG || lin_left > 40LL*65536LL)
		{
			if(motors_enabled)
				do_start = 1;
		}
	}
	else
	{
		if(ang_err < -7*ANG_1_DEG || ang_err > 7*ANG_1_DEG || lin_left > 60LL*65536LL)
		{
			if(motors_enabled)
				do_start = 1;
		}
	}

	if(do_start && !run)
	{
		if(stop_indicators == 0)
			beep(150, 800, -300, 50);

		motor_torque_lim(0, 50);
		motor_torque_lim(1, 50);
		enable_motors();
		ang_speed = 0;
		lin_speed = 0;
		run = 1;
	}

	if(!run)
	{
//		motor_let_stop(0);
//		motor_let_stop(1);
	}

	static int nonrun_cnt;
	if(run)
		nonrun_cnt = 0;

	if(!run || motors_enabled < 1)
	{
		if(nonrun_cnt < 200)
		{
			nonrun_cnt++;
		}
		else
		{

			disable_motors();
		}
	}




	if(motors_enabled > 0) motors_enabled--;

	if(err_cnt > 0) err_cnt--;

	if(stop_indicators > 0) stop_indicators--;

	if(drive_diag)
	{
		drive_diag->ang_err = ang_err;
		drive_diag->lin_err = lin_left>>16;
		drive_diag->cur_x = cur_pos.x>>16;
		drive_diag->cur_y = cur_pos.y>>16;
		drive_diag->target_x = target_pos.x>>16;
		drive_diag->target_y = target_pos.y>>16;
		drive_diag->id = cur_id;
		drive_diag->remaining = lin_left>>16;
		drive_diag->micronavi_stop_flags = micronavi_status;
		drive_diag->run = run;
/*
		drive_diag->dbg1 = gyro_dc_z[0]>>15; // motors_enabled;
		drive_diag->dbg2 = gyro_dc_z[1]>>15; // mpos[0];
		drive_diag->dbg3 = gyro_dc_z[2]>>15; //= bldc_pos_set[1];
		drive_diag->dbg4 = gyro_dc_z[3]>>15; //= mpos[1];
		drive_diag->dbg5 = gyro_dc_z[4]>>15;
		drive_diag->dbg6 = gyro_dc_z[5]>>15;
*/
		drive_diag->dbg1 = dbg1;
		drive_diag->dbg2 = dbg2;
		drive_diag->dbg3 = dbg3;
		drive_diag->dbg4 = dbg4;
		drive_diag->dbg5 = dbg5;
		drive_diag->dbg6 = reverse;
		drive_diag->ang_speed_i = ang_speed_i;
		drive_diag->lin_speed_i = lin_speed_i;
	}



}

