#pragma once
#include <stdint.h>
#include "../robotsoft/api_soft_to_board.h"


typedef struct
{
	uint32_t ang; // uint32_t range --> 0..+360 deg; let it overflow freely. 1 unit = 83.81903171539 ndeg
	int32_t pitch;
	int32_t roll;
	int64_t x;   // 1/65536 mm
	int64_t y;   // 1/65536 mm
} hires_pos_t;

void hires_pos_to_hw_pose(hw_pose_t* out, hires_pos_t* in);

extern hires_pos_t cur_pos;


void cmd_motors(int enabled_ms);
void cmd_go_to(s2b_move_abs_t* m);
void cmd_corr_pos(s2b_corr_pos_t* cmd);
void execute_corr_pos();
//void cmd_set_pose(s2b_set_pose_t* cmd);

uint32_t get_micronavi_status();


void cmd_stop_movement();


int is_driving();
void set_top_speed_max(int old_style_value);

void straight_rel(int32_t mm);
void rotate_rel(int32_t ang32, int accurate);
void rotate_rel_on_fly(int32_t ang32, int accurate);
void rotate_and_straight_rel(int32_t ang32, int32_t mm, int accurate_rotation_first);
void obstacle_avoidance_ignore(int dir, int ms);

int32_t get_remaining_lin();

int drive_is_robot_moving();

void self_calib(s2b_self_calib_request_t* cmd);

void drive_inject_gyrocal(gyro_cal_t* gc);

void drive_freerunning_fsm();

void drive_manual_drive(s2b_manual_drive_t* msg);

