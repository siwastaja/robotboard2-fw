#pragma once
#include <stdint.h>
#include "../robotsoft/api_soft_to_board.h"

#define ANG_180_DEG 2147483648UL
#define ANG_90_DEG  1073741824
#define ANG_2_5_DEG   29826162
#define ANG_1_DEG     11930465
#define ANG_0_5_DEG    5965232
#define ANG_0_25_DEG   2982616
#define ANG_0_125_DEG  1491308
#define ANG_0_1_DEG    1193047
#define ANG_0_05_DEG    596523
#define ANG_0_01_DEG    119305
#define ANG_0_001_DEG    11930

#define ANG_1PER16_DEG  745654  // cumulated full circle rounding error: 0.000006%


typedef struct
{
	uint32_t ang; // uint32_t range --> 0..+360 deg; let it overflow freely. 1 unit = 83.81903171539 ndeg
	int32_t pitch;
	int32_t roll;
	int64_t x;   // 1/65536 mm
	int64_t y;   // 1/65536 mm
} hires_pos_t;

extern hires_pos_t cur_pos;


void cmd_motors(int enabled_ms);
void cmd_go_to(s2b_move_abs_t* m);


void cmd_corr_pos(s2b_corr_pos_t* cmd);
void execute_corr_pos();


void cmd_stop_movement();


int is_driving();
void set_top_speed_max(int old_style_value);

void straight_rel(int32_t mm);
void rotate_rel(int32_t ang32);
void rotate_and_straight_rel(int32_t ang32, int32_t mm, int accurate_rotation_first);
void obstacle_avoidance_ignore(int dir, int ms);

int32_t get_remaining_lin();


