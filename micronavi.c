#include <stdint.h>
#include "ext_include/stm32h7xx.h"
#include "stm32_cmsis_extension.h"

#include "drive.h"

#if 0


int chafind_nearest_hit_x;
int chafind_nearest_hit_y;
int chafind_left_accum, chafind_left_accum_cnt;
int chafind_right_accum, chafind_right_accum_cnt;
int chafind_middle_min_y, chafind_middle_max_y;
int chafind_middle_cnt;
int chafind_total_front_accum, chafind_total_front_accum_cnt;

#define CHAFIND_FIRST_DIST 520
#define CHAFIND_MIDDLE_BAR_DEPTH 80
#define CHAFIND_MIDDLE_BAR_WIDTH 70
#define CHAFIND_FIRST_DIST_TOLERANCE 50
#define CHAFIND_LOOK_SIDEWAY_MAX 200
#define CHAFIND_LOOK_SIDEWAY_MIN 130
#define CHAFIND_SIDE (CHAFIND_LOOK_SIDEWAY_MAX+CHAFIND_LOOK_SIDEWAY_MIN)  //  /2 * 2  // y dist between the two side points looked at

#define CHAFIND_PASS1_ACCEPT_ANGLE (2*ANG_1_DEG) 
#define CHAFIND_PASS1_ACCEPT_SHIFT 20

#define CHAFIND_PUSH_TUNE 0 // in mm, lower number = go further

#define CHAFIND_ACCEPT_MILLIVOLTS 21000 // Voltage to expect to stop the push

#define CHAFIND_AIM_Y_TUNE 10  // Positive = go more right



static void chafind_empty_accum1()
{
	chafind_nearest_hit_x = 9999;
	chafind_nearest_hit_y = 0;
}

static void chafind_empty_accum2()
{
	chafind_left_accum = chafind_left_accum_cnt = 0;
	chafind_right_accum = chafind_right_accum_cnt = 0;
	chafind_middle_min_y = 9999;
	chafind_middle_max_y = -9999;
	chafind_middle_cnt = 0;
	chafind_total_front_accum = 0;
	chafind_total_front_accum_cnt = 0;
}


void micronavi_point_in_chafind(int32_t x, int32_t y, int16_t z, int stop_if_necessary, int source)
{
	if(source != 0)
		return;

	y += CHAFIND_AIM_Y_TUNE;

//	int dist_to_hit;

//	dist_to_hit = x - robot_origin_to_front_TIGHT;

	if(y < (robot_ys_TIGHT/2+80) && y > (-1*robot_ys_TIGHT/2-80) && x > robot_origin_to_front_TIGHT+150)
	{
		// Look at area in front of the robot
		if(x < chafind_nearest_hit_x)
		{
			chafind_nearest_hit_x = x;
			chafind_nearest_hit_y = y;
		}
	}

//	dbg[1] = chafind_nearest_hit_x;
//	dbg[2] = chafind_nearest_hit_y;

	if(y < (chafind_nearest_hit_y-CHAFIND_LOOK_SIDEWAY_MIN) && y > (chafind_nearest_hit_y-CHAFIND_LOOK_SIDEWAY_MAX) &&
		x > CHAFIND_FIRST_DIST-CHAFIND_FIRST_DIST_TOLERANCE-150+CHAFIND_MIDDLE_BAR_DEPTH && x < CHAFIND_FIRST_DIST+CHAFIND_FIRST_DIST_TOLERANCE+150+CHAFIND_MIDDLE_BAR_DEPTH)
	{
		chafind_left_accum += x;
		chafind_left_accum_cnt++;
//		dbg[] = chafind_left_accum/chafind_left_accum_cnt;
//		dbg[] = chafind_left_accum_cnt;
	
	}
	else if(y > (chafind_nearest_hit_y+CHAFIND_LOOK_SIDEWAY_MIN) && y < (chafind_nearest_hit_y+CHAFIND_LOOK_SIDEWAY_MAX) &&
		x > CHAFIND_FIRST_DIST-CHAFIND_FIRST_DIST_TOLERANCE-150+CHAFIND_MIDDLE_BAR_DEPTH && x < CHAFIND_FIRST_DIST+CHAFIND_FIRST_DIST_TOLERANCE+150+CHAFIND_MIDDLE_BAR_DEPTH)
	{
		chafind_right_accum += x;
		chafind_right_accum_cnt++;
//		dbg[] = chafind_right_accum/chafind_right_accum_cnt;
//		dbg[] = chafind_right_accum_cnt;
	}

	if(x >= chafind_nearest_hit_x && x < chafind_nearest_hit_x+60 && y > chafind_nearest_hit_y-CHAFIND_MIDDLE_BAR_WIDTH-25 && y < chafind_nearest_hit_y+CHAFIND_MIDDLE_BAR_WIDTH+25)
	{
		chafind_middle_cnt++;
		if(y < chafind_middle_min_y) chafind_middle_min_y = y;
		if(y > chafind_middle_max_y) chafind_middle_max_y = y;
//		dbg[] = chafind_middle_cnt;
	}

	if(y < (robot_ys_TIGHT/2) && y > -1*(robot_ys_TIGHT/2) && x > 200 && x < 1000)
	{
		chafind_total_front_accum += x;
		chafind_total_front_accum_cnt++;
	}

}

static int chafind_calc(int* p_ang, int* p_shift, int* p_dist)
{
	int left = chafind_left_accum/chafind_left_accum_cnt;
	int right = chafind_right_accum/chafind_right_accum_cnt;
	//int avgdist = (left+right+(chafind_nearest_hit_x+CHAFIND_MIDDLE_BAR_DEPTH))/3;

	int ang = atan2(right-left, CHAFIND_SIDE)*(4294967296.0/(2.0*M_PI));
	*p_ang = ang;
//	dbg[1] = ang/ANG_0_1_DEG;

	int midmark_x = chafind_nearest_hit_x;
	int midmark_y = (chafind_middle_min_y + chafind_middle_max_y)/2;

	//dbg[] = midmark_x;
	//dbg[] = midmark_y;
	// Rotate middle mark coordinates to find required sideway drift

	int rotcos = sin_lut[(uint32_t)(ANG_90_DEG-ang)>>SIN_LUT_SHIFT];
	int rotsin = sin_lut[(uint32_t)(ang)>>SIN_LUT_SHIFT];

	int midmark_rot_x = (midmark_x * rotcos - midmark_y * rotsin)>>15;
	int midmark_rot_y = (midmark_x * rotsin + midmark_y * rotcos)>>15;

	*p_shift = midmark_rot_y;
	*p_dist = midmark_rot_x-robot_origin_to_front_TIGHT;

//	dbg[2] = *p_shift;
//	dbg[3] = *p_dist;

	return 0;
}


typedef enum {
	CHAFIND_IDLE 			= 0,
	CHAFIND_START          		= 1,
	CHAFIND_WAIT_DISTANCE		= 2,
	CHAFIND_WAIT_FWD1		= 3,
	CHAFIND_WAIT_FWD1_STOPEXTRA1	= 4,
	CHAFIND_WAIT_FWD1_STOPEXTRA2	= 5,
	CHAFIND_ACCUM_DATA		= 6,
	CHAFIND_WAIT_BACKING		= 7,
	CHAFIND_START_REBACKING		= 8,
	CHAFIND_WAIT_REBACKING		= 9,
	CHAFIND_WAIT_FWD2		= 10,
	CHAFIND_WAIT_FWD2_STOPEXTRA1	= 11,
	CHAFIND_ACCUM_FRONTAVG		= 12,
	CHAFIND_WAIT_PUSH		= 13,
	CHAFIND_SUCCESS                 = 14,
	CHAFIND_FAIL			= 99
} chafind_state_t;

chafind_state_t chafind_state = CHAFIND_IDLE;

volatile int start_charger = 0;

void micronavi_point_in(int32_t x, int32_t y, int16_t z, int stop_if_necessary, int source)
{
	if(chafind_state)
	{
//		dbg[0] = chafind_state;
		micronavi_point_in_chafind(x,y,z,stop_if_necessary, source);
	}
	else
	{
		micronavi_point_in_normal(x,y,z,stop_if_necessary);
	}
}

chafind_results_t chafind_results;
volatile int send_chafind_results;

void navig_fsm2_for_charger()
{
	static int pass;
	static int timer;
	static int store_back, store_shift_ang;
	switch(chafind_state)
	{
		case CHAFIND_START:
		{
			memset(&chafind_results, 0, sizeof(chafind_results_t));
			pass = 0;
//			dbg[1] = dbg[2] = dbg[3] = dbg[5] = dbg[5] = dbg[6] = dbg[7] = 0;
			chafind_empty_accum1();
			timer = 1000;
			chafind_state++;
		}
		break;

		case CHAFIND_WAIT_DISTANCE:
		{
			if(--timer == 0)
			{
				if(chafind_nearest_hit_x < 200 || chafind_nearest_hit_x > 1200) // todo: also check obstacles from back
				{
					chafind_state = CHAFIND_FAIL;
				}
				else
				{
					int movement = chafind_nearest_hit_x - CHAFIND_FIRST_DIST;
					if(movement < -CHAFIND_FIRST_DIST_TOLERANCE || movement > CHAFIND_FIRST_DIST_TOLERANCE)
					{
						set_top_speed_max(12);
						straight_rel(movement);
						chafind_results.first_movement_needed = movement;
						chafind_state = CHAFIND_WAIT_FWD1;
					}
					else
					{
						chafind_empty_accum2();
						chafind_results.first_movement_needed = 0;
						chafind_state = CHAFIND_ACCUM_DATA;
					}
				}
			}
		}
		break;

		case CHAFIND_WAIT_FWD1:
		{
			if(!correcting_either())
			{
				timer = 200;
				chafind_state++;
			}
		}
		break;

		case CHAFIND_WAIT_FWD1_STOPEXTRA1:
		{
			if(--timer == 0)
			{
				timer = 2700;
				chafind_empty_accum1();
				chafind_state++;
			}
		}
		break;

		case CHAFIND_WAIT_FWD1_STOPEXTRA2:
		{
			if(--timer == 0)
			{
				chafind_empty_accum2();
				chafind_state++;
			}
		}
		break;

		case CHAFIND_ACCUM_DATA:
		{
			if((pass == 0 && chafind_left_accum_cnt > 30 && chafind_right_accum_cnt > 30 && chafind_middle_cnt > 20) ||
			   (pass  > 0 && chafind_left_accum_cnt > 60 && chafind_right_accum_cnt > 60 && chafind_middle_cnt > 40) )
			{
				pass++;
				int ang, shift, dist;
				chafind_calc(&ang, &shift, &dist);

				if(ang > -1*CHAFIND_PASS1_ACCEPT_ANGLE && ang < CHAFIND_PASS1_ACCEPT_ANGLE &&
					shift > -1*CHAFIND_PASS1_ACCEPT_SHIFT && shift < CHAFIND_PASS1_ACCEPT_SHIFT)
				{
					chafind_results.accepted_pos++;

					//dbg[4] = 11111;
					//dbg[5] = dbg[6] = 0;
					set_top_speed_max(12);

					int dist2 = chafind_total_front_accum/chafind_total_front_accum_cnt;
					straight_rel(dist2-robot_origin_to_front_TIGHT-270);
					chafind_state = CHAFIND_WAIT_FWD2;
				}
				else
				{
					set_top_speed_max(15);

					allow_angular(1);
					auto_disallow(0);
					if(shift > -1*CHAFIND_PASS1_ACCEPT_SHIFT && shift < CHAFIND_PASS1_ACCEPT_SHIFT) // Only turning needed
					{
						chafind_results.turning_passes_needed++;
						//dbg[4] = 22222;
						//dbg[5] = ang/ANG_0_1_DEG;
						//dbg[6] = 0;
						rotate_rel(-1*ang);
						chafind_state = CHAFIND_WAIT_REBACKING;

					}
					else // vexling needed
					{
						chafind_results.vexling_passes_needed++;
						//dbg[4] = 33333;
						allow_straight(1);
						int shift_ang;
						if(shift > 170)       shift_ang = 30*ANG_1_DEG;
						else if(shift > 70)   shift_ang = 15*ANG_1_DEG;
						else if(shift > 0)    shift_ang =  8*ANG_1_DEG;
						else if(shift < -170) shift_ang = -30*ANG_1_DEG;
						else if(shift < -70)  shift_ang = -15*ANG_1_DEG;
						else                  shift_ang =  -8*ANG_1_DEG;

						int d = ((float)shift)/tan((float)shift_ang*(2.0*M_PI)/4294967296.0);
						rotate_rel(-1*(ang+shift_ang));
						//dbg[5] = (-1*(ang+shift_ang))/ANG_0_1_DEG;
						//dbg[6] = (-1*shift_ang)/ANG_0_1_DEG;
						//dbg[7] = d;
						straight_rel(-1*d);
						store_back = d;
						store_shift_ang = shift_ang;
						chafind_state = CHAFIND_WAIT_BACKING;
					}
				}
				
			}
			else
			{
				//dbg[4] = chafind_left_accum_cnt;
				//dbg[5] = chafind_right_accum_cnt;
				//dbg[6] = chafind_middle_cnt;
			}
	
		}
		break;

		case CHAFIND_WAIT_BACKING:
		{
			if(!correcting_either())
			{
				timer=200;
				chafind_state++;
			}
		}
		break;

		case CHAFIND_START_REBACKING:
		{
			if(--timer == 0)
			{
				rotate_rel(store_shift_ang);
				straight_rel(store_back);
				chafind_state++;
			}
		}
		break;


		case CHAFIND_WAIT_REBACKING:
		{
			if(!correcting_either())
			{
				timer = 200;
				chafind_state = CHAFIND_WAIT_FWD1_STOPEXTRA1;
			}
		}
		break;

		case CHAFIND_WAIT_FWD2:
		{
			if(!correcting_either())
			{
				timer = 200;
				chafind_state++;
			}
		}
		break;

		case CHAFIND_WAIT_FWD2_STOPEXTRA1:
		{
			if(--timer == 0)
			{
				chafind_state++;
				chafind_total_front_accum = 0;
				chafind_total_front_accum_cnt = 0;
			}
		}
		break;

		case CHAFIND_ACCUM_FRONTAVG:
		{
			/*
				Average enough samples directly from the front, to measure the distance to go. Go a bit further than that.
				This prevents excessive travel in case the charger is unpowered, or we are at the wrong place.
			*/
			if(chafind_total_front_accum_cnt > 200)
			{
				int dist = chafind_total_front_accum/chafind_total_front_accum_cnt;
				chafind_results.dist_before_push = dist;
				set_top_speed_max(10);
				straight_rel(dist-robot_origin_to_front_TIGHT-CHAFIND_PUSH_TUNE);
				chafind_state = CHAFIND_WAIT_PUSH;
			}
			else
			{
				//dbg[4] = chafind_total_front_accum_cnt;
			}
		}
		break;


		case CHAFIND_WAIT_PUSH:
		{
			if(get_cha_v() > CHAFIND_ACCEPT_MILLIVOLTS)
			{
				chafind_state = CHAFIND_SUCCESS;
			}
			if(!correcting_either())
			{
				chafind_state = CHAFIND_FAIL;
			}
		}
		break;

		case CHAFIND_FAIL:
		{
			chafind_results.result = 0;
			reset_movement();
			send_chafind_results = 1;
			chafind_state = 0;
		}
		break;

		case CHAFIND_SUCCESS:
		{
			chafind_results.result = 100;
			reset_movement();
			send_chafind_results = 1;
			chafind_state = 0;
		}
		break;

		default:
		break;
	}

	//dbg[0] = chafind_state;

}


void find_charger()
{
	accurate_turngo = 1;
	chafind_state = CHAFIND_START;
}

#endif

void micronavi_point_in_chafind(int32_t x, int32_t y, int16_t z, int stop_if_necessary, int source)
{
}

