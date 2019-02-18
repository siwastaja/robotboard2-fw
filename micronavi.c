#include <stdint.h>
#include <math.h>
#include <string.h>
#include "ext_include/stm32h7xx.h"
#include "stm32_cmsis_extension.h"

#include "drive.h"
#include "sin_lut.h"
#include "charger.h"
#include "tof_process.h"
#include "own_std.h"
#include "misc.h"

#define CONTACTS_ON_BACK



int nearest_hit_x;
int nearest_hit_y;
int left_accum, left_accum_cnt;
int right_accum, right_accum_cnt;
int middle_min_y, middle_max_y;
int middle_cnt;
int total_front_accum, total_front_accum_cnt;

#define MIDDLE_BAR_DEPTH 100
#define MIDDLE_BAR_WIDTH 119
#define CHAFIND_LOOK_SIDEWAY_MAX 200
#define CHAFIND_LOOK_SIDEWAY_MIN 150


#ifdef CONTACTS_ON_BACK

	#define CHAFIND_FIRST_DIST 850
	#define CHAFIND_FIRST_DIST_TOLERANCE 35

	#define CHAFIND_PASS1_ACCEPT_ANGLE (2*ANG_1_DEG) 
	#define CHAFIND_PASS1_ACCEPT_SHIFT 25

	#define CHAFIND_PUSH_TUNE 0 // in mm, lower number = go further

	#define CHAFIND_AIM_Y_TUNE 0

#else

	#define CHAFIND_FIRST_DIST 520
	#define CHAFIND_FIRST_DIST_TOLERANCE 100

	#define CHAFIND_PASS1_ACCEPT_ANGLE (2*ANG_1_DEG) 
	#define CHAFIND_PASS1_ACCEPT_SHIFT 25

	#define CHAFIND_PUSH_TUNE 0 // in mm, lower number = go further

	#define CHAFIND_AIM_Y_TUNE 0

#endif

#define CHAFIND_SIDE (CHAFIND_LOOK_SIDEWAY_MAX+CHAFIND_LOOK_SIDEWAY_MIN)  //  /2 * 2  // y dist between the two side points looked at

static int accum_state = 0;
static void chafind_empty_accum1()
{
	nearest_hit_x = 9999;
	nearest_hit_y = 0;
	accum_state = 0;
}

static void chafind_empty_accum2()
{
	accum_state = 1;
	left_accum = left_accum_cnt = 0;
	right_accum = right_accum_cnt = 0;
	middle_min_y = 9999;
	middle_max_y = -9999;
	middle_cnt = 0;
	total_front_accum = 0;
	total_front_accum_cnt = 0;
}

#define ROBOT_YS_TIGHT (460)
#define ROBOT_XS_TIGHT (650)
#define ROBOT_ORIGIN_TO_FRONT_TIGHT (142)
#define ROBOT_ORIGIN_TO_BACK_TIGHT  (650-142)

#ifdef CONTACTS_ON_BACK
#define ORIGIN_TO_CONTACTS ROBOT_ORIGIN_TO_BACK_TIGHT
#else
#define ORIGIN_TO_CONTACTS ROBOT_ORIGIN_TO_FRONT_TIGHT
#endif


void micronavi_point_in_chafind(int32_t x, int32_t y, int16_t z, int stop_if_necessary, int source)
{
	if(source != 0)
		return;

	y += CHAFIND_AIM_Y_TUNE;

//	int dist_to_hit;

	#ifdef CONTACTS_ON_BACK
	x *= -1;
	y *= -1;
	#endif

//	dist_to_hit = x - ORIGIN_TO_CONTACTS;

		// && x > ORIGIN_TO_CONTACTS+150)

	if(accum_state == 0)
	{
		if(z > 150 && z < 190 && y < (ROBOT_YS_TIGHT/2+80) && y > (-1*ROBOT_YS_TIGHT/2-80) &&
			x > ORIGIN_TO_CONTACTS)
		{
			// Look at area in front of the robot
			if(x < nearest_hit_x)
			{
				nearest_hit_x = x;
				nearest_hit_y = y;
			}
		}
	}

//	DBG_PR_VAR_I32(nearest_hit_x);

//	dbg[1] = nearest_hit_x;
//	dbg[2] = nearest_hit_y;


	if(z > 140 && z < 210 && x >= nearest_hit_x && x < nearest_hit_x+70 && y > nearest_hit_y-MIDDLE_BAR_WIDTH-25 && y < nearest_hit_y+MIDDLE_BAR_WIDTH+25)
	{
		middle_cnt++;
		if(y < middle_min_y) middle_min_y = y;
		if(y > middle_max_y) middle_max_y = y;
//		dbg[] = middle_cnt;
	}

	int midmark_y = (middle_min_y + middle_max_y)/2;

	if(z > 140 && z < 210 && y < (midmark_y-CHAFIND_LOOK_SIDEWAY_MIN) && y > (midmark_y-CHAFIND_LOOK_SIDEWAY_MAX) &&
		x > CHAFIND_FIRST_DIST-CHAFIND_FIRST_DIST_TOLERANCE-200+MIDDLE_BAR_DEPTH && x < CHAFIND_FIRST_DIST+CHAFIND_FIRST_DIST_TOLERANCE+200+MIDDLE_BAR_DEPTH)
	{
		left_accum += x;
		left_accum_cnt++;
//		dbg[] = left_accum/left_accum_cnt;
//		dbg[] = left_accum_cnt;
	
	}
	else if(z > 150 && z < 200 && y > (midmark_y+CHAFIND_LOOK_SIDEWAY_MIN) && y < (midmark_y+CHAFIND_LOOK_SIDEWAY_MAX) &&
		x > CHAFIND_FIRST_DIST-CHAFIND_FIRST_DIST_TOLERANCE-200+MIDDLE_BAR_DEPTH && x < CHAFIND_FIRST_DIST+CHAFIND_FIRST_DIST_TOLERANCE+200+MIDDLE_BAR_DEPTH)
	{
		right_accum += x;
		right_accum_cnt++;
//		dbg[] = right_accum/right_accum_cnt;
//		dbg[] = right_accum_cnt;
	}

	if(z > 150 && z < 200 && y < (ROBOT_YS_TIGHT/2) && y > -1*(ROBOT_YS_TIGHT/2) && x > 200 && x < 1000)
	{
		total_front_accum += x;
		total_front_accum_cnt++;
	}

}

static int chafind_calc(int* p_ang, int* p_shift, int* p_dist)
{
	int left = left_accum/left_accum_cnt;
	int right = right_accum/right_accum_cnt;
	//int avgdist = (left+right+(nearest_hit_x+MIDDLE_BAR_DEPTH))/3;

	int ang = atan2(right-left, CHAFIND_SIDE)*(4294967296.0/(2.0*M_PI));
	*p_ang = ang;

	if(chafind_results) chafind_results->backwall_ang = ang;

	int midmark_x = nearest_hit_x;
	int midmark_y = (middle_min_y + middle_max_y)/2;

	if(chafind_results) chafind_results->midmark_x = midmark_x;
	if(chafind_results) chafind_results->midmark_y = midmark_y;

	// Rotate middle mark coordinates to find required sideway drift

	int rotcos = lut_cos_from_u32((uint32_t)ang);
	int rotsin = lut_sin_from_u32((uint32_t)ang);


	// Rotation: xr = x*cos(a) + y*sin(a)
	//           yr = -x*sin(a) + y*cos(a)
	// It seems to me this widely touted formula has inverted y axis, don't understand why, so it should be:
	// Rotation: xr = x*cos(a) - y*sin(a)
	//           yr = x*sin(a) + y*cos(a)

	int midmark_rot_x = (midmark_x * rotcos - midmark_y * rotsin)>>SIN_LUT_RESULT_SHIFT;
	int midmark_rot_y = (midmark_x * rotsin + midmark_y * rotcos)>>SIN_LUT_RESULT_SHIFT;

	*p_shift = midmark_rot_y;
	*p_dist = midmark_rot_x-ORIGIN_TO_CONTACTS;

	if(chafind_results) chafind_results->shift = *p_shift;
	if(chafind_results) chafind_results->dist = *p_dist;

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
	CHAFIND_FAIL			= 15
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

	}
}

void navig_fsm2_for_charger()
{
	static int pass;
	static int timer;
	static int store_back, store_shift_ang;

	extern uint32_t micronavi_status;

	if(micronavi_status)
	{
		chafind_state = CHAFIND_FAIL;
	}

	if(chafind_state)
	{
		#ifdef CONTACTS_ON_BACK
			obstacle_avoidance_ignore(2, 200);
		#else
			obstacle_avoidance_ignore(0, 200);
		#endif

	}

	switch(chafind_state)
	{
		case CHAFIND_START:
		{
			if(chafind_results) memset(chafind_results, 0, sizeof(chafind_results_t));
			pass = 0;
//			dbg[1] = dbg[2] = dbg[3] = dbg[5] = dbg[5] = dbg[6] = dbg[7] = 0;
			chafind_empty_accum1();
			timer = 30;
			chafind_state++;
		}
		break;

		case CHAFIND_WAIT_DISTANCE:
		{
			if(--timer == 0)
			{
				if(nearest_hit_x < 200 || nearest_hit_x > 1200) // todo: also check obstacles from back
				{
					chafind_state = CHAFIND_FAIL;
				}
				else
				{
					int movement = nearest_hit_x - CHAFIND_FIRST_DIST;
					if(movement < -CHAFIND_FIRST_DIST_TOLERANCE || movement > CHAFIND_FIRST_DIST_TOLERANCE)
					{
						set_top_speed_max(25);
						#ifdef CONTACTS_ON_BACK
							movement *= -1;
						#endif
						cmd_motors(5000);
						straight_rel(movement);
						if(chafind_results) chafind_results->first_movement_needed = movement;
						chafind_state = CHAFIND_WAIT_FWD1;
					}
					else
					{
						chafind_empty_accum2();
						if(chafind_results) chafind_results->first_movement_needed = 0;
						chafind_state = CHAFIND_ACCUM_DATA;
					}
				}
			}
		}
		break;

		case CHAFIND_WAIT_FWD1:
		{
			if(!is_driving())
			{
				timer = 10;
				chafind_state++;
			}
		}
		break;

		case CHAFIND_WAIT_FWD1_STOPEXTRA1:
		{
			if(--timer == 0)
			{
				timer = 20;
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
			if(chafind_results) chafind_results->left_accum_cnt = left_accum_cnt;
			if(chafind_results) chafind_results->mid_accum_cnt = middle_cnt;
			if(chafind_results) chafind_results->right_accum_cnt = right_accum_cnt;

			if(left_accum_cnt > 200 && right_accum_cnt > 200 && middle_cnt > 25)
			{
				pass++;
				int ang, shift, dist;
				chafind_calc(&ang, &shift, &dist);

				if(ang > -1*CHAFIND_PASS1_ACCEPT_ANGLE && ang < CHAFIND_PASS1_ACCEPT_ANGLE &&
					shift > -1*CHAFIND_PASS1_ACCEPT_SHIFT && shift < CHAFIND_PASS1_ACCEPT_SHIFT)
				{
					if(chafind_results) chafind_results->accepted_pos++;

					//dbg[4] = 11111;
					//dbg[5] = dbg[6] = 0;
					set_top_speed_max(23);

					int dist2 = total_front_accum/total_front_accum_cnt;
					

					if(chafind_results)
						chafind_results->dist_before_push = dist2;
					int final_movement = dist2-ORIGIN_TO_CONTACTS-CHAFIND_PUSH_TUNE;
					#ifdef CONTACTS_ON_BACK
						final_movement *= -1;
					#else
					#endif
					cmd_motors(5000);
					straight_rel(final_movement);
					chafind_state = CHAFIND_WAIT_PUSH;

				}
				else
				{
					set_top_speed_max(30);

					//allow_angular(1);
					//auto_disallow(0);
					if(shift > -1*CHAFIND_PASS1_ACCEPT_SHIFT && shift < CHAFIND_PASS1_ACCEPT_SHIFT) // Only turning needed
					{
						if(chafind_results)
							chafind_results->turning_passes_needed++;
						//dbg[4] = 22222;
						//dbg[5] = ang/ANG_0_1_DEG;
						//dbg[6] = 0;
						rotate_rel(-1*ang);
						chafind_state = CHAFIND_WAIT_REBACKING;

					}
					else // vexling needed
					{
						if(chafind_results) chafind_results->vexling_passes_needed++;
						//dbg[4] = 33333;
						//allow_straight(1);
						int shift_ang;
						if(shift > 170)       shift_ang = 30*ANG_1_DEG;
						else if(shift > 70)   shift_ang = 15*ANG_1_DEG;
						else if(shift > 0)    shift_ang =  8*ANG_1_DEG;
						else if(shift < -170) shift_ang = -30*ANG_1_DEG;
						else if(shift < -70)  shift_ang = -15*ANG_1_DEG;
						else                  shift_ang =  -8*ANG_1_DEG;

						int d = ((float)shift)/tan((float)shift_ang*(2.0*M_PI)/4294967296.0);
						cmd_motors(8000);

						#ifdef CONTACTS_ON_BACK
							rotate_and_straight_rel(-1*(ang+shift_ang), +1*d, 1);
						#else
							rotate_and_straight_rel(+1*(ang-shift_ang), -1*d, 1);
						#endif

						store_back = d;
						store_shift_ang = shift_ang;
						chafind_state = CHAFIND_WAIT_BACKING;
					}
				}
				
			}
			else
			{
			}
	
		}
		break;

		case CHAFIND_WAIT_BACKING:
		{
			if(!is_driving())
			{
				timer=20;
				chafind_state++;
			}
		}
		break;

		case CHAFIND_START_REBACKING:
		{
			if(--timer == 0)
			{
				cmd_motors(8000);

				#ifdef CONTACTS_ON_BACK
					rotate_and_straight_rel(+1*store_shift_ang, -1*store_back, 1);
				#else
					rotate_and_straight_rel(-1*store_shift_ang, +1*store_back, 1);
				#endif
				chafind_state++;
			}
		}
		break;


		case CHAFIND_WAIT_REBACKING:
		{
			if(!is_driving())
			{
				timer = 20;
				chafind_state = CHAFIND_WAIT_FWD2_STOPEXTRA1;
			}
		}
		break;

		case CHAFIND_WAIT_FWD2:
		{
			error(555);
			if(!is_driving())
			{
				timer = 20;
				chafind_state++;
			}
		}
		break;

		case CHAFIND_WAIT_FWD2_STOPEXTRA1:
		{
			if(--timer == 0)
			{
				chafind_state = CHAFIND_START;
			}
		}
		break;

		case CHAFIND_ACCUM_FRONTAVG:
		{
			error(555);

			/*
				Average enough samples directly from the front, to measure the distance to go. Only try to go a tiny bit further than that.
				This prevents excessive travel in case the charger is unpowered, or we are at a wrong place despite matching success.
			*/
			if(total_front_accum_cnt > 200)
			{
				int dist = total_front_accum/total_front_accum_cnt;
				if(chafind_results)
					chafind_results->dist_before_push = dist;
				set_top_speed_max(18);
				int final_movement = dist-ORIGIN_TO_CONTACTS-CHAFIND_PUSH_TUNE;
				#ifdef CONTACTS_ON_BACK
					final_movement *= -1;
				#else
				#endif
				cmd_motors(5000);

				straight_rel(final_movement);
				chafind_state = CHAFIND_WAIT_PUSH;
			}
			else
			{
				//dbg[4] = total_front_accum_cnt;
			}
		}
		break;


		case CHAFIND_WAIT_PUSH:
		{
			if(abso(get_remaining_lin()) < 200)
			{
				set_top_speed_max(16);
			}
			else if(abso(get_remaining_lin()) < 120)
			{
				set_top_speed_max(13);
			}
			if(charger_is_mounted())
			{
				chafind_state = CHAFIND_SUCCESS;
				cmd_stop_movement();
			}
			if(!is_driving())
			{
				chafind_state = CHAFIND_FAIL;
			}
		}
		break;

		case CHAFIND_FAIL:
		{
			if(chafind_results)
				chafind_results->result = 99;
			cmd_stop_movement();
			chafind_state = 0;
			tof_disable_chafind_datapoints();
		}
		break;

		case CHAFIND_SUCCESS:
		{
			if(chafind_results)
				chafind_results->result = 100;
			cmd_stop_movement();
			chafind_state = 0;
			tof_disable_chafind_datapoints();
		}
		break;

		default:
		break;
	}

	if(chafind_results)
	{
		chafind_results->cur_state = chafind_state;
		chafind_results->nearest_hit_x = nearest_hit_x;
		chafind_results->nearest_hit_y = nearest_hit_y;
		chafind_results->middle_bar_min_y = middle_min_y;
		chafind_results->middle_bar_max_y = middle_max_y;
	}

}


void micronavi_fsm()
{
	if(chafind_state)
		navig_fsm2_for_charger();
}

void find_charger()
{
	uart_print_string_blocking("find_charger()\r\n"); 

//	accurate_turngo = 1;
	chafind_state = CHAFIND_START;
	tof_enable_chafind_datapoints();
}


