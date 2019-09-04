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

#include "micronavi.h"



int nearest_hit_x;
int nearest_hit_y;
int nearest_hit_z;
int left_accum, left_accum_cnt;
int right_accum, right_accum_cnt;
int middle_min_y, middle_max_y;
int middle_cnt;
//int total_front_accum, total_front_accum_cnt;

static chafind_results_t results;

#define MIDDLE_BAR_DEPTH 103
#define MIDDLE_BAR_WIDTH 130
#define CHAFIND_LOOK_SIDEWAY_MAX 200
#define CHAFIND_LOOK_SIDEWAY_MIN 150


#ifdef CONTACTS_ON_BACK

	#define CHAFIND_FIRST_DIST 920
	#define CHAFIND_FIRST_DIST_TOLERANCE 50

	#define CHAFIND_PASS1_ACCEPT_ANGLE (7*ANG_1_DEG) 
	#define CHAFIND_PASS1_ACCEPT_SHIFT 40

	#define CHAFIND_PUSH_TUNE 0 // in mm, lower number = go further

	#define CHAFIND_AIM_Y_TUNE 0 // positive = aim right

#else

	#define CHAFIND_FIRST_DIST 550
	#define CHAFIND_FIRST_DIST_TOLERANCE 50

	#define CHAFIND_PASS1_ACCEPT_ANGLE (7*ANG_1_DEG) 
	#define CHAFIND_PASS1_ACCEPT_SHIFT 40

	#define CHAFIND_PUSH_TUNE 0 // in mm, lower number = go further

	#define CHAFIND_AIM_Y_TUNE 0

#endif

//#define CHAFIND_SIDE (CHAFIND_LOOK_SIDEWAY_MAX+CHAFIND_LOOK_SIDEWAY_MIN)  //  /2 * 2  // y dist between the two side points looked at
#define CHAFIND_SIDE (500) // Use a bit more than in reality to reduce corrections to avoid overcorrecting

static int accum_state = 0;
static void chafind_empty_accum1()
{
	accum_state = 0;
	nearest_hit_x = 9999;
	nearest_hit_y = 0;
	nearest_hit_z = 0;
}

static void chafind_empty_accum2()
{
	accum_state = 1;
	middle_min_y = 9999;
	middle_max_y = -9999;
	middle_cnt = 0;
//	total_front_accum = 0;
//	total_front_accum_cnt = 0;
}

static void chafind_empty_accum3()
{
	accum_state = 2;
	left_accum = left_accum_cnt = 0;
	right_accum = right_accum_cnt = 0;
}

#define ROBOT_YS_TIGHT (460)
#define ROBOT_XS_TIGHT (650)
#define ROBOT_ORIGIN_TO_FRONT_TIGHT (130)
#define ROBOT_ORIGIN_TO_BACK_TIGHT  (530)

#ifdef CONTACTS_ON_BACK
#define ORIGIN_TO_CONTACTS ROBOT_ORIGIN_TO_BACK_TIGHT
#else
#define ORIGIN_TO_CONTACTS ROBOT_ORIGIN_TO_FRONT_TIGHT
#endif


void micronavi_point_in_chafind(int32_t x, int32_t y, int16_t z, int stop_if_necessary, int source)
{
//	y += CHAFIND_AIM_Y_TUNE;

	#ifdef CONTACTS_ON_BACK
	x *= -1;
	y *= -1;
	#endif

	// Z already filtered below 290mm
	if(accum_state == 0)
	{
		if(z > 230)
			return;
		if(y < (ROBOT_YS_TIGHT/2+80) && y > (-1*ROBOT_YS_TIGHT/2-80) &&
			x > ORIGIN_TO_CONTACTS)
		{
			// Look at area in front of the robot
			if(x < nearest_hit_x)
			{
				nearest_hit_x = x;
				nearest_hit_y = y;
				nearest_hit_z = z;
			}
		}
	}
	else if(accum_state == 1)
	{
		if(z > 230)
			return;
		if(
//			(x < nearest_hit_x+60 && y > nearest_hit_y-MIDDLE_BAR_WIDTH/2 && y < nearest_hit_y+MIDDLE_BAR_WIDTH/2) ||
//			(x < nearest_hit_x+40 && y > nearest_hit_y-MIDDLE_BAR_WIDTH-10 && y < nearest_hit_y+MIDDLE_BAR_WIDTH+10) )

			(x < nearest_hit_x+50 && y > nearest_hit_y-MIDDLE_BAR_WIDTH-10 && y < nearest_hit_y+MIDDLE_BAR_WIDTH+10) )
		{
			middle_cnt++;
			if(y < middle_min_y) middle_min_y = y;
			if(y > middle_max_y) middle_max_y = y;
		}
	}
	else if(accum_state == 2)
	{

		if(x > nearest_hit_x && x < nearest_hit_x+MIDDLE_BAR_DEPTH+200)
		{
			if(y < middle_min_y-80 && y > middle_min_y-160)
			{
				right_accum += x;
				right_accum_cnt++;
			}
			else if(y > middle_max_y+80 && y < middle_max_y+160)
			{
				left_accum += x;
				left_accum_cnt++;
			}
		}
	}


/*
	if(y < (ROBOT_YS_TIGHT/2) && y > -1*(ROBOT_YS_TIGHT/2) && x > ORIGIN_TO_CONTACTS && x < ORIGIN_TO_CONTACTS+800)
	{
		total_front_accum += x;
		total_front_accum_cnt++;
	}
*/
}

static int chafind_calc(int* p_ang, int* p_shift, int* p_dist)
{
	int left = left_accum/left_accum_cnt;
	int right = right_accum/right_accum_cnt;
	//int avgdist = (left+right+(nearest_hit_x+MIDDLE_BAR_DEPTH))/3;

	int ang = atan2(left-right, CHAFIND_SIDE)*(4294967296.0/(2.0*M_PI));
	*p_ang = ang;

	results.backwall_ang = ang;

	int midmark_x = nearest_hit_x;
	int midmark_y = (middle_min_y + middle_max_y)/2;

	results.midmark_x = midmark_x;
	results.midmark_y = midmark_y;

	// Rotate middle mark coordinates to find required sideway drift

	int rotcos = lut_cos_from_u32((uint32_t)ang);
	int rotsin = lut_sin_from_u32((uint32_t)ang);

	// Rotation: xr = x*cos(a) - y*sin(a)
	//           yr = x*sin(a) + y*cos(a)

	int midmark_rot_x = (midmark_x * rotcos - midmark_y * rotsin)>>SIN_LUT_RESULT_SHIFT;
	int midmark_rot_y = (midmark_x * rotsin + midmark_y * rotcos)>>SIN_LUT_RESULT_SHIFT;

	*p_shift = midmark_rot_y;
	*p_dist = midmark_rot_x-ORIGIN_TO_CONTACTS;

	results.shift = *p_shift;
	results.dist = *p_dist;

//	dbg[2] = *p_shift;
//	dbg[3] = *p_dist;

	return 0;
}


typedef enum {
	CHAFIND_IDLE 			= 0,
	CHAFIND_START          		= 1,
	CHAFIND_WAIT_FWD1		= 2,
	CHAFIND_WAIT_FWD1_STOPEXTRA	= 3,
	CHAFIND_ACCUM_DATA		= 4,
	CHAFIND_WAIT_BACKING		= 5,
	CHAFIND_START_REBACKING		= 6,
	CHAFIND_WAIT_REBACKING		= 7,
	CHAFIND_WAIT_FWD2		= 8,
	CHAFIND_WAIT_FWD2_STOPEXTRA	= 9,
	CHAFIND_WAIT_PUSH		= 10,
	CHAFIND_SUCCESS                 = 11,
	CHAFIND_FAIL			= 12
} chafind_state_t;

chafind_state_t chafind_state = CHAFIND_IDLE;

volatile int start_charger = 0;

static int pass;


extern void request_chamount_full_images();
extern void request_chamount_mid_image();

#define CHAFIND_DO_IT


static void ignore_obst()
{
	#ifdef CONTACTS_ON_BACK
		obstacle_avoidance_ignore(2, 400);
	#else
		obstacle_avoidance_ignore(0, 400);
	#endif
}

void chamount_freerun_fsm()
{
	static int timer;
	static int store_back, store_shift_ang;
	static int final_push_store_ang;

	extern uint32_t micronavi_status;

//	if(micronavi_status)
//	{
//		chafind_state = CHAFIND_FAIL;
//	}

	static int substate;
	switch(chafind_state)
	{
		case CHAFIND_START:
		{
			// Get nearest_hit
			chafind_empty_accum1();
			request_chamount_full_images();

			if(nearest_hit_x < 200 || nearest_hit_x > 1200) // todo: also check obstacles from back
			{
				chafind_state = CHAFIND_FAIL;
			}
			else
			{
				int movement = nearest_hit_x - CHAFIND_FIRST_DIST;
				if(movement < -CHAFIND_FIRST_DIST_TOLERANCE || movement > CHAFIND_FIRST_DIST_TOLERANCE)
				{
					#ifdef CONTACTS_ON_BACK
						movement *= -1;
					#endif
					results.first_movement_needed = movement;

					uart_print_string_blocking("initial distance adjust\r\n"); 
					DBG_PR_VAR_I32(movement);

					#ifdef CHAFIND_DO_IT
						ignore_obst();
						cmd_motors(8000);
						set_top_speed_max(25);
						straight_rel(movement);
						chafind_state = CHAFIND_WAIT_FWD1;
					#else
						chafind_state = CHAFIND_SUCCESS;
					#endif

				}
				else
				{
					uart_print_string_blocking("initial distance ok\r\n"); 
					DBG_PR_VAR_I32(movement);

					chafind_empty_accum2();
					results.first_movement_needed = 0;
					chafind_state = CHAFIND_ACCUM_DATA;
					substate = 0;
				}
			}

		}
		break;

		case CHAFIND_WAIT_FWD1:
		{
			if(!is_driving())
			{
				timer = 15;
				chafind_state = CHAFIND_WAIT_FWD1_STOPEXTRA;
			}
		}
		break;

		case CHAFIND_WAIT_FWD1_STOPEXTRA:
		{
			if(--timer == 0)
			{
				chafind_state = CHAFIND_ACCUM_DATA;
				substate = 0;
			}
		}
		break;

		case CHAFIND_ACCUM_DATA:
		{
			// get nearest_hit:
			if(substate == 0)
			{
				chafind_empty_accum1();
				request_chamount_full_images();
				DBG_PR_VAR_I32(nearest_hit_x);
				DBG_PR_VAR_I32(nearest_hit_y);
				substate++;
			}
			else if(substate == 1)
			{
				// get middle tower:
				chafind_empty_accum2();
				request_chamount_full_images();
				request_chamount_full_images();
				DBG_PR_VAR_I32(middle_cnt);
				DBG_PR_VAR_I32(middle_min_y);
				DBG_PR_VAR_I32(middle_max_y);
				substate++;
			}
			else if(substate == 2)
			{
				// get left and right:
				chafind_empty_accum3();
				request_chamount_full_images();
				request_chamount_full_images();
				substate++;
			}
			else if(substate == 3)
			{
				substate++; // to set erroneous state
				request_chamount_full_images();
				request_chamount_full_images();
				DBG_PR_VAR_I32(left_accum/left_accum_cnt);
				DBG_PR_VAR_I32(left_accum_cnt);
				DBG_PR_VAR_I32(right_accum/right_accum_cnt);
				DBG_PR_VAR_I32(right_accum_cnt);


				results.left_accum_cnt = left_accum_cnt;
				results.mid_accum_cnt = middle_cnt;
				results.right_accum_cnt = right_accum_cnt;

				int middle_w = middle_max_y - middle_min_y;
				if(middle_cnt < 150 || left_accum_cnt < 400 || right_accum_cnt < 400 || middle_w < MIDDLE_BAR_WIDTH-30 || middle_w > MIDDLE_BAR_WIDTH+40)
				{
					chafind_state = CHAFIND_FAIL;
				}
				else
				{
					pass++;
					int ang, shift, dist;
					chafind_calc(&ang, &shift, &dist);

					DBG_PR_VAR_I32(ang/ANG_0_1_DEG);
					DBG_PR_VAR_I32(shift);
					DBG_PR_VAR_I32(dist);

					// dist-100, because the final angle correction is done approx. 100mm before the targer
					int32_t corr_ang_to_hit_mid = atan2(shift, (dist-100))*(4294967296.0/(2.0*M_PI));
					final_push_store_ang = -1*ang - corr_ang_to_hit_mid;

					if(ang > -1*CHAFIND_PASS1_ACCEPT_ANGLE && ang < CHAFIND_PASS1_ACCEPT_ANGLE &&
						shift > -1*CHAFIND_PASS1_ACCEPT_SHIFT && shift < CHAFIND_PASS1_ACCEPT_SHIFT &&
						corr_ang_to_hit_mid > -15*ANG_1_DEG && corr_ang_to_hit_mid < 15*ANG_1_DEG &&
						final_push_store_ang > -8*ANG_1_DEG && final_push_store_ang < 8*ANG_1_DEG)
					{
						results.accepted_pos++;
						set_top_speed_max(30);

						int dist2 = (left_accum+right_accum)/(left_accum_cnt+right_accum_cnt);

						results.dist_before_push = dist2;

						int final_movement = dist2-ORIGIN_TO_CONTACTS-CHAFIND_PUSH_TUNE;
						#ifdef CONTACTS_ON_BACK
							final_movement *= -1;
						#endif


						uart_print_string_blocking("Accepted!\r\n"); 
						DBG_PR_VAR_I32(dist2);
						DBG_PR_VAR_I32(final_movement);
						DBG_PR_VAR_I32(corr_ang_to_hit_mid/ANG_0_1_DEG);
						DBG_PR_VAR_I32(final_push_store_ang/ANG_0_1_DEG);


						#ifdef CHAFIND_DO_IT
							ignore_obst();
							cmd_motors(8000);
							rotate_and_straight_rel(corr_ang_to_hit_mid, final_movement, 1);
							chafind_state = CHAFIND_WAIT_PUSH;
						#else
							chafind_state = CHAFIND_SUCCESS;
						#endif
					}
					else
					{
						set_top_speed_max(30);

						if(shift > -1*CHAFIND_PASS1_ACCEPT_SHIFT && shift < CHAFIND_PASS1_ACCEPT_SHIFT &&
						   corr_ang_to_hit_mid > -15*ANG_1_DEG && corr_ang_to_hit_mid < 15*ANG_1_DEG &&
						   final_push_store_ang > -8*ANG_1_DEG && final_push_store_ang < 8*ANG_1_DEG)
						{
							results.turning_passes_needed++;
							uart_print_string_blocking("rotation only\r\n"); 

							#ifdef CHAFIND_DO_IT
								ignore_obst();
								rotate_rel(-1*ang, 1);
								chafind_state = CHAFIND_WAIT_REBACKING;
							#else
								chafind_state = CHAFIND_SUCCESS;
							#endif

						}
						else // vexling needed
						{
							results.vexling_passes_needed++;

							int shift_ang;
							if(shift > 170)       shift_ang = 30*ANG_1_DEG;
							else if(shift > 70)   shift_ang = 15*ANG_1_DEG;
							else if(shift > 0)    shift_ang =  8*ANG_1_DEG;
							else if(shift < -170) shift_ang = -30*ANG_1_DEG;
							else if(shift < -70)  shift_ang = -15*ANG_1_DEG;
							else                  shift_ang =  -8*ANG_1_DEG;

							int d = ((float)shift)/tan((float)shift_ang*(2.0*M_PI)/4294967296.0);

							uart_print_string_blocking("vexling needed\r\n");
							int32_t mov_ang =  
								#ifdef CONTACTS_ON_BACK
									-1*(ang+shift_ang);
								#else
									-1*(ang+shift_ang);
								#endif

							int32_t mov_d = 
								#ifdef CONTACTS_ON_BACK
									+1*d;
								#else
									-1*d;
								#endif
							DBG_PR_VAR_I32(mov_ang/ANG_0_1_DEG);
							DBG_PR_VAR_I32(mov_d);

							#ifdef CHAFIND_DO_IT
								ignore_obst();
								cmd_motors(8000);

								rotate_and_straight_rel(mov_ang, mov_d, 1);

								store_back = d;
								store_shift_ang = shift_ang;
								chafind_state = CHAFIND_WAIT_BACKING;
							#else
								chafind_state = CHAFIND_SUCCESS;
							#endif

						}
					}
					
				}
			}
			else
				error(444);	
		}
		break;

		case CHAFIND_WAIT_BACKING:
		{
			if(!is_driving())
			{
				chafind_state = CHAFIND_START_REBACKING;
			}
		}
		break;

		case CHAFIND_START_REBACKING:
		{
			cmd_motors(8000);

			uart_print_string_blocking("rebacking\r\n"); 

			#ifdef CONTACTS_ON_BACK
				rotate_and_straight_rel(+1*store_shift_ang, -1*store_back, 1);
			#else
				rotate_and_straight_rel(+1*store_shift_ang, +1*store_back, 1);
			#endif
			chafind_state = CHAFIND_WAIT_REBACKING;
		}
		break;


		case CHAFIND_WAIT_REBACKING:
		{
			if(!is_driving())
			{
				timer = 10;
				chafind_state = CHAFIND_WAIT_FWD2_STOPEXTRA;
			}
		}
		break;

		case CHAFIND_WAIT_FWD2_STOPEXTRA:
		{
			if(--timer == 0)
			{
				chafind_state = CHAFIND_START;
			}
		}
		break;

		case CHAFIND_WAIT_PUSH:
		{
			ignore_obst();

			int remai = abso(get_remaining_lin());

			//DBG_PR_VAR_I32(remai);
			if(remai < 150)
			{
				set_top_speed_max(16);

				if(final_push_store_ang != 0)
				{
					cmd_motors(5000);

					rotate_rel_on_fly(final_push_store_ang, 1);
					uart_print_string_blocking("final rota\r\n"); 
					final_push_store_ang = 0;
				}
			}
			else if(remai < 200)
			{
				set_top_speed_max(20);
			}
			else if(remai < 250)
			{
				set_top_speed_max(24);
			}

			charger_freerunning_fsm();

			static int mounted_cnt;
			if(charger_is_mounted())
			{
				mounted_cnt++;
				set_top_speed_max(5);
			}
			else
			{
				mounted_cnt = 0;
			}

			if(mounted_cnt > 2)
			{
				chafind_state = CHAFIND_SUCCESS;
				mounted_cnt = 0;
				cmd_stop_movement();
			}

			if(!is_driving())
			{
				chafind_state = CHAFIND_FAIL;
				mounted_cnt = 0;
			}
		}
		break;

		case CHAFIND_FAIL:
		{
			results.result = 99;
			cmd_stop_movement();
		}
		break;

		case CHAFIND_SUCCESS:
		{
			results.result = 100;
			cmd_stop_movement();
		}
		break;

		default:
		break;
	}

	results.cur_state = chafind_state;
	results.nearest_hit_x = nearest_hit_x;
	results.nearest_hit_y = nearest_hit_y;
	results.nearest_hit_z = nearest_hit_z;
	results.middle_bar_min_y = middle_min_y;
	results.middle_bar_max_y = middle_max_y;

	if(chafind_results)
	{
		memcpy(chafind_results, &results, sizeof(chafind_results_t));
	}

}


void find_charger()
{
	uart_print_string_blocking("find_charger()\r\n"); 

//	accurate_turngo = 1;
	memset(&results, 0, sizeof(chafind_results_t));
	pass = 0;
	chafind_state = CHAFIND_START;
}

void stop_chafind()
{
	chafind_state = 0;
}

