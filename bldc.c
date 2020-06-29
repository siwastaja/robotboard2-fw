#include <stdint.h>
#include "ext_include/stm32h7xx.h"
#include "own_std.h"
#include "stm32_cmsis_extension.h"
#include "misc.h"
#include "adcs.h"
#include "bldc.h"
#include "sin_lut.h"
#include "../robotsoft/datatypes.h" // ANG_xx_DEG

#define PWM_MAX 8192
#define PWM_MID (PWM_MAX/2)
#define MIN_FREQ 1*65536
#define MAX_FREQ 100*65536

// Gates enabled, or just freewheeling.
// Same mappings for REV2A,B
#define MC0_EN_GATE()  HI(GPIOG,8)
#define MC0_DIS_GATE() LO(GPIOG,8)

#define MC1_EN_GATE()  HI(GPIOI,4)
#define MC1_DIS_GATE() LO(GPIOI,4)

// Read hall sensor signals HALLC,HALLB,HALLA and concatenate to a 3-bit value, to index a table
// Same mappings for REV2A,B
#define MC0_HALL_CBA() ( ((GPIOD->IDR & (0b11<<14))>>14) | (GPIOG->IDR & (1<<2)) )
#define MC1_HALL_CBA() ( ((GPIOE->IDR & (0b11<<5))>>5)   | ((GPIOI->IDR & (1<<8))>>6) )

#define DLED_ON() {}


volatile int timing_shift = 0; //5800*65536; // hall sensors are not exactly where you could think they are.

volatile int precise_curr_lim; // if exceeded, the duty cycle multiplier is kept (not increased like normally)
volatile int higher_curr_lim;  // if exceeded, the duty cycle multiplier is lowered
volatile int highest_curr_lim;

/*
	three adjacent hall io pins (active low) form a 3-bit number which can be used to index hall_loc table,
	to determine the 6-step position of the rotor.
*/
const int hall_loc[8] =
{
 -1, // 000  ERROR
 5, // 001
 3, // 010
 4, // 011
 1, // 100
 0, // 101  Middle hall active - we'll call this zero position
 2, // 110
 -1  // 111  ERROR
};
/*
{
 -1, // 000  ERROR
 1, // 001
 3, // 010
 2, // 011
 5, // 100
 0, // 101  Middle hall active - we'll call this zero position
 4, // 110
 -1  // 111  ERROR
};
*/
#define PH120SHIFT (1431655765UL)  // 120 deg shift between the phases in the 32-bit range.
#define PH90SHIFT (1073741824UL)   // 90 deg shift between the phases in the 32-bit range.
#define PH60SHIFT (715827882UL) 
#define PH45SHIFT (PH90SHIFT/2)
#define PH30SHIFT (357913941UL) 
#define PH1SHIFT   (11930465UL)
#define PH01SHIFT   (1193046UL)
#define PH001SHIFT   (119305UL)

const int base_hall_aims[6] =
{
	0,
	PH120SHIFT/2,
	PH120SHIFT,
	PH120SHIFT+PH120SHIFT/2,
	PH120SHIFT*2,
	PH120SHIFT*2+PH120SHIFT/2
};

/*
The sine table is indexed with 8 MSBs of uint32; this way, a huge resolution is implemented for the frequency,
since the frequency is a term added to the indexing each round.

Sine table is 256 long. It is pointed to by the MSByte of uint32_t which rolls over, for
good resolution of the fundamental frequency.

*/

const int sine[256] =
{
0,804,1607,2410,3211,4011,4807,5601,6392,7179,7961,8739,9511,10278,11038,11792,
12539,13278,14009,14732,15446,16150,16845,17530,18204,18867,19519,20159,20787,21402,22004,22594,
23169,23731,24278,24811,25329,25831,26318,26789,27244,27683,28105,28510,28897,29268,29621,29955,
30272,30571,30851,31113,31356,31580,31785,31970,32137,32284,32412,32520,32609,32678,32727,32757,
32767,32757,32727,32678,32609,32520,32412,32284,32137,31970,31785,31580,31356,31113,30851,30571,
30272,29955,29621,29268,28897,28510,28105,27683,27244,26789,26318,25831,25329,24811,24278,23731,
23169,22594,22004,21402,20787,20159,19519,18867,18204,17530,16845,16150,15446,14732,14009,13278,
12539,11792,11038,10278,9511,8739,7961,7179,6392,5601,4807,4011,3211,2410,1607,804,
0,-804,-1607,-2410,-3211,-4011,-4807,-5601,-6392,-7179,-7961,-8739,-9511,-10278,-11038,-11792,
-12539,-13278,-14009,-14732,-15446,-16150,-16845,-17530,-18204,-18867,-19519,-20159,-20787,-21402,-22004,-22594,
-23169,-23731,-24278,-24811,-25329,-25831,-26318,-26789,-27244,-27683,-28105,-28510,-28897,-29268,-29621,-29955,
-30272,-30571,-30851,-31113,-31356,-31580,-31785,-31970,-32137,-32284,-32412,-32520,-32609,-32678,-32727,-32757,
-32767,-32757,-32727,-32678,-32609,-32520,-32412,-32284,-32137,-31970,-31785,-31580,-31356,-31113,-30851,-30571,
-30272,-29955,-29621,-29268,-28897,-28510,-28105,-27683,-27244,-26789,-26318,-25831,-25329,-24811,-24278,-23731,
-23169,-22594,-22004,-21402,-20787,-20159,-19519,-18867,-18204,-17530,-16845,-16150,-15446,-14732,-14009,-13278,
-12539,-11792,-11038,-10278,-9511,-8739,-7961,-7179,-6392,-5601,-4807,-4011,-3211,-2410,-1607,-804
};



void set_curr_lim(int ma)
{
	static int prev_val = 9999999;

	if(ma == prev_val)
		return;

	prev_val = ma;

	if(ma < 0 || ma > 25000)
		error(124);

	int lim = (7*ma)/8+200;
	int hilim = (6*lim)/5+500;
	int highestlim = (3*lim)/2+2000;

	if(lim > 30000) lim = 30000;
	if(hilim > 30000) hilim = 30000;
	if(highestlim > 30000) highestlim = 30000;

	DIS_IRQ();
	precise_curr_lim = lim;
	higher_curr_lim = hilim;
	highest_curr_lim = highestlim;
	ENA_IRQ();
}


static uint32_t pid_i_max = 3000*256;
static int32_t pid_p = 60;
static int32_t pid_i = 250;
static int32_t pid_i_threshold = (3*SUBSTEPS)/2;
static int32_t pid_d = 0;


volatile int32_t currlim_mult_flt;
volatile int32_t curr_info_ma[2];

#define ADC_MID 8192
#define ADC_CURR_SAFETY_LIMIT 4000  // 7500

#if ADC_CURR_SAFETY_LIMIT > (ADC_MID-2)
#error Recheck ADC_CURR_SAFETY_LIMIT
#endif

// Rshunt=1mOhm -> 1mA ~ 1uV. Gain = 50 -> 1mA ~ 50uV
// 16384 ~ 3.3V -> 201.416 uV/LSB
// --> multiplier = 4.0283
#define MULT_ADC_TO_MA  4

#define MAX_MULT (200*256)
#define MIN_MULT (-200*256)
#define MIN_MULT_OFFSET 8 // no point in outputting extremely low amplitudes
#define CURRLIM_RATE1  200
#define CURRLIM_RATE2    2

#define ERR_SAT (10*SUBSTEPS)

#define P_SHIFT 4
#define I_SHIFT 12
#define D_SHIFT 4

uint32_t bldc_pos_set[2];
uint32_t bldc_pos[2];

volatile int dbg_mult;
volatile int32_t pid_int_info;

int run[2] = {0};
int wanna_stop[2] = {0};

#if 0
// Old test code
volatile int go;
volatile int gospeed = 12;
void bldc_1khz()
{
	static int cnt = 0;
	if(++cnt > gospeed)
	{
		cnt = 0;

		if(go == 1)
		{
			bldc_pos_set[0] += 32;
			bldc_pos_set[1] += 32;
		}
		else if(go == 2)
		{
			bldc_pos_set[0] -= 32;
			bldc_pos_set[1] -= 32;
		}
	}
}
#endif

//#define STOP_LEN 6000 // approx half a second.
#define STOP_LEN 9000
#define STOP_THRESHOLD (SUBSTEPS*3/2)

volatile int dbg_err0, dbg_err1, dbg_sin_mult, dbg_m;

#define TRACE_LEN 2048
int trace_at = -1;

typedef struct __attribute__((packed))
{
	uint8_t hall;
	uint8_t rotor_ang;
	int16_t ia;
	int16_t ib;
	int16_t ic;
	int16_t ialpha;
	int16_t ibeta;
	int16_t id;
	int16_t sp_id;
	int16_t iq;
	int16_t sp_iq;
	int16_t cmd_alpha;
	int16_t cmd_beta;
	uint16_t pwma;
	uint16_t pwmb;
	uint16_t pwmc;
	int32_t errint_id;
	int32_t errint_iq;
} trace_elem_t;

volatile trace_elem_t trace[TRACE_LEN];


volatile int latest_ia, latest_ib, latest_ic;
volatile int max_ia, max_ib, max_ic;
volatile int min_ia, min_ib, min_ic;
//volatile int sp_ib, sp_ic;

volatile int dbg_pwma, dbg_pwmb, dbg_pwmc;

volatile int cc_p = 24000;
volatile int cc_i = 1200;
volatile int powder;
static const int powders[10] = {0, -2200, -1600, -1000, -500, 0, 500, 1000, 1600, 2200};

volatile int velo_p = 0;
volatile int velo_i = 0;
volatile int velo_d = 0;

volatile int motor_enabled[2];
volatile int sp_velo[2];
volatile int dbg_velo;

volatile int enable_interp;


volatile int mode;

volatile int max_errint;
static void calc_max_errint()
{
	#define MAX_PWM_EXCURSION_BY_I 5000
	if(cc_i == 0)
		max_errint = 0;
	else
		max_errint = 65536/cc_i * MAX_PWM_EXCURSION_BY_I;
}

volatile int max_errint_v;
static void calc_max_errint_v()
{
	#define MAX_CURR_COMMAND_BY_I 2000
	if(velo_i == 0)
		max_errint_v = 0;
	else
		max_errint_v = 65536/velo_i * MAX_CURR_COMMAND_BY_I;
}


// To help tuning PI
volatile int32_t avgd_err_id;
volatile int32_t avgd_err_iq;

volatile uint32_t dbg_rotor_ang;
volatile int dbg_rotcos, dbg_rotsin;

volatile int stop_state_dbg[2];

volatile int dbg_score;

void ITCM bldc_handler(int mc, int ib, int ic, int hall)
{
	static int illegal_hall_err_cnt; // Increased when hall value is either illegal, or a step is skipped

	static uint32_t cur_time[2];
	static uint32_t time_last_expected_edge[2];

	static int32_t prev_sp_v[2];

	static int32_t interp_ang_per_timeunit[2];
	static uint32_t interp_time_left[2];

	static uint32_t rotor_ang[2]; // Estimate of the rotor angle, interpolated based on hall-sensor inputs

	static int32_t velo[2];

	static int32_t errint_id[2], errint_iq[2];


	cur_time[mc]++;

	int32_t ia = -ib - ic;

	int32_t sp_iq, sp_id;

	sp_id = 0;
	sp_iq = 0;


	int32_t sp_v = sp_velo[mc]; // read the volatile var once here
	int32_t sp_acc = sp_v - prev_sp_v[mc];
	prev_sp_v[mc] = sp_v;


	// positive velocity setpoint = negative iq = hall,rotor_ang values increase

	static int prev_hall[2] = {-1, -1};

	/*
		When hall == expected_hall, the current hall position has JUST increased
		to the correct direction (defined by the direction of velocity setpoint).

		This means:
			If velocity > 0:
				Rotor is at current_hall_angle - 30 deg
				Rotor is turning towards current_hall_angle + 30 deg

			If velocity < 0:
				Rotor is at current_hall_angle + 30 deg
				Rotor is turning towards current_hall_angle - 30 deg

			-> action: if possible, interpolate between these. If interpolation
			is uncertain, use fixed current_hall_angle, because error is then guaranteed to be
			30 deg max.
		

		When hall == unexpected_hall, the current hall position has JUST decreased
		to the opposite direction (from what is defined by the direction of velocity setpoint).

		This means:
			If velocity > 0:
				Rotor is at current_hall_angle + 30 deg
				Rotor is turning towards current_hall_angle + 60 + 30 deg

				-> action: use fixed current_hall_angle + 30 deg as the estimate (instead of current_hall_angle),
					more torque is generated to the right direction to take the rotor back to the
					expected direction

			If velocity < 0:
				Rotor is at current_hall_angle - 30 deg
				Rotor is turning towards current_hall_angle - 60 - 30 deg

				-> action: use fixed current_hall_angle - 30 deg as the estimate (instead of current_hall_angle),
					more torque is generated to the right direction to take the rotor back to the
					expected direction


		Consider 6-step resolution (60 deg step), directly from hall. Whenever the rotor is
		rotating in the expected direction, the rotor angle estimation accuracy swings between
		-30 and +30 deg, around zero error. As a result, the quadrature current vector swings between
		60 and 120 degrees.


		

	*/

	int hall_forward  = prev_hall[mc] + 1; if(hall_forward > 5) hall_forward = 0;
	int hall_backward = prev_hall[mc] - 1; if(hall_backward < 0) hall_backward = 5;

	int expected_hall;
	int unexpected_hall;
	if(sp_v >= 0)
	{
		expected_hall = hall_forward;
		unexpected_hall = hall_backward;
	}
	else
	{
		expected_hall = hall_backward;
		unexpected_hall = hall_forward;
	}


	// Velocity calculation
	// Note: in absence of hall pulses, the velocity number needs to start ramping down, obviously
	// (i.e., it can't only be calculated at the pulse edge.)

	static uint32_t time_last_velo[2];
	uint32_t dt_for_velo = cur_time[mc] - time_last_velo[mc];
	static int prev_dir[2];

	if(prev_hall[mc] == -1)
	{
		velo[mc] = 0;
	}
	else if(hall == prev_hall[mc])
	{
		int32_t velo_would_be = (prev_dir[mc]?-65536:+65536) / (int32_t)dt_for_velo;

		if(abso(velo_would_be) < abso(velo[mc]))
		{
			velo[mc] = velo_would_be;
		}
	}
	else if(hall == hall_forward)
	{
		time_last_velo[mc] = cur_time[mc];

		if(prev_dir[mc]) // previously backward, now forward - speed really is 0
			velo[mc] = 0;
		else
			velo[mc] = +65536/(int32_t)dt_for_velo; // really going forward

		prev_dir[mc] = 0;

	}
	else if(hall == hall_backward)
	{
		time_last_velo[mc] = cur_time[mc];

		if(!prev_dir[mc]) // previously forward, now backward - speed really is 0
			velo[mc] = 0;
		else
			velo[mc] = -65536/(int32_t)dt_for_velo; // really going backward

		prev_dir[mc] = 1;

	}


	if(mc == 0)
		dbg_velo = velo[mc];

	int pwma, pwmb, pwmc;

//	static int prev_motor_enabled[2];	

//	int was_disabled = !prev_motor_enabled[mc];
//	prev_motor_enabled[mc] = motor_enabled[mc];

	if(!motor_enabled[mc])
	{
		pwma = PWM_MID;
		pwmb = PWM_MID;
		pwmc = PWM_MID;

		errint_id[mc] = 0;
		errint_iq[mc] = 0;

		if(hall >= 0 && hall <= 5)
			rotor_ang[mc] = base_hall_aims[hall] + timing_shift;

		interp_ang_per_timeunit[mc] = 0;
		interp_time_left[mc] = 0;

		goto SKIP_CONTROLLING;
	}

	// Rotor angle estimation, and hall signal error handling

	if(prev_hall[mc] == -1)
	{
		if(hall >= 0 && hall <= 5)
		{
			rotor_ang[mc] = base_hall_aims[hall] + timing_shift;
			interp_ang_per_timeunit[mc] = 0;
			interp_time_left[mc] = 0;
		}
		else
		{
			illegal_hall_err_cnt += 200;
		}
	}
	else if(hall == prev_hall[mc])
	{
		// running between hall edges. Interpolate according to the parameters
		// decided earlier (at the previous edge).

		if(enable_interp && interp_time_left[mc] > 0)
		{
			rotor_ang[mc] += interp_ang_per_timeunit[mc];
			interp_time_left[mc]--;
		}

	}
	else if(hall == expected_hall) 
	{
		// At the hall step edge: in correct direction
		// The motor produces torque in this direction.
		if(illegal_hall_err_cnt > 0 )
			illegal_hall_err_cnt--;

		static uint32_t prev_dts[2];
		uint32_t dt = cur_time[mc] - time_last_expected_edge[mc];
		uint32_t prev_dt = prev_dts[mc];
		prev_dts[mc] = dt;

		time_last_expected_edge[mc] = cur_time[mc];

		rotor_ang[mc] = base_hall_aims[hall] + timing_shift;


		if(enable_interp)
		{
			if(dt < 3)
			{
				// The motor just can't turn this fast, the hall must be wrong.
				// Do not interpolate.
				illegal_hall_err_cnt += 800;
				interp_ang_per_timeunit[mc] = 0;
				interp_time_left[mc] = 0;
			}
			else if(dt > 6000)
			{
				// Too slow to guess what's actually happening, do not try to interpolate at all.
				// Use the middle point.
				interp_ang_per_timeunit[mc] = 0;
				interp_time_left[mc] = 0;
			}
			else
			{
				// If the rotation is quick and steady, do wide interpolation
				// If the rotational speed is changing, or rotation is too slow to be certain,
				// do limited, shallow interpolation around the middle point.
				int score = 4;
				if(dt < 4500) score++;
				if(dt < 3000) score++;

				// dt and prev_dt are quite close
				// example, dt=100, prev_dt must be 67 .. 149
				if(dt*2 < prev_dt*3 && dt*3 > prev_dt*2) score++;

				// dt and prev_dt are very close
				// example, dt=100, prev_dt must be 81 .. 124
				if(dt*4 < prev_dt*5 && dt*5 > prev_dt*4) score++;

				// score = 4 to 8
				// interpolation depth = 20 deg @ score 4
				// interpolation depth = 30 deg @ score 6
				// interpolation depth = 40 deg @ score 8

				//dbg_score = score;

				int interp_depth = 5*ANG_1_DEG*score;

				if(sp_v > 0)
				{
					rotor_ang[mc] -= interp_depth>>1;
					interp_ang_per_timeunit[mc] = interp_depth / (int32_t)dt;

				}
				else if(sp_v < 0)
				{

					rotor_ang[mc] += interp_depth>>1;
					interp_ang_per_timeunit[mc] = -interp_depth / (int32_t)dt;
				}

				interp_time_left[mc] = dt;
			}
		}
		else
		{
			interp_ang_per_timeunit[mc] = 0;
			interp_time_left[mc] = 0;
		}

	}
	else if(hall == unexpected_hall)
	{
		// At the hall step edge: to the wrong direction
		// Note: the motor does not produce torque in this direction, but the opposite
		// This state is caused by a fairly strong physical jerk or oscillation.

		// Just no interpolation.

		illegal_hall_err_cnt += 2;

		rotor_ang[mc] = base_hall_aims[hall] + timing_shift;
		interp_ang_per_timeunit[mc] = 0;
		interp_time_left[mc] = 0;

	}
	else
	{
		// Completely skipped a step, or got an illegal signal combination (6 legal and 2 illegal combinations exist)
		// Both scenarios likely mean the connections are intermittent and going to break.
		// Even if the hall combination is legal, it may be wrong.
		// We have no idea how to estimate the angle.
		// Keep the previous and increase the error counter fast.

		illegal_hall_err_cnt += 200;
		error(57);

	}

	if(illegal_hall_err_cnt > 20*200)
	{
		MC0_DIS_GATE();
		MC1_DIS_GATE();
		error(19);
	}


	//rotor_ang[mc] = base_hall_aims[hall] + timing_shift;

	int err_v = sp_v - velo[mc];

	static int32_t errint_v[2];

	if(sp_v == 0)
	{
		errint_v[mc] = 0;
	}
	else
	{
		errint_v[mc] += err_v;
	}


	if(errint_v[mc] > max_errint_v) errint_v[mc] = max_errint_v;
	if(errint_v[mc] < -max_errint_v) errint_v[mc] = -max_errint_v;

	int current = 
	       (((int64_t)velo_p  * (int64_t)err_v)>>16) +
	       (((int64_t)velo_i  * (int64_t)errint_v[mc])>>16);


	#define current_limit 2000

	if(current > current_limit) current = current_limit;
	else if(current < -current_limit) current = -current_limit;


//	sp_iq = -1*current;
	sp_iq = -2*sp_v;

	/*
		Transform from 3-phase coordinate space defined by 3 vectors 120 deg apart, to
		the more understandable 2-phase x,y coordinate space (defined by 2 vectors 90 deg apart)
		This operation of two multiplications by constants and one summation 
		is called "Clarke transform" and considered high end math wizardry
		by the literature which typically does not show the math to make
		sure it stays mysterious.

		i_alpha = ia
		i_beta = (1/sqrt(3)) * (ia + 2ib)

		1/sqrt(3) = 0.5773503 ~= 37387/65536 = 0.5773468
	*/

	int i_alpha = ia;
	int i_beta = (37387*(ia + 2*ib))>>16;

	/*
		Now, rotate the current measurement vectors so that they are referenced to the rotor.

		For some reason, in context of motors, this is called "Park transform". 

		positive iq is the current 90 degrees ahead of the rotor when going backwards
		negative iq is the current 90 degrees ahead of the rotor when going forward
		positive id is the current pulling directly to the current direction of rotor. Typically regulated as 0
	*/

	int rotcos = lut_cos_from_u32((uint32_t)rotor_ang[mc]);
	int rotsin = lut_sin_from_u32((uint32_t)rotor_ang[mc]);

	int id = (i_alpha * rotcos + i_beta  * rotsin)>>SIN_LUT_RESULT_SHIFT;
	int iq = (i_beta  * rotcos - i_alpha * rotsin)>>SIN_LUT_RESULT_SHIFT;

	// Park done! Didn't need a library for that either.

	// Now the PI control loops for id, iq

	int err_id = sp_id - id;
	int err_iq = sp_iq - iq;

//	avgd_err_id = (65535*(int64_t)avgd_err_id + (int64_t)(abso(err_id)<<8))>>16;
//	avgd_err_iq = (65535*(int64_t)avgd_err_iq + (int64_t)(abso(err_iq)<<8))>>16;

	errint_id[mc] += err_id;
	errint_iq[mc] += err_iq;

	if(errint_id[mc] > max_errint) errint_id[mc] = max_errint;
	if(errint_id[mc] < -max_errint) errint_id[mc] = -max_errint;
	if(errint_iq[mc] > max_errint) errint_iq[mc] = max_errint;
	if(errint_iq[mc] < -max_errint) errint_iq[mc] = -max_errint;

	int cmd_d = 
	       (((int64_t)cc_p  * (int64_t)err_id)>>16) +
	       (((int64_t)cc_i  * (int64_t)errint_id[mc])>>16);

	int cmd_q = 
	       (((int64_t)cc_p  * (int64_t)err_iq)>>16) +
	       (((int64_t)cc_i  * (int64_t)errint_iq[mc])>>16);


	// Now rotate commands back (inverse Park):

	int cmd_alpha = (cmd_d * rotcos - cmd_q * rotsin)>>SIN_LUT_RESULT_SHIFT;
	int cmd_beta  = (cmd_q * rotcos + cmd_d * rotsin)>>SIN_LUT_RESULT_SHIFT;

	/*
		Now the funny thing:

		Outer Space Vectors have absolutely nothing to do with FOC.

		Math doesn't even know about a concept called "space vector".

		"Space Vector Modulation" is just a completely made-up name for a certain, specific class of methods of generating PWM
		waveforms for gate drivers. The result is just PWM. There is absolutely nothing wrong generating PWMs in any other way, including
		the bog standard timer-based PWM generation all microcontrollers provide in hardware.

		The argument for using "Space Vector Modulation", is to get rid of doing Inverse Clarke, saving from a
		few trivial multiplications! All of this "advantage" is naturally lost due to fact that microcontrollers
		do not implement Space Vector Modulation hardware.

		Depending on the actual "Space vector modulation" implementation, it may or may not reduce switching losses compared
		to the classical triangle-wave-comparison PWM.

		Obviously, using the classical way here because it's implemented in MCU hardware and, uhm, well, it just works.

		So finally, do the inverse clarke - i.e., convert the 2-phase pwm commands to 3-phase, using these formulae:

		va = v_alpha
		vb = (-v_alpha + sqrt(3)*v_beta)/2
		vc = (-v_alpha - sqrt(3)*v_beta)/2


		sqrt(3) = 1.732051 ~= 14189/2^13 = 1.732056
	*/
	
	pwma = PWM_MID + cmd_alpha;
	pwmb = PWM_MID + ((-cmd_alpha + ((14189*cmd_beta)>>13))>>1);
	pwmc = PWM_MID + ((-cmd_alpha - ((14189*cmd_beta)>>13))>>1);


	int clipped = 0;
	if(pwma < 100){ clipped = 1; pwma = 100;}
	if(pwma > 8192-100){ clipped = 1; pwma = 8192-100;}
	if(pwmb < 100){ clipped = 1; pwmb = 100;}
	if(pwmb > 8192-100){ clipped = 1; pwmb = 8192-100;}
	if(pwmc < 100){ clipped = 1; pwmc = 100;}
	if(pwmc > 8192-100){ clipped = 1; pwmc = 8192-100;}

	static int clipped_cnt;

	if(clipped)
	{
		clipped_cnt += 5;
		if(clipped_cnt > 5*1000000)
		{
			MC0_DIS_GATE();
			MC1_DIS_GATE();
			error(17);

		}
	}
	else
	{
		if(clipped_cnt > 0)
			clipped_cnt -= 1;
	}
	

	SKIP_CONTROLLING:;

	prev_hall[mc] = hall;
	
	if(mc == 0)
	{
		TIM1->CCR1 = pwma;
		TIM1->CCR2 = pwmb;
		TIM1->CCR3 = pwmc;
	}
	else
	{
		TIM8->CCR1 = pwma;
		TIM8->CCR2 = pwmb;
		TIM8->CCR3 = pwmc;
	}


/*
	static int decim;
	if(trace_at >= 0)
	{
		if(++decim >= 12)
		{
			decim = 0;
			trace[trace_at].hall = hall;
			trace[trace_at].rotor_ang = rotor_ang>>24;
			trace[trace_at].ia = ia;
			trace[trace_at].ib = ib;
			trace[trace_at].ic = ic;
			trace[trace_at].ialpha = i_alpha;
			trace[trace_at].ibeta = i_beta;
			trace[trace_at].id = id;
			trace[trace_at].sp_id = sp_id;
			trace[trace_at].iq = iq;
			trace[trace_at].sp_iq = sp_iq;
			trace[trace_at].cmd_beta = cmd_alpha;
			trace[trace_at].cmd_beta = cmd_beta;
			trace[trace_at].pwma = pwma;
			trace[trace_at].pwmb = pwmb;
			trace[trace_at].pwmc = pwmc;
			trace[trace_at].errint_id = errint_id;
			trace[trace_at].errint_iq = errint_iq;

			trace_at++;
			if(trace_at == TRACE_LEN)
				trace_at = -1;
		}
	}

*/

}


// runs at 12200Hz
void bldc0_inthandler() __attribute__((section(".text_itcm")));
void bldc0_inthandler()
{
	TIM1->SR = 0; // Clear interrupt flags
	__DSB();

	int ib = ADC_MID-adc1.s.mc0_imeasb;
	int ic = adc1.s.mc0_imeasc-ADC_MID; // Swapped polarity (PCB layout optimization)

	if(ib < -ADC_CURR_SAFETY_LIMIT || ib > ADC_CURR_SAFETY_LIMIT || ic < -ADC_CURR_SAFETY_LIMIT || ic > ADC_CURR_SAFETY_LIMIT)
	{
		MC0_DIS_GATE();
		error(16);
	}

	bldc_handler(0, ib, ic, hall_loc[MC0_HALL_CBA()]);
}

void bldc1_inthandler() __attribute__((section(".text_itcm")));
void bldc1_inthandler()
{
	TIM8->SR = 0; // Clear interrupt flags
	__DSB();

	int ib = ADC_MID-adc1.s.mc1_imeasb;
	int ic = adc1.s.mc1_imeasc-ADC_MID; // Swapped polarity (PCB layout optimization)

	if(ib < -ADC_CURR_SAFETY_LIMIT || ib > ADC_CURR_SAFETY_LIMIT || ic < -ADC_CURR_SAFETY_LIMIT || ic > ADC_CURR_SAFETY_LIMIT)
	{
		MC1_DIS_GATE();
		error(16);
	}

	bldc_handler(1, ib, ic, hall_loc[MC1_HALL_CBA()]);
}


void bldc_safety_shutdown() __attribute__((section(".text_itcm")));
void bldc_safety_shutdown()
{
	MC0_DIS_GATE();
	MC1_DIS_GATE();

}

void motor_run(int m) __attribute__((section(".text_itcm")));
void motor_run(int m)
{
	if(m < 0 || m > 1) error(120);

	if(m==0)
		MC0_EN_GATE();
	else
		MC1_EN_GATE();

	DIS_IRQ();
	bldc_pos_set[m] = bldc_pos[m];
	run[m] = 1;
	wanna_stop[m] = 0;
	ENA_IRQ();
}

void motor_run_a_bit(int m, int amount) __attribute__((section(".text_itcm")));
void motor_run_a_bit(int m, int amount)
{
	if(m < 0 || m > 1) error(120);

	if(m==0)
		MC0_EN_GATE();
	else
		MC1_EN_GATE();

	DIS_IRQ();
	bldc_pos_set[m] = bldc_pos[m] + amount;
	run[m] = 1;
	wanna_stop[m] = 1;
	ENA_IRQ();
}


void motor_let_stop(int m) __attribute__((section(".text_itcm")));
void motor_let_stop(int m)
{
	if(m < 0 || m > 1) error(121);
	wanna_stop[m] = 1;
}

void motor_stop_now(int m) __attribute__((section(".text_itcm")));
void motor_stop_now(int m)
{
	if(m < 0 || m > 1) error(122);
	DIS_IRQ();
	run[m] = 0;
	bldc_pos_set[m] = bldc_pos[m];
	ENA_IRQ();
}

void motor_torque_lim(int m, int percent) __attribute__((section(".text_itcm")));
void motor_torque_lim(int m, int percent)
{
	if(m < 0 || m > 1 || percent<0 || percent>100) error(122);

	int curr = (percent*25000)/100;
	int pp = 50 + (percent*50)/100;
	int pi = 150 + (percent*250)/100;

	pid_p = pp;
	pid_i = pi;

	set_curr_lim(curr);
}

void motor_release(int m) __attribute__((section(".text_itcm")));
void motor_release(int m)
{
	if(m < 0 || m > 1) error(123);

	if(m==0)
		MC0_DIS_GATE();
	else
		MC1_DIS_GATE();

	run[m] = 0;
	bldc_pos_set[m] = bldc_pos[m];
}

int get_motor_torque(int m) __attribute__((section(".text_itcm")));
int get_motor_torque(int m)
{
	if(m < 0 || m > 1) error(124);
	return (curr_info_ma[m]*100)/25000;
}

void bldc_print_debug()
{
	uart_print_string_blocking("\r\n");
	DBG_PR_VAR_I32(run[0]);
	DBG_PR_VAR_I32(run[1]);
	DBG_PR_VAR_I32(wanna_stop[0]);
	DBG_PR_VAR_I32(wanna_stop[1]);
	DBG_PR_VAR_I32(bldc_pos_set[0]);
	DBG_PR_VAR_I32(bldc_pos[0]);
	DBG_PR_VAR_I32(dbg_err0);
	DBG_PR_VAR_I32(bldc_pos_set[1]);
	DBG_PR_VAR_I32(bldc_pos[1]);
	DBG_PR_VAR_I32(dbg_err1);
}

#if 1
void bldc_test()
{
	init_cpu_profiler();

	int32_t loc = 0;

	calc_max_errint();
	calc_max_errint_v();

	int do_trace = 0;
	int print = 1;
	while(1)
	{
		profile_cpu_blocking_20ms();
/*
		DIS_IRQ();
		int ia = latest_ia;
		int ib = latest_ib;
		int ic = latest_ic;
		ENA_IRQ();
*/
		if(do_trace && trace_at == -1) // Trace done
		{
			do_trace = 0;
			MC0_DIS_GATE();
			MC1_DIS_GATE();

			uart_print_string_blocking("hall,ang,ia,ib,ic,ialpha,ibeta,id,sp_id,iq,sp_iq,cmd_alpha,cmd_beta,pwma,pwmb,pwmc,errint_id,errint_iq\r\n");
			for(int i=0; i<TRACE_LEN; i++)
			{
				o_itoa16(trace[i].hall, printbuf); uart_print_string_blocking(printbuf);
				uart_print_string_blocking(",");
				o_itoa16((int)((360*(uint32_t)trace[i].rotor_ang)>>8)-180, printbuf); uart_print_string_blocking(printbuf);
				uart_print_string_blocking(",");
				o_itoa16(trace[i].ia, printbuf); uart_print_string_blocking(printbuf);
				uart_print_string_blocking(",");
				o_itoa16(trace[i].ib, printbuf); uart_print_string_blocking(printbuf);
				uart_print_string_blocking(",");
				o_itoa16(trace[i].ic, printbuf); uart_print_string_blocking(printbuf);
				uart_print_string_blocking(",");
				o_itoa16(trace[i].ialpha, printbuf); uart_print_string_blocking(printbuf);
				uart_print_string_blocking(",");
				o_itoa16(trace[i].ibeta, printbuf); uart_print_string_blocking(printbuf);
				uart_print_string_blocking(",");
				o_itoa16(trace[i].id, printbuf); uart_print_string_blocking(printbuf);
				uart_print_string_blocking(",");
				o_itoa16(trace[i].sp_id, printbuf); uart_print_string_blocking(printbuf);
				uart_print_string_blocking(",");
				o_itoa16(trace[i].iq, printbuf); uart_print_string_blocking(printbuf);
				uart_print_string_blocking(",");
				o_itoa16(trace[i].sp_iq, printbuf); uart_print_string_blocking(printbuf);
				uart_print_string_blocking(",");
				o_itoa16(trace[i].cmd_alpha, printbuf); uart_print_string_blocking(printbuf);
				uart_print_string_blocking(",");
				o_itoa16(trace[i].cmd_beta, printbuf); uart_print_string_blocking(printbuf);
				uart_print_string_blocking(",");
				o_itoa16(trace[i].pwma, printbuf); uart_print_string_blocking(printbuf);
				uart_print_string_blocking(",");
				o_itoa16(trace[i].pwmb, printbuf); uart_print_string_blocking(printbuf);
				uart_print_string_blocking(",");
				o_itoa16(trace[i].pwmc, printbuf); uart_print_string_blocking(printbuf);
				uart_print_string_blocking(",");
				o_itoa32(trace[i].errint_id, printbuf); uart_print_string_blocking(printbuf);
				uart_print_string_blocking(",");
				o_itoa32(trace[i].errint_iq, printbuf); uart_print_string_blocking(printbuf);
				uart_print_string_blocking("\r\n");
			}
		}

/*
		int hall = hall_loc[MC0_HALL_CBA()];


		uart_print_string_blocking(" ia = "); o_itoa16_fixed(ia, printbuf); uart_print_string_blocking(printbuf);
		uart_print_string_blocking(" ib = "); o_itoa16_fixed(ib, printbuf); uart_print_string_blocking(printbuf);
		uart_print_string_blocking(" ic = "); o_itoa16_fixed(ic, printbuf); uart_print_string_blocking(printbuf);
		uart_print_string_blocking(" hall = "); o_utoa8_fixed(hall, printbuf); uart_print_string_blocking(printbuf);

		uart_print_string_blocking("\r\n");

		uart_print_string_blocking(" cc_p = "); o_utoa16_fixed(cc_p, printbuf); uart_print_string_blocking(printbuf);
		uart_print_string_blocking(" cc_i = "); o_utoa16_fixed(cc_i, printbuf); uart_print_string_blocking(printbuf);
		uart_print_string_blocking(" avgd_err_id = "); o_itoa16_fixed(avgd_err_id>>8, printbuf); uart_print_string_blocking(printbuf);
		uart_print_string_blocking(" avgd_err_iq = "); o_itoa16_fixed(avgd_err_iq>>8, printbuf); uart_print_string_blocking(printbuf);

		uart_print_string_blocking(" rotor_ang = "); o_utoa32(dbg_rotor_ang, printbuf); uart_print_string_blocking(printbuf);
		uart_print_string_blocking(" rotor_ang_deg = "); o_utoa32(dbg_rotor_ang/ANG_1_DEG, printbuf); uart_print_string_blocking(printbuf);
		uart_print_string_blocking(" rotcos = "); o_itoa32(dbg_rotcos, printbuf); uart_print_string_blocking(printbuf);
		uart_print_string_blocking(" rotsin = "); o_itoa32(dbg_rotsin, printbuf); uart_print_string_blocking(printbuf);

		uart_print_string_blocking("\r\n");


		uart_print_string_blocking("\r\n");
		uart_print_string_blocking("\r\n");
*/

		uart_print_string_blocking(" velo = "); o_itoa32(dbg_velo, printbuf); uart_print_string_blocking(printbuf);
		uart_print_string_blocking(" score = "); o_itoa32(dbg_score, printbuf); uart_print_string_blocking(printbuf);
		uart_print_string_blocking("\r\n");


		if(print)
		{		
			uart_print_string_blocking(" velo_p = "); o_utoa32(velo_p, printbuf); uart_print_string_blocking(printbuf);
			uart_print_string_blocking(" velo_i = "); o_utoa32(velo_i, printbuf); uart_print_string_blocking(printbuf);

			uart_print_string_blocking(" interp = "); o_itoa32(enable_interp, printbuf); uart_print_string_blocking(printbuf);

			
			uart_print_string_blocking("\r\n");
			uart_print_string_blocking("\r\n");

			print = 0;
		}

		for(int i=0; i<100; i++)
		{
			delay_ms(1);

			uint8_t cmd = uart_input();

			if(cmd == '0')
			{
				powder = 0;
			}
			else if(cmd == '1')
			{
				sp_velo[0] = -800;
				sp_velo[1] = -800;
			}
			else if(cmd == '2')
			{
				sp_velo[0] = -400;
				sp_velo[1] = -400;
			}
			else if(cmd == '3')
			{
				sp_velo[0] = -200;
				sp_velo[1] = -200;
			}
			else if(cmd == '4')
			{
				sp_velo[0] = -100;
				sp_velo[1] = -100;
			}
			else if(cmd == '5')
			{
				sp_velo[0] = 0;
				sp_velo[1] = 0;
			}
			else if(cmd == '6')
			{
				sp_velo[0] = 100;
				sp_velo[1] = 100;
			}
			else if(cmd == '7')
			{
				sp_velo[0] = 200;
				sp_velo[1] = 200;
			}
			else if(cmd == '8')
			{
				sp_velo[0] = 400;
				sp_velo[1] = 400;
			}
			else if(cmd == '9')
			{
				sp_velo[0] = 800;
				sp_velo[1] = 800;
			}
			else if(cmd == 'q')
			{
				do_trace = 1;
				trace_at = 0;

				powder = 1;
				delay_ms(400);
				powder = 4;
				delay_ms(800);
				powder = 9;
				delay_ms(500);
				powder = 6;
				delay_ms(700);
				powder = 5;
			}
			else if(cmd == 'w')
			{
				enable_interp = !enable_interp;
				print = 1;
			}
			else if(cmd == 'e')
			{
			}
			else if(cmd == 'r')
			{
			}
			else if(cmd == 'a')
			{
				motor_enabled[0] = 1;
				motor_enabled[1] = 1;
				MC0_EN_GATE();
				MC1_EN_GATE();
			}
			else if(cmd == 's')
			{
				motor_enabled[0] = 0;
				motor_enabled[1] = 0;
				MC0_DIS_GATE();
				MC1_DIS_GATE();
			}
/*
			else if(cmd == 'P')
			{
				cc_p += 1000;
				print = 1;
			}
			else if(cmd == 'p')
			{
				if(cc_p > 1000)
					cc_p -= 1000;
				else
					cc_p = 0;
				print = 1;
			}
			else if(cmd == 'I')
			{
				cc_i += 20;
				calc_max_errint();
				print = 1;
			}
			else if(cmd == 'i')
			{
				if(cc_i > 20)
					cc_i -= 20;
				else
					cc_i = 0;
				calc_max_errint();
				print = 1;

			}
*/
			else if(cmd == 'P')
			{
				velo_p += 4000;
				print = 1;
			}
			else if(cmd == 'p')
			{
				if(velo_p > 4000)
					velo_p -= 4000;
				else
					velo_p = 0;
				print = 1;
			}
			else if(cmd == 'I')
			{
				velo_i += 10;
				calc_max_errint_v();
				print = 1;
			}
			else if(cmd == 'i')
			{
				if(velo_i > 10)
					velo_i -= 10;
				else
					velo_i = 0;
				calc_max_errint_v();
				print = 1;

			}
			else if(cmd == 't')
			{
				delay_ms(2000);
				do_trace = 1;
				trace_at = 0;
				delay_ms(2000);
			}
		}
	}
}
#endif
/*

ADC1 is dedicated to motor phase current measurements (see adc.h)

TIM1 controls MC0
TIM8 controls MC1

Both TIM1 and TIM8 always run simultaneously, with the same PWM frequency.

To reduce DC bus ripple, TIM8 phase is 180 offset from TIM1.

TIM8 is synchronized to TIM1, by configuring:
	* TIM1 TRGO: OC1REF is used as trigger output
	* TIM8 trigger: ITR0 (TIM1 TRGO) is used as trigger input
	* TIM8 slave mode: In trigger mode: only counter start is controlled.

After both counters have started, TIM1 OC1 can be used for the intended purpose (PWM)


ADC runs in discontinous groups, converting MC0 and MC1 currents alternatively, with
two separate triggers. ADC is triggered at the midpoint (when the counter hits the top value,
right when it changes direction to downcounting).

Since the trigger source has to be fixed to either TIM1 or TIM8, we use TIM1. Thus, we
need to create two ADC triggers per cycle, which will be:
	* TIM1 TRGO2 (Master Mode 2) = update event (which will happen for both overflow
	  and underflow).
This way, we can't trig the ADC exactly centered, it'll send the trigger _start_ at the exact center,
but this is no big deal, the ADC is fast compared to our slow PWM.


Timers run at 200MHz
Timer is up/down counting, halving the freq.
With PWM_MAX = 8192, PWM will run at 12.2kHz

ADC converts the two currents in 0.6us, let's assume DMA takes 0.2us more to finish, and add 0.2us
safety margin. Hence, we want the IRQ earliest 1us after the ADC trigger (update event).

On the other hand, if we want to allocate at least 2us for worst-case calculation in the ISR,
the IRQ needs to happen latest 2us before the update event (so the new PWM values are in at update).

It would be nice to be able to trig the IRQ at OCR4 rising edge 1us after the ADC trigger, but this
is impossible: the TIMer generates two triggers (rising and falling).

The only sane way is to make the IRQ trigger from TIM1 only, at equidistant intervals, this is, at PWM_MID,
so it happens about 41us after the ADC trigger, and 41us before the update event of the relevant timer.

CORRECTION TO ABOVE:
As an undocumented and unexpected feature, CC IRQs only happen at the rising edge - exactly where we want them
to be! We can use separate interrupt triggers for both timers.

*/
void init_bldc()
{
	// Same enable and hall pins for REV2A,B
	MC0_DIS_GATE();
	MC1_DIS_GATE();
	IO_TO_GPO(GPIOG,8);
	IO_TO_GPO(GPIOI,4);

	// Hall pins:
	IO_TO_GPI(GPIOE,5);
	IO_TO_GPI(GPIOE,6);
	IO_TO_GPI(GPIOI,8);

	IO_TO_GPI(GPIOD,14);
	IO_TO_GPI(GPIOD,15);
	IO_TO_GPI(GPIOG,2);

	RCC->APB2ENR |= 0b11; // TIM8, TIM1
	__DSB();


//	TIM1->PSC = 2-1;	
//	TIM8->PSC = 2-1;	

	TIM1->CR1 = 0b01UL<<5 /*centermode 1*/;
	TIM1->CR2 = 0b0010UL<<20 /*TRGO2: Update event*/ | 0b100UL<<4 /*TRGO: OC1REF*/;

	TIM1->CCMR1 = 1UL<<3 /*OC1 Preload enable*/  | 0b110UL<<4  /*OC1 PWMmode 1*/ |
	              1UL<<11 /*OC2 Preload Enable*/ | 0b110UL<<12 /*OC2 PWMmode 1*/;
	TIM1->CCMR2 = 1UL<<3 /*OC3 Preload enable*/  | 0b110UL<<4  /*OC3 PWMmode 1*/;
	TIM1->CCER =  1UL<<0 /*OC1 on*/ | 1UL<<2 /*OC1 complementary output enable*/ |
	              1UL<<4 /*OC2 on*/ | 1UL<<6 /*OC2 complementary output enable*/ |
		      1UL<<8 /*OC3 on*/ | 1UL<<10 /*OC3 complementary output enable*/;


	TIM1->ARR = PWM_MAX;

	// CCR1 is reused for the temporary purpose of sync-starting TIM8:
	TIM1->CCR1 = PWM_MAX-10; // Sync pulse for 180 deg phase diff.
	// Tested that "1" is actually long enough to sync. Using a bit wider pulse (-10) to be sure.

	TIM1->CCR2 = PWM_MID;
	TIM1->CCR3 = PWM_MID;
	TIM1->CCR4 = PWM_MAX-400; // Interrupt trigger 2us after ADC trigger
	TIM1->BDTR = 1UL<<15 /*Main output enable*/ | 1UL /*21ns deadtime*/;
	TIM1->EGR |= 1; // Generate Reinit+update
	__DSB();
	TIM1->CR1 |= 1; // Enable the timer
	__DSB();



	TIM8->CR1 = 0b01UL<<5 /*centermode 1*/;
	TIM8->SMCR = 0UL<<7 /*unclearly documented mystery bit, let's keep it off*/ | 
		0UL<<16 | 0b110UL<<0 /*Slave mode 0110: Trigger mode: only start is triggered*/;
		// ITR0 as trigger sourceby zeroes

	TIM8->CCMR1 = 1UL<<3 /*OC1 Preload enable*/  | 0b110UL<<4  /*OC1 PWMmode 1*/ |
	              1UL<<11 /*OC2 Preload Enable*/ | 0b110UL<<12 /*OC2 PWMmode 1*/;
	TIM8->CCMR2 = 1UL<<3 /*OC3 Preload enable*/  | 0b110UL<<4  /*OC3 PWMmode 1*/;
	TIM8->CCER =  1UL<<0 /*OC1 on*/ | 1UL<<2 /*OC1 complementary output enable*/ |
	              1UL<<4 /*OC2 on*/ | 1UL<<6 /*OC2 complementary output enable*/ |
		      1UL<<8 /*OC3 on*/ | 1UL<<10 /*OC3 complementary output enable*/;

	TIM8->ARR = PWM_MAX;

	TIM8->CCR1 = PWM_MID;
	TIM8->CCR2 = PWM_MID;
	TIM8->CCR3 = PWM_MID;
	TIM8->CCR4 = PWM_MAX-400; // Interrupt trigger 2us after ADC trigger

	TIM8->BDTR = 1UL<<15 /*Main output enable*/ | 1UL /*21ns deadtime*/;
	TIM8->EGR |= 1; // Generate Reinit+update
//	TIM8->DIER = 1UL /*Update interrupt enable*/;
	__DSB();
	// Don't enable the timer, let it sync to TIM1

	while(!(TIM8->CR1 & 1UL)) ;
	__DSB();

	TIM1->CCR1 = PWM_MID; // CCR1 not needed for syncing anymore.

	delay_us(100); // Make sure CCR1 updates before turning on outputs:
	// Now both timers are running OK, enable the output signals:

	// Same timer mapping for REV2A,B
	// TIM1:
	IO_ALTFUNC(GPIOA,8,  1);
	IO_ALTFUNC(GPIOA,9,  1);
	IO_ALTFUNC(GPIOA,10, 1);

	// TIM8:
	IO_ALTFUNC(GPIOI,5,  3);
	IO_ALTFUNC(GPIOI,6,  3);
	IO_ALTFUNC(GPIOI,7,  3);

	TIM1->DIER = 1UL<<4 /*Compare 4 interrupt*/;
	NVIC_SetPriority(TIM1_CC_IRQn, INTPRIO_BLDC);
	NVIC_EnableIRQ(TIM1_CC_IRQn);

	TIM8->DIER = 1UL<<4 /*Compare 4 interrupt*/;
	NVIC_SetPriority(TIM8_CC_IRQn, INTPRIO_BLDC);
	NVIC_EnableIRQ(TIM8_CC_IRQn);

}

void init_bldc_dummy()
{
	RCC->APB2ENR |= 0b11; // TIM8, TIM1
	__DSB();

	TIM1->CR1 = 0b01UL<<5 /*centermode 1*/;
	TIM1->CR2 = 0b0010UL<<20 /*TRGO2: Update event*/ | 0b100UL<<4 /*TRGO: OC1REF*/;

	TIM1->CCMR1 = 1UL<<3 /*OC1 Preload enable*/  | 0b110UL<<4  /*OC1 PWMmode 1*/ |
	              1UL<<11 /*OC2 Preload Enable*/ | 0b110UL<<12 /*OC2 PWMmode 1*/;
	TIM1->CCMR2 = 1UL<<3 /*OC3 Preload enable*/  | 0b110UL<<4  /*OC3 PWMmode 1*/;
	TIM1->CCER =  1UL<<0 /*OC1 on*/ | 1UL<<2 /*OC1 complementary output enable*/ |
	              1UL<<4 /*OC2 on*/ | 1UL<<6 /*OC2 complementary output enable*/ |
		      1UL<<8 /*OC3 on*/ | 1UL<<10 /*OC3 complementary output enable*/;


	TIM1->ARR = PWM_MAX;

	// CCR1 is reused for the temporary purpose of sync-starting TIM8:
	TIM1->CCR1 = PWM_MAX-10; // Sync pulse for 180 deg phase diff.
	// Tested that "1" is actually long enough to sync. Using a bit wider pulse (-10) to be sure.

	TIM1->CCR2 = PWM_MID;
	TIM1->CCR3 = PWM_MID;
	TIM1->CCR4 = PWM_MAX-400; // Interrupt trigger 2us after ADC trigger
	// TIM1->BDTR = 1UL<<15 /*Main output enable*/ | 1UL /*21ns deadtime*/; <-- nope
	TIM1->EGR |= 1; // Generate Reinit+update
	__DSB();
	TIM1->CR1 |= 1; // Enable the timer
	__DSB();


	TIM8->CR1 = 0b01UL<<5 /*centermode 1*/;
	TIM8->SMCR = 0UL<<7 /*unclearly documented mystery bit, let's keep it off*/ | 
		0UL<<16 | 0b110UL<<0 /*Slave mode 0110: Trigger mode: only start is triggered*/;
		// ITR0 as trigger sourceby zeroes

	TIM8->CCMR1 = 1UL<<3 /*OC1 Preload enable*/  | 0b110UL<<4  /*OC1 PWMmode 1*/ |
	              1UL<<11 /*OC2 Preload Enable*/ | 0b110UL<<12 /*OC2 PWMmode 1*/;
	TIM8->CCMR2 = 1UL<<3 /*OC3 Preload enable*/  | 0b110UL<<4  /*OC3 PWMmode 1*/;
	TIM8->CCER =  1UL<<0 /*OC1 on*/ | 1UL<<2 /*OC1 complementary output enable*/ |
	              1UL<<4 /*OC2 on*/ | 1UL<<6 /*OC2 complementary output enable*/ |
		      1UL<<8 /*OC3 on*/ | 1UL<<10 /*OC3 complementary output enable*/;

	TIM8->ARR = PWM_MAX;

	TIM8->CCR1 = PWM_MID;
	TIM8->CCR2 = PWM_MID;
	TIM8->CCR3 = PWM_MID;
	TIM8->CCR4 = PWM_MAX-400; // Interrupt trigger 2us after ADC trigger

	// TIM8->BDTR = 1UL<<15 /*Main output enable*/ | 1UL /*21ns deadtime*/; <-- nope
	TIM8->EGR |= 1; // Generate Reinit+update
//	TIM8->DIER = 1UL /*Update interrupt enable*/;
	__DSB();
	// Don't enable the timer, let it sync to TIM1

	while(!(TIM8->CR1 & 1UL)) ;
	__DSB();

	TIM1->CCR1 = PWM_MID; // CCR1 not needed for syncing anymore.

}
