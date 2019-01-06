#include <stdint.h>
#include "ext_include/stm32h7xx.h"
#include "own_std.h"
#include "stm32_cmsis_extension.h"
#include "misc.h"
#include "adcs.h"

#include "bldc.h"

#define PWM_MAX 8192
#define PWM_MID (PWM_MAX/2)
#define MIN_FREQ 1*65536
#define MAX_FREQ 100*65536

// Gates enabled, or just freewheeling.
#define MC0_EN_GATE()  HI(GPIOG,8)
#define MC0_DIS_GATE() LO(GPIOG,8)

#define MC1_EN_GATE()  HI(GPIOI,4)
#define MC1_DIS_GATE() LO(GPIOI,4)

// Read hall sensor signals HALLC,HALLB,HALLA and concatenate to a 3-bit value, to index a table
#define MC0_HALL_CBA() ( ((GPIOD->IDR & (0b11<<14))>>14) | (GPIOG->IDR & (1<<2)) )
#define MC1_HALL_CBA() ( ((GPIOE->IDR & (0b11<<5))>>5)   | ((GPIOI->IDR & (1<<8))>>6) )

#define DLED_ON() {}


int timing_shift = 5000*65536; // hall sensors are not exactly where you could think they are.

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

	if(ma < 0 || ma > 20000)
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
#define TRACE_LEN 8
int trace_at = 0;
volatile int ib_trace[TRACE_LEN], ic_trace[TRACE_LEN];

#define ADC_MID 8192
#define ADC_CURR_SAFETY_LIMIT 7500

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


#define STOP_LEN 6000 // approx half a second.
#define STOP_THRESHOLD (SUBSTEPS*3/2)

volatile int dbg_err, dbg_sin_mult, dbg_m;

volatile int stop_state_dbg[2];
void bldc0_inthandler() __attribute__((section(".text_itcm")));
void bldc0_inthandler()
{
	TIM1->SR = 0; // Clear interrupt flags
	__DSB();

	int ib = ADC_MID-adc1.s.mc0_imeasb;
	int ic = ADC_MID-adc1.s.mc0_imeasc;

	if(ib < -ADC_CURR_SAFETY_LIMIT || ib > ADC_CURR_SAFETY_LIMIT || ic < -ADC_CURR_SAFETY_LIMIT || ic > ADC_CURR_SAFETY_LIMIT)
	{
		MC0_DIS_GATE();
		error(16);
	}

//	ib_trace[trace_at] = ib;
//	ic_trace[trace_at] = ic;
//	trace_at++;
//	if(trace_at >= TRACE_LEN)
//		trace_at=0;


	static int reverse = 0;
	static int mult = 0;
	static int currlim_mult = 255;
	static uint32_t cnt = 0;
	static int expected_fwd_hall_pos;  // for step counting only
	static int expected_back_hall_pos; // for step counting only
	static int prev_hall_pos;
	static int64_t pid_integral = 0;

	static int pid_prev_err = 0;

	static int stop_reverse = 0;
	static int stop_state = 0;
	static int stop_initial_hall_pos = 0;

	int hall_pos = hall_loc[MC0_HALL_CBA()];
	if(hall_pos == -1) hall_pos = prev_hall_pos;

	if(hall_pos == expected_fwd_hall_pos)
	{
		bldc_pos[0]+=SUBSTEPS;
	}
	else if(hall_pos == expected_back_hall_pos)
	{
		bldc_pos[0]-=SUBSTEPS;
	}

	prev_hall_pos = hall_pos;

	expected_fwd_hall_pos  = hall_pos+1; if(expected_fwd_hall_pos > 5) expected_fwd_hall_pos = 0;
	expected_back_hall_pos = hall_pos-1; if(expected_back_hall_pos < 0) expected_back_hall_pos = 5;


	int32_t err = bldc_pos_set[0] - bldc_pos[0];

	if(err < 0 ) reverse = 1; else reverse = 0;

	static uint32_t prev_bldc_pos_set;
	int change = bldc_pos_set[0] - prev_bldc_pos_set;
	if(run[0]==0 || change < -5*SUBSTEPS || change > 5*SUBSTEPS)
	{
		pid_integral = 0;
	}
	prev_bldc_pos_set = bldc_pos_set[0];

	if(stop_state == 0 && run[0] && wanna_stop[0] && err > -STOP_THRESHOLD && err < STOP_THRESHOLD)
	{
//		stop_state = abso(err)*STOP_LEN/STOP_THRESHOLD;
		stop_state = STOP_LEN;
		if(stop_state < 1000) stop_state = 1000;
		stop_initial_hall_pos = hall_pos;
		stop_reverse = reverse;
	}


	int loc;
	int sin_mult;

	if(stop_state == 0) // normal op
	{
		if(run[0] == 0)
			sin_mult = 0;
		else
		{
			if(!(cnt & 63))
			{
				dbg_err = err;
				if(err > ERR_SAT) err = ERR_SAT;
				else if(err < -ERR_SAT) err = -ERR_SAT;

				int derr = (err - pid_prev_err);
				pid_prev_err = err;

				if(err > pid_i_threshold) pid_integral += 256; //err - pid_i_threshold;
				else if(err < -1*pid_i_threshold) pid_integral += -256; //err + pid_i_threshold;
				else
				{
					pid_integral = 0;

//					if(pid_integral>0) pid_integral--;
//					else if(pid_integral<0) pid_integral++;
				}

				int64_t pid_i_max_extended = (int64_t)pid_i_max;
				int64_t pid_i_min_extended = -1*pid_i_max_extended;
				if(pid_integral > pid_i_max_extended) pid_integral = pid_i_max_extended;
				else if(pid_integral < pid_i_min_extended) pid_integral = pid_i_min_extended;

				mult = 
					  (((int64_t)pid_p*(int64_t)err)>>P_SHIFT) /* P */
					+ (((int64_t)pid_i*(int64_t)pid_integral)>>I_SHIFT)  /* I */
					+ (((int64_t)pid_d*(int64_t)derr)>>D_SHIFT); /* D */

				dbg_mult = mult;

				pid_int_info = pid_integral;

			}


			if(mult > MAX_MULT) mult = MAX_MULT;
			else if(mult < MIN_MULT) mult = MIN_MULT;

			sin_mult = mult>>8;

			if(reverse) sin_mult *= -1;	

			if(sin_mult < 0) sin_mult = 0;

			if(sin_mult != 0)
				sin_mult += MIN_MULT_OFFSET;

			dbg_sin_mult = sin_mult;

		}
		loc = base_hall_aims[hall_pos] + timing_shift + (reverse?(-PH90SHIFT):(PH90SHIFT));

	}
	else
	{
		int32_t phshift = PH90SHIFT/STOP_LEN * (STOP_LEN-stop_state);
		loc = base_hall_aims[stop_initial_hall_pos] + timing_shift + (stop_reverse?(-1*phshift):(phshift));
		sin_mult = 40;
		if(--stop_state == 0)
			run[0] = 0;

		// Went out of spec, don't stop
		if(err < -STOP_THRESHOLD || err > STOP_THRESHOLD)
		{
			sin_mult = 0;
			stop_state = 0;
		}
	}


	int idxa = (((uint32_t)loc)&0xff000000)>>24;
	int idxb = (((uint32_t)loc+PH120SHIFT)&0xff000000)>>24;
	int idxc = (((uint32_t)loc+2*PH120SHIFT)&0xff000000)>>24;


	uint8_t m = (sin_mult * currlim_mult)>>8; // 254 max

	dbg_m = m;
	TIM1->CCR1 = (PWM_MID) + ((m*sine[idxc])>>11); // sine: -32768..32767, times 254 max: multiplication result ranges +/- 4064
	TIM1->CCR2 = (PWM_MID) + ((m*sine[idxb])>>11);
	TIM1->CCR3 = (PWM_MID) + ((m*sine[idxa])>>11);

	cnt++;

	int current_b = ib;
	int current_c = ic;
	if(current_b < 0) current_b *= -1;
	if(current_c < 0) current_c *= -1;

	int current;
	if(current_c > current_b) current = current_c; else current = current_b;

	static int32_t current_flt = 0;

	current_flt = ((current<<8) + 63*current_flt)>>6;

	int32_t current_ma = current*MULT_ADC_TO_MA;
	int32_t filtered_current_ma = (current_flt>>8)*MULT_ADC_TO_MA;
	curr_info_ma[0] = filtered_current_ma;

	if(current_ma > highest_curr_lim)
	{
		currlim_mult -= CURRLIM_RATE1;
	}
	else if(current_ma > higher_curr_lim)
	{
		currlim_mult-= CURRLIM_RATE2;
	}
	else if(filtered_current_ma > precise_curr_lim)
	{
		// keep the currlim_mult
	}
	else if(currlim_mult < 255)
		currlim_mult++;

	if(currlim_mult < 5) currlim_mult = 5;
	stop_state_dbg[0] = stop_state;
}

void bldc1_inthandler() __attribute__((section(".text_itcm")));
void bldc1_inthandler()
{
	TIM8->SR = 0; // Clear interrupt flags
	__DSB();

	int ib = ADC_MID-adc1.s.mc1_imeasb;
	int ic = ADC_MID-adc1.s.mc1_imeasc;

	if(ib < -ADC_CURR_SAFETY_LIMIT || ib > ADC_CURR_SAFETY_LIMIT || ic < -ADC_CURR_SAFETY_LIMIT || ic > ADC_CURR_SAFETY_LIMIT)
	{
		MC1_DIS_GATE();
		error(16);
	}

//	ib_trace[trace_at] = ib;
//	ic_trace[trace_at] = ic;
//	trace_at++;
//	if(trace_at >= TRACE_LEN)
//		trace_at=0;


	static int reverse = 0;
	static int mult = 0;
	static int currlim_mult = 255;
	static uint32_t cnt = 0;
	static int expected_fwd_hall_pos;  // for step counting only
	static int expected_back_hall_pos; // for step counting only
	static int prev_hall_pos;
	static int64_t pid_integral = 0;

	static int pid_prev_err = 0;
	static int stop_reverse = 0;
	static int stop_state = 0;
	static int stop_initial_hall_pos = 0;

	int hall_pos = hall_loc[MC1_HALL_CBA()];
	if(hall_pos == -1) hall_pos = prev_hall_pos;

	if(hall_pos == expected_fwd_hall_pos)
	{
		bldc_pos[1]+=SUBSTEPS;
	}
	else if(hall_pos == expected_back_hall_pos)
	{
		bldc_pos[1]-=SUBSTEPS;
	}

	prev_hall_pos = hall_pos;

	expected_fwd_hall_pos  = hall_pos+1; if(expected_fwd_hall_pos > 5) expected_fwd_hall_pos = 0;
	expected_back_hall_pos = hall_pos-1; if(expected_back_hall_pos < 0) expected_back_hall_pos = 5;


	int32_t err = bldc_pos_set[1] - bldc_pos[1];

	if(err < 0 ) reverse = 1; else reverse = 0;

	static uint32_t prev_bldc_pos_set;
	int change = bldc_pos_set[1] - prev_bldc_pos_set;
	if(run[1]==0 || change < -5*SUBSTEPS || change > 5*SUBSTEPS)
	{
		pid_integral = 0;
	}
	prev_bldc_pos_set = bldc_pos_set[1];

	if(stop_state == 0 && run[1] && wanna_stop[1] && err > -STOP_THRESHOLD && err < STOP_THRESHOLD)
	{
//		stop_state = abso(err)*STOP_LEN/STOP_THRESHOLD;
		stop_state = STOP_LEN;
		if(stop_state < 1000) stop_state = 1000;
		stop_initial_hall_pos = hall_pos;
		stop_reverse = reverse;
	}


	int loc;
	int sin_mult;

	if(stop_state == 0) // normal op
	{
		if(run[1] == 0)
			sin_mult = 0;
		else
		{
			if(!(cnt & 63))
			{
				if(err > ERR_SAT) err = ERR_SAT;
				else if(err < -ERR_SAT) err = -ERR_SAT;

				int derr = (err - pid_prev_err);
				pid_prev_err = err;

				if(err > pid_i_threshold) pid_integral += 256; //err - pid_i_threshold;
				else if(err < -1*pid_i_threshold) pid_integral += -256; //err + pid_i_threshold;
				else
				{
					pid_integral = 0;
//					if(pid_integral>0) pid_integral--;
//					else if(pid_integral<0) pid_integral++;
				}

				int64_t pid_i_max_extended = (int64_t)pid_i_max;
				int64_t pid_i_min_extended = -1*pid_i_max_extended;
				if(pid_integral > pid_i_max_extended) pid_integral = pid_i_max_extended;
				else if(pid_integral < pid_i_min_extended) pid_integral = pid_i_min_extended;

				mult =
					  (((int64_t)pid_p*(int64_t)err)>>P_SHIFT) /* P */
					+ (((int64_t)pid_i*(int64_t)pid_integral)>>I_SHIFT)  /* I */
					+ (((int64_t)pid_d*(int64_t)derr)>>D_SHIFT); /* D */

				dbg_mult = mult;

				pid_int_info = pid_integral;

			}


			if(mult > MAX_MULT) mult = MAX_MULT;
			else if(mult < MIN_MULT) mult = MIN_MULT;

			sin_mult = mult>>8;

			if(reverse) sin_mult *= -1;	

			if(sin_mult < 0) sin_mult = 0;

			if(sin_mult != 0)
				sin_mult += MIN_MULT_OFFSET;

		}
		loc = base_hall_aims[hall_pos] + timing_shift + (reverse?(-PH90SHIFT):(PH90SHIFT));

	}
	else
	{
		int32_t phshift = PH90SHIFT/STOP_LEN * (STOP_LEN-stop_state);
		loc = base_hall_aims[stop_initial_hall_pos] + timing_shift + (stop_reverse?(-1*phshift):(phshift));
		sin_mult = 40;
		if(--stop_state == 0)
			run[1] = 0;

		if(err < -STOP_THRESHOLD || err > STOP_THRESHOLD)
		{
			sin_mult = 0;
			stop_state = 0;
		}
	}


	int idxa = (((uint32_t)loc)&0xff000000)>>24;
	int idxb = (((uint32_t)loc+PH120SHIFT)&0xff000000)>>24;
	int idxc = (((uint32_t)loc+2*PH120SHIFT)&0xff000000)>>24;


	uint8_t m = (sin_mult * currlim_mult)>>8; // 254 max
	TIM8->CCR1 = (PWM_MID) + ((m*sine[idxc])>>11); // sine: -32768..32767, times 254 max: multiplication result ranges +/- 4064
	TIM8->CCR2 = (PWM_MID) + ((m*sine[idxb])>>11);
	TIM8->CCR3 = (PWM_MID) + ((m*sine[idxa])>>11);

	cnt++;

	int current_b = ib;
	int current_c = ic;
	if(current_b < 0) current_b *= -1;
	if(current_c < 0) current_c *= -1;

	int current;
	if(current_c > current_b) current = current_c; else current = current_b;

	static int32_t current_flt = 0;

	current_flt = ((current<<8) + 63*current_flt)>>6;

	int32_t current_ma = current*MULT_ADC_TO_MA;
	int32_t filtered_current_ma = (current_flt>>8)*MULT_ADC_TO_MA;
	curr_info_ma[1] = filtered_current_ma;

	if(current_ma > highest_curr_lim)
	{
		currlim_mult -= CURRLIM_RATE1;
	}
	else if(current_ma > higher_curr_lim)
	{
		currlim_mult-= CURRLIM_RATE2;
	}
	else if(filtered_current_ma > precise_curr_lim)
	{
		// keep the currlim_mult
	}
	else if(currlim_mult < 255)
		currlim_mult++;

	if(currlim_mult < 5) currlim_mult = 5;
	stop_state_dbg[1] = stop_state;

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

	run[m] = 1;
	wanna_stop[m] = 0;
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
	run[m] = 0;
	bldc_pos_set[m] = bldc_pos[m];
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

void bldc_test()
{
	init_cpu_profiler();

	set_curr_lim(20000);

	while(1)
	{
		//profile_cpu_blocking_20ms();

//		for(int i=0; i<TRACE_LEN; i++)
//		{
//			DBG_PR_VAR_I16(ib_trace[i]);
//			DBG_PR_VAR_I16(ic_trace[i]);
//		}

//		int hall0 = hall_loc[MC0_HALL_CBA()];
//		int hall1 = hall_loc[MC1_HALL_CBA()];

//		DBG_PR_VAR_I16(hall0);
//		DBG_PR_VAR_I16(hall1);
		DBG_PR_VAR_I32(run[0]);
		DBG_PR_VAR_I32(run[1]);
		DBG_PR_VAR_I32(bldc_pos_set[0]);
		DBG_PR_VAR_I32(bldc_pos[0]);
		DBG_PR_VAR_I32(bldc_pos_set[1]);
		DBG_PR_VAR_I32(bldc_pos[1]);


		DBG_PR_VAR_I32(currlim_mult_flt>>8);
		DBG_PR_VAR_I32(curr_info_ma[0]);
		DBG_PR_VAR_I32(curr_info_ma[1]);
		DBG_PR_VAR_I32(dbg_mult);
//		DBG_PR_VAR_I32(timing_shift>>16);

		DBG_PR_VAR_I32(pid_i_max);
		DBG_PR_VAR_I32(pid_int_info);

		DBG_PR_VAR_I32(pid_p);
		DBG_PR_VAR_I32(pid_i);
		DBG_PR_VAR_I32(pid_d);

		DBG_PR_VAR_I32(stop_state_dbg[0]);
		DBG_PR_VAR_I32(stop_state_dbg[1]);

		DBG_PR_VAR_I32(dbg_err);
		DBG_PR_VAR_I32(dbg_sin_mult);
		DBG_PR_VAR_I32(dbg_m);


		uart_print_string_blocking("\r\n");
	
		for(int i=0; i<100; i++)
		{
			uint8_t cmd = uart_input();

			if(cmd == 'o')
			{
				motor_stop_now(0);
				motor_stop_now(1);
				motor_run(0);
				motor_run(1);
			}
			else if(cmd == 'p' || cmd == 'P')
			{
				motor_release(0);
				motor_release(1);
			}
			else if(cmd == '1')
			{
				motor_run(0);
				motor_run(1);
				bldc_pos_set[0]-=SUBSTEPS;
				bldc_pos_set[1]-=SUBSTEPS;
			}
			else if(cmd == '2')
			{
				motor_run(0);
				motor_run(1);
				bldc_pos_set[0]+=SUBSTEPS;
				bldc_pos_set[1]+=SUBSTEPS;
			}
			else if(cmd == '!')
			{
				motor_run(0);
				motor_run(1);
				bldc_pos_set[0]-=4*SUBSTEPS;
				bldc_pos_set[1]-=4*SUBSTEPS;
			}
			else if(cmd == '\"')
			{
				motor_run(0);
				motor_run(1);
				bldc_pos_set[0]+=4*SUBSTEPS;
				bldc_pos_set[1]+=4*SUBSTEPS;
			}
			else if(cmd == '8')
			{
				motor_run(0);
				motor_run(1);
				bldc_pos_set[0]-=1*SUBSTEPS;
				bldc_pos_set[1]+=1*SUBSTEPS;
			}
			else if(cmd == '9')
			{
				motor_run(0);
				motor_run(1);
				bldc_pos_set[0]+=1*SUBSTEPS;
				bldc_pos_set[1]-=1*SUBSTEPS;
			}
			else if(cmd == '(')
			{
				motor_run(0);
				motor_run(1);
				bldc_pos_set[0]-=4*SUBSTEPS;
				bldc_pos_set[1]+=4*SUBSTEPS;
			}
			else if(cmd == ')')
			{
				motor_run(0);
				motor_run(1);
				bldc_pos_set[0]+=4*SUBSTEPS;
				bldc_pos_set[1]-=4*SUBSTEPS;
			}
			else if(cmd == 'q')
			{
				motor_let_stop(0);
				motor_let_stop(1);
			}
			else if(cmd == 'r')
			{
				gospeed = 48;
			}
			else if(cmd == 't')
			{
				gospeed = 24;
			}
			else if(cmd == 'y')
			{
				gospeed = 12;
			}
			else if(cmd == 'u')
			{
				gospeed = 6;
			}
			else if(cmd == 'i')
			{
				gospeed = 3;
			}
			else if(cmd == 'G')
			{
				go = 1;
				motor_run(0);
				motor_run(1);
			}
			else if(cmd == 'g' || cmd == 'h')
			{
				go = 0;
				motor_let_stop(0);
				motor_let_stop(1);
			}
			else if(cmd == 'H')
			{
				go = 2;
				motor_run(0);
				motor_run(1);
			}
			else if(cmd == 'a')
				pid_p += 10;
			else if(cmd == 'z')
				pid_p -= 10;
			else if(cmd == 's')
				pid_i += 10;
			else if(cmd == 'x')
				pid_i -= 10;
			else if(cmd == 'd')
				pid_d += 10;
			else if(cmd == 'c')
				pid_d -= 10;

			delay_ms(1);

		}		

	}
}

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
