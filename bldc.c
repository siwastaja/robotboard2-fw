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


int timing_shift = -5000*65536; // hall sensors are not exactly where you could think they are.

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
/*
const int base_hall_aims[6] =
{
	0,
	PH120SHIFT/2,
	PH120SHIFT,
	PH120SHIFT+PH120SHIFT/2,
	PH120SHIFT*2,
	PH120SHIFT*2+PH120SHIFT/2
};
*/
const int base_hall_aims[6] =
{
	PH120SHIFT*2+PH120SHIFT/2,
	PH120SHIFT*2,
	PH120SHIFT+PH120SHIFT/2,
	PH120SHIFT,
	PH120SHIFT/2,
	0
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

#define TRACE_LEN 8192
int trace_at = -1;

typedef struct __attribute__((packed))
{
	int16_t ib;
	int16_t sp_ib;
	uint16_t pwmb;
	int16_t ic;
	int16_t sp_ic;
	uint16_t pwmc;
} trace_elem_t;

volatile trace_elem_t trace[TRACE_LEN];


volatile int latest_ia, latest_ib, latest_ic;
volatile int max_ia, max_ib, max_ic;
volatile int min_ia, min_ib, min_ic;
//volatile int sp_ib, sp_ic;

volatile int dbg_pwma, dbg_pwmb, dbg_pwmc;

volatile int cc_fw = 30000; //21845; // 1000 PWM units result in 3000 current units -> 1/3 = 21845>>16
volatile int cc_p = 0;
volatile int cc_i = 0;
volatile int errint_ib, errint_ic;
volatile int powder;
volatile int mode;

volatile int stop_state_dbg[2];
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
	int ia = -ib - ic;

	latest_ia = ia;
	latest_ib = ib;
	latest_ic = ic;

	if(ia > max_ia) max_ia = ia;
	if(ib > max_ib) max_ib = ib;
	if(ic > max_ic) max_ic = ic;

	if(ia < min_ia) min_ia = ia;
	if(ib < min_ib) min_ib = ib;
	if(ic < min_ic) min_ic = ic;

	int pwma, pwmb, pwmc;


	int sp_ib, sp_ic;

	static int loc;

	if(mode == 0)
	{
		int hall_pos = hall_loc[MC0_HALL_CBA()];
		loc = base_hall_aims[hall_pos] + timing_shift + PH90SHIFT; // (reverse?(-PH90SHIFT):(PH90SHIFT));
		
	}
	else if(mode == 1)
	{
		loc += PH01SHIFT;
	}
	else
	{
		static int prev_hall_pos;
		int hall_pos = hall_loc[MC0_HALL_CBA()];

		static int pekka;
		if(prev_hall_pos != hall_pos)
		{
			loc = base_hall_aims[hall_pos] + timing_shift + 65*PH1SHIFT; // (reverse?(-PH90SHIFT):(PH90SHIFT));
			pekka = 0;
		}
		else
		{
			if(pekka < 50*PH1SHIFT)
			{
				pekka += PH01SHIFT + 5*PH001SHIFT;
				loc += PH01SHIFT + 5*PH001SHIFT;
			}
		}

		prev_hall_pos = hall_pos;
	}

	int idxb = (((uint32_t)loc+PH120SHIFT)&0xff000000)>>24;
	int idxc = (((uint32_t)loc+2*PH120SHIFT)&0xff000000)>>24;

	if(powder == 1)
	{
		sp_ib = (sine[idxb]>>6);
		sp_ic = (sine[idxc]>>6);
	}
	else if(powder == 2)
	{
		sp_ib = (sine[idxb]>>5);
		sp_ic = (sine[idxc]>>5);
	}
	else if(powder == 3)
	{
		sp_ib = (sine[idxb]>>4);
		sp_ic = (sine[idxc]>>4);
	}
	else
	{
		sp_ib = 0;
		sp_ic = 0;
	}


	int err_ib = sp_ib - ib;
	int err_ic = sp_ic - ic;


	errint_ib += err_ib;
	errint_ic += err_ic;

	#define MAXI (16*5000)
	if(errint_ib > MAXI) errint_ib = MAXI;
	if(errint_ib < -MAXI) errint_ib = -MAXI;
	if(errint_ic > MAXI) errint_ic = MAXI;
	if(errint_ic < -MAXI) errint_ic = -MAXI;

	pwmb = (((int64_t)cc_fw * (int64_t)sp_ib)>>16) + 
	       (((int64_t)cc_p  * (int64_t)err_ib)>>16) +
	       (((int64_t)cc_i  * (int64_t)errint_ib)>>16);

	pwmc = (((int64_t)cc_fw * (int64_t)sp_ic)>>16) + 
	       (((int64_t)cc_p  * (int64_t)err_ic)>>16) +
	       (((int64_t)cc_i  * (int64_t)errint_ic)>>16);

	pwma = -pwmb -pwmc;

	pwma += PWM_MID;
	pwmb += PWM_MID;
	pwmc += PWM_MID;

	if(pwma < 2000 || pwma > 6000 || pwmb < 2000 || pwmb > 6000 || pwmc < 2000 || pwmc > 6000)
	{
		MC0_DIS_GATE();
		error(17);
	}

	dbg_pwma = pwma;
	dbg_pwmb = pwmb;
	dbg_pwmc = pwmc;

	TIM1->CCR1 = pwma;
	TIM1->CCR2 = pwmb;
	TIM1->CCR3 = pwmc;

	if(trace_at >= 0)
	{
		trace[trace_at].ib = ib;
		trace[trace_at].sp_ib = sp_ib;
		trace[trace_at].pwmb = pwmb;
		trace[trace_at].ic = ic;
		trace[trace_at].sp_ic = sp_ic;
		trace[trace_at].pwmc = pwmc;

		trace_at++;
		if(trace_at == TRACE_LEN)
			trace_at = -1;
	}
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

	int tmp_p = 12000, tmp_i = 400;

	int do_trace = 0;
	while(1)
	{
		//profile_cpu_blocking_20ms();

		DIS_IRQ();
		int ia = latest_ia;
		int ib = latest_ib;
		int ic = latest_ic;
		ENA_IRQ();

		if(do_trace && trace_at == -1) // Trace done
		{
			do_trace = 0;
			uart_print_string_blocking("ib,sp_ib,pwmb,ic,sp_ic,pwmc\r\n");
			for(int i=0; i<TRACE_LEN; i++)
			{
				o_itoa16(trace[i].ib, printbuf); uart_print_string_blocking(printbuf);
				uart_print_string_blocking(",");
				o_itoa16(trace[i].sp_ib, printbuf); uart_print_string_blocking(printbuf);
				uart_print_string_blocking(",");
				o_itoa16(trace[i].pwmb, printbuf); uart_print_string_blocking(printbuf);
				uart_print_string_blocking(",");
				o_itoa16(trace[i].ic, printbuf); uart_print_string_blocking(printbuf);
				uart_print_string_blocking(",");
				o_itoa16(trace[i].sp_ic, printbuf); uart_print_string_blocking(printbuf);
				uart_print_string_blocking(",");
				o_itoa16(trace[i].pwmc, printbuf); uart_print_string_blocking(printbuf);
				uart_print_string_blocking("\r\n");
			}
		}


		int hall = hall_loc[MC0_HALL_CBA()];


		uart_print_string_blocking(" ia = "); o_itoa16_fixed(ia, printbuf); uart_print_string_blocking(printbuf);
		uart_print_string_blocking(" ib = "); o_itoa16_fixed(ib, printbuf); uart_print_string_blocking(printbuf);
		uart_print_string_blocking(" ic = "); o_itoa16_fixed(ic, printbuf); uart_print_string_blocking(printbuf);
		uart_print_string_blocking(" hall = "); o_utoa8_fixed(hall, printbuf); uart_print_string_blocking(printbuf);

		uart_print_string_blocking("\r\n");
/*
		int idxa = (((uint32_t)loc)&0xff000000)>>24;
		int idxb = (((uint32_t)loc+PH120SHIFT)&0xff000000)>>24;
		int idxc = (((uint32_t)loc+2*PH120SHIFT)&0xff000000)>>24;


		uart_print_string_blocking(" loc = "); o_itoa16_fixed(loc>>16, printbuf); uart_print_string_blocking(printbuf);
		uart_print_string_blocking(" idxa = "); o_utoa16_fixed(idxa, printbuf); uart_print_string_blocking(printbuf);
		uart_print_string_blocking(" idxb = "); o_utoa16_fixed(idxb, printbuf); uart_print_string_blocking(printbuf);
		uart_print_string_blocking(" idxc = "); o_utoa16_fixed(idxc, printbuf); uart_print_string_blocking(printbuf);
*/
		uart_print_string_blocking("\r\n");

		uart_print_string_blocking(" cc_fw = "); o_utoa16_fixed(cc_fw, printbuf); uart_print_string_blocking(printbuf);
		uart_print_string_blocking(" cc_p = "); o_utoa16_fixed(cc_p, printbuf); uart_print_string_blocking(printbuf);
		uart_print_string_blocking(" cc_i = "); o_utoa16_fixed(cc_i, printbuf); uart_print_string_blocking(printbuf);

		uart_print_string_blocking("\r\n");

//		uart_print_string_blocking(" sp_ib = "); o_itoa16_fixed(sp_ib, printbuf); uart_print_string_blocking(printbuf);
//		uart_print_string_blocking(" sp_ic = "); o_itoa16_fixed(sp_ic, printbuf); uart_print_string_blocking(printbuf);
		uart_print_string_blocking(" errint_ib = "); o_itoa32(errint_ib, printbuf); uart_print_string_blocking(printbuf);
		uart_print_string_blocking(" errint_ic = "); o_itoa32(errint_ic, printbuf); uart_print_string_blocking(printbuf);

		uart_print_string_blocking("\r\n");

		uart_print_string_blocking(" pwma = "); o_utoa16_fixed(dbg_pwma, printbuf); uart_print_string_blocking(printbuf);
		uart_print_string_blocking(" pwmb = "); o_utoa16_fixed(dbg_pwmb, printbuf); uart_print_string_blocking(printbuf);
		uart_print_string_blocking(" pwmc = "); o_utoa16_fixed(dbg_pwmc, printbuf); uart_print_string_blocking(printbuf);

		uart_print_string_blocking("\r\n");

		uart_print_string_blocking("\r\n");

		for(int i=0; i<100; i++)
		{
			delay_ms(1);

			uint8_t cmd = uart_input();

			if(cmd == '0')
			{
				errint_ib = 0;
				errint_ic = 0;
				powder = 0;
//				sp_ib = 0;
//				sp_ic = 0;
			}
			else if(cmd == '1')
			{
				errint_ib = 0;
				errint_ic = 0;
				powder = 1;
//				sp_ib = (sine[idxb]>>6);
//				sp_ic = (sine[idxc]>>6);
			}
			else if(cmd == '2')
			{
				errint_ib = 0;
				errint_ic = 0;
				powder = 2;
//				sp_ib = (sine[idxb]>>5);
//				sp_ic = (sine[idxc]>>5);
			}
			else if(cmd == '3')
			{
				errint_ib = 0;
				errint_ic = 0;
				powder = 3;
//				sp_ib = (sine[idxb]>>4);
//				sp_ic = (sine[idxc]>>4);
			}
			else if(cmd == 'q')
			{
				loc -= 1<<24;
			}
			else if(cmd == 'w')
			{
				loc += 1<<24;
			}
			else if(cmd == 'a')
			{
				MC0_EN_GATE();
			}
			else if(cmd == 's')
			{
				MC0_DIS_GATE();
			}
			else if(cmd == 'P')
			{
				tmp_p += 200;
				cc_p = tmp_p;
			}
			else if(cmd == 'p')
			{
				if(tmp_p > 200)
					tmp_p -= 200;
				else
					tmp_p = 0;
				cc_p = tmp_p;
			}
			else if(cmd == 'I')
			{
				tmp_i += 10;
				cc_i = tmp_i;
			}
			else if(cmd == 'i')
			{
				if(tmp_i > 10)
					tmp_i -= 10;
				else
					tmp_i = 0;
				cc_i = tmp_i;
			}
			else if(cmd == 'z')
			{
				cc_fw = 30000;
				cc_i = 0;
				cc_p = 0;
			}
			else if(cmd == 'x')
			{
				cc_fw = 30000;
				cc_i = tmp_i;
				cc_p = tmp_p;
			}
			else if(cmd == '5')
			{
				mode = 0;
			}
			else if(cmd == '6')
			{
				mode = 1;
			}
			else if(cmd == '7')
			{
				mode = 2;
			}
			else if(cmd == 't')
			{
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
