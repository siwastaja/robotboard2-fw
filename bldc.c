#include <stdint.h>
#include "ext_include/stm32h7xx.h"
#include "own_std.h"
#include "stm32_cmsis_extension.h"
#include "misc.h"

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
#define MC0_HALL_CBA() ( (GPIOD->IDR & (0b11<<14))>>14) | (GPIOG->IDR & (1<<2)) )
#define MC1_HALL_CBA() ( (GPIOE->IDR & (0b11<<5))>>5) | ((GPIOP->IDR & (1<<8))>>6) )

#define DLED_ON() {}

int timing_shift = 5000*65536; // hall sensors are not exactly where you could think they are.

volatile int precise_curr_lim; // if exceeded, the duty cycle multiplier is kept (not increased like normally)
volatile int higher_curr_lim;  // if exceeded, the duty cycle multiplier is lowered

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

#define PH120SHIFT (1431655765UL)  // 120 deg shift between the phases in the 32-bit range.
#define PH90SHIFT (1073741824UL)   // 90 deg shift between the phases in the 32-bit range.
#define PH45SHIFT (PH90SHIFT/2)

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


/*
	set_curr_lim sets the ADC based normal current limit, and also the protection limit.
	The protection limit is set considerably higher so that it should never be hit if the ADC-based
	works as it should.
*/

// 1024 equals 3V3; after gain=40, it's 82.5mV; with 1mOhm, it's 82.5A.
// i.e., 81mA per unit.
// 1024 equals 3V3; after gain=40, it's 82.5mV; with 1.5mOhm, it's 55.0A.
// i.e., 55mA per unit.

/*
#ifdef PCB1A
#define CUR_LIM_DIV 81
#endif
#ifdef PCB1B
#define CUR_LIM_DIV 55
#endif

void set_curr_lim(int ma)
{
	static int prev_val = 9999999;

	if(ma == prev_val)
		return;

	prev_val = ma;

	if(ma < 0 || ma > 25000)
		ma = 0;

	int lim = (7*ma)/(8*CUR_LIM_DIV);
	int hilim = (6*lim)/5;
	__disable_irq();
	precise_curr_lim = lim;
	higher_curr_lim = hilim;
	__enable_irq();

	// Protection limit set at 1.25x soft limit + 2A constant extra.
	set_prot_lim(((ma*5)>>2)+2000);
}
*/

static volatile uint8_t pid_i_max = 30;
static volatile uint8_t pid_feedfwd = 30;
static volatile uint8_t pid_p = 80;
static volatile uint8_t pid_i = 50;
static volatile uint8_t pid_d = 50;

#if 0
void tim1_inthandler()
{
	static int reverse = 0;
	static int prev_reverse = 1;
	static int mult = 0;
	static int currlim_mult = 255;
	static int32_t cnt = 0;
	static int expected_next_hall_pos;
	static int expected_fwd_hall_pos;  // for step counting only
	static int expected_back_hall_pos; // for step counting only
	static int32_t expected_next_hall_cnt;
	static int cnt_at_prev_hall_valid = 0;
	static int cnt_at_prev_hall;
	static int prev_hall_pos;
	static int resync = 5;
	static int16_t pos_info;
	static int prev_ferr = 0;
	static int f = 0;
	static int loc;
	static int pid_f_set;
	static int64_t pid_integral = 0;

	DLED_ON();
	TIM1->SR = 0; // Clear interrupt flags

	int hall_pos = hall_loc[HALL_ABC()];
	if(hall_pos == -1) hall_pos = prev_hall_pos;

	if(pid_f_set < 0 ) reverse = 1; else reverse = 0;
	if(reverse != prev_reverse)
	{
		resync = 2;
		f = 0;
		pid_integral = 0;
	}
	prev_reverse = reverse;

/*
	Note: this code did sine interpolation earlier on. Now the relevant parts are commented out so it does
	a simple 6-step: it's working more robustly at low speeds.
*/

	if(resync)
	{
		loc = (base_hall_aims[hall_pos] + timing_shift + (reverse?(-PH90SHIFT):(PH90SHIFT)));
		cnt_at_prev_hall_valid = 0;
		expected_next_hall_cnt = cnt+22000;
		f = 0;
	}
	else if(cnt_at_prev_hall_valid && hall_pos == prev_hall_pos) // We haven't advanced a step yet, do sine interpolation
	{
		loc = (base_hall_aims[hall_pos] + timing_shift + (reverse?(-PH90SHIFT):(PH90SHIFT)));
		// Interpolation freezes if the hall sync is not received on time.
		if((cnt - expected_next_hall_cnt) < 0) // this comparison works with counter wrapping around.
		{
			// Sine interpolation hasn't advanced to the next hall sync point, run it
//			if(reverse)
//				loc -= f;
//			else
//				loc += f;
		}
		else // We are overdue: recalculate the freq.
		{
			f = (PH120SHIFT)  /  ((cnt-cnt_at_prev_hall));
		}

		// TODO: prev_error feedforward to frequency generation.
	}
	else if(hall_pos == expected_next_hall_pos) 
	{
		// We have advanced one step in the right dirction - we can synchronize the sine phase, 
		// and calculate a valid frequency from the delta time.
		loc = (base_hall_aims[hall_pos] + timing_shift + (reverse?(-PH90SHIFT):(PH90SHIFT)));

		if(cnt_at_prev_hall_valid)
		{
//			f = (reverse?(-PH120SHIFT):(PH120SHIFT))  /  ((cnt-cnt_at_prev_hall));
			f = (PH120SHIFT)  /  ((cnt-cnt_at_prev_hall));
			expected_next_hall_cnt = cnt+(cnt-cnt_at_prev_hall);
		}
		else
		{
			f = 0;
			expected_next_hall_cnt = cnt+22000; // at smallest possible freq, we'll wait for 0.5 sec for the next hall pulse.
		}

		cnt_at_prev_hall_valid = 1;
		cnt_at_prev_hall = cnt;
	}
	else // We are lost - synchronize the phase to the current hall status
	{
		//lost_count++;
		loc = (base_hall_aims[hall_pos] + timing_shift + (reverse?(-PH90SHIFT):(PH90SHIFT)));
		cnt_at_prev_hall_valid = 0;
		expected_next_hall_cnt = cnt+22000;
		f = 0;
	}

	if(resync) resync--;

	if(hall_pos == expected_fwd_hall_pos)
	{
		pos_info++;
//		spi_tx_data.pos = pos_info;
	}
	else if(hall_pos == expected_back_hall_pos)
	{
		pos_info--;
//		spi_tx_data.pos = pos_info;
	}


	prev_hall_pos = hall_pos;

	expected_next_hall_pos = hall_pos;
	if(!reverse) { expected_next_hall_pos++; if(expected_next_hall_pos > 5) expected_next_hall_pos = 0; }
	else         { expected_next_hall_pos--; if(expected_next_hall_pos < 0) expected_next_hall_pos = 5; }

	expected_fwd_hall_pos  = hall_pos+1; if(expected_fwd_hall_pos > 5) expected_fwd_hall_pos = 0;
	expected_back_hall_pos = hall_pos-1; if(expected_back_hall_pos < 0) expected_back_hall_pos = 5;

	int idxa = (((uint32_t)loc)&0xff000000)>>24;
	int idxb = (((uint32_t)loc+PH120SHIFT)&0xff000000)>>24;
	int idxc = (((uint32_t)loc+2*PH120SHIFT)&0xff000000)>>24;

	// Ramp the setpoint.
	int next_pid_f_set = (spi_rx_data.speed*100);

	if(next_pid_f_set > pid_f_set) pid_f_set+=256;
	else if(next_pid_f_set < pid_f_set) pid_f_set-=256;
	if((next_pid_f_set - pid_f_set) > -256 && (next_pid_f_set - pid_f_set) < 256) next_pid_f_set = pid_f_set;

	int pid_f_meas = f>>3;
	if(reverse) pid_f_meas *= -1;

	int ferr;

	ferr = pid_f_set - pid_f_meas;

//	spi_tx_data.speed = pid_f_meas>>8;

	// Run the speed PID loop - not on every ISR cycle
	if(!(cnt & 15))
	{
		int dferr = (ferr - prev_ferr);
		prev_ferr = ferr;

		pid_integral += ferr;

		int64_t pid_i_max_extended = (int64_t)pid_i_max<<25;
		int64_t pid_i_min_extended = -1*pid_i_max_extended;
		if(pid_integral > pid_i_max_extended) pid_integral = pid_i_max_extended;
		else if(pid_integral < pid_i_min_extended) pid_integral = pid_i_min_extended;

		mult = (((int64_t)pid_feedfwd*(int64_t)pid_f_set)>>10) /* feedforward */
			+ (((int64_t)pid_p*(int64_t)ferr)>>9) /* P */
			+ (((int64_t)pid_i*(int64_t)pid_integral)>>21)  /* I */
			+ (((int64_t)pid_d*(int64_t)dferr)>>14); /* D */

	}

	
	#define MAX_MULT (200*256)
	#define MIN_MULT (-200*256)

	if(mult > MAX_MULT) mult = MAX_MULT;
	else if(mult < MIN_MULT) mult = MIN_MULT;

	int sin_mult = mult>>8;

	if(reverse) sin_mult *= -1;	

	if(sin_mult < 0) sin_mult = 0;

#define MIN_MULT_OFFSET 8 // no point in outputting extremely low amplitudes
	if(sin_mult != 0)
		sin_mult += MIN_MULT_OFFSET;


	uint8_t m = (sin_mult * currlim_mult)>>8; // 254 max
	TIM1->CCR1 = (PWM_MID) + ((m*sine[idxa])>>14); // 4 to 1019. 
	TIM1->CCR2 = (PWM_MID) + ((m*sine[idxb])>>14);
	TIM1->CCR3 = (PWM_MID) + ((m*sine[idxc])>>14);

	cnt++;

	int current_b = latest_adc[0].cur_b-dccal_b;
	int current_c = latest_adc[0].cur_c-dccal_c;

	if(current_b < 0) current_b *= -1;
	if(current_c < 0) current_c *= -1;

	int current;
	if(current_c > current_b) current = current_c; else current = current_b;

	static int32_t current_flt = 0;

	current_flt = ((current<<8) + 63*current_flt)>>6;

	int32_t current_ma = (current_flt>>8)*CUR_LIM_DIV;
	if(current_ma > 32767) current_ma = 32767; else if(current_ma < -32768) current_ma = -32768;
//	spi_tx_data.current = current_ma;


	if(OVERCURR()) // hard overcurrent, protection has acted, quickly ramp down the multiplier to avoid hitting it again
	{
//		spi_tx_data.num_hard_limits++;
		LED_ON(); led_short = 0;
		currlim_mult-=40;
	}
	else if(current > higher_curr_lim)
	{
		LED_ON(); led_short = 1;
		currlim_mult-=2;
	}
	else if(current_flt > precise_curr_lim)
	{
		// keep the currlim_mult
	}
	else if(currlim_mult < 255)
		currlim_mult++;

	if(currlim_mult < 5) currlim_mult = 5;

	static int32_t currlim_mult_flt = 0;

	currlim_mult_flt = ((currlim_mult<<8) + 63*currlim_mult_flt)>>6;

//	spi_tx_data.cur_limit_mul = currlim_mult_flt>>8;


	DLED_OFF();
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


Timers run at 200MHz
Timer is up/down counting, halving the freq.
With PWM_MAX = 8192, PWM will run at 12.2kHz


*/
void init_bldc()
{
	MC0_DIS_GATE();
	MC1_DIS_GATE();
	IO_TO_GPO(GPIOG,8);
	IO_TO_GPO(GPIOI,4);

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


//	TIM1->CCR1 = PWM_MID; // Sync pulse for 90 deg phase diff
	TIM1->CCR1 = PWM_MAX-10; // Sync pulse for 180 deg phase diff.

	// Tested that "1" is actually long enough to sync. Using a bit wider pulse to be sure.

	TIM1->ARR = PWM_MAX;

//	TIM1->CCR1 = PWM_MID;
	TIM1->CCR2 = PWM_MID/2;
	TIM1->CCR3 = PWM_MID;
	TIM1->CCR4 = 1020; // Generate the ADC trigger near the middle of the PWM waveforms
	TIM1->BDTR = 1UL<<15 /*Main output enable*/ | 1UL /*21ns deadtime*/;
	TIM1->EGR |= 1; // Generate Reinit+update
//	TIM1->DIER = 1UL /*Update interrupt enable*/;
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
	TIM8->CCR2 = PWM_MID+PWM_MID/2;
	TIM8->CCR3 = PWM_MID;
	TIM8->CCR4 = 1020; // Generate the ADC trigger near the middle of the PWM waveforms
	TIM8->BDTR = 1UL<<15 /*Main output enable*/ | 1UL /*21ns deadtime*/;
	TIM8->EGR |= 1; // Generate Reinit+update
//	TIM8->DIER = 1UL /*Update interrupt enable*/;
	__DSB();
	// Don't enable the timer, let it sync to TIM1

	while(!(TIM8->CR1 & 1UL)) ;
	__DSB();

	TIM1->CCR1 = PWM_MID;


	// TIM1:
	IO_ALTFUNC(GPIOA,8,  1);
	IO_ALTFUNC(GPIOA,9,  1);
	IO_ALTFUNC(GPIOA,10, 1);

	// TIM8:
	IO_ALTFUNC(GPIOI,5,  3);
	IO_ALTFUNC(GPIOI,6,  3);
	IO_ALTFUNC(GPIOI,7,  3);

//	NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);

}
