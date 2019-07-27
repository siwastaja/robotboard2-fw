#include <stdint.h>
#include "misc.h"
#include "own_std.h"
#include "ext_include/stm32h7xx.h"
#include "ext_vacuum_boost.h"
#include "stm32_cmsis_extension.h"
#include "adcs.h"
#include "pwrswitch.h"

#ifdef REV2A
#define NOZZLE_DIR_UP()    do{HI(GPIOI, 9);}while(0)
#define NOZZLE_DIR_DOWN()  do{LO(GPIOI, 9);}while(0)
#define NOZZLE_POWER_EN()  do{HI(GPIOH, 9);}while(0)
#define NOZZLE_POWER_DIS() do{LO(GPIOH, 9);}while(0)
#endif

#ifdef REV2B
#define NOZZLE_DIR_UP()    do{PO6_ON();}while(0)
#define NOZZLE_DIR_DOWN()  do{PO6_OFF();}while(0)
#define NOZZLE_POWER_EN()  do{PO1_ON();}while(0)
#define NOZZLE_POWER_DIS() do{PO1_OFF();}while(0)

#endif

typedef enum
{
	IDLE = 0,
	NOZZLE_RISE_START = 1,
	NOZZLE_RISE_RUN = 2,
	NOZZLE_LOWER_START = 3,
	NOZZLE_LOWER_RUN = 4
}
state_t;

static state_t state;

void ext_vacuum_fsm()
{
	static int cnt;
	switch(state)
	{
		case IDLE:
		{
			cnt = 0;
			NOZZLE_DIR_DOWN();
			NOZZLE_POWER_DIS();
		}
		break;

		case NOZZLE_RISE_START:
		{
			NOZZLE_DIR_UP();
			NOZZLE_POWER_DIS();

			if(++cnt >= 50)
			{
				state = NOZZLE_RISE_RUN;
				cnt = 0;
			}
		}
		break;

		case NOZZLE_RISE_RUN:
		{
			NOZZLE_DIR_UP();
			NOZZLE_POWER_EN();

			if(++cnt >= 3000)
			{
				state = IDLE;
				cnt = 0;
			}
		}
		break;


		case NOZZLE_LOWER_START:
		{
			NOZZLE_DIR_DOWN();
			NOZZLE_POWER_DIS();

			if(++cnt >= 50)
			{
				state = NOZZLE_LOWER_RUN;
				cnt = 0;
			}
		}
		break;

		case NOZZLE_LOWER_RUN:
		{
			// Take a bit back
			if(cnt < 2300)
				NOZZLE_DIR_DOWN();
			else
				NOZZLE_DIR_UP();

			NOZZLE_POWER_EN();
			if(++cnt >= 2600)
			{
				state = IDLE;
				cnt = 0;
			}
		}
		break;


		default:
		{
			cnt = 0;
			NOZZLE_DIR_DOWN();
			NOZZLE_POWER_DIS();
		}
		break;
	}

	#ifdef VACUUM_REV2
		void boost_converter_fsm();
		boost_converter_fsm();
	#endif
}

static void ext_vacuum_nozzle_rise()
{
	state = NOZZLE_RISE_START;
}

static void ext_vacuum_nozzle_lower()
{
	state = NOZZLE_LOWER_START;
}
volatile int target_power_bypass = 99;
volatile int nozzle_request = 1; // 1 = up, 0 = down
void ext_vacuum_freerunning_fsm(int temporary_rise)
{
	int rq = nozzle_request;
	if(temporary_rise)
	{
		rq = 1;
		target_power_bypass = 5;
	}
	else
	{
		if(target_power_bypass < 11)
			target_power_bypass++;
	}
	static int prev_rq = -1;

	if(rq != prev_rq)
	{
		if(rq == 0)
			ext_vacuum_nozzle_lower();
		else
			ext_vacuum_nozzle_rise();
	}

	prev_rq = rq;
}



#ifdef VACUUM_REV2

	/*
		BOOST	EXTB	EXTB	EXTB	ACTUAL
		PCB	CONN	"NAME"	PIN	FUNC
		
		2	6	MOSI	PF9	V_ADJ
		4	8	MISO	PF8	ILIM_ADJ
		6	10	SCK	PF7	ENABLE
		8	12	NCS	PF6	PGOOD
		10	14	SCL	PF1	(reserved for future use)


	*/

	#define VACUUM_BOOST_EN()  do{ HI(GPIOF,7); }while(0)
	#define VACUUM_BOOST_DIS() do{ LO(GPIOF,7); }while(0)

	static void vacuum_set_v(int v) // 0 .. 4
	{
		switch(v)
		{
			case 0:	// 46V: ctrl = +3V3
				HI(GPIOF,9);
				IO_PULLUPDOWN_OFF(GPIOF,9);
				IO_TO_GPO(GPIOF,9);
			break;

			case 1:	// 55V: ctrl = pullup 40k to +3V3
				IO_PULLUP_ON(GPIOF,9);
				IO_TO_GPI(GPIOF,9);
			break;

			case 2:	// 64V: ctrl = floating
				IO_PULLUPDOWN_OFF(GPIOF,9);
				IO_TO_GPI(GPIOF,9);
			break;

			case 3:	// 69V: ctrl = pulldown 40k to GND
				IO_PULLDOWN_ON(GPIOF,9);
				IO_TO_GPI(GPIOF,9);
			break;

			case 4:	// 74V: ctrl = GND
				LO(GPIOF,9);
				IO_PULLUPDOWN_OFF(GPIOF,9);
				IO_TO_GPO(GPIOF,9);
			break;

			default:
				error(251);
		}
	}

	static void vacuum_set_ilim(int i) // 0 .. 3
	{
		switch(i)
		{
			case 0: // 19A: ctrl = GND
				LO(GPIOF,8);
				IO_PULLUPDOWN_OFF(GPIOF,8);
				IO_TO_GPO(GPIOF,8);
			break;

			case 1: // 26A: ctrl = Pulldown 40k to GND
				IO_PULLDOWN_ON(GPIOF,8);
				IO_TO_GPI(GPIOF,8);
			break;

			case 2: // 31A: ctrl = float
				IO_PULLUPDOWN_OFF(GPIOF,8);
				IO_TO_GPI(GPIOF,8);
			break;

			case 3: // 35A: ctrl = +3V3
				HI(GPIOF,8);
				IO_PULLUPDOWN_OFF(GPIOF,8);
				IO_TO_GPO(GPIOF,8);
			break;

			default:
				error(251);

		}
	}

	void ext_safety_shutdown() __attribute__((section(".text_itcm")));
	void ext_safety_shutdown()
	{
		VACUUM_BOOST_DIS();
	}


	#define IS_VACUUM_PGOOD() (IN_LOGICAL(GPIOF,6))

	static volatile int target_power; // 5..9 maps to voltage levels 0..4. Use 1 if you want to run really slow. Don't use 2..4, they are same as 5.
	static volatile int boost_fsm_cnt;
	static volatile int initialized;

	void boost_converter_fsm()
	{
		if(!initialized)
			return;

		static int cur_power = 0;

		if(cur_power < 0 || cur_power > 9 || target_power < 0 || target_power > 9 || target_power_bypass < 0)
			error(478);

		if(++boost_fsm_cnt >= 200)
		{
			boost_fsm_cnt = 0;
			if(target_power > cur_power && target_power_bypass >= cur_power)
				cur_power++;

			if(target_power < cur_power || target_power_bypass < cur_power)
				cur_power--;

			if(cur_power == 0)
			{
				VACUUM_BOOST_DIS();
				app_power_off();
			}
			else if(cur_power == 1)
			{
				VACUUM_BOOST_DIS();
				app_power_on();
			}
			else if(cur_power < 5)
			{
				vacuum_set_v(0);
				vacuum_set_ilim(3);
				VACUUM_BOOST_EN();
				app_power_on();
			}
			else
			{
				vacuum_set_v(cur_power-5);
				vacuum_set_ilim(3);
				VACUUM_BOOST_EN();
				app_power_on();
			}


			// Monitor POWER_GOOD for safety (for broken/shorted vacuum cleaner, for example):
			static int stable_cnt;
			static int prev_cur_power;
			if(cur_power > 1 && prev_cur_power == cur_power)
			{
				if(stable_cnt > 5)
				{
					if(!IS_VACUUM_PGOOD())
					{
						error(333);
					}
				}
				else
					stable_cnt++;
			}
			else
				stable_cnt=0;

			prev_cur_power = cur_power;

		}

	}

	#if 0
	static void vacuum_boost_test()
	{
		static int cnt = 0;

		cnt++;
		if(cnt == 8000)
		{
			DBG_PR_VAR_U16(IS_VACUUM_PGOOD());
			cnt = 0;
		}

		uint8_t cmd = uart_input();
		if(cmd == '1')
			vacuum_set_v(0);
		else if(cmd == '2')
			vacuum_set_v(1);
		else if(cmd == '3')
			vacuum_set_v(2);
		else if(cmd == '4')
			vacuum_set_v(3);
		else if(cmd == '5')
			vacuum_set_v(4);
		else if(cmd == 'q')
			vacuum_set_ilim(0);
		else if(cmd == 'w')
			vacuum_set_ilim(1);
		else if(cmd == 'e')
			vacuum_set_ilim(2);
		else if(cmd == 'r')
			vacuum_set_ilim(3);
		else if(cmd == 'o')
		{
			app_power_on();
		}
		else if(cmd == 'p')
		{
			app_power_off();
		}
		else if(cmd == 'O')
		{
			VACUUM_BOOST_EN();
		}
		else if(cmd == 'P')
		{
			VACUUM_BOOST_DIS();
		}
		else if(cmd == 'k')
		{
			NOZZLE_DIR_UP();
			delay_ms(50);
			NOZZLE_POWER_EN();
			delay_ms(3000);
			NOZZLE_POWER_DIS();
			NOZZLE_DIR_DOWN();
		}
		else if(cmd == 'm')
		{
			NOZZLE_DIR_DOWN();
			delay_ms(50);
			NOZZLE_POWER_EN();
			delay_ms(3000);
			NOZZLE_POWER_DIS();
			NOZZLE_DIR_DOWN();
		}

		delay_us(12);
	}

	static void hilevel_test()
	{
		void ext_vacuum_cmd(int power, int nozzle);

		uint8_t cmd = uart_input();
		if(cmd == '0')
			ext_vacuum_cmd(0, 1);
		if(cmd == '1')
			ext_vacuum_cmd(11, 0);
		if(cmd == '2')
			ext_vacuum_cmd(22, 0);
		if(cmd == '3')
			ext_vacuum_cmd(33, 0);
		if(cmd == '4')
			ext_vacuum_cmd(44, 0);
		if(cmd == '5')
			ext_vacuum_cmd(55, 0);
		if(cmd == '6')
			ext_vacuum_cmd(66, 0);
		if(cmd == '7')
			ext_vacuum_cmd(77, 0);
		if(cmd == '8')
			ext_vacuum_cmd(88, 0);
		if(cmd == '9')
			ext_vacuum_cmd(100, 0);

		static int temprise;
		if(cmd == 'g')
			temprise = 500000;

		app_power_freerunning_fsm();
		ext_vacuum_freerunning_fsm(temprise);
		if(temprise) temprise--;

		delay_us(12);

	}
	#endif

	void init_ext_vacuum_boost()
	{
		VACUUM_BOOST_DIS();
		IO_TO_GPO(GPIOF,7);
		vacuum_set_v(0);
		vacuum_set_ilim(0);

		IO_TO_GPI(GPIOF,6);
		IO_PULLUP_ON(GPIOF, 6);

		initialized = 1;

//		while(1)
//			hilevel_test();
//			vacuum_boost_test();
	}


	void ext_vacuum_cmd(int power, int nozzle) // power 0 - 100. Nozzle: 0 down, 1 up
	{
		nozzle_request = nozzle;

		/*
			0 = off
			
		*/

		if(power < 0 || power > 100)
			error(334);

		power /= 10;
		if(power > 9) power = 9;
		if(power < 0) power = 0;
		if(power != 0 && power < 5) power = 1;


		// 5..9 maps to voltage levels 0..4. Use 1 if you want to run really slow. Don't use 2..4, they are same as 5.
		target_power = power; 
	}


#else
	#define VACUUM_BOOST_GATE_EN() do{ HI(GPIOF,7); }while(0)
	#define VACUUM_BOOST_GATE_DIS() do{ LO(GPIOF,7); }while(0)


	#define PERIOD 2000
	//#define MAX_DUTY ((PERIOD*6)/10)
	#define MAX_DUTY (950)
	#define DUTY_REG (TIM16->CCR1)

	volatile static int target_duty = PERIOD/2;
	volatile static int duty = 0;

	static volatile int intcnt;
	static void ext_vacuum_boost_inthandler() __attribute__((section(".text_itcm")));
	static void ext_vacuum_boost_inthandler()
	{
		static uint32_t cnt;
		cnt++;
		TIM16->SR = 0;
		__DSB();

		// 16384 ~ 3.3V ~ 66mV ~ 132A
		// For some reason, -8A offset. Can't read down to zero.
		int curr_a = 8+adc3.s.eb_analog2/124;

		// 114k - 1.5k divider
		// 16384 ~ 3.3V ~ 254V
		int outvolt = adc3.s.eb_analog1>>6;

		if(outvolt > 180)
		{
			VACUUM_BOOST_GATE_DIS();
			error(210);
		}

		if(curr_a > 60)
		{
			VACUUM_BOOST_GATE_DIS();
			error(211);
		}
		else if(curr_a > 53)
		{
			duty>>=2;
		}
		else if(curr_a > 48)
		{
			duty>>=1;
		}
		else if(curr_a > 45)
		{
			duty-=2;
		}
		else if(curr_a > 42)
		{
			duty--;
		}
		else
		{
			if(duty < target_duty)
			{
				if((cnt&255) == 255)
					duty++;
			}
		}

		if(duty > target_duty) duty = target_duty;

		if(duty > MAX_DUTY) duty = MAX_DUTY;
		else if(duty < 0) duty = 0;


		DUTY_REG = duty;


		intcnt++;
	}


	void ext_vacuum_power(int percent)
	{
		if(percent < 0 || percent > 100)
			error(251);

		if(percent > 0 && percent < 40)
			percent = 40;


		if(percent == 0)
		{
			duty = 0;
			target_duty = 0;
			VACUUM_BOOST_GATE_DIS();
			app_power_off();
		}
		else
		{
			duty = 0;
			target_duty = 9*percent;
			app_power_on();
			VACUUM_BOOST_GATE_EN();
		}
	}

	void ext_vacuum_cmd(int power, int nozzle) // power 0 - 100. Nozzle: 0 down, 1 up
	{
		nozzle_request = nozzle;

		int prev_power = -1;

		if(power != prev_power)
		{
			ext_vacuum_power(power);
		}

		prev_power = power;
	}

	static void vacuum_boost_test()
	{
		static int cnt = 0;

		cnt++;
		if(cnt == 8000)
		{
			DBG_PR_VAR_U16(adc3.s.eb_analog1);
			DBG_PR_VAR_U16(adc3.s.eb_analog2);
			DBG_PR_VAR_I32(intcnt);

			// 16384 ~ 3.3V ~ 66mV ~ 132A
			int curr_a = adc3.s.eb_analog2/124;

			// 114k - 1.5k divider
			// 16384 ~ 3.3V ~ 254V
			int outvolt = adc3.s.eb_analog1/64;

			DBG_PR_VAR_U16(curr_a);
			DBG_PR_VAR_U16(outvolt);
			cnt = 0;
		}

		uint8_t cmd = uart_input();
		if(cmd >= '0' && cmd <= '9')
		{
			int val = cmd-'0';
			val *= 100;
			target_duty = val;
	//		DUTY_REG = val;
		}
		else if(cmd == 'o')
		{
			duty = 0;
			app_power_on();
			VACUUM_BOOST_GATE_EN();
		}
		else if(cmd == 'p')
		{
			duty = 0;
			VACUUM_BOOST_GATE_DIS();
			app_power_off();
		}
		else if(cmd == 'k')
		{
			NOZZLE_DIR_UP();
			delay_ms(50);
			NOZZLE_POWER_EN();
			delay_ms(2000);
			NOZZLE_POWER_DIS();
			NOZZLE_DIR_DOWN();
		}
		else if(cmd == 'm')
		{
			NOZZLE_DIR_DOWN();
			delay_ms(50);
			NOZZLE_POWER_EN();
			delay_ms(2000);
			NOZZLE_POWER_DIS();
			NOZZLE_DIR_DOWN();
		}

		delay_us(12);
	}

	void ext_safety_shutdown() __attribute__((section(".text_itcm")));
	void ext_safety_shutdown()
	{
		VACUUM_BOOST_GATE_DIS();
		DUTY_REG = 0;
	}

	void init_ext_vacuum_boost()
	{
		VACUUM_BOOST_GATE_DIS();
		IO_TO_GPO(GPIOF,7);
		IO_SPEED(GPIOF,7,2);

		IO_ALTFUNC(GPIOF,6,1);
		IO_SPEED(GPIOF,6,3);

		RCC->APB2ENR |= 1UL<<17; // TIM16

	//	TIM16->CR1 = 1UL<<7 /*Auto-reload is buffered*/;
		TIM16->DIER = 1UL; // update interrupt
		TIM16->CCMR1 = 0b110UL<<4 /*PWM mode 1 (active from 0 until CCR)*/ | 1UL<<3 /*CCR preloader*/;
		TIM16->CCER = 1UL<<0; // Channel 1 output enable

		TIM16->ARR = PERIOD;

		DUTY_REG = 0;

		TIM16->BDTR = 1UL<<15 /*main output enable*/;

		*((volatile uint32_t*)(VECTORS+0x0214)) = (uint32_t)(ext_vacuum_boost_inthandler);
		
		NVIC_SetPriority(TIM16_IRQn, INTPRIO_URGENT_APP);
		NVIC_EnableIRQ(TIM16_IRQn);


		TIM16->EGR = 1UL; // Generate update event to transfer to shadow regs
		__DSB(); __ISB();
		TIM16->CR1 |= 1UL; // enable



		IO_TO_GPO(GPIOI, 9); // PO5: Lower/rise direction
		IO_TO_GPO(GPIOH, 9); // PO6: Lower/rise power

	//	while(1)
	//		vacuum_boost_test();
	}
#endif
