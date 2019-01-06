#include <stdint.h>
#include "misc.h"
#include "own_std.h"
#include "ext_include/stm32h7xx.h"
#include "ext_vacuum_boost.h"
#include "stm32_cmsis_extension.h"
#include "adcs.h"
#include "pwrswitch.h"

#define VACUUM_BOOST_GATE_EN() do{ HI(GPIOF,7); }while(0)
#define VACUUM_BOOST_GATE_DIS() do{ LO(GPIOF,7); }while(0)


#define NOZZLE_DIR_UP()    do{HI(GPIOI, 9);}while(0)
#define NOZZLE_DIR_DOWN()  do{LO(GPIOI, 9);}while(0)
#define NOZZLE_POWER_EN()  do{HI(GPIOH, 9);}while(0)
#define NOZZLE_POWER_DIS() do{LO(GPIOH, 9);}while(0)


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

			if(++cnt >= 1200)
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
			NOZZLE_DIR_DOWN();
			NOZZLE_POWER_EN();

			if(++cnt >= 1200)
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
}

void ext_vacuum_nozzle_rise()
{
	state = NOZZLE_RISE_START;
}

void ext_vacuum_nozzle_lower()
{
	state = NOZZLE_LOWER_START;
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
	int prev_nozzle = -1;
	int prev_power = -1;

	if(power != prev_power)
	{
		ext_vacuum_power(power);
	}

	if(nozzle != prev_nozzle)
	{
		if(nozzle == 1)
			ext_vacuum_nozzle_rise();
		else
			ext_vacuum_nozzle_lower();
	}

	prev_nozzle = nozzle;
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
