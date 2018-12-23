#include <stdint.h>
#include "misc.h"
#include "own_std.h"
#include "ext_include/stm32h7xx.h"
#include "ext_vacuum_boost.h"
#include "stm32_cmsis_extension.h"
#include "adcs.h"

#define VACUUM_BOOST_GATE_EN() do{ HI(GPIOF,7); }while(0)
#define VACUUM_BOOST_GATE_DIS() do{ LO(GPIOF,7); }while(0)

#define PERIOD 2000
#define MAX_DUTY ((PERIOD*6)/10)
#define DUTY_REG (TIM16->CCR1)

static int target_duty = PERIOD/2;
static int duty = 0;

static volatile int intcnt;
static void ext_vacuum_boost_inthandler()
{
	TIM16->SR = 0;
	__DSB();

	// 16384 ~ 3.3V ~ 66mV ~ 132A
	// For some reason, -8A offset. Can't read down to zero.
	int curr_a = 8+adc3.s.eb_analog2/124;

	// 114k - 1.5k divider
	// 16384 ~ 3.3V ~ 254V
	int outvolt = adc3.s.eb_analog1>>6;

	if(outvolt > 230)
	{
		VACUUM_BOOST_GATE_DIS();
		error(210);
	}

	if(curr_a > 65)
	{
		VACUUM_BOOST_GATE_DIS();
		error(211);
	}
	else if(curr_a > 60)
	{
		duty>>=2;
	}
	else if(curr_a > 55)
	{
		duty>>=1;
	}
	else if(curr_a > 50)
	{
		duty-=2;
	}
	else if(curr_a > 45)
	{
		duty--;
	}
	else
	{
		if(duty < target_duty)
			duty++;
	}


	if(duty > MAX_DUTY) duty = MAX_DUTY;
	else if(duty < 0) duty = 0;

	intcnt++;
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
		val *= 80;
		DUTY_REG = val;
	}
	else if(cmd == 'o')
	{
		VACUUM_BOOST_GATE_EN();
	}
	else if(cmd == 'p')
	{
		VACUUM_BOOST_GATE_DIS();
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

	while(1)
		vacuum_boost_test();
}
