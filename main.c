#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "ext_include/stm32h7xx.h"
#include "stm32_cmsis_extension.h"

#include "misc.h"
#include "flash.h"
#include "tof_muxing.h"
#include "tof_process.h"
#include "tof_ctrl.h"
#include "micronavi.h"
#include "adcs.h"
#include "pwrswitch.h"
#include "charger.h"
#include "bldc.h"
#include "imu.h"
#include "audio.h"
#include "sbc_comm.h"
#include "timebase.h"
#include "backup_ram.h"

#include "run.h"

#include "own_std.h"

#ifdef CALIBRATOR
#include "../robotboard2-fw-calibrator/tof_calibrator.h"
#endif

#define DBG_UART

#ifdef DBG_UART
void uart_print_string_blocking(const char *buf)
{
	while(buf[0] != 0)
	{
		while((USART1->ISR & (1UL<<7)) == 0) ;
		USART1->TDR = buf[0];
		buf++;
	}
}

uint8_t uart_input()
{
	if(USART1->ISR & (1UL<<5))
	{
		return USART1->RDR;
	}
	else
		return 0;
}
#endif


static char printbuf[128];


void dump_scb()
{
	uart_print_string_blocking("\r\n");
	DBG_PR_VAR_U32_HEX(SCB->CPUID);
	DBG_PR_VAR_U32_HEX(SCB->ICSR);
	DBG_PR_VAR_U32_HEX(SCB->VTOR);
	DBG_PR_VAR_U32_HEX(SCB->AIRCR);
	DBG_PR_VAR_U32_HEX(SCB->SCR);
	DBG_PR_VAR_U32_HEX(SCB->CCR);
	DBG_PR_VAR_U32_HEX(SCB->SHCSR);
	DBG_PR_VAR_U32_HEX(SCB->CFSR);
	DBG_PR_VAR_U32_HEX(SCB->HFSR);
	DBG_PR_VAR_U32_HEX(SCB->DFSR);
	DBG_PR_VAR_U32_HEX(SCB->MMFAR);
	DBG_PR_VAR_U32_HEX(SCB->BFAR);
	DBG_PR_VAR_U32_HEX(SCB->AFSR);
	DBG_PR_VAR_U32_HEX(SCB->ID_PFR[0]);
	DBG_PR_VAR_U32_HEX(SCB->ID_PFR[1]);
	DBG_PR_VAR_U32_HEX(SCB->ID_DFR);
	DBG_PR_VAR_U32_HEX(SCB->ID_AFR);
	DBG_PR_VAR_U32_HEX(SCB->ID_MFR[0]);
	DBG_PR_VAR_U32_HEX(SCB->ID_MFR[1]);
	DBG_PR_VAR_U32_HEX(SCB->ID_MFR[2]);
	DBG_PR_VAR_U32_HEX(SCB->ID_MFR[3]);
	DBG_PR_VAR_U32_HEX(SCB->ID_ISAR[0]);
	DBG_PR_VAR_U32_HEX(SCB->ID_ISAR[1]);
	DBG_PR_VAR_U32_HEX(SCB->ID_ISAR[2]);
	DBG_PR_VAR_U32_HEX(SCB->ID_ISAR[3]);
	DBG_PR_VAR_U32_HEX(SCB->ID_ISAR[4]);
	DBG_PR_VAR_U32_HEX(SCB->CPACR);
	DBG_PR_VAR_U32_HEX(SCB->STIR);
	DBG_PR_VAR_U32_HEX(SCB->MVFR0);
	DBG_PR_VAR_U32_HEX(SCB->MVFR1);
	DBG_PR_VAR_U32_HEX(SCB->MVFR2);
	DBG_PR_VAR_U32_HEX(SCB->ITCMCR);
	DBG_PR_VAR_U32_HEX(SCB->DTCMCR);
	DBG_PR_VAR_U32_HEX(SCB->AHBPCR);
	DBG_PR_VAR_U32_HEX(SCB->AHBSCR);
	DBG_PR_VAR_U32_HEX(SCB->ABFSR);
	uart_print_string_blocking("\r\n");

}

void dump_stack()
{
	register int sp asm ("sp");
	uart_print_string_blocking("\r\nSP = "); o_utoa32_hex(sp, printbuf); uart_print_string_blocking(printbuf);
	uart_print_string_blocking("\r\nStack dump = \r\n");
	uint8_t* p = (uint8_t*)sp;

	int i = 0;

	extern unsigned int _STACKTOP;
	
	while(p < (uint8_t*)&_STACKTOP)
	{
		o_utoa32_hex(*(uint32_t*)p, printbuf); uart_print_string_blocking(printbuf);
		uart_print_string_blocking(" "); 
//		if(i%4 == 3)
//			uart_print_string_blocking(" "); 
		if(i%32 == 0)
			uart_print_string_blocking("\r\n"); 

		if(i>16384)
		{
			uart_print_string_blocking("(STOPPED)\r\n"); 
			break;
		}
		p+=4;
		i+=4;
	}

}

void error(int code)
{
	__disable_irq();
	SAFETY_SHUTDOWN();

	NVIC_SetPriority(TIM5_IRQn, 0); // to keep the power on
	NVIC_SetPriority(EXTI15_10_IRQn, 1); // for detecting flasher magic code

	SET_TIMEBASE_VECTOR_TO_KEEPON();

	// Disable all interrupts, except the charge pump, and for reflashing, the SPI nCS handler
	// TIM5_IRQn is #50, which is in register 1.
	// EXTI15_10_IRQn is #40, which is in register 1.
	NVIC->ICER[0] = 0xffffffffUL;
	NVIC->ICER[1] = ~(   (uint32_t)(1UL << ((50UL) & 0x1FUL))  |  (uint32_t)(1UL << ((40UL) & 0x1FUL))    );
	NVIC->ICER[2] = 0xffffffffUL;
	NVIC->ICER[3] = 0xffffffffUL;
	NVIC->ICER[4] = 0xffffffffUL;
	NVIC->ICER[5] = 0xffffffffUL;
	NVIC->ICER[6] = 0xffffffffUL;
	NVIC->ICER[7] = 0xffffffffUL;
	__DSB();
	__ISB();
	__enable_irq();

	uart_print_string_blocking("\r\nERROR "); o_itoa32(code, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
//	dump_scb();
//	dump_stack();

	if(code < 1)
	{
		while(1);
	}

	int volume = 200;
	while(1)
	{
		int tens = code/10;
		int ones = code - tens*10;

		for(int i=0; i<tens; i++)
		{
			LED_ON();
			beep_blocking(150, 1000, volume);
			delay_ms(300);
			LED_OFF();
			delay_ms(500);
		}

		delay_ms(500);

		for(int i=0; i<ones; i++)
		{
			LED_ON();
			beep_blocking(50, 700, volume);
			delay_ms(70);
			LED_OFF();
			delay_ms(400);
		}

		delay_ms(1200);
		volume=70;
	}
}

void init_cpu_profiler()
{
	// TIM4 used for CPU usage profiling
	RCC->APB1LENR |= 1UL<<2;
	__DSB();
	TIM4->PSC = 200-1;
	TIM4->CR1 = 1UL<<3 /* one pulse mode */;
	TIM4->ARR = 0xffffffff;
}

void profile_cpu_blocking_20ms()
{
	TIM4->CNT = 0;
	__DSB();
	TIM4->CR1 = 1UL<<3 | 1UL; // Enable
	__DSB();
	delay_ms(20);
	uint32_t time = TIM4->CNT;		
	TIM4->CR1 = 1UL<<3;
	__DSB();
	uart_print_string_blocking("CPU overhead : "); 
	char* p_joo = o_utoa32(((time-20000))*10000/20000, printbuf);
	p_joo[0] = p_joo[-1];
	p_joo[-1] = p_joo[-2];
	p_joo[-2] = '.';
	p_joo[1] = 0;

	uart_print_string_blocking(printbuf); uart_print_string_blocking(" %\r\n");
}

void shutdown_handler()
{
	__disable_irq();
	charger_safety_shutdown();
	epc_shutdown(); // blocks for 100ms
	while(1);
}

void delay_us(uint32_t i) __attribute__((section(".text_itcm")));
void delay_us(uint32_t i)
{
	i *= 100;
	i -= 7;
	while(i--)
		__asm__ __volatile__ ("nop");
}

void delay_tenth_us(uint32_t i) __attribute__((section(".text_itcm")));
void delay_tenth_us(uint32_t i)
{
	i *= 10;
	i -= 7;
	while(i--)
		__asm__ __volatile__ ("nop");
}

void delay_ms(uint32_t i) __attribute__((section(".text_itcm")));
void delay_ms(uint32_t i)
{
	while(i--)
	{
		delay_us(1000);
	}
}

void pointer_system_test();
void sbc_comm_test();
extern void init_sram1234();
extern void init_axi_data();
extern void init_dtcm();
extern void relocate_vectors();

void main()
{

// At 18.0V:
// 28.3 mA before clock init
// 36 mA after all init
// 27.5mA wfi
// 27.6mA wfe
// 27.5mA without SRAM123

	// Let the capacitors precharge, so that the precharge current diminishes,
	// so that the voltage drop over the precharge resistors is below the desaturation
	// protection circuit's trip voltage.
	// Simple delay is fine, since the actual time required is defined by the RC constant
	// (DC bus capacitance * precharge resistance)
	// Block for 500ms, but generate charge pump drive enough to keep it charged if it already is
	// (for example, when soft-resetting the system without pressing power switch)

	pwrswitch_chargepump_init();
	chargepump_pulsetrain_low_power(500); // if this is removed, change the next replenish_pulsetrain to initial_pulsetrain.


	// Pulse trains are needed with < 9ms intervals, until the proper timer takes over.
	chargepump_replenish_pulsetrain();

	RCC->AHB4ENR |= 0b111111111; // enable GPIOA to GPIOI (J and K do not exist on the device)

	IO_TO_GPO(GPIOC, 13); // LED

	LED_ON();
	delay_ms(6/6);
	chargepump_replenish_pulsetrain();
	LED_OFF();

/*
	sys_ck (for CPU, etc.) is pll1_p_ck
	Fvco = Fref * divN
	Fout = Fvco / (postdiv_reg+1)
	VCO must be between 192 and 836MHz
	Multiplier (divN) must be from 4 to 512
	
	Fractional PLL mode not needed - initialize in integer mode
	Input clock = 8 MHz

	Let:
	AHB1 aka hclk1 = 200 MHz   HPRE    = 2
	AHB2 aka hclk2 = 200 MHz  (HPRE    = 2)
	AHB3 aka hclk3 = 200 MHz  (HPRE    = 2)
	APB1 aka pclk1 = 100 MHz   D2PPRE1 = 2
	APB2 aka pclk2 = 100 MHz   D2PPRE2 = 2
	APB3 aka pclk3 = 100 MHz   D1PPRE  = 2
	APB4 aka pclk4 = 100 MHz   D3PPRE  = 2

	Some APB* peripheral kernel clocks are 100MHz max, and cannot be connected to
	PLL1. So, if 200MHz bus on APB1 or APB2 is ever needed, PLL2 needs to be configured
	to drive the kernel clocks for such peripherals. For example, SPI4,5.

	For now, it's simplest to drive these peripherals directly on the bus clock, so we
	use 100MHz bus. It's highly unlikely this would never be an performance issue.

	The most busy/biggest bandwidth SPI is SPI1, which is in APB2. Consider increasing APB2
	to 200MHz.


	PLL1_P: 400MHz: sysck (cpu, etc.)
	PLL1_Q: 100MHz: SPI1,2,3
	PLL2_P: 33.33MHz:  ADC1,2, ADC3
	
*/
	RCC->CR |= 1UL<<16; // HSE clock on

	delay_ms(6/6);
	chargepump_replenish_pulsetrain();

	while(!(RCC->CR & 1UL<<17)) ; // Wait for HSE oscillator stabilization

	delay_ms(6/6);
	chargepump_replenish_pulsetrain();

	// M prescalers - set to 0 when PLL is disabled. 1 = div by 1 (bypass)
	RCC->PLLCKSELR = 0UL<<20 /* PLL3 M(prescaler)*/ | 
			 1UL<<12 /* PLL2 M(prescaler)*/ | 
			 1UL<<4 /* PLL1 M(prescaler)*/ |
			 0b10UL /*HSE as PLL source*/;
			
	RCC->PLLCFGR =  0b000UL<<22 /*PLL3 divider output enables: R,Q,P */ |
			0b001UL<<19 /*PLL2 divider output enables: R,Q,P */ |
			0b011UL<<16 /*PLL1 divider output enables: R,Q,P */ |
			0b10UL<<10 /*PLL3 input between 4..8MHz*/ |
			0b10UL<<6  /*PLL2 input between 4..8MHz*/ |
			0b10UL<<2  /*PLL1 input between 4..8MHz*/;
	

	RCC->PLL1DIVR = 0UL<<24 /*R*/ | (4UL  -1UL)<<16 /*Q ->pll1_q_ck (100MHz)*/ | (1UL  -1UL)<<9 /*P ->sys_ck (400MHz)  */ | (50UL   -1UL)<<0 /*N  8MHz->400MHz*/;
	RCC->PLL2DIVR = 0UL<<24 /*R*/ | 0UL<<16 /*Q                             */ | (12UL  -1UL)<<9 /*P ->ADC   (33.33MHz)*/ | (50UL   -1UL)<<0 /*N  8MHz->400MHz*/;

	RCC->CR |= 1UL<<24; // PLL1 on
	// Configure other registers while waiting PLL1 stabilization:

	FLASH->ACR = 0b10UL<<4 /*WRHIGHFREQ programming delay: 185..210MHz AXI bus in VOS1 */ |
		     2UL<<0    /*LATENCY = 2 wait states: 185..210MHz in VOS1 */;


	RCC->D1CFGR = 0UL<<8 /*D1CPRE = sysck divider = 1*/ | 0b100UL<<4 /*D1PPRE = 2*/ | 0b1000UL<<0 /*HPRE = 2*/;
	RCC->D2CFGR = 0b100UL<<8 /*D2PPRE2 = 2*/ | 0b100UL<<4 /*D2PPRE1 = 2*/;
	RCC->D3CFGR = 0b100UL<<4 /*D3PPRE = 2*/;

	RCC->AHB1ENR |= 1UL<<1 /*DMA2*/ | 1UL<<0 /*DMA1*/;
	RCC->AHB4ENR |= 1UL<<21 /*BDMA*/;

	chargepump_replenish_pulsetrain();

	while(!(RCC->CR & 1UL<<25)) ; // Wait for PLL1 ready

	chargepump_replenish_pulsetrain();

	RCC->CFGR |= 0b011; // Change PLL to system clock
	while((RCC->CFGR & (0b111UL<<3)) != (0b011UL<<3)) ; // Wait for switchover to PLL.

	RCC->CR |= 1UL<<26; // PLL2 on

	relocate_vectors();
	init_sram1234();
	chargepump_replenish_pulsetrain();
	init_axi_data();
	init_dtcm();

	while(!(RCC->CR & 1UL<<27)) ; // Wait for PLL2 ready

	chargepump_replenish_pulsetrain();

	NVIC_SetPriorityGrouping(0);

	init_pwrswitch_and_led();
	init_timebase();
	

//	RCC->CR |= 1UL<<28; // PLL3 on
//	while(!(RCC->CR & 1UL<<29)) ; // Wait for PLL3 ready

	
	#ifdef DBG_UART

		/*
			USART1 = the raspi USART @ APB2 = 100 MHz
		*/

		RCC->APB2ENR |= 1UL<<4;
		IO_TO_ALTFUNC(GPIOB, 14);
		IO_TO_ALTFUNC(GPIOB, 15);
		IO_SET_ALTFUNC(GPIOB, 14, 4);
		IO_SET_ALTFUNC(GPIOB, 15, 4);
		USART1->BRR = 100000000/460800; //115200;
		USART1->CR1 = 0UL<<5 /*RX interrupt*/ | 1UL<<3 /*TX ena*/ | 1UL<<2 /*RX ena*/ |  1UL /*USART ENA*/;

	#endif

	init_adcs();
	init_audio();
	init_sbc_comm();

	delay_ms(2); // let the ADC convert...

	extern int main_power_enabled;

	int vgplat = VGPLAT_MEAS_TO_MV(adc3.s.bms_mainfet_g_meas);
	uart_print_string_blocking("Vgplat converted mV = ");
	o_utoa16(vgplat, printbuf); uart_print_string_blocking(printbuf);
	if(vgplat < 6*3000 + 10000)
	{
		beep_blocking(20, 4000, 1500);
		main_power_enabled = 1; // blink and die
		while(1);
	}
	main_power_enabled = 2;


	// Enable FPU

	SCB->CPACR |= 0b1111UL<<20;
	__DSB();
	__ISB();


	// Setup Memory Protection Unit
	// MPU is disabled during hardfault and NMI
	MPU->CTRL = 1UL<<2 /*Default backgroung map for privileged code*/ | 1UL /*Enable MPU*/;
	__DSB();

	MPU->RNR = 0;
	MPU->RBAR = 0UL; // Base address - keep last five bits cleared (minimum granularity 32 bytes)
	MPU->RASR = 0UL<<28 /*Allow instr fetch*/ | 0b110UL<<24 /*Read Only*/ |
		0b001000UL<<16 /*TEX=001, C=0, B=0, S=0 normal, non-shareable, noncacheable*/ |
		(15UL  -1UL)<<1 /*Len = 32K = 2^15 bytes*/ |
		1UL /*Enable it!*/;
	// Test code to trig the fault: *(uint32_t*)0x00000080 = 1234;


	__DSB(); __ISB();

	/*
		Interrupts will have 16 levels of pre-emptive priority.

		Interrupt with more urgent (lower number) pre-emptive priority will interrupt another interrupt running.

		Priority 0 is the highest quick-safety-shutdown level which won't be disabled for atomic operations.
	*/


	PWR->CR1 |= 0b110UL<<5 /*Power Voltage Detector level: 2.85V*/ | 1UL<<4 /*PVD on*/;

	// PVD event is EXTI event #16

	EXTI->RTSR1 |= 1UL<<16; // Get the rising edge interrupt
	EXTI->FTSR1 |= 1UL<<16; // Also get the falling edge interrupt
	EXTI_D1->IMR1 |= 1UL<<16; // Enable CPU interrupt

	NVIC_SetPriority(PVD_IRQn, INTPRIO_PVD);
	NVIC_EnableIRQ(PVD_IRQn);

	beep_blocking(100, 250, 1000);

	IO_TO_GPO(GPIOF, 5);
	BIG5V_ON();


	delay_ms(2000);


	uart_print_string_blocking("No terse\r\n\r\n"); 

	DBG_PR_VAR_U32_HEX(backup_ram.immediate_5v);
	backup_ram.immediate_5v = 11;

	DBG_PR_VAR_U32(backup_ram.boot_cnt);
	DBG_PR_VAR_U32(backup_ram.dummy);

	backup_ram.boot_cnt++;
	backup_ram.dummy = 55;

	DBG_PR_VAR_U32(backup_ram.boot_cnt);
	DBG_PR_VAR_U32(backup_ram.dummy);



	init_imu();
	extern void timer_test();
	init_bldc(); // Gives triggers to ADC1. Init ADCs first so they sync correctly.
	init_charger(); // Requires working ADC1 data, so init_bldc() first.

	extern void adc_test();
//	while(1)
//		adc_test();

	delay_ms(10);

	init_cpu_profiler();

	extern void bldc_test();

	tof_ctrl_init();
	uart_print_string_blocking("init ok\r\n"); 


	#ifdef CALIBRATOR
		init_tofcal_ambient();
	#endif

	while(1)
	{
		run_cycle();
	}

//	while(1);
}
