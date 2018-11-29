/*
	
	Definitions and stuff needed from everywhere
*/

#pragma once

void error(int code);
void delay_us(uint32_t i) __attribute__((section(".text_itcm")));
void delay_tenth_us(uint32_t i) __attribute__((section(".text_itcm")));
void delay_ms(uint32_t i) __attribute__((section(".text_itcm")));


#define LED_ON()  do{HI(GPIOC, 13);}while(0)
#define LED_OFF() do{LO(GPIOC, 13);}while(0)


#define BIG5V_ON()  do{HI(GPIOF, 5);}while(0)
#define BIG5V_OFF()  do{HI(GPIOF, 5);}while(0)

void uart_print_string_blocking(const char *buf);
uint8_t uart_input();


#define VECTORS 0x0000FC00UL
#define SET_TIM3_VECTOR(v) do{*((volatile uint32_t*)(VECTORS+0x00B4)) = (uint32_t)(v);}while(0)
#define SET_ADC12_VECTOR(v) do{*((volatile uint32_t*)(VECTORS+0x0088)) = (uint32_t)(v);}while(0)

void timebase_alternative_inthandler();
#define SET_TIMEBASE_VECTOR_TO_KEEPON() do{*((volatile uint32_t*)(VECTORS+0x0108)) = (uint32_t)(timebase_alternative_inthandler);}while(0)


void charger_safety_shutdown();
void epc_safety_shutdown();
void pwrswitch_safety_shutdown();
void bldc_safety_shutdown();

#ifdef CALIBRATOR
	extern void tofcal_ambient_lvl(int);
	#define SAFETY_SHUTDOWN() do{charger_safety_shutdown(); bldc_safety_shutdown(); epc_safety_shutdown(); pwrswitch_safety_shutdown(); tofcal_ambient_lvl(0);}while(0)
#else
	#define SAFETY_SHUTDOWN() do{charger_safety_shutdown(); bldc_safety_shutdown(); epc_safety_shutdown(); pwrswitch_safety_shutdown();}while(0)
#endif


#define DBG_PR_VAR_U32_HEX(n_) do{uart_print_string_blocking(#n_ " = "); o_utoa32_hex((n_), printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");}while(0)
#define DBG_PR_VAR_U32(n_) do{uart_print_string_blocking(#n_ " = "); o_utoa32((n_), printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");}while(0)
#define DBG_PR_VAR_I32(n_) do{uart_print_string_blocking(#n_ " = "); o_itoa32((n_), printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");}while(0)
#define DBG_PR_VAR_U16(n_) do{uart_print_string_blocking(#n_ " = "); o_utoa16((n_), printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");}while(0)
#define DBG_PR_VAR_I16(n_) do{uart_print_string_blocking(#n_ " = "); o_itoa16((n_), printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");}while(0)


void init_cpu_profiler();
void profile_cpu_blocking_20ms();


#define sq(x) ((x)*(x))
#define abso(x) ((x<0)?(-x):(x))




// INTERRUPT PRIORITIES while running normally
// Shutdown (safety or otherwise) may reorder these. The requirement is that all electrical systems are shut in safe state
// (for example, gate drive signals down), and there is no way back to normal operation from such shutdown.

// Priorities 0,1,2 are the highest quick-safety-shutdown level which won't be disabled for atomic operations.
#define DIS_IRQ() do{__DSB(); __set_BASEPRI(3UL << (8 - __NVIC_PRIO_BITS)); __DSB();}while(0)
#define ENA_IRQ() do{__DSB(); __set_BASEPRI(0UL); __DSB();}while(0)

// These are never masked off. ISRs must not communicate using non-atomic things, since these cannot be disabled even temporarily:
#define INTPRIO_PVD          0    // 3V3 dropping. Basically a quick shutdown handler. No recovery possible.
#define INTPRIO_ADC12        2    // Must be the highest in operating (non-shutdown) system, super critical for charger!
#define INTPRIO_ADC3         2

// These may be masked off for atomic operations:
#define INTPRIO_IMU_DMA      3
#define INTPRIO_IMU_TIMER    4
#define INTPRIO_BLDC         5
#define INTPRIO_SBC_COMM     6
#define INTPRIO_TOF_I2C      7

// By having the timebase handler with the least priority, this means if any ISR in the system gets stuck, the timebase handler never gets
// called, the power switch is not replenished and power is cut really quickly. A great watchdog. Note: in shutdown (error or safety), the priority
// is changed so that power is kept on, to wait for the flasher command. Same is true for the SBC priority.
#define INTPRIO_TIMEBASE     13

#define INTPRIO_SOFTWARE     14


// Cool white    4,255,255,190
// Neutral white 4,255,210,110
// Warm white    4,255,150, 50

// Blinker yellow bright 4,255,120,0
// Blinker yellow dim    2,255,100,0

// Traffic light green  4,0,255,10
void led_status(int sid, uint32_t val, int mode);

#define BLACK  0x00000000
#define WHITE  0x01ffe090
#define YELLOW 0x01ff8000
#define RED    0x04ff0000
#define LED_MODE_FADE  0
#define LED_MODE_KEEP  1
#define LED_MODE_BLINK 2



