/*
	
	Definitions and stuff needed from everywhere
*/

#pragma once

void error(int code);
void delay_us(uint32_t i) __attribute__((section(".text_itcm")));
void delay_tenth_us(uint32_t i) __attribute__((section(".text_itcm")));
void delay_ms(uint32_t i) __attribute__((section(".text_itcm")));

// Priority 0 is the highest quick-safety-shutdown level which won't be disabled for atomic operations.
#define DIS_IRQ() do{__DSB(); __set_BASEPRI(1UL << (8 - __NVIC_PRIO_BITS)); __DSB();}while(0)
#define ENA_IRQ() do{__DSB(); __set_BASEPRI(0UL); __DSB();}while(0)

#define LED_ON()  do{HI(GPIOC, 13);}while(0)
#define LED_OFF() do{LO(GPIOC, 13);}while(0)

void uart_print_string_blocking(const char *buf);
uint8_t uart_input();


#define VECTORS 0x0000FC00UL
#define SET_TIM3_VECTOR(v) do{*((volatile uint32_t*)(VECTORS+0x00B4)) = (uint32_t)(v);}while(0)
#define SET_ADC12_VECTOR(v) do{*((volatile uint32_t*)(VECTORS+0x0088)) = (uint32_t)(v);}while(0)


void charger_safety_shutdown();
void epc_safety_shutdown();

#define SAFETY_SHUTDOWN() do{charger_safety_shutdown(); epc_safety_shutdown();}while(0)



#define DBG_PR_VAR_U32_HEX(n_) do{uart_print_string_blocking(#n_ " = "); o_utoa32_hex((n_), printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");}while(0)
#define DBG_PR_VAR_U32(n_) do{uart_print_string_blocking(#n_ " = "); o_utoa32((n_), printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");}while(0)
#define DBG_PR_VAR_I32(n_) do{uart_print_string_blocking(#n_ " = "); o_itoa32((n_), printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");}while(0)
#define DBG_PR_VAR_U16(n_) do{uart_print_string_blocking(#n_ " = "); o_utoa16((n_), printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");}while(0)
#define DBG_PR_VAR_I16(n_) do{uart_print_string_blocking(#n_ " = "); o_itoa16((n_), printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");}while(0)

