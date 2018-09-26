#include <stdint.h>
#include "ext_include/stm32h7xx.h"
#include "stm32_cmsis_extension.h"

#include "own_std.h"
#include "tof_ctrl.h"

// Including the proprietary sensor firmware for EPC635, (c) Espros Photonics Corporation,
// not available for public distribution. Pulu Robotics has a permission to redistribute this
// small binary to our own customers. Please contant us for more information.
#include "epc635_seq_v10.c"


#include "misc.h"

#include "tof_muxing.h"

#define RSTN_HIGH()     HI(GPIOA,13)
#define RSTN_LOW()      LO(GPIOA,13)

#define LEDWIDE_ON()    HI(GPIOD,12)
#define LEDWIDE_OFF()   LO(GPIOD,12)

#define LEDNARROW_ON()  HI(GPIOD,11)
#define LEDNARROW_OFF() LO(GPIOD,11)

#define PLUS3V3_ON()    LO(GPIOH,13)
#define PLUS3V3_OFF()   HI(GPIOH,13)

#define PLUS10V_ON()    HI(GPIOH,14)
#define PLUS10V_OFF()   LO(GPIOH,14)

#define MINUS10V_ON()   HI(GPIOD,13)
#define MINUS10V_OFF()  LO(GPIOD,13)



static char printbuf[128];


/*
	All sensors use a single I2C bus, with the multiplexer connecting one sensor at a time.
	This makes everything simple.
*/

#define N_SENSORS 1

#define WR_DMA DMA1
#define WR_DMA_STREAM DMA1_Stream2
#define WR_DMA_STREAM_NUM 2 // stream number

#define RD_DMA DMA1
#define RD_DMA_STREAM DMA1_Stream3
#define RD_DMA_STREAM_NUM 3 // stream number
#define RD_DMA_STREAM_IRQ DMA1_Stream3_IRQn

#define DCMI_DMA DMA2
#define DCMI_DMA_STREAM DMA2_Stream0
#define DCMI_DMA_STREAM_NUM 0
#define DCMI_DMA_STREAM_IRQ DMA2_Stream0_IRQn

static const uint8_t i2c_addr = 0b0100000;
static const uint8_t rgb_addr = 0b1101000;

// Temperature sensor readout procedure is weird, and requires storing and restoring some undocumented internal registers to the chip:
static uint8_t epc_tempsens_regx[N_SENSORS], epc_tempsens_regy[N_SENSORS];
static float epc_tempsens_factory_offset[N_SENSORS]; // in some intermediate format, as specified in datasheet parameter "z"
int temperatures[N_SENSORS]; // in degs/10


volatile uint8_t epc_wrbuf[16] __attribute__((aligned(4)));
volatile uint8_t epc_rdbuf[16] __attribute__((aligned(4)));



// Things written to I2C CR1 every time:
#define I2C_CR1_BASICS_OFF (0UL<<8 /*Digital filter len 0 to 15*/)
#define I2C_CR1_BASICS_ON (I2C_CR1_BASICS_OFF | 1UL /*keep it on*/)



void epc_safety_shutdown()
{
	// It's most important to bring down the analog supplies in the correct order, and do this quickly.
	// When it's done, it's an optional plus to turn 3V3 off.
	RSTN_LOW();
	MINUS10V_OFF();
	PLUS10V_OFF();
	tof_mux_all_off();
	LEDWIDE_OFF();
	LEDNARROW_OFF();
	delay_ms(100);
	PLUS3V3_OFF();
}






volatile int epc_i2c_write_busy, epc_i2c_read_busy;
volatile int epc_i2c_read_state;
uint8_t epc_i2c_read_slave_addr;
volatile uint8_t *epc_i2c_read_buf;
uint8_t epc_i2c_read_len;
volatile int init_err_cnt;

void epc_i2c_write_dma(uint8_t slave_addr_7b, volatile uint8_t *buf, uint8_t len)
{
	// Pipeline flush is almost always needed when using this function, so easier to do it here.
	// If data cache is enabled later, remember to use non-cacheable sections (or invalidate the cache)
	__DSB();

	if(epc_i2c_write_busy || epc_i2c_read_busy || epc_i2c_read_state)
		error(4);

	if(DMA1->LISR & 1UL<<22)
		error(5);
	if(DMA1->LISR & 1UL<<24)
		error(6);
	if(DMA1->LISR & 1UL<<25)
		error(7);

	if(WR_DMA_STREAM->CR & 1UL)
	{
		if(DMA1->LISR & 1UL<<27)
			error(8);

		uart_print_string_blocking("I2C SR = "); o_btoa16_fixed(I2C1->ISR&0xffff, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");

		error(15+init_err_cnt);
	}

	epc_i2c_write_busy = 1;
	I2C1->ICR = 1UL<<5; // Clear any pending STOPF interrupt 
	NVIC_ClearPendingIRQ(I2C1_EV_IRQn);

	if(len > 1) // Actually use DMA
	{		
		WR_DMA_STREAM->CR = 0b01UL<<16 /*med prio*/ | 
				   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
			           1UL<<10 /*mem increment*/ | 0b01UL<<6 /*mem-to-periph*/;

		WR_DMA_STREAM->NDTR = len;
		WR_DMA_STREAM->M0AR = (uint32_t)buf;

		DMA_CLEAR_INTFLAGS(WR_DMA, WR_DMA_STREAM_NUM);
		WR_DMA_STREAM->CR |= 1; // Enable DMA // OPT_TODO: try combining with previous CR write.

		I2C1->CR1 = I2C_CR1_BASICS_ON |  1UL<<5 /*STOPF interrupt - the only specified way to detect finished transfer when AUTOEND=1 and RELOAD=0*/ | 1UL<<14 /*TX DMA*/;

	}
	else	// Just do the single write right now.
	{
		I2C1->CR1 = I2C_CR1_BASICS_ON |  1UL<<5 /*STOPF interrupt - the only specified way to detect finished transfer when AUTOEND=1 and RELOAD=0*/; // no DMA
		I2C1->TXDR = buf[0];
	}

	I2C1->CR2 = 1UL<<25 /*AUTOEND: stop is generated after len bytes*/ | ((uint32_t)len)<<16 | 0UL<<10 /*write*/ | slave_addr_7b<<1;
	I2C1->CR2 |= 1UL<<13; // START. OPT_TODO: try combining with previous CR2 write.
}


#define epc_i2c_write epc_i2c_write_dma

void epc_i2c_read_dma(uint8_t slave_addr_7b, volatile uint8_t *buf, uint8_t len)
{
	// Pipeline flush is almost always needed when using this function, so easier to do it here.
	// If data cache is enabled later, remember to use non-cacheable sections (or invalidate the cache)
	__DSB();

	RD_DMA_STREAM->CR = 0b01UL<<16 /*med prio*/ | 
			   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/ | 1UL<<4 /*transfer complete interrupt*/;

	RD_DMA_STREAM->NDTR = len;
	RD_DMA_STREAM->M0AR = (uint32_t)buf;

	I2C1->CR1 = I2C_CR1_BASICS_ON | 0UL<<5 /*OFF FOR READ: STOPF interrupt - the only specified way to detect finished transfer when AUTOEND=1 and RELOAD=0*/ | 1UL<<15 /*RX DMA*/ ;

	I2C1->CR2 = 1UL<<25 /*AUTOEND: stop is generated after len bytes*/ | ((uint32_t)len)<<16 | 1UL<<10 /*read*/ | slave_addr_7b<<1;

	DMA_CLEAR_INTFLAGS(RD_DMA, RD_DMA_STREAM_NUM);
	RD_DMA_STREAM->CR |= 1; // Enable DMA // OPT_TODO: try combining with previous CR write.

	I2C1->CR2 |= 1UL<<13; // START. OPT_TODO: try combining with previous CR2 write.
}

/*
	In I2C, a million things can go wrong. STM32 I2C implementations are notoriously buggy, device implementations often are, too.
	It makes little sense to try to detect every possible error condition, since they most often end up in total bus lockup
	anyway, and can only be reliably solved by device&bus reset. Since this is time-consuming anyway, and always leads to loss
	of data and hangup in execution time, we can as well solve all error checking using a simple watchdog that does the same,
	with much less code footprint (good for reliability, execution speed, flash usage, and code maintainability) compared to trying
	to catch all unexpected things in actual interrupt services or API functions.
	TODO: actually implement said watchdog
*/

void epc_i2c_read(uint8_t slave_addr_7b, uint8_t reg_addr, volatile uint8_t *buf, uint8_t len)
{
	// Pipeline flush is almost always needed when using this function, so easier to do it here.
	// If data cache is enabled later, remember to use non-cacheable sections (or invalidate the cache)
	__DSB();

	if(epc_i2c_write_busy || epc_i2c_read_busy)
		error(10);
	/*
		Full I2C read cycle consists of a write cycle (payload = reg_addr), without stop condition,
		then the actual read cycle starting with the repeated start condition.

		Since the write is always one byte of payload, we run it without DMA.

		After the write is complete, there is no AUTOEND generated, instead we get an interrupt, allowing
		us to go on with the read.
	*/
	epc_i2c_read_busy = 1;

	I2C1->CR1 = I2C_CR1_BASICS_ON | 1UL<<6 /*Transfer Complete interrupt - happens because AUTOEND=0 and RELOAD=0*/; // no DMA
	I2C1->CR2 = 0UL<<25 /*AUTOEND off*/ | 1UL<<16 /*len=1*/ | 0UL<<10 /*write*/ | slave_addr_7b<<1;
	I2C1->TXDR = reg_addr;
	I2C1->CR2 |= 1UL<<13; // START. OPT_TODO: try combining with previous CR2 write.
	epc_i2c_read_state = 1;
	epc_i2c_read_slave_addr = slave_addr_7b;
	epc_i2c_read_buf = buf;
	epc_i2c_read_len = len;

}

// Returns true whenever read or write operation is going on.
int epc_i2c_is_busy()
{
	return epc_i2c_write_busy || epc_i2c_read_busy;
}










void epc_rx_dma_inthandler() // read complete.
{
	DMA_CLEAR_INTFLAGS(RD_DMA, RD_DMA_STREAM_NUM);
	epc_i2c_read_busy = 0;
}

/*
	For some reason, I can't get repeated start (Sr) condition out of the new STM32 I2C - it generates a stop-start sequence even when only asked for a START.
	Luckily, the EPC chip still works correctly even though this violates the spec.
*/
void epc_i2c_inthandler()
{
	I2C1->ICR = 1UL<<5; // Clear the STOPF flag.
	if(epc_i2c_read_state)
	{
		// Writing START to 1 (in epc_i2c_read_dma()) clears the interrupt flag
		epc_i2c_read_dma(epc_i2c_read_slave_addr, epc_i2c_read_buf, epc_i2c_read_len);
		epc_i2c_read_state=0;

	}
	else  // STOPF interrupt - this was a write.
	{
		if(I2C1->ISR & (1UL<<15)) // busy shouldn't be high - at least fail instead of doing random shit
		{
			error(11);
		}

		// Write is now finished.
		epc_i2c_write_busy = 0;
		I2C1->CR1 = I2C_CR1_BASICS_ON;
	}
}


void epc_i2c_init()
{
	// For now, init in Fast mode 400 kHz

	// I2C1 runs on APB2 at 100 MHz

	// Use the datasheet example of 48MHz input clock, with prescaler 2 times bigger for 100 MHz.
	// Silicon bug errata: tSU;DAT (do they mean SDADEL register??) must be at least one I2CCLK period
	// Silicon bug errata: bus errors are spuriously generated for no reason, the bug is not said to affect the transfer: just ignore BERR.
 
	WR_DMA_STREAM->PAR = (uint32_t)&(I2C1->TXDR);
	RD_DMA_STREAM->PAR = (uint32_t)&(I2C1->RXDR);

	// open drain:
	GPIOB->OTYPER  |= 1UL<<6;
	GPIOB->OTYPER  |= 1UL<<7;

	IO_ALTFUNC(GPIOB, 6, 4);
	IO_ALTFUNC(GPIOB, 7, 4);

	I2C1->TIMINGR = 11UL<<28/*PRESCALER*/  | 9UL<<0/*SCLL*/  | 3UL<<8/*SCLH*/  | 3UL<<16 /*SDADEL*/  | 3UL<<20 /*SCLDEL*/;
	I2C1->CR1 = I2C_CR1_BASICS_OFF;

	I2C1->CR1 |= 1UL; // Enable

	NVIC_SetPriority(I2C1_EV_IRQn, 5);
	NVIC_EnableIRQ(I2C1_EV_IRQn);
	NVIC_SetPriority(RD_DMA_STREAM_IRQ, 5);
	NVIC_EnableIRQ(RD_DMA_STREAM_IRQ);
}


volatile int epc_capture_finished = 0;
void epc_dcmi_dma_inthandler()
{
	// DMA finished
	DMA_CLEAR_INTFLAGS(DMA2, 7);
	epc_capture_finished = 1;
}


void dcmi_init()
{
	IO_ALTFUNC(GPIOC,6,  13);
	IO_ALTFUNC(GPIOC,7,  13);
	IO_ALTFUNC(GPIOC,8,  13);
	IO_ALTFUNC(GPIOC,9,  13);
	IO_ALTFUNC(GPIOC,11, 13);
	IO_ALTFUNC(GPIOD,3,  13);
	IO_ALTFUNC(GPIOB,8,  13);
	IO_ALTFUNC(GPIOB,9,  13);
	IO_ALTFUNC(GPIOA,6,  13);

	DCMI->CR = 0b00UL<<10 /*8-bit data*/ | 0UL<<5 /* CLK falling edge*/ | 1UL<<4 /*Embedded sync*/ | 0UL<<1 /*continuous grab*/;

	// Program the default synchronization codes used in the epc635
	DCMI->ESCR = 0x1eUL<<0 /*frame start*/ | 0xffUL<<24 /*frame end*/ | 0xaaUL<<8 /*line start*/ | 0x55<<16 /*line end*/;
	DCMI->ESUR = 0xffffffffUL; // mask for the previous: all bits compared

	DCMI->CR |= 1UL<<14; // Enable

	DCMI_DMA_STREAM->PAR = (uint32_t)&(DCMI->DR);
	DCMI_DMA_STREAM->CR = 0b01UL<<16 /*med prio*/ | 0UL<<18 /*double buffer OFF*/ | 0UL<<8 /*circular OFF*/ |
			   0b10UL<<13 /*32-bit mem*/ | 0b10UL<<11 /*32-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/ | 0UL<<4 /* Transfer complete interrupt: OFF!*/;

	// NDTR is not set yet because it varies. Stream is not enabled yet.

//	NVIC_SetPriority(DCMI_DMA_STREAM_IRQ, 10);
//	NVIC_EnableIRQ(DCMI_DMA_STREAM_IRQ);

	DCMI->CR |= 1UL<<0; // Start CAPTURE
}


void dcmi_start_dma(void *data, int size)
{
	// Disable the stream first
	DCMI_DMA_STREAM->CR = 0b01UL<<16 /*med prio*/ |
			   0b10UL<<13 /*32-bit mem*/ | 0b10UL<<11 /*32-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/;

	while(DCMI_DMA_STREAM->CR & 1UL) ;

	DMA_CLEAR_INTFLAGS(DCMI_DMA, 0);

	DCMI_DMA_STREAM->M0AR = (uint32_t)data;

	DCMI_DMA_STREAM->NDTR = size/4; // Num of 32-bit transfers
	DCMI_DMA_STREAM->CR = 0b01UL<<16 /*med prio*/ | 
			   0b10UL<<13 /*32-bit mem*/ | 0b10UL<<11 /*32-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/ | 1UL<<4 /* Transfer complete interrupt*/;

	DMA_CLEAR_INTFLAGS(DCMI_DMA, DCMI_DMA_STREAM_NUM);
	DCMI_DMA_STREAM->CR |= 1; // Enable DMA
	__DSB();
}

void trig()
{
	static volatile uint8_t b[2] = {0xa4, 1};
	epc_i2c_write(i2c_addr, b, 2);
	while(epc_i2c_is_busy());
}


void epc_clk_div(int div)
{
	epc_wrbuf[0] = 0x85;
	epc_wrbuf[1] = div-1;
	epc_i2c_write(i2c_addr, epc_wrbuf, 2);
}

void epc_dis_leds()
{
	epc_wrbuf[0] = 0x90;
	epc_wrbuf[1] = 0b11001000; // leds disabled
	epc_i2c_write(i2c_addr, epc_wrbuf, 2);
}

void epc_ena_wide_leds()
{
	epc_wrbuf[0] = 0x90;
	epc_wrbuf[1] = 0b11101000; // "LED2" output on
	epc_i2c_write(i2c_addr, epc_wrbuf, 2);
}

void epc_ena_narrow_leds()
{
	epc_wrbuf[0] = 0x90;
	epc_wrbuf[1] = 0b11001100; // "LED" output on
	epc_i2c_write(i2c_addr, epc_wrbuf, 2);
}

void epc_greyscale() // OK to do while acquiring: shadow registered: applied to next trigger.
{
	epc_wrbuf[0] = 0x92;
	epc_wrbuf[1] = 0b11000100; // greyscale modulation
	epc_i2c_write(i2c_addr, epc_wrbuf, 2);
}

void epc_2dcs() // OK to do while acquiring: shadow registered: applied to next trigger.
{
	epc_wrbuf[0] = 0x92;
	epc_wrbuf[1] = 0b00010100; // 2dcs modulation
	epc_i2c_write(i2c_addr, epc_wrbuf, 2);
}

void epc_4dcs() // OK to do while acquiring: shadow registered: applied to next trigger.
{
	epc_wrbuf[0] = 0x92;
	epc_wrbuf[1] = 0b00110100; // 4dcs modulation
	epc_i2c_write(i2c_addr, epc_wrbuf, 2);
}


void epc_4dcs_dualint() // OK to do while acquiring: shadow registered: applied to next trigger.
{
	epc_wrbuf[0] = 0x92;
	epc_wrbuf[1] = 0b00111100;
	epc_i2c_write(i2c_addr, epc_wrbuf, 2);
}

// Required for dual integration time or dual phase mode:
void epc_dualphase_or_int() // OK to do while acquiring: shadow registered: applied to next trigger.
{
	epc_wrbuf[0] = 0x94;
	epc_wrbuf[1] = 0x80;
	epc_i2c_write(i2c_addr, epc_wrbuf, 2);
}

// Back to normal mode (non-dual int.time or non-dual phase)
void epc_normalphase_or_int() // OK to do while acquiring: shadow registered: applied to next trigger.
{
	epc_wrbuf[0] = 0x94;
	epc_wrbuf[1] = 0x00;
	epc_i2c_write(i2c_addr, epc_wrbuf, 2);
}


void epc_intlen(uint8_t multiplier, uint16_t time) // OK to do while acquiring: shadow registered: applied to next trigger.
{
	int intlen = ((int)time<<2)-1;
	epc_wrbuf[0] = 0xA1;
	epc_wrbuf[1] = multiplier;
	epc_wrbuf[2] = (intlen&0xff00)>>8;
	epc_wrbuf[3] = intlen&0xff;

	epc_i2c_write(i2c_addr, epc_wrbuf, 4);
}

// time1 = odd rows, time2 = even rows
// epc_4dcs_dualint() and epc_dualphase_or_int() must be called first
void epc_intlen_dual(uint8_t multiplier, uint16_t time1, uint16_t time2) // OK to do while acquiring: shadow registered: applied to next trigger.
{
	int intlen1 = ((int)time1<<2)-1;
	int intlen2 = ((int)time2<<2)-1;
	epc_wrbuf[0] = 0x9E;
	epc_wrbuf[1] = (intlen2&0xff00)>>8;
	epc_wrbuf[2] = intlen2&0xff;
	epc_wrbuf[3] = 0;
	epc_wrbuf[4] = multiplier;
	epc_wrbuf[5] = (intlen1&0xff00)>>8;
	epc_wrbuf[6] = intlen1&0xff;

	epc_i2c_write(i2c_addr, epc_wrbuf, 7);
}

// Do the magic stuff specified in the datasheet to enable temperature sensor conversion:
void epc_temperature_magic_mode(int idx)
{
	epc_wrbuf[0] = 0xD3;
	epc_wrbuf[1] = epc_tempsens_regx[idx] | 0x60;
	epc_i2c_write(i2c_addr, epc_wrbuf, 2);
	while(epc_i2c_is_busy());

	epc_wrbuf[0] = 0xD5;
	epc_wrbuf[1] = epc_tempsens_regy[idx] & 0x0f;
	epc_i2c_write(i2c_addr, epc_wrbuf, 2);
	while(epc_i2c_is_busy());
}


// Do the magic stuff specified in the datasheet to disable temperature sensor conversion, back to normal operation
void epc_temperature_magic_mode_off(int idx)
{
	epc_wrbuf[0] = 0xD3;
	epc_wrbuf[1] = epc_tempsens_regx[idx];
	epc_i2c_write(i2c_addr, epc_wrbuf, 2);
	while(epc_i2c_is_busy());

	epc_wrbuf[0] = 0xD5;
	epc_wrbuf[1] = epc_tempsens_regy[idx];
	epc_i2c_write(i2c_addr, epc_wrbuf, 2);
	while(epc_i2c_is_busy());
}

uint16_t epc_read_temperature_regs()
{
	uint8_t hi, lo;
	epc_i2c_read(i2c_addr, 0x60, &hi, 1);
	while(epc_i2c_is_busy());
	epc_i2c_read(i2c_addr, 0x61, &lo, 1);
	while(epc_i2c_is_busy());

	delay_us(50); // see the comment about i2c read bug in top_init

	return ((uint16_t)hi<<8) | ((uint16_t)lo);
}


void epc_fix_modulation_table_defaults()
{
	epc_wrbuf[0] = 0x22;
	epc_wrbuf[1] = 0x30;
	epc_i2c_write(i2c_addr, epc_wrbuf, 2);
	while(epc_i2c_is_busy());
	epc_wrbuf[0] = 0x25;
	epc_wrbuf[1] = 0x35;
	epc_i2c_write(i2c_addr, epc_wrbuf, 2);
	while(epc_i2c_is_busy());
	epc_wrbuf[0] = 0x28;
	epc_wrbuf[1] = 0x3A;
	epc_i2c_write(i2c_addr, epc_wrbuf, 2);
	while(epc_i2c_is_busy());
	epc_wrbuf[0] = 0x2B;
	epc_wrbuf[1] = 0x3F;
	epc_i2c_write(i2c_addr, epc_wrbuf, 2);
	while(epc_i2c_is_busy());
}

static const uint8_t curr_settings[5] = {0b01000 /*5 mA*/, 0b01000 /*5 mA*/, 0b00100 /*10mA*/, 0b10000 /*17.5mA*/, 0b01100 /*30mA*/};
void rgb_update(int bright, uint8_t r, uint8_t g, uint8_t b)
{
	if(bright < 0) bright = 0;
	else if(bright > 4) bright = 4;

	if(bright == 0)
	{
		epc_wrbuf[0] = 0x00;
		epc_wrbuf[1] = 0<<5 /*EN*/ | 1<<0 /*Software shutdown?*/;
		epc_i2c_write(rgb_addr, epc_wrbuf, 2);
		while(epc_i2c_is_busy());
	}
	else
	{
		epc_wrbuf[0] = 0x00;
		epc_wrbuf[1] = 1<<5 /*EN*/ | 0<<0 /*Software shutdown?*/;
		epc_i2c_write(rgb_addr, epc_wrbuf, 2);
		while(epc_i2c_is_busy());

		epc_wrbuf[0] = 0x03;
		epc_wrbuf[1] = curr_settings[bright];
		epc_i2c_write(rgb_addr, epc_wrbuf, 2);
		while(epc_i2c_is_busy());

		epc_wrbuf[0] = 0x04;
		epc_wrbuf[1] = g;
		epc_i2c_write(rgb_addr, epc_wrbuf, 2);
		while(epc_i2c_is_busy());

		epc_wrbuf[0] = 0x05;
		epc_wrbuf[1] = r;
		epc_i2c_write(rgb_addr, epc_wrbuf, 2);
		while(epc_i2c_is_busy());

		epc_wrbuf[0] = 0x06;
		epc_wrbuf[1] = b;
		epc_i2c_write(rgb_addr, epc_wrbuf, 2);
		while(epc_i2c_is_busy());

		epc_wrbuf[0] = 0x07; // perform update
		epc_wrbuf[1] = 0;
		epc_i2c_write(rgb_addr, epc_wrbuf, 2);
		while(epc_i2c_is_busy());
	}
}

static int err_cnt = 0;

int poll_capt_with_timeout()
{
	int timeout = 1600000; // 40000 tested to be barely ok with exposure time 125*5*5

	while(DCMI_DMA_STREAM->NDTR > 1 && timeout>0) timeout--;

	if(timeout == 0)
	{
		// Disable the stream
		DCMI_DMA_STREAM->CR = 1UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ |
				   0b10UL<<13 /*32-bit mem*/ | 0b10UL<<11 /*32-bit periph*/ |
			           1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/;

		DMA_CLEAR_INTFLAGS(DCMI_DMA, DCMI_DMA_STREAM_NUM);

		err_cnt++;
		if(err_cnt > 200)
		{
			error(13);
		}
		return 1;
	}

	return 0;
}


int poll_capt_with_timeout_complete()
{
	int timeout = 1600000;

	while(DCMI_DMA_STREAM->NDTR > 0 && timeout>0) timeout--;

	if(timeout == 0)
	{
		// Disable the stream
		DCMI_DMA_STREAM->CR = 1UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ |
				   0b10UL<<13 /*32-bit mem*/ | 0b10UL<<11 /*32-bit periph*/ |
			           1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/;

		DMA_CLEAR_INTFLAGS(DCMI_DMA, DCMI_DMA_STREAM_NUM);

		err_cnt++;
		if(err_cnt > 200)
		{
			error(14);
		}
		return 1;
	}

	return 0;
}

static epc_img_t mono_comp __attribute__((aligned(4)));
static epc_4dcs_t dcsa __attribute__((aligned(4)));


void epc_run()
{
	int idx = 0;
	int cnt = 0;

	tof_mux_select(0);

	while(1)
	{
		epc_clk_div(1);
		while(epc_i2c_is_busy());

		if(cnt==0)
			epc_dis_leds();
		else if(cnt==1)
			epc_ena_wide_leds();
		else if(cnt==2)
			epc_ena_narrow_leds();

		while(epc_i2c_is_busy());

		epc_4dcs(idx);
		while(epc_i2c_is_busy());

		epc_intlen(8, 600);
		while(epc_i2c_is_busy());

		dcmi_start_dma(&dcsa, SIZEOF_4DCS);
		trig();
		LED_ON();
		if(poll_capt_with_timeout()) error(8);
		LED_OFF();

		delay_ms(100);
//		uart_print_string_blocking("val middle = "); o_utoa16_fixed(mono_comp.img[40*EPC_XS+80], printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
		rgb_update(1, 32,  13,   0);
		delay_ms(100);
		rgb_update(1,   0,   0,   0);
		delay_ms(200);

		cnt++;
		if(cnt==3) cnt=0;
	}


}




void init_sensors()
{
	PLUS3V3_ON();


	delay_ms(300);
	PLUS10V_ON();


	delay_ms(100);
	MINUS10V_ON();

	delay_ms(100);
	RSTN_HIGH();


	LEDWIDE_ON();
	LEDNARROW_ON();

	uart_print_string_blocking("Power init done\r\n");
	delay_ms(100);

//	delay_ms(4000);


	epc_i2c_init();

	/*
		Even with 40cm cable, 40MHz (div 2) works well!

	*/
//	uart_print_string_blocking("I2C SR = "); o_btoa16_fixed(I2C1->ISR&0xffff, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");

	for(int idx = 0; idx < N_SENSORS; idx++)
//	for(int idx = 0; idx < 1; idx++)
	{
		tof_mux_select(idx);
		init_err_cnt = idx;
		delay_ms(10);

//		uart_print_string_blocking("I2C SR = "); o_btoa16_fixed(I2C1->ISR&0xffff, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");


//		{
//			epc_wrbuf[0] = 0xaa; 
//			epc_wrbuf[1] = 4;
//			epc_i2c_write(i2c_addr, epc_wrbuf, 2);
//			while(epc_i2c_is_busy());
//		}

//		uart_print_string_blocking("I2C SR = "); o_btoa16_fixed(I2C1->ISR&0xffff, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
//		delay_ms(10);
//		uart_print_string_blocking("I2C SR = "); o_btoa16_fixed(I2C1->ISR&0xffff, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");


//		while(1);

		// Reprogram the sequencer binary:
		for(int seq=0; seq<SEQ_LEN; seq++)
		{
			epc_i2c_write(i2c_addr, epc_seq[seq].d, epc_seq[seq].len);
			while(epc_i2c_is_busy());
		}

		delay_ms(10);


		{
			epc_wrbuf[0] = 0xaa; 
			epc_wrbuf[1] = 4;
			epc_i2c_write(i2c_addr, epc_wrbuf, 2);
			while(epc_i2c_is_busy());
		}

		{
			epc_wrbuf[0] = 0xab; 
			epc_wrbuf[1] = 4;
			epc_i2c_write(i2c_addr, epc_wrbuf, 2);
			while(epc_i2c_is_busy());
		}


// Temperature sensor readout procedure is weird, and requires storing and restoring some undocumented internal registers to the chip:

		{

			epc_i2c_read(i2c_addr, 0xd3, &epc_tempsens_regx[idx], 1);
			while(epc_i2c_is_busy());

			epc_i2c_read(i2c_addr, 0xd5, &epc_tempsens_regy[idx], 1);
			while(epc_i2c_is_busy());

			uint8_t temp_offset;
			epc_i2c_read(i2c_addr, 0xe8, &temp_offset, 1);
			while(epc_i2c_is_busy());
			__DSB(); __ISB();

			epc_tempsens_factory_offset[idx] = (float)temp_offset/4.7 - (float)0x12b;

		}
			/*
				Weird i2c bug: if epc_i2c_read is used, then write operations, _two_ write operations later the write fails
				because the previous write DMA is unfinished. It seems this is because of spurious STOPF interrupt happening
				somewhere between the first (succesful) epc_i2c_write and while(epc_i2c_is_busy()).  I can't get the interrupt
				away, no matter what I try. A small delay after the i2c read operation fixes the issue temporarily. 
				Empirically found out that delay_us(8) is not enough while delay_us(10) is (of course add a safety margin there).

			*/
			delay_us(50);


		{
			epc_wrbuf[0] = 0xcc; // tcmi polarity settings
			epc_wrbuf[1] = 1<<0 /*dclk rising edge*/ | 1<<7 /*saturate data*/; // bit 0 actually defaults to 1, at least on stock sequencer
			epc_i2c_write(i2c_addr, epc_wrbuf, 2);
			while(epc_i2c_is_busy());
			__DSB(); __ISB();
		}


		{
			epc_wrbuf[0] = 0x89; 
			epc_wrbuf[1] = (2 /*TCMI clock div 2..16, default 4*/ -1) | 0<<7 /*add clock delay?*/;
			epc_i2c_write(i2c_addr, epc_wrbuf, 2);
			while(epc_i2c_is_busy());
		}

		{
			epc_wrbuf[0] = 0xcb; // i2c&tcmi control
			epc_wrbuf[1] = 0b01101111; // saturation bit, split mode, gated dclk, ESM
			epc_i2c_write(i2c_addr, epc_wrbuf, 2);
			while(epc_i2c_is_busy());
		}

		{
			epc_wrbuf[0] = 0x90; // led driver control
			epc_wrbuf[1] = 0b11001000;
			epc_i2c_write(i2c_addr, epc_wrbuf, 2);
			while(epc_i2c_is_busy());
		}

		{
			epc_wrbuf[0] = 0x92; // modulation select
			epc_wrbuf[1] = 0b11000100; // grayscale
			epc_i2c_write(i2c_addr, epc_wrbuf, 2);
			while(epc_i2c_is_busy());
		}


	
	}

//	LEDWIDE_ON();

	delay_ms(100);

	dcmi_init();

	// Take a dummy frame, which will eventually output the end-of-frame sync marker, to get the DCMI sync marker parser in the right state
	// Any camera works for this, let's use 0.

	tof_mux_select(0);
	{
		dcmi_start_dma(&mono_comp, SIZEOF_MONO);
		trig();
		delay_ms(100);
	}
	tof_mux_all_off();

}


void tof_ctrl_init()
{
	tof_mux_init();

	RSTN_LOW();
	PLUS3V3_OFF();

	// Reset and power supplies:
	IO_TO_GPO(GPIOA,13);
	IO_TO_GPO(GPIOD,12);
	IO_TO_GPO(GPIOD,11);
	IO_TO_GPO(GPIOH,13);
	IO_TO_GPO(GPIOH,14);
	IO_TO_GPO(GPIOD,13);

	RCC->AHB2ENR |= 1UL<<0 /*DCMI*/;
	RCC->APB1LENR |= 1UL<<21 /*I2C1*/;

	uart_print_string_blocking("IO init done\r\n");
	//uart_print_string_blocking("SPI TSIZE = "); o_utoa16(SPI1->TSIZE, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");	


	DMAMUX1_Channel2->CCR = 34;  //DMA1 Stream2
	DMAMUX1_Channel3->CCR = 33;  //DMA1 Stream3
	DMAMUX1_Channel8->CCR = 75;  //DMA2 Strem0


	init_sensors();

	epc_run();
}
