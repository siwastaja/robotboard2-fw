#include <stdint.h>
#include "ext_include/stm32h7xx.h"
#include "stm32_cmsis_extension.h"

#include "own_std.h"
#include "tof_ctrl.h"
#include "tof_process.h" // for epc_img_t for taking the dummy image

#include "audio.h"
// Including the proprietary sensor firmware for EPC635, (c) Espros Photonics Corporation,
// not available for public distribution. Pulu Robotics has a permission to redistribute this
// small binary to our own customers. Please contant us for more information.
#include "epc635_seq_v10.c"


#include "misc.h"

#include "tof_muxing.h"

// Same for REV2A, REV2B
#define RSTN_HIGH()     do{HI(GPIOA,13);}while(0)
#define RSTN_LOW()      do{LO(GPIOA,13);}while(0)

#ifdef REV2A

	// REV2A has separate supplies for wide and narrow LEDs:

	#define LEDWIDE_ON()    do{HI(GPIOD,12);}while(0)
	#define LEDWIDE_OFF()   do{LO(GPIOD,12);}while(0)

	#define LEDNARROW_ON()  do{HI(GPIOD,11);}while(0)
	#define LEDNARROW_OFF() do{LO(GPIOD,11);}while(0)

	#define LEDSUP_ON()	do{LEDWIDE_ON(); LEDNARROW_ON();}while(0)
	#define LEDSUP_OFF()	do{LEDWIDE_OFF(); LEDNARROW_OFF();}while(0)

#endif

#ifdef REV2B

	#define LEDSUP_ON()	do{HI(GPIOD,12);}while(0)
	#define LEDSUP_OFF()	do{LO(GPIOD,12);}while(0)

#endif



// Same for REV2A, REV2B
#define PLUS3V3_ON()    do{LO(GPIOH,13);}while(0)
#define PLUS3V3_OFF()   do{HI(GPIOH,13);}while(0)

// Same for REV2A, REV2B
#define PLUS10V_ON()    do{HI(GPIOH,14);}while(0)
#define PLUS10V_OFF()   do{LO(GPIOH,14);}while(0)

// Same for REV2A, REV2B
#define MINUS10V_ON()   do{HI(GPIOD,13);}while(0)
#define MINUS10V_OFF()  do{LO(GPIOD,13);}while(0)




/*
	All sensors use a single I2C bus, with the multiplexer connecting one sensor at a time.
	The I2C bus, at any time, only sees one PuluTOF2 module (the EPC635, and the ISSI LED driver).
	This makes everything simple.
*/

// 1 or 0                                  0 1 2 3 4   5 6 7 8 9
const uint8_t sensors_in_use[N_SENSORS] = 
#ifdef CALIBRATOR_BOX
// 0 1 2 3 4   5 6 7 8 9
  {1,0,0,0,0,  0,0,0,0,0};
#else
// 0 1 2 3 4   5 6 7 8 9
#ifdef EXT_VACUUM
  {0,1,1,1,1,  1,1,1,1,1};
#else
  {1,1,1,1,1,  1,1,1,1,1};
#endif
#endif

uint32_t sensor_silicon_ids[N_SENSORS];

#define WR_DMA DMA1
#define WR_DMA_STREAM DMA1_Stream2
#define WR_DMA_STREAM_NUM 2

#define RD_DMA DMA1
#define RD_DMA_STREAM DMA1_Stream3
#define RD_DMA_STREAM_NUM 3
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

volatile uint8_t epc_wrbuf[16] __attribute__((aligned(4)));
volatile uint8_t epc_rdbuf[16] __attribute__((aligned(4)));



// Tested digital filter up to max 15 without ill effects. Use a bit shorter anyway...
// Things written to I2C CR1 every time:
#define I2C_CR1_BASICS_OFF (12UL<<8 /*Digital filter len 0 to 15*/)
#define I2C_CR1_BASICS_ON (I2C_CR1_BASICS_OFF | 1UL /*keep it on*/)


void epc_safety_shutdown() __attribute__((section(".text_itcm")));
void epc_safety_shutdown()
{
	// It's most important to bring down the analog supplies in the correct order, and do this quickly.
	// When it's done, it's an optional plus to turn 3V3 off.
	RSTN_LOW();
	MINUS10V_OFF();
	PLUS10V_OFF();
	tof_mux_all_off();
	LEDSUP_OFF();
//	delay_ms(100);
//	PLUS3V3_OFF();
}


void epc_shutdown() __attribute__((section(".text_itcm")));
void epc_shutdown()
{
	// It's most important to bring down the analog supplies in the correct order, and do this quickly.
	// When it's done, it's an optional plus to turn 3V3 off.
	RSTN_LOW();
	MINUS10V_OFF();
	PLUS10V_OFF();
	tof_mux_all_off();
	LEDSUP_OFF();
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
		error(61);

	if(DMA1->LISR & 1UL<<22)
		error(62);
	if(DMA1->LISR & 1UL<<24)
		error(63);
	if(DMA1->LISR & 1UL<<25)
		error(64);

	if(WR_DMA_STREAM->CR & 1UL)
	{
		if(DMA1->LISR & 1UL<<27)
			error(65);

		uart_print_string_blocking("I2C SR = "); o_btoa16_fixed(I2C1->ISR&0xffff, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");

		error(80+init_err_cnt);
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
		error(66);
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
		// Seems to be high, sometimes: let's try removing the check?
		if(I2C1->ISR & (1UL<<15)) // busy shouldn't be high - at least fail instead of doing random shit
		{
			error(67);
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

	NVIC_SetPriority(I2C1_EV_IRQn, INTPRIO_TOF_I2C);
	NVIC_EnableIRQ(I2C1_EV_IRQn);
	NVIC_SetPriority(RD_DMA_STREAM_IRQ, INTPRIO_TOF_I2C);
	NVIC_EnableIRQ(RD_DMA_STREAM_IRQ);
}


volatile int epc_capture_finished = 0;
void epc_dcmi_dma_inthandler()
{
	// DMA finished
	DMA_CLEAR_INTFLAGS(DMA2, 7);
	epc_capture_finished = 1;
}

#define DCMI_DMA_CR (0b01UL<<16 /*med prio*/ | 0b10UL<<13 /*32-bit mem*/ | 0b10UL<<11 /*32-bit periph*/ | 1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/)

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

	// Program the crop values for narrow beam data. Crop enable bit is toggled on or off later.
	DCMI->CWSTRTR = (TOF_NARROW_Y_START)<<16 | ((TOF_NARROW_X_START*2));
	DCMI->CWSIZER = (TOF_YS_NARROW -1)<<16 | ((TOF_XS_NARROW*2)-1);


	DCMI->CR |= 1UL<<14; // Enable

	DCMI_DMA_STREAM->PAR = (uint32_t)&(DCMI->DR);
	DCMI_DMA_STREAM->CR = DCMI_DMA_CR;

//	DCMI_DMA_STREAM->FCR = 1UL<<2 /* Enable FIFO */ | 0b01UL /*FIFO threshold 50% (2 words)*/;
	// NDTR is not set yet because it varies. Stream is not enabled yet.

	DCMI->CR |= 1UL<<0; // Start CAPTURE
}

void dcmi_start_dma(void *data, int size)
{
	// Disable the stream first
	DCMI_DMA_STREAM->CR = DCMI_DMA_CR;

	while(DCMI_DMA_STREAM->CR & 1UL) ;

	DMA_CLEAR_INTFLAGS(DCMI_DMA, 0);

	DCMI_DMA_STREAM->M0AR = (uint32_t)data;

	DCMI_DMA_STREAM->NDTR = size/4; // Num of 32-bit transfers
	DCMI_DMA_STREAM->CR = DCMI_DMA_CR | 1UL<<4 /* Transfer complete interrupt*/;

	DMA_CLEAR_INTFLAGS(DCMI_DMA, DCMI_DMA_STREAM_NUM);
	DCMI_DMA_STREAM->CR |= 1; // Enable DMA
	__DSB();
}

void epc_trig()
{
	static volatile uint8_t b[2] = {0xa4, 1};
	epc_i2c_write(i2c_addr, b, 2);
}


// WARNING: this parameter changed from earlier (pulutof1) codebase.
// 0 (not 1) is now 20MHz.
void epc_clk_div(int div)
{
	epc_wrbuf[0] = 0x85;
	epc_wrbuf[1] = div;
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

void epc_ena_wide_and_narrow_leds()
{
	epc_wrbuf[0] = 0x90;
	epc_wrbuf[1] = 0b11101100;
	epc_i2c_write(i2c_addr, epc_wrbuf, 2);
}

void epc_greyread_single_ended()
{
	epc_wrbuf[0] = 0x3a;
	epc_wrbuf[1] = 0x10;
	epc_i2c_write(i2c_addr, epc_wrbuf, 2);
}

void epc_greyread_differential()
{
	epc_wrbuf[0] = 0x3a;
	epc_wrbuf[1] = 0x00;
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

// Actually a 10-bit multiplier is supported, but we never want such long integration times, and save one register write.
// Multiplier: 1 to 255
// time: From 1 to 16384
// Integration length will be:
// clk_div/80MHz * multiplier * (regval+1), which equals:
// clk_div/80MHz * multiplier * time*4, which equals:
// clk_div/20MHz * multiplier * time, or,
// 1/led_freq * multiplier * time.

// Set the multiplier to LED freq in MHz, and time is us directly!
// 20MHz LED, mult=20, time=1000 -> 1 ms.
// Remember this is the length of one frame: 4dcs has four times total exposure.


void epc_intlen(uint8_t multiplier, uint16_t time) // OK to do while acquiring: shadow registered: applied to next trigger.
{
	if(time > 16383)
		error(454);
//	DBG_PR_VAR_I32(multiplier);
//	DBG_PR_VAR_I32(time);
	int intlen = ((int)time<<2)-1; // Actual regval must be a multiple of four minus 1: this guarantees this condition.
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
	block_epc_i2c(0);

	epc_wrbuf[0] = 0xD5;
	epc_wrbuf[1] = epc_tempsens_regy[idx] & 0x0f;
	epc_i2c_write(i2c_addr, epc_wrbuf, 2);
	block_epc_i2c(0);
}


// Do the magic stuff specified in the datasheet to disable temperature sensor conversion, back to normal operation
void epc_temperature_magic_mode_off(int idx)
{
	epc_wrbuf[0] = 0xD3;
	epc_wrbuf[1] = epc_tempsens_regx[idx];
	epc_i2c_write(i2c_addr, epc_wrbuf, 2);
	block_epc_i2c(0);

	epc_wrbuf[0] = 0xD5;
	epc_wrbuf[1] = epc_tempsens_regy[idx];
	epc_i2c_write(i2c_addr, epc_wrbuf, 2);
	block_epc_i2c(0);
}

static uint16_t epc_read_temperature_regs()
{
	static uint8_t hi, lo; // Not in stack! Stack in DTCM isn't accessible by DMA.
	epc_i2c_read(i2c_addr, 0x60, &hi, 1);
	block_epc_i2c(5);
	delay_us(50); // see the comment about i2c read bug in top_init

	epc_i2c_read(i2c_addr, 0x61, &lo, 1);
	block_epc_i2c(6);

	delay_us(50); // see the comment about i2c read bug in top_init
	__DSB();
	return ((uint16_t)hi<<8) | ((uint16_t)lo);
}

int32_t epc_read_temperature(int idx)
{
	int32_t temp = epc_read_temperature_regs();
	return ((float)(temp-0x2000)*0.134 + epc_tempsens_factory_offset[idx])*10.0;
}

uint8_t epc_reg_read(uint8_t addr)
{
	static uint8_t val; // Not in stack! Stack in DTCM isn't accessible by DMA.
	epc_i2c_read(i2c_addr, addr, &val, 1);
	block_epc_i2c(7);
	delay_us(50); // see the comment about i2c read bug in top_init
	__DSB();
	return val;
}

void epc_fix_modulation_table_defaults()
{
	epc_wrbuf[0] = 0x22;
	epc_wrbuf[1] = 0x30;
	epc_i2c_write(i2c_addr, epc_wrbuf, 2);
	block_epc_i2c(0);
	epc_wrbuf[0] = 0x25;
	epc_wrbuf[1] = 0x35;
	epc_i2c_write(i2c_addr, epc_wrbuf, 2);
	block_epc_i2c(0);
	epc_wrbuf[0] = 0x28;
	epc_wrbuf[1] = 0x3A;
	epc_i2c_write(i2c_addr, epc_wrbuf, 2);
	block_epc_i2c(0);
	epc_wrbuf[0] = 0x2B;
	epc_wrbuf[1] = 0x3F;
	epc_i2c_write(i2c_addr, epc_wrbuf, 2);
	block_epc_i2c(0);
}

void epc_enable_dll()
{
	epc_wrbuf[0] = 0xae;
	epc_wrbuf[1] = 0x04;
	epc_i2c_write(i2c_addr, epc_wrbuf, 2);
}

void epc_disable_dll()
{
	epc_wrbuf[0] = 0xae;
	epc_wrbuf[1] = 0x01;
	epc_i2c_write(i2c_addr, epc_wrbuf, 2);
}

void epc_coarse_dll_steps(int steps)
{
	epc_wrbuf[0] = 0x73;
	epc_wrbuf[1] = steps;
	epc_i2c_write(i2c_addr, epc_wrbuf, 2);
}

void epc_fine_dll_steps(int steps)
{
	epc_wrbuf[0] = 0x71;
	epc_wrbuf[1] = (steps&0xff00)>>8;
	epc_wrbuf[2] = (steps&0x00ff)>>0;
	epc_i2c_write(i2c_addr, epc_wrbuf, 3);
}

void epc_pll_steps(int steps)
{
	epc_wrbuf[0] = 0x8b;
	epc_wrbuf[1] = steps;
	epc_i2c_write(i2c_addr, epc_wrbuf, 2);
}

void epc_start_read_eeprom(uint8_t addr)
{
	epc_wrbuf[0] = 0x11;
	epc_wrbuf[1] = addr;
	epc_i2c_write(i2c_addr, epc_wrbuf, 2);
}

uint8_t epc_read_eeprom_byte()
{
	uint8_t val = epc_reg_read(0x12);
	block_epc_i2c(1);
	delay_us(50);
	return val;
}


static const uint8_t curr_settings[5] = {0b01000 /*5 mA*/, 0b01000 /*5 mA*/, 0b00100 /*10mA*/, 0b10000 /*17.5mA*/, 0b01100 /*30mA*/};
void rgb_update(uint32_t val)
{
	int bright = (val&0xff000000)>>24;
	int r = (val&0x00ff0000)>>16;
	int g = (val&0x0000ff00)>>8;
	int b = (val&0x000000ff)>>0;
	if(bright < 0) bright = 0;
	else if(bright > 4) bright = 4;

	if(bright == 0)
	{
		epc_wrbuf[0] = 0x00;
		epc_wrbuf[1] = 0<<5 /*EN*/ | 1<<0 /*Software shutdown?*/;
		epc_i2c_write(rgb_addr, epc_wrbuf, 2);
		block_epc_i2c(2);
	}
	else
	{
		epc_wrbuf[0] = 0x00;
		epc_wrbuf[1] = 1<<5 /*EN*/ | 0<<0 /*Software shutdown?*/;
		epc_i2c_write(rgb_addr, epc_wrbuf, 2);
		block_epc_i2c(2);

		epc_wrbuf[0] = 0x03;
		epc_wrbuf[1] = curr_settings[bright];
		epc_i2c_write(rgb_addr, epc_wrbuf, 2);
		block_epc_i2c(2);

		epc_wrbuf[0] = 0x04;
		epc_wrbuf[1] = g;
		epc_i2c_write(rgb_addr, epc_wrbuf, 2);
		block_epc_i2c(2);

		epc_wrbuf[0] = 0x05;
		epc_wrbuf[1] = r;
		epc_i2c_write(rgb_addr, epc_wrbuf, 2);
		block_epc_i2c(2);

		epc_wrbuf[0] = 0x06;
		epc_wrbuf[1] = b;
		epc_i2c_write(rgb_addr, epc_wrbuf, 2);
		block_epc_i2c(2);

		epc_wrbuf[0] = 0x07; // perform update
		epc_wrbuf[1] = 0;
		epc_i2c_write(rgb_addr, epc_wrbuf, 2);
		block_epc_i2c(2);
	}
}

static int err_cnt = 0;

int poll_data_transfer_start_with_timeout(int size)
{
	int timeout = 2*1600000;

// DMA counter based implementation:
#if 0
	int orig_ndtr = size/4;
	while(DCMI_DMA_STREAM->NDTR >= orig_ndtr && timeout>0)
#endif

// DCMI VSYNC status bit based implementation:
#if 1
	while((DCMI->SR & 2) /* "synchronization between frames" */ && timeout>0)	
#endif
	{
//		DBG_PR_VAR_U16(DCMI_DMA_STREAM->NDTR);
		timeout--;
	}

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
			error(68);
		}
		return 1;
	}

	return 0;

}

int poll_capt_with_timeout()
{
	int timeout = 2*1600000; // 40000 tested (on 216MHz CPU) to be barely ok with exposure time 125*5*5
//	int timeout = 30;

	while(DCMI_DMA_STREAM->NDTR > 1 && timeout>0)
	{
//		DBG_PR_VAR_U16(DCMI_DMA_STREAM->NDTR);
		timeout--;
	}

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
			error(68);
		}
		return 1;
	}

	return 0;
}


int poll_capt_with_timeout_complete()
{
	int timeout = 2*1600000;

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
			error(69);
		}
		return 1;
	}

	return 0;
}

void block_epc_i2c(int err_idx)
{
	int timeout = 100000;
	while(epc_i2c_is_busy())
	{
		if(--timeout == 0)
			error(90+err_idx);
	}
}



#define SOFTSTART_PERIOD_LEN 400

void init_sensors()
{
	// 3V3 line has too much capacitance to be turned on suddenly. Otherwise, the 3V3 line dips and the CPU brownouts.
	for(int i=0; i<SOFTSTART_PERIOD_LEN/2; i++)
	{
		PLUS3V3_OFF();
		delay_us(29);
		PLUS3V3_ON();
		delay_us(1);
	}

	for(int i=0; i<SOFTSTART_PERIOD_LEN; i++)
	{
		PLUS3V3_OFF();
		delay_us(19);
		PLUS3V3_ON();
		delay_us(1);
	}

	for(int i=0; i<SOFTSTART_PERIOD_LEN; i++)
	{
		PLUS3V3_OFF();
		delay_us(18);
		PLUS3V3_ON();
		delay_us(2);
	}

	for(int i=0; i<SOFTSTART_PERIOD_LEN; i++)
	{
		PLUS3V3_OFF();
		delay_us(17);
		PLUS3V3_ON();
		delay_us(3);
	}

	for(int i=0; i<SOFTSTART_PERIOD_LEN; i++)
	{
		PLUS3V3_OFF();
		delay_us(16);
		PLUS3V3_ON();
		delay_us(4);
	}

	for(int i=0; i<SOFTSTART_PERIOD_LEN; i++)
	{
		PLUS3V3_OFF();
		delay_us(14);
		PLUS3V3_ON();
		delay_us(6);
	}

	for(int i=0; i<SOFTSTART_PERIOD_LEN; i++)
	{
		PLUS3V3_OFF();
		delay_us(12);
		PLUS3V3_ON();
		delay_us(8);
	}

	for(int i=0; i<SOFTSTART_PERIOD_LEN; i++)
	{
		PLUS3V3_OFF();
		delay_us(10);
		PLUS3V3_ON();
		delay_us(10);
	}

	for(int i=0; i<SOFTSTART_PERIOD_LEN; i++)
	{
		PLUS3V3_OFF();
		delay_us(8);
		PLUS3V3_ON();
		delay_us(12);
	}

	for(int i=0; i<SOFTSTART_PERIOD_LEN; i++)
	{
		PLUS3V3_OFF();
		delay_us(6);
		PLUS3V3_ON();
		delay_us(14);
	}

	for(int i=0; i<SOFTSTART_PERIOD_LEN; i++)
	{
		PLUS3V3_OFF();
		delay_us(3);
		PLUS3V3_ON();
		delay_us(17);
	}


	delay_ms(30);

	PLUS10V_ON();

	delay_ms(30);

	MINUS10V_ON();

	delay_ms(30);

	RSTN_HIGH();

	//uart_print_string_blocking("Power init done\r\n");
	delay_ms(50);

	LEDSUP_ON();


	epc_i2c_init();

	/*
		Even with 40cm cable, 40MHz (div 2) works well!
		But, the readout speed is limited by the ADC conversion in the sensor - so we get the same performance at 20MHz.
		Frame readout is 1.874ms anyway.

	*/
//	uart_print_string_blocking("I2C SR = "); o_btoa16_fixed(I2C1->ISR&0xffff, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");

	int last_valid_idx = 0; // any camera that exists and works
	for(int idx = 0; idx < N_SENSORS; idx++)
	{
		if(!sensors_in_use[idx])
			continue;

		//DBG_PR_VAR_U16(idx);

		last_valid_idx = idx;

		tof_mux_select(idx);
		init_err_cnt = idx;
		delay_us(100);

		rgb_update(0x0400ff00);

	//	while(1);


//		uart_print_string_blocking("I2C SR = "); o_btoa16_fixed(I2C1->ISR&0xffff, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");


//		{
//			epc_wrbuf[0] = 0xaa; 
//			epc_wrbuf[1] = 4;
//			epc_i2c_write(i2c_addr, epc_wrbuf, 2);
//			block_epc_i2c();
//		}

//		uart_print_string_blocking("I2C SR = "); o_btoa16_fixed(I2C1->ISR&0xffff, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
//		delay_ms(10);
//		uart_print_string_blocking("I2C SR = "); o_btoa16_fixed(I2C1->ISR&0xffff, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");


//		while(1);

		// Reprogram the sequencer binary:
		for(int seq=0; seq<SEQ_LEN; seq++)
		{
			epc_i2c_write(i2c_addr, epc_seq[seq].d, epc_seq[seq].len);
			block_epc_i2c(0);
		}

		delay_ms(10);

		// Registers 0xaa and 0xab (undocumented) need to be set to 4, and 4. The reason is not explained.

		{
			epc_wrbuf[0] = 0xaa; 
			epc_wrbuf[1] = 4;
			epc_i2c_write(i2c_addr, epc_wrbuf, 2);
			block_epc_i2c(0);
		}

		{
			epc_wrbuf[0] = 0xab; 
			epc_wrbuf[1] = 4;
			epc_i2c_write(i2c_addr, epc_wrbuf, 2);
			block_epc_i2c(0);
		}


// Temperature sensor readout procedure is a bit weird, and requires storing and restoring some undocumented internal registers to the chip:

		{

			epc_i2c_read(i2c_addr, 0xd3, &epc_tempsens_regx[idx], 1);
			block_epc_i2c(1);
			delay_us(50);

			epc_i2c_read(i2c_addr, 0xd5, &epc_tempsens_regy[idx], 1);
			block_epc_i2c(1);
			delay_us(50);

			static uint8_t temp_offset; // Not in stack! Stack in DTCM isn't accessible by DMA.
			epc_i2c_read(i2c_addr, 0xe8, &temp_offset, 1);
			block_epc_i2c(1);
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
			block_epc_i2c(0);
			__DSB(); __ISB();
		}


		{
			epc_wrbuf[0] = 0x89; 
			epc_wrbuf[1] = (4 /*TCMI clock div 2..16, default 4*/ -1) | 0<<7 /*add clock delay?*/;
			epc_i2c_write(i2c_addr, epc_wrbuf, 2);
			block_epc_i2c(0);
		}

		{
			epc_wrbuf[0] = 0xcb; // i2c&tcmi control
			epc_wrbuf[1] = 0b01101111; // saturation bit, split mode, gated dclk, embedded sync
			epc_i2c_write(i2c_addr, epc_wrbuf, 2);
			block_epc_i2c(0);
		}

		{
			epc_wrbuf[0] = 0x90; // led driver control
			epc_wrbuf[1] = 0b11001000;
			epc_i2c_write(i2c_addr, epc_wrbuf, 2);
			block_epc_i2c(0);
		}

		{
			epc_wrbuf[0] = 0x92; // modulation select
			epc_wrbuf[1] = 0b11000100; // grayscale
			epc_i2c_write(i2c_addr, epc_wrbuf, 2);
			block_epc_i2c(0);
		}

		epc_enable_dll(); block_epc_i2c(0);
		epc_coarse_dll_steps(0); block_epc_i2c(0);
		epc_pll_steps(0); block_epc_i2c(4); // THIS ISN'T ZERO BY DEFAULT!!


		//epc_normalphase_or_int();


		// Read and store the unique chip id:

		epc_start_read_eeprom(0xf6); block_epc_i2c(4);
		uint8_t waferid_msb = epc_read_eeprom_byte(); 
		uint8_t waferid_lsb = epc_read_eeprom_byte();

		uint8_t chipid_msb = epc_read_eeprom_byte();
		uint8_t chipid_lsb = epc_read_eeprom_byte();

		uint16_t wafer_id = ((uint16_t)waferid_msb)<<8 | ((uint16_t)waferid_lsb);
		uint16_t chip_id = ((uint16_t)chipid_msb)<<8 | ((uint16_t)chipid_lsb);

//		DBG_PR_VAR_U16(wafer_id);
//		DBG_PR_VAR_U16(chip_id);
		uint32_t combined_id = ((uint32_t)wafer_id<<16) | (uint32_t)chip_id;
//		DBG_PR_VAR_U32(combined_id);

		sensor_silicon_ids[idx] = combined_id;

		rgb_update(0);

//		uart_print_string_blocking("single sensor init success\r\n");
	}

	delay_ms(1);


	dcmi_init();

	// Take a dummy frame, which will eventually output the end-of-frame sync marker, to get the DCMI sync marker parser in the right state
	// Any camera works for this

	tof_mux_select(last_valid_idx);
	delay_us(10);

	{
		epc_intlen(1, 1); block_epc_i2c(0);
		extern epc_img_t mono_comp;
		dcmi_start_dma(&mono_comp, SIZEOF_MONO);
		epc_trig();
		delay_ms(10);
	}

	tof_mux_all_off();
	uart_print_string_blocking("sensors init success\r\n");

}

void dcmi_crop_narrow()
{
	DCMI->CR |= 1UL<<2;
}

void dcmi_crop_wide()
{
	DCMI->CR &= ~(1UL<<2);
}

void tof_ctrl_init()
{
	tof_mux_init();

	RSTN_LOW();
	PLUS3V3_OFF();

	// Reset and power supplies:
	IO_TO_GPO(GPIOA,13);
	IO_TO_GPO(GPIOD,12);
	#ifdef REV2A
		IO_TO_GPO(GPIOD,11); // VLED_NARROW
	#endif
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

}
