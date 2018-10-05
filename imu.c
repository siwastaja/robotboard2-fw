/*

STM32H7 MASSIVE TRAP WARNING
(So-called "vittu mitä paskaa")

BDMA ON D3 CAN ONLY ACCESS SRAM4, NOT FLASH NOR AXISRAM, SRAM1,2,3

THIS OBVIOUS TRAP IS NOT CLEARLY DOCUMENTED ANYWHERE IN THE REFERENCE MANUAL.

https://community.st.com/s/article/FAQ-DMA-is-not-working-on-STM32H7-devices


*/

#include <stdint.h>
#include <string.h>
#include "ext_include/stm32h7xx.h"
#include "stm32_cmsis_extension.h"
#include "misc.h"
#include "own_std.h"

volatile int kakka_cnt, kukka_cnt;

static char printbuf[128];

#define IMU0_PRESENT
#define IMU1_PRESENT
#define IMU2_PRESENT
#define IMU3_PRESENT
#define IMU4_PRESENT
#define IMU5_PRESENT

#define SPI_CLKDIV_REGVAL (0b011UL) // 6.25MHz SPI clock. Datasheet maximum: 10MHz

#define IMU024_G_NCS_PORT GPIOA
#define IMU024_G_NCS_PIN  14
#define IMU024_A_NCS_PORT GPIOC
#define IMU024_A_NCS_PIN  10
#define IMU024_M_NCS_PORT GPIOD
#define IMU024_M_NCS_PIN  0

#define IMU135_G_NCS_PORT GPIOD
#define IMU135_G_NCS_PIN  1
#define IMU135_A_NCS_PORT GPIOH
#define IMU135_A_NCS_PIN  15
#define IMU135_M_NCS_PORT GPIOI
#define IMU135_M_NCS_PIN  0

#define SEL_G024()   do{LO(IMU024_G_NCS_PORT,IMU024_G_NCS_PIN); __DSB();}while(0)
#define DESEL_G024() do{HI(IMU024_G_NCS_PORT,IMU024_G_NCS_PIN); __DSB();}while(0)
#define SEL_A024()   do{LO(IMU024_A_NCS_PORT,IMU024_A_NCS_PIN); __DSB();}while(0)
#define DESEL_A024() do{HI(IMU024_A_NCS_PORT,IMU024_A_NCS_PIN); __DSB();}while(0)
#define SEL_M024()   do{LO(IMU024_M_NCS_PORT,IMU024_M_NCS_PIN); __DSB();}while(0)
#define DESEL_M024() do{HI(IMU024_M_NCS_PORT,IMU024_M_NCS_PIN); __DSB();}while(0)

#define SEL_G135()   do{LO(IMU135_G_NCS_PORT,IMU135_G_NCS_PIN); __DSB();}while(0)
#define DESEL_G135() do{HI(IMU135_G_NCS_PORT,IMU135_G_NCS_PIN); __DSB();}while(0)
#define SEL_A135()   do{LO(IMU135_A_NCS_PORT,IMU135_A_NCS_PIN); __DSB();}while(0)
#define DESEL_A135() do{HI(IMU135_A_NCS_PORT,IMU135_A_NCS_PIN); __DSB();}while(0)
#define SEL_M135()   do{LO(IMU135_M_NCS_PORT,IMU135_M_NCS_PIN); __DSB();}while(0)
#define DESEL_M135() do{HI(IMU135_M_NCS_PORT,IMU135_M_NCS_PIN); __DSB();}while(0)


#define AGM01_SPI SPI4
#define AGM01_SPI_IRQ SPI4_IRQn

#define AGM23_SPI SPI6
#define AGM23_SPI_IRQ SPI6_IRQn

#define AGM45_SPI SPI2
#define AGM45_SPI_IRQ SPI2_IRQn


/*
Stupid heterogenous structure. Instead of 3 similar DMA's, 1 of them is different "BDMA", which is actually still
almost the same, just a bit different (for added complexity). Nomenclature note: "channel" is the same as "stream".

SPI6 cannot be mapped to DMA1 or DMA2, so it has to be mapped to BDMA.

*/

#define AGM01_TX_DMA DMA1
#define AGM01_TX_DMA_STREAM DMA1_Stream6
#define AGM01_TX_DMA_STREAM_NUM 6
#define AGM01_TX_DMA_STREAM_IRQ DMA1_Stream6_IRQn
#define AGM01_TX_DMAMUX() do{DMAMUX1_Channel6->CCR = 84;}while(0)

#define AGM23_TX_DMA BDMA
#define AGM23_TX_DMA_STREAM BDMA_Channel1
#define AGM23_TX_DMA_STREAM_NUM 1
#define AGM23_TX_DMA_STREAM_IRQ BDMA_Channel1_IRQn
#define AGM23_TX_DMAMUX() do{DMAMUX2_Channel1->CCR = 12;}while(0)

#define AGM45_TX_DMA DMA1
#define AGM45_TX_DMA_STREAM DMA1_Stream7
#define AGM45_TX_DMA_STREAM_NUM 7
#define AGM45_TX_DMA_STREAM_IRQ DMA1_Stream7_IRQn
#define AGM45_TX_DMAMUX() do{DMAMUX1_Channel7->CCR = 40;}while(0)


#define AGM01_RX_DMA DMA1
#define AGM01_RX_DMA_STREAM DMA1_Stream4
#define AGM01_RX_DMA_STREAM_NUM 4
#define AGM01_RX_DMA_STREAM_IRQ DMA1_Stream4_IRQn
#define AGM01_RX_DMAMUX() do{DMAMUX1_Channel4->CCR = 83;}while(0)

#define AGM23_RX_DMA BDMA
#define AGM23_RX_DMA_STREAM BDMA_Channel0
#define AGM23_RX_DMA_STREAM_NUM 0
#define AGM23_RX_DMA_STREAM_IRQ BDMA_Channel0_IRQn
#define AGM23_RX_DMAMUX() do{DMAMUX2_Channel0->CCR = 11;}while(0)

#define AGM45_RX_DMA DMA1
#define AGM45_RX_DMA_STREAM DMA1_Stream5
#define AGM45_RX_DMA_STREAM_NUM 5
#define AGM45_RX_DMA_STREAM_IRQ DMA1_Stream5_IRQn
#define AGM45_RX_DMAMUX() do{DMAMUX1_Channel5->CCR = 39;}while(0)

void agm01_errhandler()
{
	uart_print_string_blocking("\r\n\r\nSTOPPED: ");
	uart_print_string_blocking(__func__);
	uart_print_string_blocking("\r\n");

	if(AGM01_TX_DMA_STREAM->CR & 1) uart_print_string_blocking("TX01 ");
	if(AGM01_RX_DMA_STREAM->CR & 1) uart_print_string_blocking("RX01 ");
	uart_print_string_blocking("\r\n01SPICR = "); o_btoa16_fixed(AGM01_SPI->CR1&0xffff, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("01SPISR = "); o_btoa16_fixed(AGM01_SPI->SR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("CTSIZE = "); o_utoa16_fixed(AGM01_SPI->SR>>16, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("01TXDMA = "); o_btoa8_fixed(DMA_INTFLAGS(AGM01_TX_DMA, AGM01_TX_DMA_STREAM_NUM), printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("01RXDMA = "); o_btoa8_fixed(DMA_INTFLAGS(AGM01_RX_DMA, AGM01_RX_DMA_STREAM_NUM), printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("01TX NDTR  = "); o_utoa16(AGM01_TX_DMA_STREAM->NDTR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("01RX NDTR  = "); o_utoa16(AGM01_RX_DMA_STREAM->NDTR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("01SPICFG2 = "); o_utoa32_hex(AGM01_SPI->CFG2, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");

	uart_print_string_blocking("\r\n");
	error(20);

}

void agm23_errhandler()
{
	uart_print_string_blocking("\r\n\r\nSTOPPED: ");
	uart_print_string_blocking(__func__);
	uart_print_string_blocking("\r\n");

	if(AGM23_TX_DMA_STREAM->CCR & 1) uart_print_string_blocking("TX23 ");
	if(AGM23_RX_DMA_STREAM->CCR & 1) uart_print_string_blocking("RX23 ");
	uart_print_string_blocking("\r\n23SPICR = "); o_btoa16_fixed(AGM23_SPI->CR1&0xffff, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("23SPISR = "); o_btoa16_fixed(AGM23_SPI->SR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("CTSIZE = "); o_utoa16_fixed(AGM23_SPI->SR>>16, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("23TXDMA = "); o_btoa8_fixed(BDMA_INTFLAGS(AGM23_TX_DMA, AGM23_TX_DMA_STREAM_NUM), printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("23RXDMA = "); o_btoa8_fixed(BDMA_INTFLAGS(AGM23_RX_DMA, AGM23_RX_DMA_STREAM_NUM), printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("23TX NDTR  = "); o_utoa16(AGM23_TX_DMA_STREAM->CNDTR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("23RX NDTR  = "); o_utoa16(AGM23_RX_DMA_STREAM->CNDTR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("23SPICFG2 = "); o_utoa32_hex(AGM23_SPI->CFG2, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");

	uart_print_string_blocking("\r\n");
	error(20);

}

void agm45_errhandler()
{
	uart_print_string_blocking("\r\n\r\nSTOPPED: ");
	uart_print_string_blocking(__func__);
	uart_print_string_blocking("\r\n");

	if(AGM45_TX_DMA_STREAM->CR & 1) uart_print_string_blocking("TX45 ");
	if(AGM45_RX_DMA_STREAM->CR & 1) uart_print_string_blocking("RX45 ");
	uart_print_string_blocking("\r\n45SPICR = "); o_btoa16_fixed(AGM45_SPI->CR1&0xffff, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("45SPISR = "); o_btoa16_fixed(AGM45_SPI->SR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("CTSIZE = "); o_utoa16_fixed(AGM45_SPI->SR>>16, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("45TXDMA = "); o_btoa8_fixed(DMA_INTFLAGS(AGM45_TX_DMA, AGM45_TX_DMA_STREAM_NUM), printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("45RXDMA = "); o_btoa8_fixed(DMA_INTFLAGS(AGM45_RX_DMA, AGM45_RX_DMA_STREAM_NUM), printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("45TX NDTR  = "); o_utoa16(AGM45_TX_DMA_STREAM->NDTR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("45RX NDTR  = "); o_utoa16(AGM45_RX_DMA_STREAM->NDTR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("45SPICFG2 = "); o_utoa32_hex(AGM45_SPI->CFG2, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");

	uart_print_string_blocking("\r\n");
	error(20);

}


#define WR_REG(_a_, _d_) ( (_a_) | ((_d_)<<8) )
#define M_OP_NORMAL (0b00<<1)
#define M_OP_FORCED (0b01<<1)
#define M_OP_SLEEP  (0b11<<1)



/*
	Analysis for required bandwidth and latency:

	Maximum sustained rotational speed 90 deg/s
	Let's assume a meaningful short time maximum (more than a <0.5 degree jerk) angular rate is twice that, 180 deg/s.
	In 5 ms, robot turns 0.9 degrees. When looking at subjects 10 meters away, they shift 
	tan(0.9 degrees)*10000 = 157mm. Seems acceptable given that this is a fairly extreme angular rate and not something
	happening all the time during mapping.

	-> 200Hz is OK.


	Data flow synchronization:

	6 gyros and accelerometers would require 12 interrupt lines to serve in real time without polling. Such number of IOs
	were not available.

	Sensors run on their internal clocks, exact frequency being unspecified and subject to drift.

	Because we want data with low latency, we only want to keep the sensor FIFO fill level to max 1.

	Unfortunately, the register ordering makes it impossible to read the FIFO fill level on the same burst read with the data.

	The sensor is documented to give out all zeroes when reading empty FIFO.

	ISR runs at 10kHz (0.1ms)

	Rate for A: 250Hz (4ms)
	Rate for G: 200Hz (5ms)

*/

typedef union __attribute__((packed))
{
	struct __attribute__((packed))
	{
		uint8_t dummy1;
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t dummy2;
	} coords;

	struct __attribute__((packed))
	{
		uint32_t first;
		uint32_t second;
	} blocks;
} xyz_in_fifo_t;

#define A_REMOVE_STATUS_BITS(_x_) do{((xyz_in_fifo_t*)&(_x_))->blocks.first &= 0xf0fff000; ((xyz_in_fifo_t*)&(_x_))->blocks.second &= 0x00fff0ff; }while(0)

// Gyro data has no status bits to remove

typedef union __attribute__((packed))
{
	struct __attribute__((packed))
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint16_t rhall;
	} coords;

	struct __attribute__((packed))
	{
		uint32_t first;
		uint32_t second;
	} blocks;
} m_dataframe_in_fifo_t;

#define M_REMOVE_STATUS_BITS(_x_) do{((m_dataframe_in_fifo_t*)&(_x_))->blocks.first &= 0xfff8fff8; ((xyz_in_fifo_t*)&(_x_))->blocks.second &= 0xfffcfffe; }while(0)


volatile xyz_in_fifo_t latest_a[6];
volatile xyz_in_fifo_t latest_g[6];
volatile m_dataframe_in_fifo_t latest_m[6];
volatile int8_t a_temps[6];


#define AGM01_WR32 (*(volatile uint32_t*)&AGM01_SPI->TXDR)
#define AGM23_WR32 (*(volatile uint32_t*)&AGM23_SPI->TXDR)
#define AGM45_WR32 (*(volatile uint32_t*)&AGM45_SPI->TXDR)

#define AGM01_WR16 (*(volatile uint16_t*)&AGM01_SPI->TXDR)
#define AGM23_WR16 (*(volatile uint16_t*)&AGM23_SPI->TXDR)
#define AGM45_WR16 (*(volatile uint16_t*)&AGM45_SPI->TXDR)

#define AGM01_WR8  (*(volatile uint8_t*)&AGM01_SPI->TXDR)
#define AGM23_WR8  (*(volatile uint8_t*)&AGM23_SPI->TXDR)
#define AGM45_WR8  (*(volatile uint8_t*)&AGM45_SPI->TXDR)

#define AGM01_RD32 (*(volatile uint32_t*)&AGM01_SPI->RXDR)
#define AGM23_RD32 (*(volatile uint32_t*)&AGM23_SPI->RXDR)
#define AGM45_RD32 (*(volatile uint32_t*)&AGM45_SPI->RXDR)

#define AGM01_RD16 (*(volatile uint16_t*)&AGM01_SPI->RXDR)
#define AGM23_RD16 (*(volatile uint16_t*)&AGM23_SPI->RXDR)
#define AGM45_RD16 (*(volatile uint16_t*)&AGM45_SPI->RXDR)

#define AGM01_RD8 (*(volatile uint8_t*)&AGM01_SPI->RXDR)
#define AGM23_RD8 (*(volatile uint8_t*)&AGM23_SPI->RXDR)
#define AGM45_RD8 (*(volatile uint8_t*)&AGM45_SPI->RXDR)

#define AG01_READ_CMD() do{AGM01_WR32 = 0x000000bf; __DMB(); AGM01_WR16 = 0x0000; __DMB(); AGM01_WR8 = 0x00; __DMB();}while(0)
#define AG23_READ_CMD() do{AGM23_WR32 = 0x000000bf; __DMB(); AGM23_WR16 = 0x0000; __DMB(); AGM23_WR8 = 0x00; __DMB();}while(0)
#define AG45_READ_CMD() do{AGM45_WR32 = 0x000000bf; __DMB(); AGM45_WR16 = 0x0000; __DMB(); AGM45_WR8 = 0x00; __DMB();}while(0)

#define M01_READ_CMD_PART1() do{AGM01_WR32 = 0x000000c2; __DMB(); AGM01_WR32 = 0x0000; __DMB();}while(0)
#define M23_READ_CMD_PART1() do{AGM23_WR32 = 0x000000c2; __DMB(); AGM23_WR32 = 0x0000; __DMB();}while(0)
#define M45_READ_CMD_PART1() do{AGM45_WR32 = 0x000000c2; __DMB(); AGM45_WR32 = 0x0000; __DMB();}while(0)

#define M01_READ_CMD_PART2() do{AGM01_WR8 = 0x00; __DMB();}while(0)
#define M23_READ_CMD_PART2() do{AGM23_WR8 = 0x00; __DMB();}while(0)
#define M45_READ_CMD_PART2() do{AGM45_WR8 = 0x00; __DMB();}while(0)

static inline void set_timer(uint16_t tenth_us)
{
	TIM3->ARR = tenth_us-1;
	TIM3->CR1 = 1UL<<3 /* one pulse mode */ | 1UL<<2 /*only over/underflow generates update int*/ | 1UL /* enable*/;
}


// wait times in tenths of microseconds

// Status read is a 2-byte operation - 2.56 us. Leave some margin for SPI starting delay and bus delays
#define STATUS_READ_WAIT_TIME 50

// Data read is a 7-byte operation - 8.96 us. Leave some margin for SPI starting delay and bus delays
#define DATA_READ_WAIT_TIME 110

// Trigger register write is a 2-byte operation, but as a write operation, 2us extra is required per datasheet
#define TRIG_REG_WRITE_WAIT_TIME 70

// Desel waiting time defines how long the "inactive" pulse of nCS is, which is very relevant between reading status and
// then data from the same sensor, to signify separate transfers. This parameter is unspecified(!) in the datasheet. Assuming
// 1us should be generous for simple read operations.

#define DESEL_WAIT_TIME 10

// Magnetometer data read requires two steps, since it's a 9-byte operation and the SPI fifo is 8 bytes.
// This still makes sense over DMA, because the first reply byte is ditched anyway, and the actual data fits nicely in 8 bytes once.
// Part 1 is 8 bytes, part 2 is 1 byte more.
// Part 2 is supposed to run when part1 has run both TX and RX for at least 1 byte.
// 1 byte should fit in 1.28 us. Add some leeway for SPI starting delay and bus delays.
// No horrors happen if too long - but SPI clock generation stops temporarily. Sensors are OK with this.
// If you want to keep this from happening, eep well below 8*1.28us = 10 us.
// So between (1.28us + leeway) and (10 us - leeway), we choose 3 us.
// (2 us has been tested working, and 1 us has been tested non-working, as expected)

#define M_READ_PART1_WAIT_TIME 30

// With 1 byte out of equation, the remaining data read is a 8-byte operation - 10.24 us.
#define M_READ_PART2_WAIT_TIME 120

// Finally, this parameter defines how quickly the whole thing runs.
// You should never encounter more than 1 item in any FIFO. If this happens, reduce the wait time to run faster.
// Running faster reduces readout latencies. Don't go too low mostly because the fsm inthandler starts to eat up
// some considerable CPU time.
#define FINAL_WAIT_TIME 5000
#define FINAL_WAIT_TIME_WITH_TEMP_AND_M_READOUTS (FINAL_WAIT_TIME - (TRIG_REG_WRITE_WAIT_TIME*2 + \
		STATUS_READ_WAIT_TIME*2 + M_READ_PART1_WAIT_TIME*2 + M_READ_PART2_WAIT_TIME*2 + 7*3)) /*Estimate of 0.3us per code execution per state*/

//#if ((FINAL_WAIT_TIME < 3000) || (FINAL_WAIT_TIME_WITH_TEMP_AND_M_READOUTS < 2000))
//#error "You really shouldn't be doing this. The ISR will hog up the CPU."
//#endif

#define REPOLL_WAIT_TIME 5000

// 1.28us per byte -> add 10% extra: 1.4us per byte. Then, add 5 us extra to the total.
#define A_DMA_WAIT_TIME ((3*6+1)*14 + 500)
#define G_DMA_WAIT_TIME ((5*6+1)*14 + 500)

volatile int a_status_cnt[6];
volatile int a_nonstatus_cnt[6];
volatile int g_status_cnt[6];
volatile int g_nonstatus_cnt[6];

volatile int trig_m;

volatile int cur_state;

/*
	Both gyro and accelerometer in BMX055 are free-running, on internal, non-precision oscillator.
	The data generation cannot be synced.

	We may want to have equitemporal (is that a word? Trying to sound fancy) -- same-time samples
	of all six devices. 

	The only way to achieve something closer to that, is to minimize time difference by using
	higher-than-necessary sampling rate, then average the samples together to form the desired output rate.

	Gyro sampling rate = 1000Hz (internal filter BW = 116Hz)
	Xcel sampling rate = 1000Hz (internal filter BW = 500Hz)

	Values are accumulated in the BMX055 FIFOs until there is at least:
	* 4 gyro samples
	* 4 xcel samples

	It's acceptable to have more (7 gyro, 7 xcel). More than that is considered a timing error, meaning
	significant data rate error between the devices.

	amount to read: Up to 7: fifo_status & 0b00000111 (0x07)
	mask:   0b00000100 (0x04) must equal
	        0b00000100 (0x04) --> (4,5,6,or 7 data)

*/

#define REQUIRED_FIRST_MASK  0x0404040404040404ULL
#define REQUIRED_FIRST       0x0404040404040404ULL

#define REQUIRED_SECOND_MASK 0x04040404UL
#define REQUIRED_SECOND      0x04040404UL

#define ERR_FIRST_MASK  0xf8f8f8f8f8f8f8f8ULL
#define ERR_SECOND_MASK 0xf8f8f8f8UL


//#define READ_LEN_FIRST_MASK  0x0505030303030303ULL
//#define READ_LEN_SECOND_MASK 0x05050505UL
#define READ_LEN_FIRST_MASK  0x0707070707070707ULL
#define READ_LEN_SECOND_MASK 0x07070707UL

static void ag_do_fifo_lvl_read() __attribute__((section(".text_itcm")));
static void ag_do_fifo_lvl_read()
{
//	delay_us(4);

	AGM01_WR16 = 0x008e;
	AGM23_WR16 = 0x008e;
	AGM45_WR16 = 0x008e;
}

// Mask bit7 away to get the actual count.
static inline uint8_t ag01_read_fifo_lvl()
{
	uint16_t status = AGM01_RD16;
	return status>>8;
}

static inline uint8_t ag23_read_fifo_lvl()
{
	uint16_t status = AGM23_RD16;
	return status>>8;
}

static inline uint8_t ag45_read_fifo_lvl()
{
	uint16_t status = AGM45_RD16;
	return status>>8;
}

typedef struct __attribute__((packed))
{
	int16_t x;
	int16_t y;
	int16_t z;
} xyz_i16_packed_t;


// n field:
// Works as a "read fifo" command (0x3f register address) on the TX packet
// Comes inevitably back on the RX packet, containing random, do not care data
// Is reused as a "number of samples" field, after DMA is done :-).
typedef struct __attribute__((packed))
{
	uint8_t n;
	xyz_i16_packed_t xyz[8]; //5
} g_dma_packet_t;

typedef struct __attribute__((packed))
{
	uint8_t n;
	xyz_i16_packed_t xyz[8]; //3
} a_dma_packet_t;

volatile a_dma_packet_t a_packet0 __attribute__((aligned(4))) __attribute__((section(".sram3_bss")));
volatile g_dma_packet_t g_packet0 __attribute__((aligned(4))) __attribute__((section(".sram3_bss")));
volatile a_dma_packet_t a_packet1 __attribute__((aligned(4))) __attribute__((section(".sram3_bss")));
volatile g_dma_packet_t g_packet1 __attribute__((aligned(4))) __attribute__((section(".sram3_bss")));
volatile a_dma_packet_t a_packet2 __attribute__((aligned(4))) __attribute__((section(".sram4_bss")));
volatile g_dma_packet_t g_packet2 __attribute__((aligned(4))) __attribute__((section(".sram4_bss")));
volatile a_dma_packet_t a_packet3 __attribute__((aligned(4))) __attribute__((section(".sram4_bss")));
volatile g_dma_packet_t g_packet3 __attribute__((aligned(4))) __attribute__((section(".sram4_bss")));
volatile a_dma_packet_t a_packet4 __attribute__((aligned(4))) __attribute__((section(".sram3_bss")));
volatile g_dma_packet_t g_packet4 __attribute__((aligned(4))) __attribute__((section(".sram3_bss")));
volatile a_dma_packet_t a_packet5 __attribute__((aligned(4))) __attribute__((section(".sram3_bss")));
volatile g_dma_packet_t g_packet5 __attribute__((aligned(4))) __attribute__((section(".sram3_bss")));


static const g_dma_packet_t dma_command __attribute__((aligned(4))) __attribute__((section(".sram3_data"))) = {0xbf, {{0}}};
static const g_dma_packet_t bdma_command __attribute__((aligned(4))) __attribute__((section(".sram4_data"))) = {0xbf, {{0}}};


#define TX_DMA_CONFIG  (0b01UL<<16 /*med prio*/ | 0b10UL<<13 /*32-bit mem*/ | 0b10UL<<11 /*32-bit periph*/ | 1UL<<10 /*mem increment*/ | 0b01UL<<6 /*mem-to-periph*/ | 0b110 /*err interrupts*/)
#define RX_DMA_CONFIG  (0b01UL<<16 /*med prio*/ | 0b10UL<<13 /*32-bit mem*/ | 0b10UL<<11 /*32-bit periph*/ | 1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/ | 0b110 /*err interrupts*/)

#define TX_BDMA_CONFIG (0b01UL<<12 /*med prio*/ | 0b10UL<<10 /*32-bit mem*/ | 0b10UL<<8 /*32-bit periph*/ | 1UL<<7 /*mem increment*/ | 1UL<<4 /*mem-to-periph*/ | 0b1000 /*err int*/)
#define RX_BDMA_CONFIG (0b01UL<<12 /*med prio*/ | 0b10UL<<10 /*32-bit mem*/ | 0b10UL<<8 /*32-bit periph*/ | 1UL<<7 /*mem increment*/ | 0UL<<4 /*periph-to-mem*/ | 0b1000 /*err int*/)

#define SPI_CR_OFF (1UL<<12 /*SSI*/)
#define SPI_CR_ON (SPI_CR_OFF | 1UL /*ena*/)

#define SPI_CFG1_NONDMA (SPI_CLKDIV_REGVAL<<28 | 0UL<<15 /*TX DMA*/ | 0UL<<14 /*RX DMA*/ | \
                        (1/*FIFO threshold*/   -1)<<5 | (8/*bits per frame*/   -1))

#define SPI_CFG1_RXDMA (SPI_CLKDIV_REGVAL<<28 | 0UL<<15 /*TX DMA*/ | 1UL<<14 /*RX DMA*/ | \
                        (4/*FIFO threshold*/   -1)<<5 | (8/*bits per frame*/   -1))

#define SPI_CFG1_RXTXDMA (SPI_CLKDIV_REGVAL<<28 | 1UL<<15 /*TX DMA*/ | 1UL<<14 /*RX DMA*/ | \
                        (4/*FIFO threshold*/   -1)<<5 | (8/*bits per frame*/   -1))


static void ag01_nondma()
{
	if(!(AGM01_SPI->SR & 1UL<<3))
	{
		uart_print_string_blocking("SPI01 NOT DONE!");
		error(15);
	}

	AGM01_SPI->CR1 = SPI_CR_OFF;
	__DSB();
	AGM01_SPI->IFCR = 0b11000; // Clear EOT and TXTF flags
	AGM01_SPI->CFG1 = SPI_CFG1_NONDMA;
	AGM01_SPI->TSIZE = 0;
	__DSB();
	AGM01_SPI->CR1 = SPI_CR_ON;
	__DSB();
	AGM01_SPI->CR1 |= 1UL<<9;
}

static void ag23_nondma()
{
	if(!(AGM23_SPI->SR & 1UL<<3))
	{
		uart_print_string_blocking("SPI23 NOT DONE!");
		error(15);
	}
	AGM23_SPI->CR1 = SPI_CR_OFF;
	__DSB();
	AGM23_SPI->IFCR = 0b11000; // Clear EOT and TXTF flags
	AGM23_SPI->CFG1 = SPI_CFG1_NONDMA;
	AGM23_SPI->TSIZE = 0;
	__DSB();
	AGM23_SPI->CR1 = SPI_CR_ON;
	__DSB();
	AGM23_SPI->CR1 |= 1UL<<9;
}

static void ag45_nondma()
{
	if(!(AGM45_SPI->SR & 1UL<<3))
	{
		uart_print_string_blocking("SPI45 NOT DONE!");
		error(15);
	}

	AGM45_SPI->CR1 = SPI_CR_OFF;
	__DSB();
	AGM45_SPI->IFCR = 0b11000; // Clear EOT and TXTF flags
	AGM45_SPI->CFG1 = SPI_CFG1_NONDMA;
	AGM45_SPI->TSIZE = 0;
	__DSB();
	AGM45_SPI->CR1 = SPI_CR_ON;
	__DSB();
	AGM45_SPI->CR1 |= 1UL<<9;
}

static void ag23_dma_start(uint8_t n_samples, void* p_packet) __attribute__((section(".text_itcm")));
static void ag23_dma_start(uint8_t n_samples, void* p_packet)
{
	if(n_samples < 1 || n_samples > 7)
		error(7);

//	delay_us(4);

	// BDMA doesn't clear the enable bit after completion, so we need to do it manually before starting the new transfer:
	AGM23_RX_DMA_STREAM->CCR = RX_BDMA_CONFIG;
	AGM23_TX_DMA_STREAM->CCR = TX_BDMA_CONFIG;

	uint16_t len = n_samples*6+1;

	AGM23_SPI->CR1 = SPI_CR_OFF;
	AGM23_SPI->CFG1 = SPI_CFG1_RXDMA;
	AGM23_SPI->TSIZE = len;

	AGM23_SPI->IFCR = 0b11000; // Clear EOT and TXTF flags

	AGM23_RX_DMA_STREAM->CM0AR = (uint32_t)p_packet;
	AGM23_RX_DMA_STREAM->CNDTR = (len+3)/4;
	BDMA_CLEAR_INTFLAGS(AGM23_RX_DMA, AGM23_RX_DMA_STREAM_NUM);
	__DSB();
	AGM23_RX_DMA_STREAM->CCR = RX_BDMA_CONFIG | 1UL;
	__DSB();

	AGM23_TX_DMA_STREAM->CNDTR = (len+3)/4;
	BDMA_CLEAR_INTFLAGS(AGM23_TX_DMA, AGM23_TX_DMA_STREAM_NUM);
	AGM23_TX_DMA_STREAM->CCR = TX_BDMA_CONFIG | 1UL;

	__DSB();
	AGM23_SPI->CFG1 = SPI_CFG1_RXTXDMA;

	__DSB();
	AGM23_SPI->CR1 = SPI_CR_ON;
	__DSB();
	AGM23_SPI->CR1 = SPI_CR_ON | (1UL<<9);
}


static void ag01_dma_start(uint8_t n_samples, void* p_packet) __attribute__((section(".text_itcm")));
static void ag01_dma_start(uint8_t n_samples, void* p_packet)
{
	if(n_samples < 1 || n_samples > 7)
		error(7);

//	delay_us(4);

	uint16_t len = n_samples*6+1;

	AGM01_SPI->CR1 = SPI_CR_OFF;
	__DSB();
	AGM01_SPI->CFG1 = SPI_CFG1_RXDMA;
	AGM01_SPI->TSIZE = len;

	// STM32 SPI TOTAL BROKENNESS CRAP TRAP WARNING:
	// SPI just is broken and doesn't start to do anything (neither gives errors; just doensn't start)
	// if you don't manually clear these informative flag bits:
	AGM01_SPI->IFCR = 0b11000; // Clear EOT and TXTF flags

	AGM01_RX_DMA_STREAM->M0AR = (uint32_t)p_packet;
	AGM01_RX_DMA_STREAM->NDTR = (len+3)/4;
	DMA_CLEAR_INTFLAGS(AGM01_RX_DMA, AGM01_RX_DMA_STREAM_NUM);
	AGM01_RX_DMA_STREAM->CR = RX_DMA_CONFIG | 1UL;

	AGM01_TX_DMA_STREAM->NDTR = (len+3)/4;
	DMA_CLEAR_INTFLAGS(AGM01_TX_DMA, AGM01_TX_DMA_STREAM_NUM);
	AGM01_TX_DMA_STREAM->CR = TX_DMA_CONFIG | 1UL;

	__DSB();
	AGM01_SPI->CFG1 = SPI_CFG1_RXTXDMA;

	__DSB();
	AGM01_SPI->CR1 = SPI_CR_ON;
	__DSB();
	AGM01_SPI->CR1 = SPI_CR_ON | (1UL<<9);
	__DSB();
}

static void ag45_dma_start(uint8_t n_samples, void* p_packet) __attribute__((section(".text_itcm")));
static void ag45_dma_start(uint8_t n_samples, void* p_packet)
{
	if(n_samples < 1 || n_samples > 7)
		error(7);

//	delay_us(4);

	uint16_t len = n_samples*6+1;

	AGM45_SPI->CR1 = SPI_CR_OFF;
	__DSB();
	AGM45_SPI->CFG1 = SPI_CFG1_RXDMA;
	AGM45_SPI->TSIZE = len;

	AGM45_SPI->IFCR = 0b11000; // Clear EOT and TXTF flags

	AGM45_RX_DMA_STREAM->M0AR = (uint32_t)p_packet;
	AGM45_RX_DMA_STREAM->NDTR = (len+3)/4;
	DMA_CLEAR_INTFLAGS(AGM45_RX_DMA, AGM45_RX_DMA_STREAM_NUM);
	AGM45_RX_DMA_STREAM->CR = RX_DMA_CONFIG | 1UL;

	AGM45_TX_DMA_STREAM->NDTR = (len+3)/4;
	DMA_CLEAR_INTFLAGS(AGM45_TX_DMA, AGM45_TX_DMA_STREAM_NUM);
	AGM45_TX_DMA_STREAM->CR = TX_DMA_CONFIG | 1UL;

	__DSB();
	AGM45_SPI->CFG1 = SPI_CFG1_RXTXDMA;

	__DSB();
	AGM45_SPI->CR1 = SPI_CR_ON;
	__DSB();
	AGM45_SPI->CR1 = SPI_CR_ON | (1UL<<9);
	__DSB();

}

volatile int data_ok;

volatile static union
{
	struct __attribute__((packed))
	{
		uint8_t a[6];
		uint8_t g[6];
	} lvls;

	struct __attribute__((packed))
	{
		uint64_t first;
		uint32_t second;
	} quick;
} fifo;

#define DLEN 8000
uint8_t dbg[DLEN][12];
int curd;

void imu_fsm_inthandler() __attribute__((section(".text_itcm")));
void imu_fsm_inthandler()
{
//	LED_ON();
	TIM3->SR = 0UL;

	switch(cur_state)
	{
		case 0: // Trig by timer: Assert A024, start reading FIFO fill level
		{
			fifo.quick.first = fifo.quick.second = 0;

			SEL_A024();
			ag_do_fifo_lvl_read();
			set_timer(STATUS_READ_WAIT_TIME);
			cur_state++;
		} break;

		case 1: // Trig by timer: Deassert A024, Read A024 level, assert A135, start reading FIFO fill level
		{
			DESEL_A024();
//			delay_us(1);
			fifo.lvls.a[0] = ag01_read_fifo_lvl();
			fifo.lvls.a[2] = ag23_read_fifo_lvl();
			fifo.lvls.a[4] = ag45_read_fifo_lvl();
			SEL_A135();
			ag_do_fifo_lvl_read();
			set_timer(STATUS_READ_WAIT_TIME);
			cur_state++;
		} break;

		case 2: // Trig by timer: Deassert A135, Read A135 level, assert G024, start reading FIFO fill level
		{
			DESEL_A135();
//			delay_us(1);
			fifo.lvls.a[1] = ag01_read_fifo_lvl();
			fifo.lvls.a[3] = ag23_read_fifo_lvl();
			fifo.lvls.a[5] = ag45_read_fifo_lvl();
			SEL_G024();
			ag_do_fifo_lvl_read();
			set_timer(STATUS_READ_WAIT_TIME);
			cur_state++;			
		} break;

		case 3: // Trig by timer: Deassert G024, Read G024 level, assert G135, start reading FIFO fill level
		{
			DESEL_G024();
//			delay_us(1);
			fifo.lvls.g[0] = ag01_read_fifo_lvl();
			fifo.lvls.g[2] = ag23_read_fifo_lvl();
			fifo.lvls.g[4] = ag45_read_fifo_lvl();
			SEL_G135();
			ag_do_fifo_lvl_read();
			set_timer(STATUS_READ_WAIT_TIME);
			cur_state++;			
		} break;

		case 4: // Trig by timer: Deassert G135, Read G135 level, wait to start over? or Assert & DMA read A024?
		{
			DESEL_G135();
			fifo.lvls.g[1] = ag01_read_fifo_lvl();
			fifo.lvls.g[3] = ag23_read_fifo_lvl();
			fifo.lvls.g[5] = ag45_read_fifo_lvl();
			__DSB();

			dbg[curd][0] = fifo.lvls.a[0];
			dbg[curd][1] = fifo.lvls.a[1];
			dbg[curd][2] = fifo.lvls.a[2];
			dbg[curd][3] = fifo.lvls.a[3];
			dbg[curd][4] = fifo.lvls.a[4];
			dbg[curd][5] = fifo.lvls.a[5];
			dbg[curd][6] = fifo.lvls.g[0];
			dbg[curd][7] = fifo.lvls.g[1];
			dbg[curd][8] = fifo.lvls.g[2];
			dbg[curd][9] = fifo.lvls.g[3];
			dbg[curd][10] = fifo.lvls.g[4];
			dbg[curd][11] = fifo.lvls.g[5];

			if(
			    (fifo.quick.first & ERR_FIRST_MASK) || 
			    (fifo.quick.second & ERR_SECOND_MASK))

/*
				fifo.lvls.a[0] > 7 ||
				fifo.lvls.a[1] > 7 ||
				fifo.lvls.a[2] > 7 ||
				fifo.lvls.a[3] > 7 ||
				fifo.lvls.a[4] > 7 ||
				fifo.lvls.a[5] > 7 ||
				fifo.lvls.g[0] > 7 ||
				fifo.lvls.g[1] > 7 ||
				fifo.lvls.g[2] > 7 ||
				fifo.lvls.g[3] > 7 ||
				fifo.lvls.g[4] > 7 ||
				fifo.lvls.g[5] > 7)
*/
			{
				uart_print_string_blocking("\r\nSTOPPED: too many samples\r\n");

				for(int i=0; i<6; i++)
				{
					uart_print_string_blocking("a: "); o_utoa16(fifo.lvls.a[i], printbuf); uart_print_string_blocking(printbuf); 
					if(fifo.lvls.a[i] > 3) uart_print_string_blocking(" !");
					uart_print_string_blocking("\r\n");
				}
				for(int i=0; i<6; i++)
				{
					uart_print_string_blocking("g: "); o_utoa16(fifo.lvls.g[i], printbuf); uart_print_string_blocking(printbuf);
					if(fifo.lvls.g[i] > 5) uart_print_string_blocking(" !");
					uart_print_string_blocking("\r\n");
				}

				uart_print_string_blocking("TRACE:\r\n");

				for(int i=0; i<=curd; i++)
				{
					int weird = 0;
					for(int j=0; j<12; j++)
					{
						if(i>0 && (dbg[i][j] > dbg[i-1][j]+1)) weird = 1;
						o_utoa16(dbg[i][j], printbuf); uart_print_string_blocking(printbuf);
//						if(j==5)
						uart_print_string_blocking(",");
					}
					if(weird)
						uart_print_string_blocking("!!!!!");
					uart_print_string_blocking("\r\n");
				}
				uart_print_string_blocking("\r\nEND\r\n");

				error(13);
			}

			curd++;
			if(curd >= DLEN) curd = 0;



			if(
			    (fifo.quick.first & REQUIRED_FIRST_MASK) == REQUIRED_FIRST  && 
			    (fifo.quick.second & REQUIRED_SECOND_MASK) == REQUIRED_SECOND)
/*				fifo.lvls.a[0] >= 4 &&
				fifo.lvls.a[1] >= 4 &&
				fifo.lvls.a[2] >= 4 &&
				fifo.lvls.a[3] >= 4 &&
				fifo.lvls.a[4] >= 4 &&
				fifo.lvls.a[5] >= 4 &&
				fifo.lvls.g[0] >= 4 &&
				fifo.lvls.g[1] >= 4 &&
				fifo.lvls.g[2] >= 4 &&
				fifo.lvls.g[3] >= 4 &&
				fifo.lvls.g[4] >= 4 &&
				fifo.lvls.g[5] >= 4)*/

			{
				fifo.quick.first &= READ_LEN_FIRST_MASK;
				fifo.quick.second &= READ_LEN_SECOND_MASK;

				data_ok = 0;
				// We don't need the actual FIFO level anymore, but we need the "read length" for now on,
				// in two places (configuring the DMA channel, and later, after DMA is finished, to overwrite
				// the n field). Let's overwrite fifo_lvls in a quick operation:

				SEL_A024();
				ag01_dma_start(fifo.lvls.a[0], &a_packet0);
				ag23_dma_start(fifo.lvls.a[2], &a_packet2);
				ag45_dma_start(fifo.lvls.a[4], &a_packet4);
				set_timer(A_DMA_WAIT_TIME);
				cur_state++;
			}
			else
			{				
				cur_state = 0;
				set_timer(REPOLL_WAIT_TIME);
			}
		} break;

		case 5: // Trig by DMA completion: Deassert A024, Assert & DMA read A135
		{
			DESEL_A024();
			a_packet0.n = fifo.lvls.a[0];
			a_packet2.n = fifo.lvls.a[2];
			a_packet4.n = fifo.lvls.a[4];
			SEL_A135();
			ag01_dma_start(fifo.lvls.a[1], &a_packet1);
			ag23_dma_start(fifo.lvls.a[3], &a_packet3);
			ag45_dma_start(fifo.lvls.a[5], &a_packet5);
			cur_state++;
			set_timer(A_DMA_WAIT_TIME);
		} break;

		case 6: // Trig by DMA completion: Deassert A135, Assert & DMA read G024
		{
			DESEL_A135();
			a_packet1.n = fifo.lvls.a[1];
			a_packet3.n = fifo.lvls.a[3];
			a_packet5.n = fifo.lvls.a[5];
			SEL_G024();
			ag01_dma_start(fifo.lvls.g[0], &g_packet0);
			ag23_dma_start(fifo.lvls.g[2], &g_packet2);
			ag45_dma_start(fifo.lvls.g[4], &g_packet4);
			set_timer(G_DMA_WAIT_TIME);
			cur_state++;

		} break;

		case 7: // Trig by DMA completion: Deassert G024, Assert & DMA read G135
		{
			DESEL_G024();

			g_packet0.n = fifo.lvls.g[0];
			g_packet2.n = fifo.lvls.g[2];
			g_packet4.n = fifo.lvls.g[4];
			SEL_G135();
			ag01_dma_start(fifo.lvls.g[1], &g_packet1);
			ag23_dma_start(fifo.lvls.g[3], &g_packet3);
			ag45_dma_start(fifo.lvls.g[5], &g_packet5);
			set_timer(G_DMA_WAIT_TIME);
			cur_state++;
		} break;

		case 8: // Trig by DMA completion: Deassert G135, wait to start over? or Assert & Trig M024?
		{
			DESEL_G135();

			ag01_nondma();
			ag23_nondma();
			ag45_nondma();
			g_packet1.n = fifo.lvls.g[1];
			g_packet3.n = fifo.lvls.g[3];
			g_packet5.n = fifo.lvls.g[5];
			cur_state = 0;
			data_ok = 1;
			kakka_cnt++;
			set_timer(FINAL_WAIT_TIME);
		} break;

		case 9: // Trig by timer: Deassert M024, Assert & Trig M135
		{

		} break;

		case 10: // Trig by timer: Deassert M135, Assert & DMA read M024
		{

		} break;

		case 11: // Trig by DMA completion: Deassert M024, Assert & DMA read M135
		{

		} break;

		case 12: // Trig by DMA completion: Deassert M135, Assert A024 & start temperature read
		{

		} break;

		case 13: // Trig by timer: Deassert A024, read temp, assert A135 & start temp read
		{

		} break;

		case 14: // Trig by timer: Deassert A135, read temp, start final wait
		{

		} break;

		case 15: // Trig by timer:
		{

		} break;

		case 16: // Trig by timer:
		{

		} break;

#if 0
		case START_A024_STATUS_READ:
		{
			SEL_A024();
			__DSB();
			AGM01_WR16 = 0x008e;
			AGM23_WR16 = 0x008e;
			AGM45_WR16 = 0x008e;
			set_timer(STATUS_READ_WAIT_TIME);
			cur_state++;
		} break;

		case READ_A024_STATUS__START_DATA_READ:
		{
			uint16_t status0 = AGM01_RD16;
			uint16_t status2 = AGM23_RD16;
			uint16_t status4 = AGM45_RD16;

			SEL_A024();
			__DSB();
			reading0 = reading1 = reading2 = 0;
			if(status0 & STATUS_FILL_LEVEL_MASK)
			{
				a_status_cnt[0]++;
				reading0 = 1;
				AG01_READ_CMD();
			}
			else
				a_nonstatus_cnt[0]++;

			if(status2 & STATUS_FILL_LEVEL_MASK)
			{
				a_status_cnt[2]++;
				reading1 = 1;
				AG23_READ_CMD();
			}
			else
				a_nonstatus_cnt[2]++;

			if(status4 & STATUS_FILL_LEVEL_MASK)
			{
				a_status_cnt[4]++;
				reading2 = 1;
				AG45_READ_CMD();
			}
			else
				a_nonstatus_cnt[4]++;

			set_timer(DATA_READ_WAIT_TIME);
			cur_state++;
		} break;

		case READ_A024_DATA__START_A135_STATUS_READ:
		{
			if(reading0)
			{
				latest_a[0].blocks.first  = AGM01_RD32; __DMB();
				latest_a[0].blocks.second = AGM01_RD32;
				A_REMOVE_STATUS_BITS(latest_a[0]);
			}
			if(reading1)
			{
				latest_a[2].blocks.first  = AGM23_RD32; __DMB();
				latest_a[2].blocks.second = AGM23_RD32;
				A_REMOVE_STATUS_BITS(latest_a[2]);
			}
			if(reading2)
			{
				latest_a[4].blocks.first  = AGM45_RD32; __DMB();
				latest_a[4].blocks.second = AGM45_RD32;
				A_REMOVE_STATUS_BITS(latest_a[4]);
			}

			SEL_A135();
			__DSB();
			AGM01_WR16 = 0x008e;
			AGM23_WR16 = 0x008e;
			AGM45_WR16 = 0x008e;
			set_timer(STATUS_READ_WAIT_TIME);
			cur_state++;
		} break;

		case READ_A135_STATUS__START_DATA_READ:
		{
			uint16_t status1 = AGM01_RD16;
			uint16_t status3 = AGM23_RD16;
			uint16_t status5 = AGM45_RD16;

			SEL_A135();
			__DSB();
			reading0 = reading1 = reading2 = 0;
			if(status1 & STATUS_FILL_LEVEL_MASK)
			{
				a_status_cnt[1]++;
				reading0 = 1;
				AG01_READ_CMD();
			}
			else
				a_nonstatus_cnt[1]++;

			if(status3 & STATUS_FILL_LEVEL_MASK)
			{
				a_status_cnt[3]++;
				reading1 = 1;
				AG23_READ_CMD();
			}
			else
				a_nonstatus_cnt[3]++;

			if(status5 & STATUS_FILL_LEVEL_MASK)
			{
				a_status_cnt[5]++;
				reading2 = 1;
				AG45_READ_CMD();
			}
			else
				a_nonstatus_cnt[5]++;

			set_timer(DATA_READ_WAIT_TIME);
			cur_state++;
		} break;

		case READ_A135_DATA__START_G024_STATUS_READ:
		{
			if(reading0)
			{
				latest_a[1].blocks.first  = AGM01_RD32; __DMB();
				latest_a[1].blocks.second = AGM01_RD32;
				A_REMOVE_STATUS_BITS(latest_a[1]);
			}
			if(reading1)
			{
				latest_a[3].blocks.first  = AGM23_RD32; __DMB();
				latest_a[3].blocks.second = AGM23_RD32;
				A_REMOVE_STATUS_BITS(latest_a[3]);
			}
			if(reading2)
			{
				latest_a[5].blocks.first  = AGM45_RD32; __DMB();
				latest_a[5].blocks.second = AGM45_RD32;
				A_REMOVE_STATUS_BITS(latest_a[5]);
			}

			SEL_G024();
			__DSB();
			AGM01_WR16 = 0x008e;
			AGM23_WR16 = 0x008e;
			AGM45_WR16 = 0x008e;
			set_timer(STATUS_READ_WAIT_TIME);
			cur_state++;

		} break;

		case READ_G024_STATUS__START_DATA_READ:
		{
			uint16_t status0 = AGM01_RD16;
			uint16_t status2 = AGM23_RD16;
			uint16_t status4 = AGM45_RD16;

			SEL_G024();
			__DSB();
			reading0 = reading1 = reading2 = 0;
			if(status0 & STATUS_FILL_LEVEL_MASK)
			{
				g_status_cnt[0]++;
				reading0 = 1;
				AG01_READ_CMD();
			}
			else
				g_nonstatus_cnt[0]++;

			if(status2 & STATUS_FILL_LEVEL_MASK)
			{
				g_status_cnt[2]++;
				reading1 = 1;
				AG23_READ_CMD();
			}
			else
				g_nonstatus_cnt[2]++;

			if(status4 & STATUS_FILL_LEVEL_MASK)
			{
				g_status_cnt[4]++;
				reading2 = 1;
				AG45_READ_CMD();
			}
			else
				g_nonstatus_cnt[4]++;

			set_timer(DATA_READ_WAIT_TIME);
			cur_state++;
		} break;

		case READ_G024_DATA__START_G135_STATUS_READ:
		{
			if(reading0)
			{
				latest_g[0].blocks.first  = AGM01_RD32; __DMB();
				latest_g[0].blocks.second = AGM01_RD32;
			}
			if(reading1)
			{
				latest_g[2].blocks.first  = AGM23_RD32; __DMB();
				latest_g[2].blocks.second = AGM23_RD32;
			}
			if(reading2)
			{
				latest_g[4].blocks.first  = AGM45_RD32; __DMB();
				latest_g[4].blocks.second = AGM45_RD32;
			}

			SEL_G135();
			__DSB();
			AGM01_WR16 = 0x008e;
			AGM23_WR16 = 0x008e;
			AGM45_WR16 = 0x008e;
			set_timer(STATUS_READ_WAIT_TIME);
			cur_state++;

		} break;

		case READ_G135_STATUS__START_DATA_READ:
		{
			uint16_t status1 = AGM01_RD16;
			uint16_t status3 = AGM23_RD16;
			uint16_t status5 = AGM45_RD16;

			SEL_G135();
			__DSB();
			reading0 = reading1 = reading2 = 0;
			if(status1 & STATUS_FILL_LEVEL_MASK)
			{
				g_status_cnt[1]++;
				reading0 = 1;
				AG01_READ_CMD();
			}
			else
				g_nonstatus_cnt[1]++;

			if(status3 & STATUS_FILL_LEVEL_MASK)
			{
				g_status_cnt[3]++;
				reading1 = 1;
				AG23_READ_CMD();
			}
			else
				g_nonstatus_cnt[3]++;

			if(status5 & STATUS_FILL_LEVEL_MASK)
			{
				g_status_cnt[5]++;
				reading2 = 1;
				AG45_READ_CMD();
			}
			else
				g_nonstatus_cnt[5]++;

			set_timer(DATA_READ_WAIT_TIME);
			cur_state++;

		} break;

		case READ_G135_DATA__TRIG_M024__OR__START_WAIT:
		{
			if(reading0)
			{
				latest_g[1].blocks.first  = AGM01_RD32; __DMB();
				latest_g[1].blocks.second = AGM01_RD32;
			}
			if(reading1)
			{
				latest_g[3].blocks.first  = AGM23_RD32; __DMB();
				latest_g[3].blocks.second = AGM23_RD32;
			}
			if(reading2)
			{
				latest_g[5].blocks.first  = AGM45_RD32; __DMB();
				latest_g[5].blocks.second = AGM45_RD32;
			}

			if(trig_m)
			{
				trig_m = 0;
				SEL_M024();
				__DSB();
				AGM01_WR16 = WR_REG(0x4c, M_OP_FORCED);
				AGM23_WR16 = WR_REG(0x4c, M_OP_FORCED);
				AGM45_WR16 = WR_REG(0x4c, M_OP_FORCED);
				set_timer(TRIG_REG_WRITE_WAIT_TIME);
				cur_state++;
			}
			else
			{
				set_timer(FINAL_WAIT_TIME);
				cur_state = START_A024_STATUS_READ;
			}

		} break;

		case TRIG_M135:
		{
			SEL_M135();
			__DSB();
			AGM01_WR16 = WR_REG(0x4c, M_OP_FORCED);
			AGM23_WR16 = WR_REG(0x4c, M_OP_FORCED);
			AGM45_WR16 = WR_REG(0x4c, M_OP_FORCED);
			set_timer(TRIG_REG_WRITE_WAIT_TIME);
			cur_state++;

		} break;

		case START_A024_TEMP_READ:
		{
			// empty the RX fifo for both TRIG operations at once:
			AGM01_RD32;
			AGM23_RD32;
			AGM45_RD32;

			SEL_A024();
			__DSB();
			AGM01_WR16 = 0x0088;
			AGM23_WR16 = 0x0088;
			AGM45_WR16 = 0x0088;
			set_timer(STATUS_READ_WAIT_TIME);
			cur_state++;

		} break;

		case READ_A024_TEMP__START_A135_TEMP_READ:
		{
			a_temps[0] = ((int8_t)(AGM01_RD16>>8))+23;
			a_temps[2] = ((int8_t)(AGM23_RD16>>8))+23;
			a_temps[4] = ((int8_t)(AGM45_RD16>>8))+23;
			SEL_A135();
			__DSB();
			AGM01_WR16 = 0x0088;
			AGM23_WR16 = 0x0088;
			AGM45_WR16 = 0x0088;
			set_timer(STATUS_READ_WAIT_TIME);
			cur_state++;

		} break;

		case READ_A135_TEMP__START_M024_DATA_READ:
		{
			a_temps[1] = ((int8_t)(AGM01_RD16>>8))+23;
			a_temps[3] = ((int8_t)(AGM23_RD16>>8))+23;
			a_temps[5] = ((int8_t)(AGM45_RD16>>8))+23;

			SEL_M024();
			__DSB();
			M01_READ_CMD_PART1();
			M23_READ_CMD_PART1();
			M45_READ_CMD_PART1();
			set_timer(M_READ_PART1_WAIT_TIME);
			cur_state++;
		} break;

		case CONTINUE_M024_DATA_READ:
		{
			// SPI FIFO is 8 bytes long.
			// Transaction length is 9 bytes, first of which is dummy, to be thrown away
			// Before the FIFO is full, we read out this dummy, and get the 8-byte payload
			// nicely in our 8-byte FIFO. At the same time, there is now space in the TX fifo
			// as well to fit the last byte of the read CMD.
			AGM01_RD8;
			AGM23_RD8;
			AGM45_RD8;
			__DSB();
			// chip selects are already active here
			M01_READ_CMD_PART2();
			M23_READ_CMD_PART2();
			M45_READ_CMD_PART2();
			set_timer(M_READ_PART2_WAIT_TIME);
			cur_state++;
		} break;

		case READ_M024_DATA__START_M135_DATA_READ:
		{
			latest_m[0].blocks.first  = AGM01_RD32; __DMB();
			latest_m[0].blocks.second = AGM01_RD32;
			__DMB();
			M_REMOVE_STATUS_BITS(latest_m[0]);

			latest_m[2].blocks.first  = AGM23_RD32; __DMB();
			latest_m[2].blocks.second = AGM23_RD32;
			__DMB();
			M_REMOVE_STATUS_BITS(latest_m[2]);

			latest_m[4].blocks.first  = AGM45_RD32; __DMB();
			latest_m[4].blocks.second = AGM45_RD32;
			__DMB();
			M_REMOVE_STATUS_BITS(latest_m[4]);

			SEL_M135();
			__DSB();
			M01_READ_CMD_PART1();
			M23_READ_CMD_PART1();
			M45_READ_CMD_PART1();
			set_timer(M_READ_PART1_WAIT_TIME);
			cur_state++;
		} break;

		case CONTINUE_M135_DATA_READ:
		{
			AGM01_RD8;
			AGM23_RD8;
			AGM45_RD8;
			__DSB();
			// chip selects are already active here
			M01_READ_CMD_PART2();
			M23_READ_CMD_PART2();
			M45_READ_CMD_PART2();
			set_timer(M_READ_PART2_WAIT_TIME);
			cur_state++;
		} break;

		case READ_M135_DATA__START_WAIT:
		{
			latest_m[1].blocks.first  = AGM01_RD32; __DMB();
			latest_m[1].blocks.second = AGM01_RD32;
			__DMB();
			M_REMOVE_STATUS_BITS(latest_m[1]);

			latest_m[3].blocks.first  = AGM23_RD32; __DMB();
			latest_m[3].blocks.second = AGM23_RD32;
			__DMB();
			M_REMOVE_STATUS_BITS(latest_m[3]);

			latest_m[5].blocks.first  = AGM45_RD32; __DMB();
			latest_m[5].blocks.second = AGM45_RD32;
			__DMB();
			M_REMOVE_STATUS_BITS(latest_m[5]);

			set_timer(FINAL_WAIT_TIME);
			cur_state = START_A024_STATUS_READ;
		} break;

		default:
		{
			DESEL_A024();
			DESEL_G024();
			DESEL_M024();
			DESEL_A135();
			DESEL_G135();
			DESEL_M135();
			set_timer(DESEL_WAIT_TIME);
			cur_state++;
		} break;
#endif
		default:
		{
			error(13);
		} break;

	}


//	LED_OFF();
	__DSB();

}


static void printings()
{
//	int ax[6], ay[6], az[6];
//	int gx[6], gy[6], gz[6];
//	int mx[6], my[6], mz[6];

	static int cnt = 0;

	a_dma_packet_t a[6];
	g_dma_packet_t g[6];

	while(!data_ok) ;
	DIS_IRQ();
	memcpy(&a[0], &a_packet0, sizeof(a[0]));
	memcpy(&a[1], &a_packet1, sizeof(a[0]));
	memcpy(&a[2], &a_packet2, sizeof(a[0]));
	memcpy(&a[3], &a_packet3, sizeof(a[0]));
	memcpy(&a[4], &a_packet4, sizeof(a[0]));
	memcpy(&a[5], &a_packet5, sizeof(a[0]));

	memcpy(&g[0], &g_packet0, sizeof(a[0]));
	memcpy(&g[1], &g_packet1, sizeof(a[0]));
	memcpy(&g[2], &g_packet2, sizeof(a[0]));
	memcpy(&g[3], &g_packet3, sizeof(a[0]));
	memcpy(&g[4], &g_packet4, sizeof(a[0]));
	memcpy(&g[5], &g_packet5, sizeof(a[0]));
	ENA_IRQ();

	uart_print_string_blocking("A# ");

	for(int i=0; i<6; i++)
	{
		o_utoa16(a[i].n, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking(" ");
//		o_utoa16(a_packets0145[i].n, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking(" ");
	}

	uart_print_string_blocking("  G# ");

	for(int i=0; i<6; i++)
	{
		o_utoa16(g[i].n, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking(" ");
//		o_utoa16(g_packets0145[i].n, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking(" ");
	}

	uart_print_string_blocking("  kakka= "); o_utoa32(kakka_cnt, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking(" ");
	uart_print_string_blocking("  kukka= "); o_utoa32(kukka_cnt, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking(" ");

	uart_print_string_blocking("\r\n");

	for(int imu=0; imu<6; imu++)
	{
		uart_print_string_blocking("A"); o_utoa16(imu, printbuf); uart_print_string_blocking(printbuf); 
		uart_print_string_blocking(": ");
		
		for(int i=0; i<a[imu].n; i++)
		{
			uart_print_string_blocking("x="); 
			o_itoa16_fixed(a[imu].xyz[i].x, printbuf); uart_print_string_blocking(printbuf); 
			uart_print_string_blocking(" y="); 
			o_itoa16_fixed(a[imu].xyz[i].y, printbuf); uart_print_string_blocking(printbuf); 
			uart_print_string_blocking(" z="); 
			o_itoa16_fixed(a[imu].xyz[i].z, printbuf); uart_print_string_blocking(printbuf);
			uart_print_string_blocking("  ");
		}
			uart_print_string_blocking("\r\n");
	}

	for(int imu=0; imu<6; imu++)
	{
		uart_print_string_blocking("G"); o_utoa16(imu, printbuf); uart_print_string_blocking(printbuf); 
		uart_print_string_blocking(": ");
		
		for(int i=0; i<g[imu].n; i++)
		{
			uart_print_string_blocking("x="); 
			o_itoa16_fixed(g[imu].xyz[i].x, printbuf); uart_print_string_blocking(printbuf); 
			uart_print_string_blocking(" y="); 
			o_itoa16_fixed(g[imu].xyz[i].y, printbuf); uart_print_string_blocking(printbuf); 
			uart_print_string_blocking(" z="); 
			o_itoa16_fixed(g[imu].xyz[i].z, printbuf); uart_print_string_blocking(printbuf);
			uart_print_string_blocking("  ");
		}
			uart_print_string_blocking("\r\n");
	}

	uart_print_string_blocking("\r\n");

/*	

		o_utoa16_fixed(as[i], printbuf); uart_print_string_blocking(printbuf); 
		uart_print_string_blocking(" / "); 
		o_utoa16_fixed(an[i], printbuf); uart_print_string_blocking(printbuf); 
		uart_print_string_blocking("  x = "); 
		o_itoa16_fixed(ax[i], printbuf); uart_print_string_blocking(printbuf); 
		uart_print_string_blocking("  y = "); 
		o_itoa16_fixed(ay[i], printbuf); uart_print_string_blocking(printbuf); 
		uart_print_string_blocking("  z = "); 
		o_itoa16_fixed(az[i], printbuf); uart_print_string_blocking(printbuf);

		uart_print_string_blocking("   G ");

		o_utoa16_fixed(gs[i], printbuf); uart_print_string_blocking(printbuf); 
		uart_print_string_blocking(" / "); 
		o_utoa16_fixed(gn[i], printbuf); uart_print_string_blocking(printbuf); 
		uart_print_string_blocking("  x = "); 
		o_itoa16_fixed(gx[i], printbuf); uart_print_string_blocking(printbuf); 
		uart_print_string_blocking("  y = "); 
		o_itoa16_fixed(gy[i], printbuf); uart_print_string_blocking(printbuf); 
		uart_print_string_blocking("  z = "); 
		o_itoa16_fixed(gz[i], printbuf); uart_print_string_blocking(printbuf);

		uart_print_string_blocking("   T=");
		o_itoa8_fixed(a_temps[i], printbuf); uart_print_string_blocking(printbuf); 



		uart_print_string_blocking("   M ");

		uart_print_string_blocking("  x = "); 
		o_itoa16_fixed(mx[i], printbuf); uart_print_string_blocking(printbuf); 
		uart_print_string_blocking("  y = "); 
		o_itoa16_fixed(my[i], printbuf); uart_print_string_blocking(printbuf); 
		uart_print_string_blocking("  z = "); 
		o_itoa16_fixed(mz[i], printbuf); uart_print_string_blocking(printbuf);

		uart_print_string_blocking("\r\n");
*/

		TIM4->CNT = 0;
		__DSB();
		TIM4->CR1 = 1UL<<3 | 1UL; // Enable
		__DSB();
		delay_ms(40);
		uint32_t time = TIM4->CNT;		
		TIM4->CR1 = 1UL<<3;
		__DSB();
		uart_print_string_blocking("CPU overhead : "); 
		char* p_joo = o_utoa32(((time-36000))*10000/36000, printbuf);
		p_joo[0] = p_joo[-1];
		p_joo[-1] = p_joo[-2];
		p_joo[-2] = '.';
		p_joo[1] = 0;

		uart_print_string_blocking(printbuf); uart_print_string_blocking(" % \r\n");


	delay_ms(200);

//				NVIC_SystemReset();
//				while(1);

}

#define XCEL_RANGE_2G  0b0011
#define XCEL_RANGE_4G  0b0101
#define XCEL_RANGE_8G  0b1000
#define XCEL_RANGE_16G 0b1100
#define XCEL_T16MS_BW31HZ  0b01010
#define XCEL_T8MS_BW63HZ   0b01011
#define XCEL_T4MS_BW125HZ  0b01100
#define XCEL_T2MS_BW250HZ  0b01101
#define XCEL_T1MS_BW500HZ  0b01110

static const uint16_t a_init_seq[] =
{
	WR_REG(0x0f, XCEL_RANGE_2G),
	WR_REG(0x10, XCEL_T1MS_BW500HZ),
	WR_REG(0x3e, 0b01<<6 /*FIFO mode*/ | 0b00 /*X,Y and Z stored*/)
};

#define NUM_ELEM(_tbl_) (sizeof(_tbl_)/sizeof(_tbl_[0]))



#define GYRO_RANGE_2000DPS  0b000
#define GYRO_RANGE_1000DPS  0b001
#define GYRO_RANGE_500DPS   0b010
#define GYRO_RANGE_250DPS   0b011
#define GYRO_RANGE_125DPS   0b100

#define GYRO_ODR100HZ_BW32HZ  0b0111
#define GYRO_ODR200HZ_BW64HZ  0b0110
#define GYRO_ODR100HZ_BW12HZ  0b0101
#define GYRO_ODR200HZ_BW23HZ  0b0100
#define GYRO_ODR400HZ_BW47HZ  0b0011
#define GYRO_ODR1000HZ_BW116HZ 0b0010
#define GYRO_ODR2000HZ_BW230HZ 0b0001
#define GYRO_ODR2000HZ_BW523HZ 0b0000




static const uint16_t g_init_seq[] =
{
	WR_REG(0x0f, GYRO_RANGE_250DPS),
	WR_REG(0x10, GYRO_ODR1000HZ_BW116HZ),
	WR_REG(0x3e, 0b01<<6 /*FIFO mode*/ | 0b00 /*X,Y and Z stored*/)
};


#define M_ODR_2HZ  (0b001<<3)
#define M_ODR_6HZ  (0b010<<3)
#define M_ODR_8HZ  (0b011<<3)
#define M_ODR_10HZ (0b000<<3)
#define M_ODR_15HZ (0b100<<3)
#define M_ODR_20HZ (0b101<<3)
#define M_ODR_25HZ (0b110<<3)
#define M_ODR_30HZ (0b111<<3)

// 0 = BOSCH regular preset: 0.5 mA
// 1 = BOSCH enhanced regular preset: 0.8 mA
// 2 = midway between 1 and 3: around 2.5 mA
// 3 = BOSCH high accuracy preset: 4.9 mA
#define M_ACCURACY 1

#if M_ACCURACY == 0
#define M_NUM_XY_REPETITIONS 9
#define M_NUM_Z_REPETITIONS  15
#define M_INTERVAL_MS 100
#endif

#if M_ACCURACY == 1
#define M_NUM_XY_REPETITIONS 15
#define M_NUM_Z_REPETITIONS  27
#define M_INTERVAL_MS 100
#endif

#if M_ACCURACY == 2
#define M_NUM_XY_REPETITIONS 31
#define M_NUM_Z_REPETITIONS  55
#define M_INTERVAL_MS 75
#endif

#if M_ACCURACY == 3
#define M_NUM_XY_REPETITIONS 47
#define M_NUM_Z_REPETITIONS  83
#define M_INTERVAL_MS 50
#endif


#define M_CONV_TIME_US (145*M_NUM_XY_REPETITIONS + 500*M_NUM_Z_REPETITIONS + 980 + 100 /*a bit extra on our own*/)

#define M_NUM_XY_REPETITIONS_REGVAL ((M_NUM_XY_REPETITIONS-1)/2)
#define M_NUM_Z_REPETITIONS_REGVAL  (M_NUM_Z_REPETITIONS-1)

static const uint16_t m_init_seq[] =
{
	WR_REG(0x4b, 1 /*Enable power*/),
//	WR_REG(0x4c, M_ODR_10HZ | M_OP_NORMAL),
	WR_REG(0x51, M_NUM_XY_REPETITIONS_REGVAL),
	WR_REG(0x52, M_NUM_Z_REPETITIONS_REGVAL)
};


#include "imu_m_compensation.c"

static void m_sensor_init(int bunch)
{
	if(!(bunch == 0 || bunch == 1))
		error(20);

	for(int m=0; m < NUM_ELEM(m_init_seq); m++)
	{
		if(bunch) SEL_M135(); else SEL_M024();
		__DSB();
		AGM01_WR16 = m_init_seq[m];
		AGM23_WR16 = m_init_seq[m];
		AGM45_WR16 = m_init_seq[m];
		__DSB();
		delay_us(6);
		if(bunch) DESEL_M135(); else DESEL_M024();
		// Clean up the RX fifo of the dummy data:
		AGM01_RD16;
		AGM23_RD16;
		AGM45_RD16;
		__DSB();
		delay_us(4);
		if(m==0)
			delay_ms(5); // Extra delay after the turn-on command (startup time in datasheet: 3ms)
	}

	uart_print_string_blocking("a\r\n");

	// Fetch calibration data from the magnetometer:

	if(bunch) SEL_M135(); else SEL_M024();
	AGM01_WR8 = 0x80 | M_CALIB_START_ADDR;
	AGM23_WR8 = 0x80 | M_CALIB_START_ADDR;
	AGM45_WR8 = 0x80 | M_CALIB_START_ADDR;
	__DSB();
	while(!(AGM01_SPI->SR & 1)) ;
	while(!(AGM23_SPI->SR & 1)) ;
	while(!(AGM45_SPI->SR & 1)) ;
	__DSB();

	AGM01_RD8;
	AGM23_RD8;
	AGM45_RD8;
	__DSB();

	for(int m=0; m < sizeof(m_calib[0]); m++)
	{
		AGM01_WR8 = 0;
		AGM23_WR8 = 0;
		AGM45_WR8 = 0;
		__DSB();

		while(!(AGM01_SPI->SR & 1)) ;
		while(!(AGM23_SPI->SR & 1)) ;
		while(!(AGM45_SPI->SR & 1)) ;
		__DSB();

		*(((uint8_t*)&m_calib[0+bunch])+m) = AGM01_RD8; __DSB();
		*(((uint8_t*)&m_calib[2+bunch])+m) = AGM23_RD8; __DSB();
		*(((uint8_t*)&m_calib[4+bunch])+m) = AGM45_RD8; __DSB();
		__DSB();
	}
	if(bunch) DESEL_M135(); else DESEL_M024();



#if 0
	for(int i=bunch; i<6; i+=2)
	{
		uart_print_string_blocking("\r\nsensor "); o_itoa16(i, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
		uart_print_string_blocking("x1="); o_itoa16(m_calib[i].x1, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
		uart_print_string_blocking("y1="); o_itoa16(m_calib[i].y1, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
		uart_print_string_blocking("z4="); o_itoa16(m_calib[i].z4, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
		uart_print_string_blocking("x2="); o_itoa16(m_calib[i].x2, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
		uart_print_string_blocking("y2="); o_itoa16(m_calib[i].y2, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
		uart_print_string_blocking("z2="); o_itoa16(m_calib[i].z2, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
		uart_print_string_blocking("z1="); o_utoa16(m_calib[i].z1, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
		uart_print_string_blocking("xyz1="); o_utoa16(m_calib[i].xyz1, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
		uart_print_string_blocking("z3="); o_itoa16(m_calib[i].z3, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
		uart_print_string_blocking("xy2="); o_itoa16(m_calib[i].xy2, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
		uart_print_string_blocking("xy1="); o_utoa16(m_calib[i].xy1, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
		uart_print_string_blocking("\r\n");
	}
#endif
}


static void ag_sensor_init(int bunch)
{
	if(!(bunch == 0 || bunch == 1))
		error(20);

	for(int a=0; a < NUM_ELEM(a_init_seq); a++)
	{
		if(bunch) SEL_A135(); else SEL_A024();
		__DSB();
		AGM01_WR16 = a_init_seq[a];
		AGM23_WR16 = a_init_seq[a];
		AGM45_WR16 = a_init_seq[a];
		__DSB();
		delay_us(6);
		if(bunch) DESEL_A135(); else DESEL_A024();
		// Clean up the RX fifo of the dummy data:
		AGM01_RD16;
		AGM23_RD16;
		AGM45_RD16;
		__DSB();
		delay_us(4);
	}

	for(int g=0; g < NUM_ELEM(g_init_seq); g++)
	{
		if(bunch) SEL_G135(); else SEL_G024();
		__DSB();
		AGM01_WR16 = g_init_seq[g];
		AGM23_WR16 = g_init_seq[g];
		AGM45_WR16 = g_init_seq[g];
		__DSB();
		delay_us(6);
		if(bunch) DESEL_G135(); else DESEL_G024();
		// Clean up the RX fifo of the dummy data:
		AGM01_RD16;
		AGM23_RD16;
		AGM45_RD16;
		__DSB();
		delay_us(4);
	}

}


/* 

	Three sensors are connected to shared nCS signals, each on separate SPI bus.
	Thus, three sensors (0,2,4 or 1,3,5) are all read simultaneously.

	While the sampling rates are the same, due to free-running oscillators, the FIFO
	fill level on the sensors won't always be the same. This means, sometimes we read
	data from a sensor which has nothing in FIFO. This is acceptable per datasheet, and
	zeroes will be read out. Note that this case is only allowed when "read or burst
	read access time [will not] exceed the sampling time". This won't become an issue
	with our fast SPI comms.




	Idle time between write accesses at least 2 us

	Write operation: 16 bit access
	R/W bit 0 = write
	7 bits  reg addr
	8 bits  data
	MISO is tri-stated by BMX055

	Single-byte read operation: 16 bit access

	TX data:
	R/W bit  1 = read
	7 bits   reg addr
	8 bits   do not care

	RX data:
	8 bits   do not care
	8 bits   data


	Multi-byte read operation: (n+1)*8 bit access

	TX data:
	R/W bit  1 = read
	7 bits   reg addr
	n*8 bits do not care

	RX data:
	8 bits    do not care
	n*8 bits  data from reg_addr, reg_addr+1, and so on

	With 6.25MHz SPI clock, max inter-data idleness setting of 15 clock cycles equals 2.4us idle time
	between write accesses.

*/

void init_imu() __attribute__((section(".text_itcm")));
void init_imu()
{
	#if defined(IMU0_PRESENT) || defined(IMU1_PRESENT)

		RCC->APB2ENR |= 1UL<<13;
		__DSB();

		AGM01_SPI->CFG1 = SPI_CFG1_NONDMA;

		AGM01_SPI->CFG2 = 1UL<<31 /*Keep pin config while SPI is disabled to prevent glitches*/ |
			1UL<<29 /*SSOE - another new incorrectly documented trap bit*/ | 1UL<<26 /*Software slave management*/ | 1UL<<22 /*Master*/ |
			0UL<<25 /*CPOL*/ | 0UL<<24 /*CPHA*/;
		
		AGM01_SPI->CR1 =  1UL<<12 /*SSI bit must always be high in software nSS managed master mode*/;

		IO_ALTFUNC(GPIOE,13, 5); // MISO

		IO_ALTFUNC(GPIOE,14, 5); // MOSI
		IO_SPEED(GPIOE,14, 2);

		IO_ALTFUNC(GPIOE,12, 5); // SCK
		IO_SPEED(GPIOE,12, 2);

		AGM01_SPI->IER = 1UL<<9 /*mode fault*/ | 1UL<<8 /*TI mode fault*/ | 1UL<<7 /*CRC error*/ | 1UL<<6 /*Overrun*/ | 1UL<<5 /*Underrun*/;

		__DSB();
		AGM01_SPI->CR1 = SPI_CR_ON;
		__DSB();
		AGM01_SPI->CR1 |= 1UL<<9;

		AGM01_TX_DMA_STREAM->PAR = (uint32_t)&(AGM01_SPI->TXDR);
		AGM01_RX_DMA_STREAM->PAR = (uint32_t)&(AGM01_SPI->RXDR);
		AGM01_TX_DMA_STREAM->M0AR = (uint32_t)&dma_command;

		// FIFO error interrupt - Unofficial errata: happens on non-FIFO related, including undocumented errors as well!
		// ST declines to update errata or correct the manual, playing dumb instead:
		// https://community.st.com/s/question/0D50X00009ZDC3LSAX/rm0433-dmabdma-wrong-memory-region-as-sourcetarget-will-set-fifo-error
		AGM01_TX_DMA_STREAM->FCR |= 1UL<<7;
		AGM01_RX_DMA_STREAM->FCR |= 1UL<<7;

		AGM01_TX_DMAMUX();
		AGM01_RX_DMAMUX();

		NVIC_SetPriority(AGM01_TX_DMA_STREAM_IRQ, 1);
		NVIC_EnableIRQ(AGM01_TX_DMA_STREAM_IRQ);
		NVIC_SetPriority(AGM01_RX_DMA_STREAM_IRQ, 1);
		NVIC_EnableIRQ(AGM01_RX_DMA_STREAM_IRQ);
		NVIC_SetPriority(AGM01_SPI_IRQ, 1);
		NVIC_EnableIRQ(AGM01_SPI_IRQ);

	#endif

	#if defined(IMU2_PRESENT) || defined(IMU3_PRESENT)

		RCC->APB4ENR |= 1UL<<5;
		__DSB();

		AGM23_SPI->CFG1 = SPI_CFG1_NONDMA;

		AGM23_SPI->CFG2 = 1UL<<31 /*Keep pin config while SPI is disabled to prevent glitches*/ |
			1UL<<29 /*SSOE - another new incorrectly documented trap bit*/ | 1UL<<26 /*Software slave management*/ | 1UL<<22 /*Master*/ |
			0UL<<25 /*CPOL*/ | 0UL<<24 /*CPHA*/;


		AGM23_SPI->CR1 =  1UL<<12 /*SSI bit must always be high in software nSS managed master mode*/;

		IO_ALTFUNC(GPIOG,12, 5); // MISO

		IO_ALTFUNC(GPIOG,14, 5); // MOSI
		IO_SPEED(GPIOG,14, 2);

		IO_ALTFUNC(GPIOG,13, 5); // SCK
		IO_SPEED(GPIOG,13, 2);

		AGM23_SPI->IER = 1UL<<9 /*mode fault*/ | 1UL<<8 /*TI mode fault*/ | 1UL<<7 /*CRC error*/ | 1UL<<6 /*Overrun*/ | 1UL<<5 /*Underrun*/;


		__DSB();
		AGM23_SPI->CR1 = SPI_CR_ON;
		__DSB();
		AGM23_SPI->CR1 |= 1UL<<9;

		AGM23_TX_DMA_STREAM->CPAR = (uint32_t)&(AGM23_SPI->TXDR);
		AGM23_RX_DMA_STREAM->CPAR = (uint32_t)&(AGM23_SPI->RXDR);

		AGM23_TX_DMA_STREAM->CM0AR = (uint32_t)&bdma_command;

		AGM23_TX_DMAMUX();
		AGM23_RX_DMAMUX();

		NVIC_SetPriority(AGM23_TX_DMA_STREAM_IRQ, 1);
		NVIC_EnableIRQ(AGM23_TX_DMA_STREAM_IRQ);
		NVIC_SetPriority(AGM23_RX_DMA_STREAM_IRQ, 1);
		NVIC_EnableIRQ(AGM23_RX_DMA_STREAM_IRQ);
		NVIC_SetPriority(AGM23_SPI_IRQ, 1);
		NVIC_EnableIRQ(AGM23_SPI_IRQ);


	#endif

	#if defined(IMU4_PRESENT) || defined(IMU5_PRESENT)

		RCC->APB1LENR |= 1UL<<14;
		__DSB();

		AGM45_SPI->CFG1 = SPI_CFG1_NONDMA;

		AGM45_SPI->CFG2 =  1UL<<31 /*Keep pin config while SPI is disabled to prevent glitches*/ |
			1UL<<29 /*SSOE - another new incorrectly documented trap bit*/ | 1UL<<26 /*Software slave management*/ | 1UL<<22 /*Master*/ |
			0UL<<25 /*CPOL*/ | 0UL<<24 /*CPHA*/;


		AGM45_SPI->CR1 =  1UL<<12 /*SSI bit must always be high in software nSS managed master mode*/;

		IO_ALTFUNC(GPIOI, 2, 5); // MISO

		IO_ALTFUNC(GPIOI, 3, 5); // MOSI
		IO_SPEED(GPIOI, 3, 2);

		IO_ALTFUNC(GPIOI, 1, 5); // SCK
		IO_SPEED(GPIOI, 1, 2);


		AGM45_SPI->IER = 1UL<<9 /*mode fault*/ | 1UL<<8 /*TI mode fault*/ | 1UL<<7 /*CRC error*/ | 1UL<<6 /*Overrun*/ | 1UL<<5 /*Underrun*/;


		__DSB();
		AGM45_SPI->CR1 = SPI_CR_ON;
		__DSB();
		AGM45_SPI->CR1 |= 1UL<<9;

		AGM45_TX_DMA_STREAM->PAR = (uint32_t)&(AGM45_SPI->TXDR);
		AGM45_RX_DMA_STREAM->PAR = (uint32_t)&(AGM45_SPI->RXDR);

		AGM45_TX_DMA_STREAM->M0AR = (uint32_t)&dma_command;

		AGM45_TX_DMA_STREAM->FCR |= 1UL<<7;
		AGM45_RX_DMA_STREAM->FCR |= 1UL<<7;

		AGM45_TX_DMAMUX();
		AGM45_RX_DMAMUX();

		NVIC_SetPriority(AGM45_TX_DMA_STREAM_IRQ, 1);
		NVIC_EnableIRQ(AGM45_TX_DMA_STREAM_IRQ);
		NVIC_SetPriority(AGM45_RX_DMA_STREAM_IRQ, 1);
		NVIC_EnableIRQ(AGM45_RX_DMA_STREAM_IRQ);
		NVIC_SetPriority(AGM45_SPI_IRQ, 1);
		NVIC_EnableIRQ(AGM45_SPI_IRQ);


	#endif

	#if defined(IMU0_PRESENT) || defined(IMU2_PRESENT) || defined(IMU4_PRESENT)
		DESEL_A024();
		DESEL_G024();
		DESEL_M024();
		IO_TO_GPO(IMU024_G_NCS_PORT,IMU024_G_NCS_PIN);
		IO_SPEED (IMU024_G_NCS_PORT,IMU024_G_NCS_PIN, 1);
		IO_TO_GPO(IMU024_A_NCS_PORT,IMU024_A_NCS_PIN);
		IO_SPEED (IMU024_A_NCS_PORT,IMU024_A_NCS_PIN, 1);
		IO_TO_GPO(IMU024_M_NCS_PORT,IMU024_M_NCS_PIN);
		IO_SPEED (IMU024_M_NCS_PORT,IMU024_M_NCS_PIN, 1);
	#endif

	#if defined(IMU1_PRESENT) || defined(IMU3_PRESENT) || defined(IMU5_PRESENT)
		DESEL_A135();
		DESEL_G135();
		DESEL_M135();
		IO_TO_GPO(IMU135_G_NCS_PORT,IMU135_G_NCS_PIN);
		IO_SPEED (IMU135_G_NCS_PORT,IMU135_G_NCS_PIN, 1);
		IO_TO_GPO(IMU135_A_NCS_PORT,IMU135_A_NCS_PIN);
		IO_SPEED (IMU135_A_NCS_PORT,IMU135_A_NCS_PIN, 1);
		IO_TO_GPO(IMU135_M_NCS_PORT,IMU135_M_NCS_PIN);
		IO_SPEED (IMU135_M_NCS_PORT,IMU135_M_NCS_PIN, 1);
	#endif
	

	/*
		TIM3 works as a counter for the interrupt-driven state machine.

		APB1 runs at 100MHz, so timer runs at 200MHz. Even if the APB1 prescaler is changed to 1
		so it runs at 200MHz, the timer will still just run at 200MHz.

		With prescaler = 20, the counter runs at 10MHz - each unit is 0.1 us.
	*/
	RCC->APB1LENR |= 1UL<<1;

	TIM3->CR1 = 1UL<<3 /* one pulse mode */ | 1UL<<2 /*only over/underflow generates update int*/;
	TIM3->DIER |= 1UL; // Update interrupt
	TIM3->PSC = 20-1;

	m_sensor_init(0);
	m_sensor_init(1);

	ag_sensor_init(0);
	ag_sensor_init(1);

	// Do not add delay here: init of the a&g flush the FIFOs, and we want to have the interrupt running right away
	// (within about 2-3 ms) so that excess data does not accumulate.
	// ISR considers excess data items in FIFOs as an error condition.
	NVIC_SetPriority(TIM3_IRQn, 5);
	NVIC_EnableIRQ(TIM3_IRQn);

#if 0

	// Code to measure in which order RX DMA END interrupt flag, RX DMA turning off, and EOT interrupt flag show up.
	// Due to limitations of measurement on CPU, exact order couldn't be established - they appear approximately the same
	int cnt = 0;
	int eot01 = 0;
	int eot23 = 0;
	int eot45 = 0;
	int dma01tx_off = 0;
	int dma01rx_off = 0;
	int dma45tx_off = 0;
	int dma45rx_off = 0;

	int dma01tx_fin = 0;
	int dma01rx_fin = 0;
	int dma23tx_fin = 0;
	int dma23rx_fin = 0;
	int dma45tx_fin = 0;
	int dma45rx_fin = 0;

	TIM3->ARR = 65000;

	ag01_dma_start(2, &a_packets0145[0]);
	ag23_dma_start(2, &a_packets23[0]);
	ag45_dma_start(2, &a_packets0145[2]);

	TIM3->CNT = 0;
	__DSB();
	TIM3->CR1 = 1UL<<3 | 1UL<<2 | 1UL;
	__DSB();

	while(1)
	{
		if(AGM01_SPI->SR & (1UL<<3)) if(!eot01) eot01 = TIM3->CNT;
//		if(AGM23_SPI->SR & (1UL<<3)) if(!eot23) eot23 = TIM3->CNT;
//		if(AGM45_SPI->SR & (1UL<<3)) if(!eot45) eot45 = TIM3->CNT;

//		if(!(AGM01_TX_DMA_STREAM->CR & 1UL)) if(!dma01tx_off) dma01tx_off = TIM3->CNT;
		if(!(AGM01_RX_DMA_STREAM->CR & 1UL)) if(!dma01rx_off) dma01rx_off = TIM3->CNT;
//		if(!(AGM45_TX_DMA_STREAM->CR & 1UL)) if(!dma45tx_off) dma45tx_off = TIM3->CNT;
//		if(!(AGM45_RX_DMA_STREAM->CR & 1UL)) if(!dma45rx_off) dma45rx_off = TIM3->CNT;

//		if(DMA_INTFLAGS(AGM01_TX_DMA, AGM01_TX_DMA_STREAM_NUM) & (1UL<<5))  if(!dma01tx_fin) dma01tx_fin = TIM3->CNT;
		if(DMA_INTFLAGS(AGM01_RX_DMA, AGM01_RX_DMA_STREAM_NUM) & (1UL<<5))  if(!dma01rx_fin) dma01rx_fin = TIM3->CNT;
//		if(BDMA_INTFLAGS(AGM23_TX_DMA, AGM23_TX_DMA_STREAM_NUM) & (1UL<<1)) if(!dma23tx_fin) dma23tx_fin = TIM3->CNT;
//		if(BDMA_INTFLAGS(AGM23_RX_DMA, AGM23_RX_DMA_STREAM_NUM) & (1UL<<1)) if(!dma23rx_fin) dma23rx_fin = TIM3->CNT;
//		if(DMA_INTFLAGS(AGM45_TX_DMA, AGM45_TX_DMA_STREAM_NUM) & (1UL<<5))  if(!dma45tx_fin) dma45tx_fin = TIM3->CNT;
//		if(DMA_INTFLAGS(AGM45_RX_DMA, AGM45_RX_DMA_STREAM_NUM) & (1UL<<5))  if(!dma45rx_fin) dma45rx_fin = TIM3->CNT;

		__DSB();

		cnt++;

		if(cnt > 1000000)
			break;
	}

	
	uart_print_string_blocking("perkele : "); o_btoa16_fixed(DMA_INTFLAGS(AGM01_TX_DMA, AGM01_TX_DMA_STREAM_NUM), printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
//	uart_print_string_blocking("perkele : "); o_utoa32_hex(DMA1->HIF), printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	
	uart_print_string_blocking("01 RXDMAFIN : "); o_utoa32(dma01rx_fin, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("01 RXDMAOFF : "); o_utoa32(dma01rx_off, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("01 TXDMAFIN : "); o_utoa32(dma01tx_fin, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("01 TXDMAOFF : "); o_utoa32(dma01tx_off, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("01       EOT: "); o_utoa32(eot01, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n\r\n");

	uart_print_string_blocking("23 RXDMAFIN : "); o_utoa32(dma23rx_fin, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("23 RXDMAOFF : N/A"); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("23 TXDMAFIN : "); o_utoa32(dma23tx_fin, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("23 TXDMAOFF : N/A"); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("23       EOT: "); o_utoa32(eot23, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n\r\n");

	uart_print_string_blocking("45 RXDMAFIN : "); o_utoa32(dma45rx_fin, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("45 RXDMAOFF : "); o_utoa32(dma45rx_off, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("45 TXDMAFIN : "); o_utoa32(dma45tx_fin, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("45 TXDMAOFF : "); o_utoa32(dma45tx_off, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("45       EOT: "); o_utoa32(eot45, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n\r\n");


	while(1);

#endif

}


/*
	CPU analysis: delay_ms(10) takes:
	9 000 us (0.0% burden), fsm disables (nothing runs, expect delay_ms() function)
	9 314 us (3.4% burden), IMU fsm running, REPOLL_WAIT = 1000, FINAL_WAIT = 1000
	9 256 us (2.8% burden),                                      FINAL_WAIT = 10000
	9 088 us (0.9% burden),                  REPOLL_WAIT = 5000, FINAL_WAIT = 10000

*/

void timer_test()
{
//	int cnt = 0;
	set_timer(50000);

	RCC->APB1LENR |= 1UL<<2;
	__DSB();
	TIM4->PSC = 200-1;
	TIM4->CR1 = 1UL<<3 /* one pulse mode */;
	TIM4->ARR = 0xffffffff;


	while(1)
	{
		printings();
	}
}


void deinit_imu()
{
	#if defined(IMU0_PRESENT) || defined(IMU1_PRESENT)

		AGM01_SPI->CR1 = 0;
		RCC->APB2ENR &= ~(1UL<<13);

		IO_TO_GPI(GPIOE,13); // MISO

		IO_TO_GPI(GPIOE,14); // MOSI
		IO_SPEED(GPIOE,14, 0);

		IO_TO_GPI(GPIOE,12); // SCK
		IO_SPEED(GPIOE,12, 0);
	#endif

	#if defined(IMU2_PRESENT) || defined(IMU3_PRESENT)

		AGM23_SPI->CR1 = 0;
		RCC->APB4ENR &= ~(1UL<<5);

		IO_TO_GPI(GPIOG,12); // MISO

		IO_TO_GPI(GPIOG,14); // MOSI
		IO_SPEED(GPIOG,14, 0);

		IO_TO_GPI(GPIOG,13); // SCK
		IO_SPEED(GPIOG,13, 0);
	#endif

	#if defined(IMU4_PRESENT) || defined(IMU5_PRESENT)

		AGM45_SPI->CR1 = 0;
		RCC->APB1LENR &= ~(1UL<<14);

		IO_TO_GPI(GPIOI, 2); // MISO

		IO_TO_GPI(GPIOI, 3); // MOSI
		IO_SPEED(GPIOI, 3, 0);

		IO_TO_GPI(GPIOI, 1); // SCK
		IO_SPEED(GPIOI, 1, 0);
	#endif

	#if defined(IMU0_PRESENT) || defined(IMU2_PRESENT) || defined(IMU4_PRESENT)
		IO_TO_GPI(IMU024_G_NCS_PORT,IMU024_G_NCS_PIN);
		IO_SPEED (IMU024_G_NCS_PORT,IMU024_G_NCS_PIN, 0);
		IO_TO_GPI(IMU024_A_NCS_PORT,IMU024_A_NCS_PIN);
		IO_SPEED (IMU024_A_NCS_PORT,IMU024_A_NCS_PIN, 0);
		IO_TO_GPI(IMU024_M_NCS_PORT,IMU024_M_NCS_PIN);
		IO_SPEED (IMU024_M_NCS_PORT,IMU024_M_NCS_PIN, 0);
	#endif

	#if defined(IMU1_PRESENT) || defined(IMU3_PRESENT) || defined(IMU5_PRESENT)
		IO_TO_GPI(IMU135_G_NCS_PORT,IMU135_G_NCS_PIN);
		IO_SPEED (IMU135_G_NCS_PORT,IMU135_G_NCS_PIN, 0);
		IO_TO_GPI(IMU135_A_NCS_PORT,IMU135_A_NCS_PIN);
		IO_SPEED (IMU135_A_NCS_PORT,IMU135_A_NCS_PIN, 0);
		IO_TO_GPI(IMU135_M_NCS_PORT,IMU135_M_NCS_PIN);
		IO_SPEED (IMU135_M_NCS_PORT,IMU135_M_NCS_PIN, 0);
	#endif


}
