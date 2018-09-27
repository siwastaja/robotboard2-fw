#include <stdint.h>
#include "ext_include/stm32h7xx.h"
#include "stm32_cmsis_extension.h"
#include "misc.h"
#include "own_std.h"

#define IMU0_PRESENT
#define IMU1_PRESENT
#define IMU2_PRESENT
#define IMU3_PRESENT
#define IMU4_PRESENT
#define IMU5_PRESENT

#define IMU_SPI_6M25 // 6.25MHz SPI clock. Datasheet maximum: 10MHz

#ifdef IMU_SPI_6M25
#define SPI2_CLKDIV_REGVAL (0b011UL)
#define SPI4_CLKDIV_REGVAL (0b011UL)
#define SPI6_CLKDIV_REGVAL (0b011UL)
#endif


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

#define SEL_IMU024_G()   do{LO(IMU024_G_NCS_PORT,IMU024_G_NCS_PIN);}while(0)
#define DESEL_IMU024_G() do{HI(IMU024_G_NCS_PORT,IMU024_G_NCS_PIN);}while(0)
#define SEL_IMU024_A()   do{LO(IMU024_A_NCS_PORT,IMU024_A_NCS_PIN);}while(0)
#define DESEL_IMU024_A() do{HI(IMU024_A_NCS_PORT,IMU024_A_NCS_PIN);}while(0)
#define SEL_IMU024_M()   do{LO(IMU024_M_NCS_PORT,IMU024_M_NCS_PIN);}while(0)
#define DESEL_IMU024_M() do{HI(IMU024_M_NCS_PORT,IMU024_M_NCS_PIN);}while(0)

#define SEL_IMU135_G()   do{LO(IMU135_G_NCS_PORT,IMU135_G_NCS_PIN);}while(0)
#define DESEL_IMU135_G() do{HI(IMU135_G_NCS_PORT,IMU135_G_NCS_PIN);}while(0)
#define SEL_IMU135_A()   do{LO(IMU135_A_NCS_PORT,IMU135_A_NCS_PIN);}while(0)
#define DESEL_IMU135_A() do{HI(IMU135_A_NCS_PORT,IMU135_A_NCS_PIN);}while(0)
#define SEL_IMU135_M()   do{LO(IMU135_M_NCS_PORT,IMU135_M_NCS_PIN);}while(0)
#define DESEL_IMU135_M() do{HI(IMU135_M_NCS_PORT,IMU135_M_NCS_PIN);}while(0)


// To convert to two's complement value:
// mask bits 3,2,1 (will contain random values) and 0 (new_data) away.
typedef struct __attribute__((packed))
{
	int16_t raw_x;
	int16_t raw_y;
	int16_t raw_z;
} xcel_fifo_access_rx_t;



void imu_fsm()
{

}

typedef struct __attribute__((packed))
{
	uint8_t reg;
	uint8_t content;
} init_item_t;


#define XCEL_RANGE_2G  0b0011
#define XCEL_RANGE_4G  0b0101
#define XCEL_RANGE_8G  0b1000
#define XCEL_RANGE_16G 0b1100
#define XCEL_BW_31HZ  0b01010
#define XCEL_BW_63HZ  0b01011
#define XCEL_BW_125HZ 0b01100
#define XCEL_BW_250HZ 0b01101
#define XCEL_BW_500HZ 0b01110


static const init_item_t xcel_init_seq[] =
{
	{0x0f, XCEL_RANGE_4G},
	{0x10, XCEL_BW_125HZ},
	{0x3e, 0b01<<6 /*FIFO mode*/ | 0b00 /*X,Y and Z stored*/}
};

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

static const init_item_t gyro_init_seq[] =
{
	{0x0f, GYRO_RANGE_1000DPS},
	{0x10, GYRO_ODR200HZ_BW64HZ},
	{0x3e, 0b01<<6 /*FIFO mode*/ | 0b00 /*X,Y and Z stored*/}
};

#define MAGN_ODR_2HZ  (0b001<<3)
#define MAGN_ODR_6HZ  (0b010<<3)
#define MAGN_ODR_8HZ  (0b011<<3)
#define MAGN_ODR_10HZ (0b000<<3)
#define MAGN_ODR_15HZ (0b100<<3)
#define MAGN_ODR_20HZ (0b101<<3)
#define MAGN_ODR_25HZ (0b110<<3)
#define MAGN_ODR_30HZ (0b111<<3)

#define MAGN_OP_NORMAL (0b00<<1)
#define MAGN_OP_FORCED (0b01<<1)
#define MAGN_OP_SLEEP  (0b11<<1)

#define MAGN_NUM_XY_REPETITIONS_REGVAL 10
#define MAGN_NUM_Z_REPETITIONS_REGVAL 10

static const init_item_t magn_init_seq[] =
{
	{0x4b, 1 /*Enable power*/},
	{0x4c, MAGN_ODR_10HZ | MAGN_OP_NORMAL},
	{0x51, MAGN_NUM_XY_REPETITIONS_REGVAL},
	{0x52, MAGN_NUM_Z_REPETITIONS_REGVAL}
};


static void init_bmx055_xcel()
{
	
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

static char printbuf[128];

	#define MAGON 0x4b01

void turn_m_on()
{
	SEL_IMU024_M();
	__DSB();
	delay_us(10);
	*(uint16_t*)&SPI4->TXDR = MAGON;
	*(uint16_t*)&SPI6->TXDR = MAGON;
	*(uint16_t*)&SPI2->TXDR = MAGON;
	__DSB();
	delay_ms(1);
	DESEL_IMU024_M();
	__DSB();

	// reply: 0x32
	uint16_t mag0 = *(uint16_t*)&SPI4->RXDR;
	uint16_t mag2 = *(uint16_t*)&SPI6->RXDR;
	uint16_t mag4 = *(uint16_t*)&SPI2->RXDR;
	uart_print_string_blocking("TURN ON REPLY mag0 = "); o_utoa16_hex(mag0, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("TURN ON REPLY mag2 = "); o_utoa16_hex(mag2, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("TURN ON REPLY mag4 = "); o_utoa16_hex(mag4, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	delay_ms(1);

	SEL_IMU135_M();
	__DSB();
	delay_us(10);
	*(uint16_t*)&SPI4->TXDR = MAGON;
	*(uint16_t*)&SPI6->TXDR = MAGON;
	*(uint16_t*)&SPI2->TXDR = MAGON;
	__DSB();
	delay_ms(1);
	DESEL_IMU135_M();
	__DSB();

	// reply: 0x32
	uint16_t mag1 = *(uint16_t*)&SPI4->RXDR;
	uint16_t mag3 = *(uint16_t*)&SPI6->RXDR;
	uint16_t mag5 = *(uint16_t*)&SPI2->RXDR;
	uart_print_string_blocking("TURN ON REPLY mag1 = "); o_utoa16_hex(mag1, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("TURN ON REPLY mag3 = "); o_utoa16_hex(mag3, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("TURN ON REPLY mag5 = "); o_utoa16_hex(mag5, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	delay_ms(1);

}

void imu_test()
{
	delay_ms(100);

	uart_print_string_blocking("testings...\r\n"); 

	#define OPER 0x8000
	#define MAGOPER 0xc000

	SEL_IMU024_A();
	__DSB();
	delay_us(10);

	*(uint16_t*)&SPI4->TXDR = OPER;
	*(uint16_t*)&SPI2->TXDR = OPER;
	*(uint16_t*)&SPI6->TXDR = OPER;
	__DSB();
	delay_ms(1);
	DESEL_IMU024_A();
	__DSB();

	// reply: 0xFA
	uint16_t acc0 = *(uint16_t*)&SPI4->RXDR;
	uint16_t acc2 = *(uint16_t*)&SPI6->RXDR;
	uint16_t acc4 = *(uint16_t*)&SPI2->RXDR;

	__DSB();

	uart_print_string_blocking("acc0 = "); o_utoa16_hex(acc0, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("acc2 = "); o_utoa16_hex(acc2, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("acc4 = "); o_utoa16_hex(acc4, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");

	delay_ms(1);

	SEL_IMU024_G();
	__DSB();
	delay_us(10);

	*(uint16_t*)&SPI4->TXDR = OPER;
	*(uint16_t*)&SPI6->TXDR = OPER;
	*(uint16_t*)&SPI2->TXDR = OPER;
	__DSB();

	delay_ms(1);
	DESEL_IMU024_G();
	__DSB();
	// reply: 0x0F
	uint16_t gyr0 = *(uint16_t*)&SPI4->RXDR;
	uint16_t gyr2 = *(uint16_t*)&SPI6->RXDR;
	uint16_t gyr4 = *(uint16_t*)&SPI2->RXDR;
	uart_print_string_blocking("gyr0 = "); o_utoa16_hex(gyr0, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("gyr2 = "); o_utoa16_hex(gyr2, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("gyr4 = "); o_utoa16_hex(gyr4, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	delay_ms(1);

	SEL_IMU024_M();
	__DSB();
	delay_us(10);
	*(uint16_t*)&SPI4->TXDR = MAGOPER;
	*(uint16_t*)&SPI6->TXDR = MAGOPER;
	*(uint16_t*)&SPI2->TXDR = MAGOPER;
	__DSB();
	delay_ms(1);
	DESEL_IMU024_M();
	__DSB();

	// reply: 0x32
	uint16_t mag0 = *(uint16_t*)&SPI4->RXDR;
	uint16_t mag2 = *(uint16_t*)&SPI6->RXDR;
	uint16_t mag4 = *(uint16_t*)&SPI2->RXDR;
	uart_print_string_blocking("mag0 = "); o_utoa16_hex(mag0, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("mag2 = "); o_utoa16_hex(mag2, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("mag4 = "); o_utoa16_hex(mag4, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	delay_ms(1);




	SEL_IMU135_A();
	__DSB();

	delay_us(10);
	*(uint16_t*)&SPI4->TXDR = OPER;
	*(uint16_t*)&SPI6->TXDR = OPER;
	*(uint16_t*)&SPI2->TXDR = OPER;
	__DSB();
	delay_ms(1);
	DESEL_IMU135_A();
	__DSB();

	// reply: 0xFA
	uint16_t acc1 = *(uint16_t*)&SPI4->RXDR;
	uint16_t acc3 = *(uint16_t*)&SPI6->RXDR;
	uint16_t acc5 = *(uint16_t*)&SPI2->RXDR;
	uart_print_string_blocking("acc1 = "); o_utoa16_hex(acc1, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("acc3 = "); o_utoa16_hex(acc3, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("acc5 = "); o_utoa16_hex(acc5, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	delay_ms(1);


	SEL_IMU135_G();
	__DSB();

	delay_us(10);
	*(uint16_t*)&SPI4->TXDR = OPER;
	*(uint16_t*)&SPI6->TXDR = OPER;
	*(uint16_t*)&SPI2->TXDR = OPER;
	__DSB();
	delay_ms(1);
	DESEL_IMU135_G();
	__DSB();

	// reply: 0x0F
	uint16_t gyr1 = *(uint16_t*)&SPI4->RXDR;
	uint16_t gyr3 = *(uint16_t*)&SPI6->RXDR;
	uint16_t gyr5 = *(uint16_t*)&SPI2->RXDR;
	uart_print_string_blocking("gyr1 = "); o_utoa16_hex(gyr1, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("gyr3 = "); o_utoa16_hex(gyr3, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("gyr5 = "); o_utoa16_hex(gyr5, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	delay_ms(1);



	SEL_IMU135_M();
	__DSB();

	delay_us(10);
	*(uint16_t*)&SPI4->TXDR = MAGOPER;
	*(uint16_t*)&SPI6->TXDR = MAGOPER;
	*(uint16_t*)&SPI2->TXDR = MAGOPER;
	__DSB();
	delay_ms(1);
	DESEL_IMU135_M();
	__DSB();

	// reply: 0x32
	uint16_t mag1 = *(uint16_t*)&SPI4->RXDR;
	uint16_t mag3 = *(uint16_t*)&SPI6->RXDR;
	uint16_t mag5 = *(uint16_t*)&SPI2->RXDR;
	uart_print_string_blocking("mag1 = "); o_utoa16_hex(mag1, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("mag3 = "); o_utoa16_hex(mag3, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("mag5 = "); o_utoa16_hex(mag5, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	delay_ms(1);


	delay_ms(1000);

}

void init_imu()
{

	#if defined(IMU0_PRESENT) || defined(IMU1_PRESENT)

		RCC->APB2ENR |= 1UL<<13;

		SPI4->CFG1 = SPI4_CLKDIV_REGVAL<<28 | 1UL<<15 /*TX DMA*/ | 1UL<<14 /*RX DMA*/ |
		             (1/*FIFO threshold*/   -1)<<5 | (16/*bits per frame*/   -1);

		SPI4->CFG2 = 1UL<<29 /*SSOE - another new incorrectly documented trap bit*/ | 1UL<<26 /*Software slave management*/ | 1UL<<22 /*Master*/ |
		             1UL<<25 /*CPOL*/ | 1UL<<24 /*CPHA*/;
		
		SPI4->CR1 =  1UL<<12 /*SSI bit must always be high in software nSS managed master mode*/;

		IO_ALTFUNC(GPIOE,13, 5); // MISO

		IO_ALTFUNC(GPIOE,14, 5); // MOSI
		IO_SPEED(GPIOE,14, 2);

		IO_ALTFUNC(GPIOE,12, 5); // SCK
		IO_SPEED(GPIOE,12, 2);

		SPI4->CR1 |= 1UL;
		SPI4->CR1 |= 1UL<<9;
	#endif

	#if defined(IMU2_PRESENT) || defined(IMU3_PRESENT)

		RCC->APB4ENR |= 1UL<<5;

		SPI6->CFG1 = SPI4_CLKDIV_REGVAL<<28 | 1UL<<15 /*TX DMA*/ | 1UL<<14 /*RX DMA*/ |
		             (1/*FIFO threshold*/   -1)<<5 | (16/*bits per frame*/   -1);

		SPI6->CFG2 = 1UL<<29 /*SSOE - another new incorrectly documented trap bit*/ | 1UL<<26 /*Software slave management*/ | 1UL<<22 /*Master*/ |
		             1UL<<25 /*CPOL*/ | 1UL<<24 /*CPHA*/;


		SPI6->CR1 =  1UL<<12 /*SSI bit must always be high in software nSS managed master mode*/;

		IO_ALTFUNC(GPIOG,12, 5); // MISO

		IO_ALTFUNC(GPIOG,14, 5); // MOSI
		IO_SPEED(GPIOG,14, 2);

		IO_ALTFUNC(GPIOG,13, 5); // SCK
		IO_SPEED(GPIOG,13, 2);

		SPI6->CR1 |= 1UL;
		SPI6->CR1 |= 1UL<<9;

	#endif

	#if defined(IMU4_PRESENT) || defined(IMU5_PRESENT)

		RCC->APB1LENR |= 1UL<<14;

		SPI2->CFG1 = SPI4_CLKDIV_REGVAL<<28 | 1UL<<15 /*TX DMA*/ | 1UL<<14 /*RX DMA*/ |
		             (1/*FIFO threshold*/   -1)<<5 | (16/*bits per frame*/   -1);

		SPI2->CFG2 = 1UL<<29 /*SSOE - another new incorrectly documented trap bit*/ | 1UL<<26 /*Software slave management*/ | 1UL<<22 /*Master*/ |
		             1UL<<25 /*CPOL*/ | 1UL<<24 /*CPHA*/;


		SPI2->CR1 =  1UL<<12 /*SSI bit must always be high in software nSS managed master mode*/;

		IO_ALTFUNC(GPIOI, 2, 5); // MISO

		IO_ALTFUNC(GPIOI, 3, 5); // MOSI
		IO_SPEED(GPIOI, 3, 2);

		IO_ALTFUNC(GPIOI, 1, 5); // SCK
		IO_SPEED(GPIOI, 1, 2);

		SPI2->CR1 |= 1UL;
		SPI2->CR1 |= 1UL<<9;

	#endif

	#if defined(IMU0_PRESENT) || defined(IMU2_PRESENT) || defined(IMU4_PRESENT)
		DESEL_IMU024_A();
		DESEL_IMU024_G();
		DESEL_IMU024_M();
		IO_TO_GPO(IMU024_G_NCS_PORT,IMU024_G_NCS_PIN);
		IO_SPEED (IMU024_G_NCS_PORT,IMU024_G_NCS_PIN, 1);
		IO_TO_GPO(IMU024_A_NCS_PORT,IMU024_A_NCS_PIN);
		IO_SPEED (IMU024_A_NCS_PORT,IMU024_A_NCS_PIN, 1);
		IO_TO_GPO(IMU024_M_NCS_PORT,IMU024_M_NCS_PIN);
		IO_SPEED (IMU024_M_NCS_PORT,IMU024_M_NCS_PIN, 1);
	#endif

	#if defined(IMU1_PRESENT) || defined(IMU3_PRESENT) || defined(IMU5_PRESENT)
		DESEL_IMU135_A();
		DESEL_IMU135_G();
		DESEL_IMU135_M();
		IO_TO_GPO(IMU135_G_NCS_PORT,IMU135_G_NCS_PIN);
		IO_SPEED (IMU135_G_NCS_PORT,IMU135_G_NCS_PIN, 1);
		IO_TO_GPO(IMU135_A_NCS_PORT,IMU135_A_NCS_PIN);
		IO_SPEED (IMU135_A_NCS_PORT,IMU135_A_NCS_PIN, 1);
		IO_TO_GPO(IMU135_M_NCS_PORT,IMU135_M_NCS_PIN);
		IO_SPEED (IMU135_M_NCS_PORT,IMU135_M_NCS_PIN, 1);
	#endif
	
}

void deinit_imu()
{
	#if defined(IMU0_PRESENT) || defined(IMU1_PRESENT)

		SPI4->CR1 = 0;
		RCC->APB2ENR &= ~(1UL<<13);

		IO_TO_GPI(GPIOE,13); // MISO

		IO_TO_GPI(GPIOE,14); // MOSI
		IO_SPEED(GPIOE,14, 0);

		IO_TO_GPI(GPIOE,12); // SCK
		IO_SPEED(GPIOE,12, 0);
	#endif

	#if defined(IMU2_PRESENT) || defined(IMU3_PRESENT)

		SPI6->CR1 = 0;
		RCC->APB4ENR &= ~(1UL<<5);

		IO_TO_GPI(GPIOG,12); // MISO

		IO_TO_GPI(GPIOG,14); // MOSI
		IO_SPEED(GPIOG,14, 0);

		IO_TO_GPI(GPIOG,13); // SCK
		IO_SPEED(GPIOG,13, 0);
	#endif

	#if defined(IMU4_PRESENT) || defined(IMU5_PRESENT)

		SPI2->CR1 = 0;
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
