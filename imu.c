#include <stdint.h>
#include "ext_include/stm32h7xx.h"
#include "stm32_cmsis_extension.h"

#define IMU_SPI_6M25


#ifdef IMU_SPI_6M25
#define SPI2_CLKDIV_REGVAL (0b011UL)
#define SPI4_CLKDIV_REGVAL (0b011UL)
#define SPI6_CLKDIV_REGVAL (0b011UL)
#endif


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

	


*/

void init_imu()
{

	#if defined(IMU0_PRESENT) || defined(IMU1_PRESENT)

		RCC->APB2ENR |= 1UL<<13;

		SPI4->CFG1 = SPI4_CLKDIV_REGVAL<<28 | 


		SPI4->CFG2 = 1UL<<26 /*Software slave management*/ | 1UL<<22 /*Master*/;
		
		SPI4->CR =  1UL<<8 /*SSI bit must always be high in software nSS managed master mode*/

	#endif

	#if defined(IMU2_PRESENT) || defined(IMU3_PRESENT)

		RCC->APB4ENR |= 1UL<<5;

		SPI6->CFG2 = 1UL<<26 /*Software slave management*/ | 1UL<<22 /*Master*/;

	#endif

	#if defined(IMU4_PRESENT) || defined(IMU5_PRESENT)

		RCC->APB1LENR |= 1UL<<14;

		SPI2->CFG2 = 1UL<<26 /*Software slave management*/ | 1UL<<22 /*Master*/;

	#endif

}

void deinit_imu()
{
	#if defined(IMU0_PRESENT) || defined(IMU1_PRESENT)

		SPI4->CR = 0;
		RCC->APB2ENR &= ~(1UL<<13);

	#endif

	#if defined(IMU2_PRESENT) || defined(IMU3_PRESENT)

		SPI6->CR = 0;
		RCC->APB4ENR &= ~(1UL<<5);

	#endif

	#if defined(IMU4_PRESENT) || defined(IMU5_PRESENT)

		SPI2->CR = 0;
		RCC->APB1LENR &= ~(1UL<<14);

	#endif

}
