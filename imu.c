#include <stdint.h>

void init_imu()
{

	#if defined(IMU0_PRESENT) || defined(IMU1_PRESENT)

		RCC->APB2ENR |= 1UL<<13;

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
