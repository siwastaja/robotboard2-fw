/*
	Originally from: PULUROBOT RN1-BRAIN RobotBoard main microcontroller firmware project

	(c) 2017-2018 Pulu Robotics and other contributors
	Maintainer: Antti Alhonen <antti.alhonen@iki.fi>

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License version 2, as 
	published by the Free Software Foundation.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	GNU General Public License version 2 is supplied in file LICENSING.



	FLASHER module.

	Allows full in-system firmware update, through the same SPI interface
	used for normal operation.

	Flashing functions are run from the ITCM RAM section, allowing reflashing
	the flasher code itself (and anything else). (TODO: Not much ITCM, it's valuable.
	Find out how to alias these functions to the same addresses as anything else on
	the ITCM; just copy them to actual memory when the flasher is run, not in the startup code.)


	For maximum robustness and reliability, the complete firmware is transferred
	to RAM first, then checked using CRC8. Only after that, flash is erased and
	written.

	For this reason, the firmware size is limited to available RAM.

	The flasher always ends up in reset.

	Hence, we just mercilessly reuse the SRAM1, starting from 0x20020000, for the huge
	firmware buffer. Former application RAM usage does not matter since we always reset
	after running the flasher. This limits the maximum firmware size to about 360K
	(SRAM1 368K minus some storage for the flasher itself). Let's say 256K since it's
	a nice number, and fits in FLASH sectors 0..4, leaving the rest (5,6,7, each 256K)
	for storing some calibration etc.

	NOTE: For this reason, any variables for flasher need to reside either in stack,
	or in their own section located after the large firmware buffer. Right now we
	don't have globals or statics here. Keep it that way if you don't know what you
	are doing.


*/

#include <stdint.h>
#include "ext_include/stm32h7xx.h"
#include "stm32_cmsis_extension.h"

#include "flash.h"
#include "own_std.h"

#define FLASH_OFFSET 0x08000000

extern void delay_us(uint32_t i);
extern void delay_ms(uint32_t i);
#define LED_ON()  HI(GPIOF, 2)
#define LED_OFF() LO(GPIOF, 2)


static void unlock_flash() __attribute__((section(".text_itcm")));
static void unlock_flash()
{
	if(FLASH->CR & (1UL<<31))
	{
		FLASH->KEYR = 0x45670123;
		FLASH->KEYR = 0xCDEF89AB;
	}
}

static void lock_flash() __attribute__((section(".text_itcm")));
static void lock_flash()
{
	FLASH->CR |= 1UL<<31;
}

static int flash_erase_sector(int sector) __attribute__((section(".text_itcm")));
static int flash_erase_sector(int sector)
{
	if(sector < 0 || sector > 7)
		return 1;

	while(FLASH->SR & (1UL<<16)) ; // Poll busy bit
	FLASH->CR |= 0b10UL<<8 /*32-bit parallelism*/ | sector<<3 | 1UL<<1 /*sector erase*/;
	FLASH->CR |= 1UL<<16; // Start
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	while(FLASH->SR & (1UL<<16)) ; // Poll busy bit

	FLASH->CR = 0; // Clear erase bit.
	return 0;
}

volatile settings_t settings __attribute__((section(".settings")));

void save_flash_settings()
{
	unlock_flash();
	flash_erase_sector(7);

	FLASH->CR = 0b10<<8 /*32-bit parallellism*/ | 1UL /*activate programming*/;

	volatile uint32_t* p_flash = (volatile uint32_t*)(FLASH_OFFSET + 0x000C0000 /*sector 7*/);
	volatile uint32_t* p_ram = (volatile uint32_t* volatile)(&settings);

	for(int i=0; i<sizeof(settings)/4; i++)
	{
		*p_flash = *p_ram;
		p_flash++;
		p_ram++;
		__DSB(); __ISB();
		while(FLASH->SR & (1UL<<16)) ; // Poll busy bit
	}

	FLASH->SR |= 1UL; // Clear End Of Operation bit by writing '1'
	FLASH->CR = 0; // Clear programming bit.

	lock_flash();
}

/*

Flashing: 

	Enter the flasher using the flasher entering sequence in normal SPI data.
	The MCU waits for about 200ms.
	Flush any SPI buffers during this wait. Everything's ignored. Wait for
	at least 500ms so that the MCU is ready to receive.


To program:	
	Do a 9-byte write:
		uint32_t little endian: 0xabba1337
		uint32_t little endian: firmware size in bytes, rounded up to next four
		data bytes padded to next four bytes: valid: 512 to 262144
		CRC8 over the full, padded firmware

	Then do as many writes as needed, max 65535 bytes each.

	Erasing & programming happens automatically after nCS goes low with enough number
	of bytes in buffer, and the data passes size sanity, size match, and CRC tests.

	Then, do 5-byte long reads (writes with any dummy data). You'll get 0x55 or 0x00
	or something similar back, the flashing is busy. When done, you'll get:
		uint32_t little endian: 0xacdc3579
		uint8_t success code:
			2 = size sanity check fail
			4 = size mismatch
			5 = len not aligned to 4
			6 = crc test fail
			123 = success
			others = unspecified error

To read:
	Do a 8-byte long write as one operation:
		uint32_t little endian: 0xbeef1234
		uint32_t little endian: memory offset. Starts at 0.

	Then, as a separate operation, do dummy writes (generate clocks) for max 65535
	bytes. If the firmware is longer, do multiple read operations (first command, then
	data clocks.)


To reset:
	Do a 5-byte write as one operation:
		uint32_t little endian: 0xdead5678
		uint8_t reset type:
			1, others = soft reset
			2 = power cycle (if implemented)

	You have to reset. There's no other way going back to the application.
*/

#define FLASHER_BUF_ADDR 0x20020000
#define FLASHER_MAX_FW_SIZE (256*1024)
#define FLASHER_BUF_SIZE (FLASHER_MAX_FW_SIZE+16)
#define FLASHER_DMA_SIZE 65535

#define FLASHER_REPLY_BUF_ADDR (FLASHER_BUF_ADDR+FLASHER_BUF_SIZE)
#define FLASHER_REPLY_BUF_SIZE 5

static void ena_tx_dma() __attribute__((section(".text_itcm")));
static void ena_tx_dma()
{
	DMA1_Stream4->PAR = (uint32_t)&(SPI2->DR);
	DMA1_Stream4->M0AR = FLASHER_REPLY_BUF_ADDR;
	DMA1_Stream4->CR = 0UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0UL<<8 /*circular OFF*/ |
			   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b01UL<<6 /*mem-to-periph*/;
	DMA1_Stream4->NDTR = FLASHER_REPLY_BUF_SIZE;
	DMA_CLEAR_INTFLAGS(DMA1, 4);
	DMA1_Stream4->CR |= 1; // Enable TX DMA
	SPI2->CR2 |= 1UL<<1 /*TX DMA ena*/;
}

static void ena_tx_dma_for_flash_read(int offset) __attribute__((section(".text_itcm")));
static void ena_tx_dma_for_flash_read(int offset)
{
	DMA1_Stream4->PAR = (uint32_t)&(SPI2->DR);
	DMA1_Stream4->M0AR = 0x08000000 + offset; // FLASH on AXIM interface
	DMA1_Stream4->CR = 0UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0UL<<8 /*circular OFF*/ |
			   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b01UL<<6 /*mem-to-periph*/;
	DMA1_Stream4->NDTR = FLASHER_DMA_SIZE;
	DMA_CLEAR_INTFLAGS(DMA1, 4);
	DMA1_Stream4->CR |= 1; // Enable TX DMA
	SPI2->CR2 |= 1UL<<1 /*TX DMA ena*/;
}

static void reset_spi() __attribute__((section(".text_itcm")));
static void reset_spi()
{
	// Disable the DMAs:
	DMA1_Stream4->CR = 0UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0UL<<8 /*circular OFF*/ |
			   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b01UL<<6 /*mem-to-periph*/;
	DMA1_Stream3->CR = 0UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0UL<<8 /*circular OFF*/ |
			   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/;

	while(DMA1_Stream4->CR & 1UL) ;
	while(DMA1_Stream3->CR & 1UL) ;

	// Hard-reset SPI - the only way to empty TXFIFO! (Go figure.)

	RCC->APB1RSTR = 1UL<<14;
	__asm__ __volatile__ ("nop");
	RCC->APB1RSTR = 0;
			
	// Re-enable:

	SPI2->CR2 = 0b0111UL<<8 /*8-bit data*/ | 1UL<<0 /*RX DMA ena*/  | 1UL<<12 /*Don't Reject The Last Byte*/;

	// RX DMA
	DMA1_Stream3->M0AR = FLASHER_BUF_ADDR;

	DMA1_Stream3->NDTR = FLASHER_DMA_SIZE;
	DMA_CLEAR_INTFLAGS(DMA1, 3);
	DMA1_Stream3->CR |= 1; // Enable DMA

	SPI2->CR1 = 1UL<<6; // Enable in slave mode

}

static void reset_spi_with_rxbuf_offset(int offset) __attribute__((section(".text_itcm")));
static void reset_spi_with_rxbuf_offset(int offset)
{
	// Disable the DMAs:
	DMA1_Stream4->CR = 0UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0UL<<8 /*circular OFF*/ |
			   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b01UL<<6 /*mem-to-periph*/;
	DMA1_Stream3->CR = 0UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0UL<<8 /*circular OFF*/ |
			   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/;

	while(DMA1_Stream4->CR & 1UL) ;
	while(DMA1_Stream3->CR & 1UL) ;

	// Hard-reset SPI - the only way to empty TXFIFO! (Go figure.)

	RCC->APB1RSTR = 1UL<<14;
	__asm__ __volatile__ ("nop");
	RCC->APB1RSTR = 0;
			
	// Re-enable:

	SPI2->CR2 = 0b0111UL<<8 /*8-bit data*/ | 1UL<<0 /*RX DMA ena*/  | 1UL<<12 /*Don't Reject The Last Byte*/;

	// RX DMA

	DMA1_Stream3->M0AR = FLASHER_BUF_ADDR+offset;

	DMA1_Stream3->NDTR = FLASHER_DMA_SIZE;
	DMA_CLEAR_INTFLAGS(DMA1, 3);
	DMA1_Stream3->CR |= 1; // Enable DMA

	SPI2->CR1 = 1UL<<6; // Enable in slave mode

}


#define CRC_INITIAL_REMAINDER 0x00
#define CRC_POLYNOMIAL 0x07 // As per CRC-8-CCITT

#define CALC_CRC(remainder) \
	for(int crc__bit = 8; crc__bit > 0; --crc__bit) \
	{ \
		if((remainder) & 0b10000000) \
		{ \
			(remainder) = ((remainder) << 1) ^ CRC_POLYNOMIAL; \
		} \
		else \
		{ \
			(remainder) = ((remainder) << 1); \
		} \
	}

uint8_t do_program(int fw_len, uint8_t expected_crc) __attribute__((section(".text_itcm")));
uint8_t do_program(int fw_len, uint8_t expected_crc)
{
	__DSB(); __ISB();

	if(fw_len < 512 || fw_len > FLASHER_MAX_FW_SIZE)
	{
		return 2;
	}

	if(fw_len%4 != 0)
	{
		return 5;
	}

	uint8_t chk = CRC_INITIAL_REMAINDER;
	for(int i=0; i<fw_len; i++)
	{
		chk ^= *((volatile uint8_t*)(FLASHER_BUF_ADDR+i));
		CALC_CRC(chk);
	}

	if(chk != expected_crc)
	{
		return 6;
	}

	unlock_flash();

	flash_erase_sector(0);
	if(fw_len > (1*32)*1024) flash_erase_sector(1);
	if(fw_len > (2*32)*1024) flash_erase_sector(2);
	if(fw_len > (3*32)*1024) flash_erase_sector(3);
	if(fw_len > (4*32)*1024) flash_erase_sector(4);

	FLASH->CR = 0b10<<8 /*32-bit parallellism*/ | 1UL /*activate programming*/;

	volatile uint32_t* p_flash = (uint32_t*)FLASH_OFFSET;
	volatile uint32_t* p_ram = (volatile uint32_t* volatile)(FLASHER_BUF_ADDR);

	for(int i=0; i<fw_len/4; i++)
	{
		*p_flash = *p_ram;
		p_flash++;
		p_ram++;
		__DSB(); __ISB();
		while(FLASH->SR & (1UL<<16)) ; // Poll busy bit
	}

	FLASH->SR |= 1UL; // Clear End Of Operation bit by writing '1'
	FLASH->CR = 0; // Clear programming bit.

	lock_flash();

	return 123; // The magical success code.
}

void flasher() __attribute__((section(".text_itcm")));
void flasher()
{
	while(1)
	{
		while(GPIOB->IDR & (1UL<<12)); // Wait for nSS->low
		delay_us(1);
		while(!(GPIOB->IDR & (1UL<<12))) ; // Wait for nSS->high again
		int xfer_len = FLASHER_DMA_SIZE - DMA1_Stream3->NDTR; // let's see how many bytes were subtracted from the DMA remaining length value.
		reset_spi();

		uint32_t cmd = *((volatile uint32_t*)FLASHER_BUF_ADDR);

		switch(cmd)
		{
			case 0xabba1337: // PROGRAM
			{
				if(xfer_len != 9)
				{
					USART3->TDR = 'x';
					reset_spi();
					break;
				}

				uint32_t fw_len = *((volatile uint32_t*)(FLASHER_BUF_ADDR+4));
				uint8_t expected_crc = *((volatile uint8_t*)(FLASHER_BUF_ADDR+8));

				int got_bytes = 0;
				while(got_bytes < fw_len)
				{
					while(GPIOB->IDR & (1UL<<12)) ; // Wait for nSS->low
					delay_us(1);
					while(!(GPIOB->IDR & (1UL<<12))) ; // Wait for nSS->high again
					int this_len = FLASHER_DMA_SIZE - DMA1_Stream3->NDTR;
					got_bytes += this_len;
					reset_spi_with_rxbuf_offset(got_bytes);
				}


				LED_OFF();
				int ret = do_program(fw_len, expected_crc);
				LED_ON();
				// Done programming. Master will be polling. Wait for the on-going poll operation to end first.
				while(!(GPIOB->IDR & (1UL<<12))) ; // Wait for nSS->high again
				reset_spi();
				*((volatile uint32_t*)FLASHER_REPLY_BUF_ADDR) = 0xacdc3579;
				*((volatile uint8_t*)(FLASHER_REPLY_BUF_ADDR+4)) = ret;
				__DSB(); __ISB();
				ena_tx_dma();
				while(GPIOB->IDR & (1UL<<12)) ; // Wait for nSS->low: DMA will do the job.
				delay_us(10);
				while(!(GPIOB->IDR & (1UL<<12))) ; // Wait for nSS->high again
				reset_spi();
			}
			break;

			case 0xbeef1234: // READ
			{
				uint32_t offset = *((volatile uint32_t*)(FLASHER_BUF_ADDR+4));

				// After 8-byte long READ message, we can accept SPI clocks right away!

				ena_tx_dma_for_flash_read(offset);
				while(GPIOB->IDR & (1UL<<12)) ; // Wait for nSS->low: DMA will do the job then
				delay_us(10);
				while(!(GPIOB->IDR & (1UL<<12))) ; // Wait for nSS->high again
				reset_spi();
			}
			break;

			case 0xdead5678: // RESET
			{
				uint8_t type = *((volatile uint8_t*)(FLASHER_BUF_ADDR+4));
				if(type==55)
				{
					while(1)
					{
						LED_ON();
						delay_ms(1000);
						LED_OFF();
						delay_ms(1000);
					}
				}
				else
				{
					LED_OFF();
					NVIC_SystemReset();
					while(1);
				}
			}
			break;

			default:
			{
			}
			break; // Just ignore the message

		}

	}
}


void run_flasher()
{
	__disable_irq();

	ADC1->CR2 = 0; // Disable ADC
	DMA2_Stream0->CR = 0; // Disable ADC DMA, so that it doesn't write stuff in our buffer

	FLASH->ACR &= ~(1UL<<9); // Disable the ART accelerator (instruction caches?) Just in case.


	// Hard-reset SPI - the only way to empty TXFIFO!

	RCC->APB1RSTR = 1UL<<14;
	__asm__ __volatile__ ("nop");
	RCC->APB1RSTR = 0;

	delay_ms(200);

	// Do it again just to be sure...
	RCC->APB1RSTR = 1UL<<14;
	__asm__ __volatile__ ("nop");
	RCC->APB1RSTR = 0;
	__asm__ __volatile__ ("nop");

	// Reconf SPI
	
	SPI2->CR2 = 0b0111UL<<8 /*8-bit data*/ | 1UL<<0 /*RX DMA ena*/ | 1UL<<12 /*Don't Reject The Last Byte*/;

	// RX DMA

	DMA1_Stream3->PAR = (uint32_t)&(SPI2->DR);
	DMA1_Stream3->M0AR = FLASHER_BUF_ADDR;
	DMA1_Stream3->CR = 0UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0UL<<8 /*circular OFF*/ |
			   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/;
	DMA1_Stream3->NDTR = FLASHER_DMA_SIZE;
	DMA_CLEAR_INTFLAGS(DMA1, 3);
	DMA1_Stream3->CR |= 1; // Enable RX DMA


	SPI2->CR1 = 1UL<<6; // Enable in slave mode

	// nCS is PB12 - use it manually. Remove EXTI config:
	EXTI->IMR = 0;
	EXTI->RTSR = 0;

	//SPI2->DR = 0x11;

	LED_ON();
	flasher();
	while(1);
}

