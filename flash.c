/*
	Flasher code is located in ITCM core-coupled RAM.

	The flasher allows erasing and writing one complete sector at a time.
	All sectors are 128*1024 bytes.

	Flasher completely uses SRAM1 (exactly 128*1024 bytes as well) as a buffer for
	the data to be written.

	The flasher disregards any other use for SRAM1 - any other data is lost. Now, using flasher
	always ends up in system reset. If the flasher is modified to allow returining to the application,
	SRAM1 contents must be rebuilt.

*/

// Programming delay (WRHIGHFREQ): 0b10 = 185..210MHz AXI bus freq in VOS1


#include <stdint.h>
#include "ext_include/stm32h7xx.h"
#include "stm32_cmsis_extension.h"

#include "flash.h"

#define FLASH_OFFSET 0x08000000

#define SECTOR_LEN (128*1024)
#if (SECTOR_LEN%32 != 0)
#error "SECTOR_LEN must be multiple of 32 (256 bits is the smallest writable block with standard means, with valid ECC)"
#endif

extern void delay_us(uint32_t i);
extern void delay_ms(uint32_t i);


static void unlock_flash(int bank) __attribute__((section(".text_itcm")));
static void unlock_flash(int bank)
{
	if(bank==1)
	{
		if(FLASH->CR1 & (1UL<<0))
		{
			FLASH->KEYR1 = 0x45670123;
			FLASH->KEYR1 = 0xCDEF89AB;
		}
	}
	else if(bank==2)
	{
		if(FLASH->CR2 & (1UL<<0))
		{
			FLASH->KEYR2 = 0x45670123;
			FLASH->KEYR2 = 0xCDEF89AB;
		}
	}
	// else (invalid bank id): flash remains locked, and no harm happens (erasing and writing fails)

}

static void lock_flash() __attribute__((section(".text_itcm")));
static void lock_flash()
{
	// Lock both banks
	FLASH->CR1 |= 1UL<<0;
	FLASH->CR2 |= 1UL<<0;
}

/*
	Delay which is surely long enough so that setting a command bit and then polling for the expected result status works.
*/
static void delay() __attribute__((section(".text_itcm")));
static void delay()
{
	for(int i=0; i<16; i++)
	{
		__asm__ __volatile__ ("nop");
	}
}

#define POLL_QW_BANK1() do{while(FLASH->SR1 & (1UL<<2));}while(0)
#define POLL_QW_BANK2() do{while(FLASH->SR2 & (1UL<<2));}while(0)

#define POLL_BSY_BANK1() do{while(FLASH->SR1 & (1UL<<0));}while(0)
#define POLL_BSY_BANK2() do{while(FLASH->SR2 & (1UL<<0));}while(0)

static int flash_erase_sector(int bank, int sector) __attribute__((section(".text_itcm")));
static int flash_erase_sector(int bank, int sector)
{
	if(bank < 1 || bank > 2 || sector < 0 || sector > 7)
		return 1;

	if(bank == 1)
	{
		POLL_QW_BANK1();
		FLASH->CR1 |= 0b11UL<<4 /*64-bit parallelism*/ | sector<<8 | 1UL<<2 /*sector erase*/;
		FLASH->CR1 |= 1UL<<7; // Start
		delay();
		POLL_QW_BANK1();
		delay();
		POLL_BSY_BANK1(); // Just to be sure everything goes well, 
		                  // we wait for the actual operation to finish, not just to get out of the queue.
		FLASH->CR1 = 0; // Clear configuration, keep flash unlocked
	}
	else
	{
		POLL_QW_BANK2();
		FLASH->CR2 |= 0b11UL<<4 /*64-bit parallelism*/ | sector<<8 | 1UL<<2 /*sector erase*/;
		FLASH->CR2 |= 1UL<<7; // Start
		delay();
		POLL_QW_BANK2();
		delay();
		POLL_BSY_BANK2(); // Just to be sure everything goes well, 
		                  // we wait for the actual operation to finish, not just to get out of the queue.
		FLASH->CR2 = 0; // Clear configuration, keep flash unlocked
	}

	return 0;
}

// write: 
// unlock
//clear PGSERR and INCERR
//PG high.
//check for write protection
//write 32-bit word at 32-bit aligned address
//poll until QW is high, then poll until QW is low

/*

Flashing: 

	Enter the flasher using the flasher entering sequence in normal SPI data.
	The MCU waits for about 200ms.
	Flush any SPI buffers during this wait. Everything's ignored. Wait for
	at least 500ms so that the MCU is ready to receive.


To program:	
	Do a 7-byte write:
		uint32_t little endian: 0xabba1337
		uint8_t: bank number (1 or 2)
		uint8_t: sector number (0 to 7)
		uint8_t: CRC8 over the 128*1024 bytes of data

	Then write 128*1024 bytes of data, in as many writes as needed, max 65535 bytes each. (Minimum
	3 writes)

	Erasing & programming happens automatically after nCS goes low with enough number
	of bytes in buffer, and the data passes the CRC test.

	Then, do 5-byte long reads (writes with any dummy data). You'll get 0x55 or 0x00
	or something similar back, the flashing is busy. When done, you'll get:
		uint32_t little endian: 0xacdc3579
		uint8_t success code:
			6 = crc test fail
			123 = success
			others = unspecified error

To read:
	Do a 8-byte long write as one operation:
		uint32_t little endian: 0xbeef1234
		uint32_t little endian: flash memory offset starting from bank0sect0 - starts from 0

	Then, as a separate operation, do dummy writes (generate clocks) for max 65535
	bytes. If you want to read more than that, do multiple read operations (first command, then
	data clocks.)


To reset:
	Do a 5-byte write as one operation:
		uint32_t little endian: 0xdead5678
		uint8_t reset type:
			1, others = soft reset
			2 = power cycle (if implemented)

	You have to reset. There's no other way going back to the application.
*/

#define FLASHER_BUF_ADDR 0x30000000
#define FLASHER_DMA_SIZE 65535

#define FLASHER_REPLY_BUF_SIZE 5
volatile uint8_t flasher_reply_buf[FLASHER_REPLY_BUF_SIZE] __attribute__((aligned(4)));


static void ena_tx_dma() __attribute__((section(".text_itcm")));
static void ena_tx_dma()
{
	DMA1_Stream4->PAR = (uint32_t)&(SPI2->DR);
	DMA1_Stream4->M0AR = flasher_reply_buf;
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
	DMA1_Stream4->M0AR = FLASH_OFFSET + offset; // FLASH on AXIM interface
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

static uint8_t do_program(int bank, int sector, uint8_t expected_crc) __attribute__((section(".text_itcm")));
static uint8_t do_program(int bank, int sector, uint8_t expected_crc)
{
	__DSB(); __ISB();

	uint8_t chk = CRC_INITIAL_REMAINDER;
	for(int i=0; i<SECTOR_LEN; i++)
	{
		chk ^= *((volatile uint8_t*)(FLASHER_BUF_ADDR+i));
		CALC_CRC(chk);
	}

	if(chk != expected_crc)
	{
		return 6;
	}

	unlock_flash();

	flash_erase_sector(bank, sector);


	if(bank == 1)
	{
		POLL_QW_BANK1();
		FLASH->CR1 |= 0b11UL<<4 /*64-bit parallelism*/ | 1UL<<1 /*misleadingly named write command*/;
	}
	else
	{
		POLL_QW_BANK2();
		FLASH->CR2 |= 0b11UL<<4 /*64-bit parallelism*/ | 1UL<<1 /*misleadingly named write command*/;
	}


	volatile uint64_t* p_flash = (uint64_t*)(FLASH_OFFSET + (bank*8+sector)*128*1024));
	volatile uint64_t* p_ram = (volatile uint64_t* volatile)(FLASHER_BUF_ADDR);

	for(int i=0; i<SECTOR_LEN/8; i++)
	{
		if(bank == 1)
			POLL_QW_BANK1();
		else
			POLL_QW_BANK2();

		*p_flash = *p_ram;
		p_flash++;
		p_ram++;
		__DSB(); __ISB();
	}

	if(bank == 1)
	{
		POLL_QW_BANK1();
		POLL_BSY_BANK1();
	}
	else
	{
		POLL_QW_BANK2();
		POLL_BSY_BANK2();
	}

	lock_flash();

	return 123; // The magical success code.
}

static void wait_nss_low()
{
	while(GPIOB->IDR & (1UL<<12));
}

static void wait_nss_high()
{
	while(!(GPIOB->IDR & (1UL<<12))) ;
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

