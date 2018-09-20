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
#include "misc.h"
#include "own_std.h"

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

#define FLASHER_BUF_ADDR 0x30000000  // SRAM1,SRAM2,SRAM3 form a contiguous 128K+128K+32K buffer
#define FLASHER_DMA_SIZE 65535

#define FLASHER_REPLY_BUF_SIZE 5
volatile uint8_t flasher_reply_buf[FLASHER_REPLY_BUF_SIZE] __attribute__((aligned(4)));


static void ena_tx_dma_for_reply() __attribute__((section(".text_itcm")));
static void ena_tx_dma_for_reply()
{

	DMA1_Stream0->PAR = (uint32_t)&(SPI1->TXDR);
	DMA1_Stream0->M0AR = (uint32_t)flasher_reply_buf;
	DMA1_Stream0->CR = 0b01UL<<16 /*med prio*/ | 0UL<<8 /*circular OFF*/ |
			   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b01UL<<6 /*mem-to-periph*/;
	DMA1_Stream0->NDTR = FLASHER_REPLY_BUF_SIZE;
	DMAMUX1_Channel0->CCR = 38;
	DMA_CLEAR_INTFLAGS(DMA1, 0);
	DMA1_Stream0->CR |= 1; // Enable TX DMA
	SPI1->CFG1 |= 1UL<<15 /*TX DMA ena*/;
}

static void ena_tx_dma_for_flash_read(int offset) __attribute__((section(".text_itcm")));
static void ena_tx_dma_for_flash_read(int offset)
{
	DMA1_Stream0->PAR = (uint32_t)&(SPI1->TXDR);
	DMA1_Stream0->M0AR = FLASH_OFFSET + offset; // FLASH on AXIM interface
	DMA1_Stream0->CR = 0b01UL<<16 /*med prio*/ | 0UL<<8 /*circular OFF*/ |
			   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b01UL<<6 /*mem-to-periph*/;
	DMA1_Stream0->NDTR = FLASHER_DMA_SIZE;
	DMAMUX1_Channel0->CCR = 38;
	DMA_CLEAR_INTFLAGS(DMA1, 0);
	DMA1_Stream0->CR |= 1; // Enable TX DMA
	SPI1->CFG1 |= 1UL<<15 /*TX DMA ena*/;
}

static void reset_spi(int rxbuf_offset) __attribute__((section(".text_itcm")));
static void reset_spi(int rxbuf_offset)
{
	SPI1->CR1 = 0; // Disable and reset the SPI - FIFO flush happens
	SPI1->CFG1 &= ~(1UL<<15); // Remove TX DMA
	__DSB();
	// Disable the DMAs:
	DMA1_Stream0->CR = 0b01UL<<16 /*med prio*/ | 0UL<<8 /*circular OFF*/ |
			   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b01UL<<6 /*mem-to-periph*/;
	DMA1_Stream1->CR = 0b01UL<<16 /*med prio*/ | 0UL<<8 /*circular OFF*/ |
			   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/;

	__DSB();
	while(DMA1_Stream0->CR & 1UL) ;
	while(DMA1_Stream1->CR & 1UL) ;

			
	// Re-enable:

	// RX DMA
	DMA1_Stream1->M0AR = FLASHER_BUF_ADDR+rxbuf_offset;

	DMA1_Stream1->NDTR = FLASHER_DMA_SIZE;
	DMA_CLEAR_INTFLAGS(DMA1, 1);
	DMA1_Stream1->CR |= 1; // Enable DMA

	SPI1->CR1 = 1UL; // Enable in slave mode
	__DSB();

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
//		char printbuf[128];

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

	unlock_flash(bank);

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

	__DSB();

	volatile uint64_t* p_flash = (uint64_t*)(FLASH_OFFSET + ((bank-1)*8+sector)*128*1024);
	volatile uint64_t* p_ram = (volatile uint64_t* volatile)(FLASHER_BUF_ADDR);

	for(int i=0; i<SECTOR_LEN/8; i++)
	{
		if(bank == 1)
			POLL_QW_BANK1();
		else
			POLL_QW_BANK2();

		*p_flash = *p_ram; //0x12345678abcdef12ULL;
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


//	uart_print_string_blocking("deep shit: "); o_utoa32_hex(*((uint32_t*)(FLASH_OFFSET + ((bank-1)*8+sector)*128*1024)), printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
//	uart_print_string_blocking("deep shit: "); o_utoa32_hex(*((uint32_t*)(4+FLASH_OFFSET + ((bank-1)*8+sector)*128*1024)), printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
//	uart_print_string_blocking("deep shit: "); o_utoa32_hex(*((uint32_t*)(1000+FLASH_OFFSET + ((bank-1)*8+sector)*128*1024)), printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");



	return 123; // The magical success code.
}

#ifdef SBC_ODROID
	#define wait_nss_low() do{while(GPIOA->IDR & (1UL<<15));}while(0)
	#define wait_nss_high() do{while(!(GPIOA->IDR & (1UL<<15)));}while(0)
#endif
#ifdef SBC_RASPI
	#define wait_nss_low() do{while(GPIOG->IDR & (1UL<<10));}while(0)
	#define wait_nss_high() do{while(!(GPIOG->IDR & (1UL<<10)));}while(0)
#endif

#ifdef SBC_AUTODETECT
	#error "Sorry, SBC_AUTODETECT mode not implemented in flasher"
#endif

void flasher() __attribute__((section(".text_itcm")));
void flasher()
{
	while(1)
	{
//		char printbuf[128];
		wait_nss_low();
		delay_us(1);
		wait_nss_high();
		int xfer_len = FLASHER_DMA_SIZE - DMA1_Stream1->NDTR; // let's see how many bytes were subtracted from the DMA remaining length value.
		__DSB();
		reset_spi(0);

		uint32_t cmd = *((volatile uint32_t*)FLASHER_BUF_ADDR);

		switch(cmd)
		{
			case 0xabba1337: // PROGRAM
			{
				if(xfer_len != 7)
				{
//					USART1->TDR = 'x';
					reset_spi(0);
					break;
				}
//				else
//					USART1->TDR = 'k';


				uint8_t bank = *((volatile uint8_t*)(FLASHER_BUF_ADDR+4));
				uint8_t sector = *((volatile uint8_t*)(FLASHER_BUF_ADDR+5));
				uint8_t expected_crc = *((volatile uint8_t*)(FLASHER_BUF_ADDR+6));

				int got_bytes = 0;
				while(got_bytes < SECTOR_LEN)
				{
					wait_nss_low();
					delay_us(1);
					wait_nss_high();
					__DSB();
					int this_len = FLASHER_DMA_SIZE - DMA1_Stream1->NDTR;
//					uart_print_string_blocking("SR="); o_btoa16_fixed(SPI1->SR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
//					uart_print_string_blocking("len = "); o_utoa16(this_len, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
					got_bytes += this_len;
					reset_spi(got_bytes);
				}


				LED_OFF();
				int ret = do_program(bank, sector, expected_crc);

//				uart_print_string_blocking("wrote to "); o_utoa16(bank, printbuf); uart_print_string_blocking(printbuf); o_utoa16(sector, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
//				uart_print_string_blocking("which is at "); o_utoa32_hex(FLASH_OFFSET + ((bank-1)*8+sector)*128*1024, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");


				LED_ON();
				// Done programming. Master will be polling. Wait for the on-going poll operation to end first.
				wait_nss_high();
				reset_spi(0);
				*((volatile uint32_t*)flasher_reply_buf) = 0xacdc3579;
				*((volatile uint8_t*)(flasher_reply_buf+4)) = ret;
				__DSB();
				ena_tx_dma_for_reply();
				wait_nss_low(); // DMA will do the job
				delay_us(10);
				wait_nss_high();
				reset_spi(0);
			}
			break;

			case 0xbeef1234: // READ
			{
				uint32_t offset = *((volatile uint32_t*)(FLASHER_BUF_ADDR+4));

				// After 8-byte long READ message, we can accept SPI clocks right away!

				ena_tx_dma_for_flash_read(offset);
				wait_nss_low(); // DMA will do the job
				delay_us(10);
				wait_nss_high();
				reset_spi(0);

//				uart_print_string_blocking("read from "); o_utoa32(offset, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
//				uart_print_string_blocking("which is at "); o_utoa32_hex(FLASH_OFFSET + offset, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");

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

	// Disable all other DMAs so they can't mess up our buffers
	DMA1_Stream0->CR = 0;
	DMA1_Stream1->CR = 0;
	DMA1_Stream2->CR = 0;
	DMA1_Stream3->CR = 0;
	DMA1_Stream4->CR = 0;
	DMA1_Stream5->CR = 0;
	DMA1_Stream6->CR = 0;
	DMA1_Stream7->CR = 0;
	DMA2_Stream0->CR = 0;
	DMA2_Stream1->CR = 0;
	DMA2_Stream2->CR = 0;
	DMA2_Stream3->CR = 0;
	DMA2_Stream4->CR = 0;
	DMA2_Stream5->CR = 0;
	DMA2_Stream6->CR = 0;
	DMA2_Stream7->CR = 0;

	RCC->APB2ENR |= 1UL<<12; // Make sure SPI1 is on
	__DSB();

	// Hard-reset SPI1, just to be sure.

	RCC->APB2RSTR = 1UL<<12;
	__DSB();
	delay();
	RCC->APB2RSTR = 0;
	__DSB();

	delay_ms(200); // At this time, master may still be sending some crap while flushing buffers. Hard-reset the SPI once more after this:

	RCC->APB2RSTR = 1UL<<12;
	__DSB();
	delay();
	RCC->APB2RSTR = 0;
	__DSB();

	// Reconf SPI
	
	SPI1->CFG1 = 1UL<<14 /*RX DMA EN*/ |(1UL -1UL)<<5 /*FIFO threshold*/ | 0b00111UL /*8-bit data*/;  // don't set TX DMA EN yet 

	// RX DMA

	DMA1_Stream1->PAR = (uint32_t)&(SPI1->RXDR);
	DMA1_Stream1->M0AR = FLASHER_BUF_ADDR;
	DMA1_Stream1->CR = 0b01UL<<16 /*med prio*/ | 0UL<<8 /*circular OFF*/ |
			   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/;
	DMA1_Stream1->NDTR = FLASHER_DMA_SIZE;
	DMAMUX1_Channel1->CCR = 37;
	DMA_CLEAR_INTFLAGS(DMA1, 1);
	DMA1_Stream1->CR |= 1; // Enable RX DMA


	SPI1->CR1 = 1UL; // Enable in slave mode

	// Remove all EXTI config. Use nCS manually.
	EXTI_D1->IMR1 = 0;
	EXTI_D1->IMR2 = 0;
	EXTI_D1->IMR3 = 0;

	LED_ON();
	flasher();
	while(1);
}

