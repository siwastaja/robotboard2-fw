#include <stdint.h>
#include <string.h>
#include "ext_include/stm32h7xx.h"
#include "stm32_cmsis_extension.h"
#include "misc.h"
#include "flash.h"
#include "own_std.h"

#define DEFINE_API_VARIABLES
#include "../robotsoft/api_board_to_soft.h"
#include "../robotsoft/api_soft_to_board.h"
#undef DEFINE_API_VARIABLES

#include "drive.h"
#include "ext_vacuum_boost.h"
#include "micronavi.h"

// 1 or 0:
#define CRC_EN 1

#ifdef SPI_DMA_1BYTE_AT_TIME
#define RX_DMA_NDTR (S2B_MAX_LEN)
#else
#define RX_DMA_NDTR (S2B_MAX_LEN/4)
#endif




// All pin mappings for sbc_comm are the same for REV2A,B



/*
define SPI_DMA_1BYTE_AT_TIME to force the SPI communication peripheral & DMA use 8-bit transfers. 
Bus bandwidth usage will be 4 times more, but transfer sizes not multiples of four will work
perfectly.

If not defined, 32-bit accessess will be used. For maximized simplicity, message sizes need to be
multiplies of four bytes, in this case. Nothing dramatic will happen with non-conforming messages,
but the reported rx length is rounded down %4. Overindexing of the buffers may happen if you are
not careful, in this case. For example, if the transfer length is 5, 8 bytes will be transferred
by the DMA.

*/
//#define SPI_DMA_1BYTE_AT_TIME

/*

Communication with the Single-Board Computer:

Using SPI:
There is a FIFO here, so that the SBC host software won't be timing critical (it'll run standard linux, and I
want to keep software development easier, so that some blocking 50ms processing loop there won't kill everything).

Due to the symmetry in SPI, there is always a concurrent TX packet sent to the SBC, and an RX packet from the SBC.

One packet contains a certain set of data structures. Which data, can be configured run-time. Due to both limited RAM,
and limited transfer speed on SPI, there is a (quite limiting) fixed max packet size, which limits the number
of concurrent subscribtions.

RX packets are usually shorter than TX packets; excess bits are ignored, the master runs the clock to get the TX.

FIFO works like this:
* Initially, wr and rd index are the same. No data can be read out.
* When the processing loop decides that the data is "ready" to be sent out, wr index is incremented (wrapped)
* When the master reads, if the wr and rd indeces are different, the slave is sent the data pointed to by rd.
* When the read is finished, the rd index is incremented. If wr==rd, the data output DMA is set to give a "no data avail" message.
* If the "processing finished" event detects that wr after increment would become == rd, this is an overrun condition. The overrun
  counter is incremented. The wr pointer is NOT incremented, so the previous packet is fully overwritten. Packets are always intact.


Peeking for status:
fifo is not incremented if the transfer length is 16 bytes or less. This way, status can be peeked at with no side effects.

Comment on nomenclature:

tx means data direction: MCU -> SBC
rx means data direction: SBC -> MCU

Both tx and rx always happen at the same time.

SBC is the master. Only the SBC can initialize the transfer.

_cpu counters refers to the actions of storing data (measurements, sensor data etc.) from the robot
_spi counters refers to the master's reading action.

RX and TX fifos are separate.

When TX FIFO overruns, data is not generated. Some kind of overrun flag will be used.

When RX FIFO overruns, this is considered catastrophic failure, and the robot stops is error().
This is because RX packets may include important commands (for example, stop the robot) and cannot be ignored.
Note that on the MCU, we control timing very well and can do RX fifo fetches as frequently as we want - processing
the commands shouldn't take a lot of time. Also, RX buffer are smaller so we can use a longer fifo.


*/

#define TX_FIFO_DEPTH  3
#define RX_FIFO_DEPTH  4

#if (B2S_MAX_LEN%8 != 0 || S2B_MAX_LEN%8 != 0)
	#error "SPI max transfer lengths must be multiples of 8"
#endif

static volatile uint8_t tx_fifo[TX_FIFO_DEPTH][B2S_MAX_LEN] __attribute__((aligned(8))) __attribute__((section(".sram1_bss")));
static volatile uint8_t rx_fifo[RX_FIFO_DEPTH][S2B_MAX_LEN] __attribute__((aligned(8))) __attribute__((section(".sram1_bss")));

static volatile int tx_fifo_cpu = 0;
static volatile int tx_fifo_spi = 0;

static volatile int rx_fifo_cpu = 0;
static volatile int rx_fifo_spi = 0;

// Enabled subscriptions, 1 bit per message ID, [0] LSb = id0, [0] MSb = id63, [1] LSb = id64, and so on:
uint64_t subs[B2S_SUBS_U64_ITEMS];

uint64_t requested_subs_copy[B2S_SUBS_U64_ITEMS];

int stored_offs;
void update_subs(uint64_t *subs_vector)
{
	for(int i=0; i<B2S_MAX_MSGIDS; i++)
	{
		if (b2s_msgs[i].p_accessor != NULL)
		{
			*(b2s_msgs[i].p_accessor) = NULL;
		}
	}


	int offs = MSGS_START_OFFSET;

	for(int i=0; i<B2S_SUBS_U64_ITEMS; i++)
	{
		subs[i] = 0;
		uint64_t t = subs_vector[i];
		for(int s=i*64; s<(i+1)*64; s++)
		{
			if(t & 1)
			{
				// enable id #s
				if(!b2s_msgs[s].p_accessor)
				{
					// Trying to add a non-existing message
					// TODO: error recovery & reporting
					SAFETY_SHUTDOWN();
					DBG_PR_VAR_I32(i);
					DBG_PR_VAR_I32(s);
					error(21); 
				}				
				if(offs + b2s_msgs[s].size > B2S_MAX_LEN-FOOTER_LEN)
				{
					// requested subscription doesn't fit: stop adding subscriptions.
					goto BREAK_SUB_ADD;
				}

				subs[i] |= 1ULL<<(s-i*64);
				*(b2s_msgs[s].p_accessor) = tx_fifo[tx_fifo_cpu] + offs;
				offs += b2s_msgs[s].size;
			}
			t >>= 1;
		}
	}
	BREAK_SUB_ADD:;

	stored_offs = offs;
/*
	for(int i=0; i<TX_FIFO_DEPTH; i++)
	{
		// When the subscription are changed, write the list of subscriptions permanently on all buffers
		// update_subs() is allowed to be slower than tx_fifo_push:		
		for(int o=0; o<B2S_SUBS_U64_ITEMS; o++)
			*(uint64_t*)&tx_fifo[i][SUBLIST_START_OFFSET+o*8] = subs[o];

		// Similarly, write the payload len:
			((b2s_header_t*)&tx_fifo[i][0])->payload_len = offs-MSGS_START_OFFSET;

		// Write the footer, which stays permanently on the buffers, until new update_subs()
		for(int o=0; o<FOOTER_LEN; o++)
		{
			tx_fifo[i][offs+o] = 0xee;
		}
	}
*/
}

void remove_sub(int idx)
{
	uint64_t copy[2];
	copy[0] = subs[0];
	copy[1] = subs[1];

	if(idx<64)
		copy[0] &= ~(1ULL<<idx);
	else
		copy[1] &= ~(1ULL<<(idx-64));

	update_subs(copy);
}

void add_sub(int idx)
{
	uint64_t copy[2];
	copy[0] = subs[0];
	copy[1] = subs[1];

	if(idx<64)
		copy[0] |= (1ULL<<idx);
	else
		copy[1] |= (1ULL<<(idx-64));

	update_subs(copy);
}

void restore_subs()
{
	update_subs(requested_subs_copy);
}

static void init_tx_fifo_fixed_content()
{
	for(int i=0; i<TX_FIFO_DEPTH; i++)
	{
		((b2s_header_t*)&tx_fifo[i][0])->magic = 0xabcd;
	}

}

int is_tx_overrun()
{
	int next_tx_fifo_cpu = tx_fifo_cpu+1; if(next_tx_fifo_cpu >= TX_FIFO_DEPTH) next_tx_fifo_cpu = 0;

	return (next_tx_fifo_cpu == tx_fifo_spi);
}

void flag_err()
{
	for(int i=0; i<TX_FIFO_DEPTH; i++)
		((b2s_header_t*)&tx_fifo[i][0])->err_flags = 1;
}

void clear_err()
{
	for(int i=0; i<TX_FIFO_DEPTH; i++)
		((b2s_header_t*)&tx_fifo[i][0])->err_flags = 0;
}


void tx_fifo_push()
{
	for(int o=0; o<B2S_SUBS_U64_ITEMS; o++)
		*(uint64_t*)&tx_fifo[tx_fifo_cpu][SUBLIST_START_OFFSET+o*8] = subs[o];

	// Similarly, write the payload len:
	((b2s_header_t*)&tx_fifo[tx_fifo_cpu][0])->payload_len = stored_offs-MSGS_START_OFFSET;

	// Write the footer,
	for(int o=0; o<FOOTER_LEN; o++)
		tx_fifo[tx_fifo_cpu][stored_offs+o] = 0xee;


	((b2s_header_t*)&tx_fifo[tx_fifo_cpu][0])->fifo_status = 0;


	// Temporarily disable all accessor pointers, to avoid race conditions
	// Data generated in interrupt handlers see these subscriptions are "off" for a short time
	for(int i=0; i<B2S_MAX_MSGIDS; i++)
	{
		if (b2s_msgs[i].p_accessor != NULL)
		{
			*(b2s_msgs[i].p_accessor) = NULL;
		}
	}


	__DSB();
	DIS_IRQ();
	tx_fifo_cpu++; if(tx_fifo_cpu >= TX_FIFO_DEPTH) tx_fifo_cpu = 0;
	ENA_IRQ();

	int total_size = 0;
	for(int i=0; i<B2S_SUBS_U64_ITEMS; i++)
	{
		uint64_t t = subs[i];
		for(int s=i*64; s<(i+1)*64; s++)
		{
			if (t & 1)     // id #s is enabled
			{
				total_size += b2s_msgs[s].size;
			}
			t >>= 1;
		}
	}

	// Clear the data, so that if some module does not write anything, old data is not retained
	memset(&tx_fifo[tx_fifo_cpu][MSGS_START_OFFSET], 0, total_size);


	// Increased tx_fifo_cpu tells the SPI procedure that the data won't change anymore, and is OK to send.
	// Having an SPI interrupt at this point in time is OK (and optimal).

	// Now, let's make the active data pointers to point to the next TX FIFO block.
	// This is basically the same offset calculation as in update_subs, but no need to check if the requests fit,
	// because it's already checked (and subs item zeroed out if necessary).

	int offs = MSGS_START_OFFSET;

	for(int i=0; i<B2S_SUBS_U64_ITEMS; i++)
	{
		uint64_t t = subs[i];
		for(int s=i*64; s<(i+1)*64; s++)
		{
			if (t & 1)     // id #s is enabled
			{
				*(b2s_msgs[s].p_accessor) = tx_fifo[tx_fifo_cpu] + offs;
				offs += b2s_msgs[s].size;
			}
			t >>= 1;
		}
	}


}

// Doesn't work properly in corner case timing, avoid using:
void flush_fifos()
{
	DIS_IRQ();
	__DSB();
	tx_fifo_cpu = tx_fifo_spi;
	rx_fifo_cpu = rx_fifo_spi;
	__DSB();
	ENA_IRQ();
}

void block_until_tx_fifo_empty()
{
	while(tx_fifo_cpu != tx_fifo_spi) ;
}


enum {RASPI=0, ODROID=1} sbc_iface;

#if !defined(SBC_RASPI) && !defined(SBC_ODROID) && !defined(SBC_AUTODETECT)
	#error "You need to define SBC_RASPI, SBC_ODROID or SBC_AUTODETECT"
#endif

#if defined(SBC_RASPI) && defined(SBC_ODROID)
	#error "Define either SBC_RASPI or SBC_ODROID, not both"
#endif

#if defined(SBC_AUTODETECT) && (defined(SBC_RASPI) || defined(SBC_ODROID))
	#error "With SBC_AUTODETECT defined, don't define SBC_RASPI or SBC_ODROID"
#endif

#ifdef SBC_AUTODETECT
static void wait_detect_sbc()
{
	// Chip select signals are pulled down:
	IO_TO_GPI(GPIOG, 10);
	IO_TO_GPI(GPIOA, 15);
	IO_PULLDOWN_ON(GPIOG, 10);
	IO_PULLDOWN_ON(GPIOA, 15);

	// Then, we'll see which one is pulled high (inactive level) by the connected single-board computer:

	int blink_cnt = 0;
	while(1)
	{
		delay_ms(1);
		blink_cnt++;
		if(blink_cnt == 1)
			LED_ON();
		else if(blink_cnt == 100)
			LED_OFF();
		else if(blink_cnt == 200)
			blink_cnt = 0;

		int raspi_high = IN(GPIOG, 10);
		int odroid_high = IN(GPIOA, 15);

		if(raspi_high && !odroid_high)
		{
			sbc_iface = RASPI;
			break;
		}
		else if(odroid_high && !raspi_high)
		{
			sbc_iface = ODROID;
			break;
		}
		// else: keep waiting for valid situation (only either one high)
	}

	LED_OFF();

	IO_PULLUPDOWN_OFF(GPIOG, 10);
	IO_PULLUPDOWN_OFF(GPIOA, 15);
}
#endif

/*
	Okay, so STM32 SPI _still_ doesn't _actually_ support nSS pin hardware management in slave mode, even when they are talking about
	it like that.

	This was the case with the previous incarnation of the SPI - now they have made it more complex, added a separate event/interrupt
	source exactly to signify end-of-transfer in slave mode as well - but for some reason, they still don't delimit the transfer by nCS
	(which is exactly designed for this purpose!), go figure. Instead, you need to know the transfer size beforehand, and a counter is
	used to give the interrupt. Of course, with our variable-length messaging, this isn't going to work.

	This is why we still use a simple EXTI interrupt from the nCS pin going high, and we still need to circumvent the whole SPI/DMA transfer
	management.

	They have fixed something - now you don't need to reset the SPI through the RCC registers anymore. Disabling SPI through the SPE bit
	flushes the FIFOs and resets the internal logic. Great!
	

	An addition:
	This is of course no surprise, but the SPI peripheral is still broken, and STILL requires the reset through the RCC registers,
	if CRC is used. Problem description:
	* Configure RX DMA for a certain length (ex., 6000 bytes or NDTR=1500 with word access)
	* Reveice a transfer which is actually longer, so that the DMA shuts down and overrun flag is set
	* Normally all this is OK - reset the SPI by disabling it (SPIEN = 0), clear all intflags and reconfing...
	* But, if you have CRCEN enabled - the SPI internally ends up in an unspecified fault state, which is not seen in any status flags.
	* Any further RX DMA operations fail right at the start - when you config RX DMA channel and enable it, it instantly sets FIFO ERROR,
	  half transfer complete, and transfer complete flags, and set NDTR to 0.
	* This internal fault state can only be resolved with RCC reset.



	Note:
	When the SPI is enabled with DMA, it always generates DMA requests right away to fill its TX fifo. When the
	nSS is asserted half a year later, the first data are from half a year ago, rest is DMA'ed from the memory at that point.
	With our communication FIFO scheme, this isn't a problem, but do remember this.


*/

volatile int new_rx;
volatile int new_rx_len;

volatile int spi_dbg1, spi_dbg2;

#ifdef SPI_DMA_1BYTE_AT_TIME
#define SPI_CFG1 (1UL<<14 /*RX DMA EN*/ | (1UL -1UL)<<5 /*FIFO threshold*/ | 0b00111UL /*8-bit data*/ | \
	             CRC_EN<<22 | (8UL  -1UL)<<16 /*CRC size*/) // don't set TX DMA EN yet
#else
#define SPI_CFG1 (1UL<<14 /*RX DMA EN*/ | (4UL -1UL)<<5 /*FIFO threshold*/ | 0b00111UL /*8-bit data*/ | \
	             CRC_EN<<22 | (8UL  -1UL)<<16 /*CRC size*/)  // don't set TX DMA EN yet
#endif


void sbc_spi_cs_end_inthandler()
{
	static int prev_tsize;
//	spi_dbg1 = DMA1_Stream0->NDTR;
//	spi_dbg2 = DMA1_Stream1->NDTR;
	// Triggered when cs goes high
//	uart_print_string_blocking("\r\n--");

/*
	uart_print_string_blocking("\r\nSTART\r\n");
	if(DMA1_Stream0->CR & 1UL) uart_print_string_blocking("TX DMA  ");
	if(DMA1_Stream1->CR & 1UL) uart_print_string_blocking("RX DMA  ");
	uart_print_string_blocking("\r\nCR = "); o_btoa16_fixed(SPI1->CR1&0xffff, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("SR = "); o_btoa16_fixed(SPI1->SR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("CTSIZE = "); o_utoa16_fixed(SPI1->SR>>16, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("TXDMA = "); o_btoa8_fixed(DMA_INTFLAGS(DMA1, 0), printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("RXDMA = "); o_btoa8_fixed(DMA_INTFLAGS(DMA1, 1), printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("TXNDTR = "); o_utoa16(DMA1_Stream0->NDTR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("RXNDTR = "); o_utoa16(DMA1_Stream1->NDTR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
*/
	// Clear the EXTI interrupt pending bit:
	if(sbc_iface == ODROID)
		EXTI_D1->PR1 = 1UL<<15;
	else // RASPI
		EXTI_D1->PR1 = 1UL<<10;

//	SPI1->CR1 = 0; // Disable and reset the SPI - FIFO flush happens 

	int crc_err = 0;
	if(SPI1->SR & (1UL<<7))
	{
		crc_err = 1;
	}

//	uint32_t rxcrc = SPI1->RXCRC;


	// Silicon Errata: Hard reset is needed if CRC is used and if RX OVR happens. Otherwise, all
	// further DMA operations fail right at the start.

	RCC->APB2RSTR = 1UL<<12;
	__DSB();
	RCC->APB2RSTR = 0;
	__DSB();


	int tx_dma_was_enabled = DMA1_Stream0->CR & 1UL;

	__DMB();


	// Reconf SPI after reset

	SPI1->CFG1 = SPI_CFG1;

	// Disable the DMAs (but keep the configuration):
	DMA1_Stream0->CR = 0b01UL<<16 /*med prio*/ | 0UL<<8 /*circular OFF*/ |
		#ifdef SPI_DMA_1BYTE_AT_TIME
	                   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
		#else
	                   0b10UL<<13 /*32-bit mem*/ | 0b10UL<<11 /*32-bit periph*/ |
		#endif
	                   1UL<<10 /*mem increment*/ | 0b01UL<<6 /*mem-to-periph*/;

	DMA1_Stream1->CR = 0b01UL<<16 /*med prio*/ | 0UL<<8 /*circular OFF*/ |
		#ifdef SPI_DMA_1BYTE_AT_TIME
	                   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
		#else
	                   0b10UL<<13 /*32-bit mem*/ | 0b10UL<<11 /*32-bit periph*/ |
		#endif
	                   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/;

	__DSB();

	// Poll DMA enable bits to make sure they are disabled before re-enabling them:
	while(DMA1_Stream0->CR & 1UL) ;
	while(DMA1_Stream1->CR & 1UL) ;

	new_rx = 1;

	#ifdef SPI_DMA_1BYTE_AT_TIME
	int len = S2B_MAX_LEN - DMA1_Stream1->NDTR;
	#else
	int len = S2B_MAX_LEN - DMA1_Stream1->NDTR*4;
	#endif
/*
	uart_print_string_blocking("\r\nMID\r\n");
	if(DMA1_Stream0->CR & 1UL) uart_print_string_blocking("TX DMA  ");
	if(DMA1_Stream1->CR & 1UL) uart_print_string_blocking("RX DMA  ");
	uart_print_string_blocking("\r\nCR = "); o_btoa16_fixed(SPI1->CR1&0xffff, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("SR = "); o_btoa16_fixed(SPI1->SR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("CTSIZE = "); o_utoa16_fixed(SPI1->SR>>16, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("TXDMA = "); o_btoa8_fixed(DMA_INTFLAGS(DMA1, 0), printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("RXDMA = "); o_btoa8_fixed(DMA_INTFLAGS(DMA1, 1), printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("TXNDTR = "); o_utoa16(DMA1_Stream0->NDTR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("RXNDTR = "); o_utoa16(DMA1_Stream1->NDTR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("len = "); o_utoa16(len, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("was_ena = "); o_utoa16(tx_dma_was_enabled, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
*/

//	uart_print_string_blocking("len = "); o_utoa16(len, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");

	// This is not needed, RCC reset clears this as well:
//	SPI1->IFCR = 0b111111111000; // TRAP: Clear EOT and TXTF flags, otherwise the TX DMA won't start if TSIZE is ever non-zero, causing EOT to be high.


	// Check if there is the maintenance magic code:
	if(*((volatile uint32_t*)&rx_fifo[rx_fifo_spi]) == 0x9876fedb)
	{
//		uart_print_string_blocking("flasher!\r\n");
		run_flasher();
	}

	if(len >= 16)
	{
		if(tx_dma_was_enabled) // data was sent out
		{
			tx_fifo_spi++;
			if(tx_fifo_spi >= TX_FIFO_DEPTH) tx_fifo_spi = 0;
		}
	}

	// Process RX FIFO and check for errors here, so we can flag the next TX packet.
	if(len >= 16 && ((s2b_header_t*)rx_fifo[rx_fifo_spi])->magic == 0x2345)
	{
		// CRC calculation in the SPI peripheral only works correctly when our tx is of known size, and longer than our rx
		// (i.e., the normal case). Prevent false positives here. The caveat is, we lose the protection of CRC for incoming
		// commands when there are little or no subscriptions turned on.
		if(prev_tsize > len && crc_err)
		{
			uart_print_string_blocking("\r\nCRC_ERR\r\n");
//			uart_print_string_blocking("\r\nRXCRC = 0x"); o_utoa32_hex(rxcrc, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
			flag_err();
		}
		else
		{
			int next_rx_fifo_spi = rx_fifo_spi+1;
			if(next_rx_fifo_spi >= RX_FIFO_DEPTH)
				next_rx_fifo_spi = 0;

			if(next_rx_fifo_spi == rx_fifo_cpu)
			{
				// RX fifo full - overrun. Considered catastrophic condition.
				error(17);
			}
			else
			{
				rx_fifo_spi = next_rx_fifo_spi;
			}
		}
	}
	else
	{
		// Just re-enable the DMA so the next operation writes to the same place.
	}	

	// TX DMA for the next transfer
	if(tx_fifo_cpu == tx_fifo_spi)
	{
		// No data to send.
		// Don't enable TX DMA, let the SPI send the content of the "underrun register"
		#if CRC_EN == 1
			SPI1->TSIZE = 0;
			prev_tsize = 0;
		#endif
	}
	else
	{
		if(tx_dma_was_enabled)
		{
			int nextnext = tx_fifo_spi+1; if(nextnext >= TX_FIFO_DEPTH) nextnext = 0;

			// If there are even more packets to send after the next transaction, indicate this
			// by setting a bit in fifo_status register, but only if the sizes match (so that it
			// can be read out with the same length transaction)
			if(nextnext != tx_fifo_cpu && 
			  ((b2s_header_t*)&tx_fifo[tx_fifo_spi][0])->payload_len == ((b2s_header_t*)&tx_fifo[nextnext][0])->payload_len)
			{
				((b2s_header_t*)&tx_fifo[tx_fifo_spi][0])->fifo_status |= 1;
			}
		}

		DMA1_Stream0->M0AR = (uint32_t)tx_fifo[tx_fifo_spi];
		// TSIZE tells the SPI peripheral when to insert the CRC byte:
		#if CRC_EN == 1
			SPI1->TSIZE = prev_tsize = ((b2s_header_t*)&tx_fifo[tx_fifo_spi][0])->payload_len + sizeof(b2s_header_t) + FOOTER_LEN;
		#endif
		__DSB();
		DMA_CLEAR_INTFLAGS(DMA1, 0);
		DMA1_Stream0->CR |= 1; // Enable TX DMA
		SPI1->CFG1 |= 1UL<<15 /*TX DMA ena*/; // not earlier, if you want to follow the refman advice
	}


	// RX DMA

	DMA1_Stream1->NDTR = RX_DMA_NDTR;
	DMA1_Stream1->M0AR = (uint32_t)rx_fifo[rx_fifo_spi];

	__DSB();
	DMA_CLEAR_INTFLAGS(DMA1, 1);
	DMA1_Stream1->CR |= 1; // Enable RX DMA
	__DSB();


	SPI1->CR1 = 1UL; // Enable in slave mode
	__DSB();

/*
	uart_print_string_blocking("\r\nEND\r\n");
	if(DMA1_Stream0->CR & 1UL) uart_print_string_blocking("TX DMA  ");
	if(DMA1_Stream1->CR & 1UL) uart_print_string_blocking("RX DMA  ");
	uart_print_string_blocking("\r\nCR = "); o_btoa16_fixed(SPI1->CR1&0xffff, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("SR = "); o_btoa16_fixed(SPI1->SR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("CTSIZE = "); o_utoa16_fixed(SPI1->SR>>16, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("TXDMA = "); o_btoa8_fixed(DMA_INTFLAGS(DMA1, 0), printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("RXDMA = "); o_btoa8_fixed(DMA_INTFLAGS(DMA1, 1), printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("TXNDTR = "); o_utoa16(DMA1_Stream0->NDTR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("RXNDTR = "); o_utoa16(DMA1_Stream1->NDTR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
	uart_print_string_blocking("len = "); o_utoa16(len, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n\r\n");
*/

}

void parse_rx_packet()
{
	s2b_header_t *p_header = (s2b_header_t*)rx_fifo[rx_fifo_cpu];

	int offs = S2B_HEADER_LEN;
//	uart_print_string_blocking("PARSE: n_cmds = "); o_utoa16(p_header->n_cmds, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");

	for(int p = 0; p < p_header->n_cmds; p++)
	{
		s2b_cmdheader_t *p_cmdheader = (s2b_cmdheader_t*)&rx_fifo[rx_fifo_cpu][offs];
		uint8_t *p_data = (uint8_t*)&rx_fifo[rx_fifo_cpu][offs+S2B_CMDHEADER_LEN];

//		uart_print_string_blocking("PARSE: paylen = "); o_utoa16(p_cmdheader->paylen, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
//		uart_print_string_blocking("PARSE: msgid = "); o_utoa16(p_cmdheader->msgid, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");

//		for(int i=0; i<42; i++)
//		{
//			o_utoa8_hex(rx_fifo[rx_fifo_cpu][i], printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking(" ");
//		}
//		 uart_print_string_blocking("\r\n");
		if(offs + p_cmdheader->paylen >= S2B_MAX_LEN)
		{
			// Message would overflow
			// Todo: raise error flag instead
			error(6);
			break;
		}


		if(p_cmdheader->paylen != s2b_msgs[p_cmdheader->msgid].size)
		{
			// Inconsistency in API (or data corruption) - size field doens't match the expected length of the message type.
			// Todo: raise error flag instead
			error(7);
			break;
		}

//		for(int i=0; i<p_cmdheader->paylen; i++)
//		{
//			o_utoa8_hex(p_data[i], printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking(" ");
//		}

		// TODO: If you want to be helpful and reduce amount of code, add a function pointer to api_board_to_soft.h s2b_message_t, defined in
		// S2B_MESSAGE_STRUCT. Make that macro define the extern function so that no header is need to be included. Functions should be of the
		// standard type always with one argument of the message type and void return value. Then, instead of this massive switch-case, check
		// if that function pointer is non-zero, automagically call it with (correct_type*)p_data
		// (just like most of the cases are doing right now). Fix those few message types like CMD_MOVE_REL which separately supply
		// arguments to the function they call, to use the message struct as-is. Parse everything inside those handler functions.

		switch(p_cmdheader->msgid)
		{
			case CMD_SUBSCRIBE:
			{
				memcpy(requested_subs_copy, p_data, sizeof(requested_subs_copy));
				update_subs(requested_subs_copy);
			}
			break;

			case CMD_ACK_ERROR:
			{
				clear_err();
			}
			break;

			#ifdef CALIBRATOR
				case CMD_CALIBRATION:
				{
					uart_print_string_blocking("PARSE: calibrator cmd\r\n");
					extern void calibrator_cmd_in(s2b_calibration_t*);
					calibrator_cmd_in((s2b_calibration_t*)p_data);
				}
				break;
			
			#else

				case CMD_MOVE_REL:
				{
					s2b_move_rel_t* m = (s2b_move_rel_t*)p_data;
					rotate_and_straight_rel(m->ang, m->fwd, 1);
				} break;

				case CMD_MOVE_ABS:
				{
					cmd_go_to((s2b_move_abs_t*)p_data);
				}
				break;

				case CMD_MOTORS:
				{
					if(((s2b_motors_t*)p_data)->enabled)
						cmd_motors(7000); // milliseconds watchdog
					else
						cmd_motors(0);
				}
				break;


				case CMD_CORR_POS:
				{
					cmd_corr_pos((s2b_corr_pos_t*)p_data);
				}
				break;

				case CMD_STOP_MOVEMENT:
				{
					cmd_stop_movement();
				}
				break;

				#ifdef EXT_VACUUM
					case CMD_EXT_VACUUM:
					{
						ext_vacuum_cmd(((s2b_ext_vacuum_t*)p_data)->power, ((s2b_ext_vacuum_t*)p_data)->nozzle);
					}
					break;
				#endif

				case CMD_MOUNT_CHARGER:
				{
					find_charger();
				}
				break;


				case CMD_SELF_CALIB_REQUEST:
				{
					self_calib((s2b_self_calib_request_t*)p_data);
				}
				break;

				case CMD_INJECT_GYROCAL:
				{
					drive_inject_gyrocal(&(((s2b_inject_gyrocal_t*)p_data)->gyrocal));
				}
				break;

				case CMD_MANUAL_DRIVE:
				{
					drive_manual_drive((s2b_manual_drive_t*)p_data);
				}
				break;
/*
				case CMD_SET_POSE:
				{
					cmd_set_pose((s2b_set_pose_t*)p_data);
				}
				break;
*/
			#endif

			
			default:
			{

			}
			break;
		}

		offs += p_cmdheader->paylen + S2B_CMDHEADER_LEN;
	}
	
}

void check_rx()
{
	if(rx_fifo_spi != rx_fifo_cpu)
	{
//		uart_print_string_blocking("rx pop:"); o_utoa32_hex(*(uint32_t*)rx_fifo[rx_fifo_cpu], printbuf); uart_print_string_blocking(printbuf); 
//		o_utoa32_hex(*(uint32_t*)&rx_fifo[rx_fifo_cpu][4], printbuf); uart_print_string_blocking(printbuf); 
//		o_utoa32_hex(*(uint32_t*)&rx_fifo[rx_fifo_cpu][8], printbuf); uart_print_string_blocking(printbuf);  uart_print_string_blocking("...\r\n");

		parse_rx_packet();

		DIS_IRQ();
		rx_fifo_cpu++; if(rx_fifo_cpu >= RX_FIFO_DEPTH) rx_fifo_cpu = 0;
		ENA_IRQ();
	}
	__DSB();
}

#if 0
void sbc_comm_test()
{
	uint8_t data = 0x42;
	int cnt = 0;

	int delays[13] = {50, 20, 70, 4, 17, 200, 40, 4, 15, 45, 4, 4, 4};
	while(1)
	{
//		delay_ms(delays[cnt]);
		for(int i = 0; i<1; i++)
		{
			delay_ms(1000);
			check_rx();
		}

		cnt++;
//		if(cnt > 12) cnt = 0;


		if(cnt==5)
		{
			cnt=0;

			int next_tx_fifo_cpu = tx_fifo_cpu+1; if(next_tx_fifo_cpu >= TX_FIFO_DEPTH) next_tx_fifo_cpu = 0;

			if(next_tx_fifo_cpu == tx_fifo_spi)
			{
				uart_print_string_blocking("\r\nTX buffer overrun! Not generating data\r\n"); 
			}
			else
			{
				data++;

				tx_fifo[next_tx_fifo_cpu][0] = 0x11;
				tx_fifo[next_tx_fifo_cpu][1] = tx_fifo_cpu;
				tx_fifo[next_tx_fifo_cpu][2] = tx_fifo_spi;
				tx_fifo[next_tx_fifo_cpu][3] = rx_fifo_cpu;
				tx_fifo[next_tx_fifo_cpu][4] = rx_fifo_spi;
				for(int i=5; i<64; i++)
					tx_fifo[next_tx_fifo_cpu][i] = data;


				DIS_IRQ();
				tx_fifo_cpu = next_tx_fifo_cpu;

				ENA_IRQ();

				uart_print_string_blocking("\r\nNew tx generated\r\n"); 
			}
		}

		uart_print_string_blocking("tx_cpu="); o_utoa16(tx_fifo_cpu, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("  ");
		uart_print_string_blocking("tx_spi="); o_utoa16(tx_fifo_spi, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("  ");
		uart_print_string_blocking("rx_cpu="); o_utoa16(rx_fifo_cpu, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("  ");
		uart_print_string_blocking("rx_spi="); o_utoa16(rx_fifo_spi, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
		
	}
}
#endif

uint64_t initial_subs[B2S_SUBS_U64_ITEMS] = {1UL<<4, 0};

uint64_t subs_test1[4] = {0b1110,0,0,0};
uint64_t subs_test2[4] = {0b1010,0,0,0};
uint64_t subs_test3[4] = {0b0000,0,0,0};
uint64_t subs_test4[4] = {0b0100,0,0,0};

static void gen_some_data1()
{
	static int cnt = 0;

	if(!test_msg1)
	{
		uart_print_string_blocking("\r\nmsg1 generation turned off\r\n"); 
	}
	else
	{
		uart_print_string_blocking("\r\nGenerating data1, pointer="); 
		o_utoa32_hex((uint32_t)test_msg1, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");

		test_msg1->a = 0xacdcabba;
		test_msg1->b = cnt;
		test_msg1->c = 0x55;
		test_msg1->d = cnt;
	}
	cnt++;
}

static void gen_some_data2()
{
	static int cnt = 0;
	if(!test_msg2)
	{
		uart_print_string_blocking("\r\nmsg2 generation turned off\r\n"); 
	}
	else
	{
		uart_print_string_blocking("\r\nGenerating data2, pointer="); 
		o_utoa32_hex((uint32_t)test_msg2, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
		for(int i=0; i<sizeof(test_msg2->buf); i++)
			test_msg2->buf[i] = cnt&0xff;
	}
	cnt++;
}

static void gen_some_data3()
{
	static int cnt = 0;
	if(!test_msg3)
	{
		uart_print_string_blocking("\r\nmsg3 generation turned off\r\n"); 
	}
	else
	{
		uart_print_string_blocking("\r\nGenerating data3, pointer="); 
		o_utoa32_hex((uint32_t)test_msg3, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");

		for(int i=0; i<sizeof(test_msg3->buf)-2; i+=2)
		{
			test_msg3->buf[i] = cnt&0xff;
			test_msg3->buf[i+1] = 0x88;
		}
	}
	cnt++;
}

void pointer_system_test()
{
	int cnt=0;
	uint8_t cnt_u8 = 0;
	while(1)
	{
		for(int i=0; i<10; i++)
		{
			uart_print_string_blocking("!");
			delay_ms(100);
//			check_rx();
		}

		if(is_tx_overrun())
		{
			uart_print_string_blocking("\r\nTX buffer overrun! Skipping data generation.\r\n"); 
		}
		else
		{
//			tx_fifo[tx_fifo_cpu][3] = cnt_u8;
			gen_some_data1();
			gen_some_data2();
			gen_some_data3();


			uart_print_string_blocking("\r\nPush!\r\n"); 

			tx_fifo_push();
/*			if(cnt == 5)
				update_subs(subs_test1);
			if(cnt == 10)
				update_subs(subs_test2);
			if(cnt == 15)
				update_subs(subs_test3);
			if(cnt == 20)
				update_subs(subs_test4);
*/

			cnt_u8++;
		}

		check_rx();

		cnt++;
	}


}

void init_sbc_comm()
{

	update_subs(initial_subs);

	RCC->APB2ENR |= 1UL<<12; // SPI1

	#ifdef SBC_AUTODETECT
		wait_detect_sbc();
	#endif

	#ifdef SBC_ODROID
		sbc_iface = ODROID;
	#endif

	#ifdef SBC_RASPI
		sbc_iface = RASPI;
	#endif

	if(sbc_iface == ODROID)
	{
		IO_ALTFUNC(GPIOB,5,  5);
		IO_ALTFUNC(GPIOB,4,  5);
		IO_ALTFUNC(GPIOB,3,  5);
		IO_ALTFUNC(GPIOA,15, 5);
		IO_SPEED(GPIOB,4, 3);

		IO_PULLUP_ON(GPIOA, 15);
	}
	else // RASPI
	{
		IO_ALTFUNC(GPIOG,9,  5);
		IO_ALTFUNC(GPIOD,7,  5);
		IO_ALTFUNC(GPIOG,11, 5);
		IO_ALTFUNC(GPIOG,10, 5);
		IO_SPEED(GPIOG,9, 3);

		IO_PULLUP_ON(GPIOG, 10);
	}

	init_tx_fifo_fixed_content();

	// Initialization order from reference manual:
	SPI1->CFG1 = SPI_CFG1;

	SPI1->UDRDR = 0;
	SPI1->CFG2 = 1UL<<31 /*Keep pins in control while disabling the SPI for FIFO flush - prevent MISO from glitching*/;
	// SPI1->CRCPOLY 0x100|0x07 by default reset value.

	// SPI1->CR2 zero - transfer length unknown

	// TX DMA - keep it off, but configure what we can.
	DMA1_Stream0->PAR = (uint32_t)&(SPI1->TXDR);
	DMA1_Stream0->CR = 0b01UL<<16 /*med prio*/ | 0UL<<8 /*circular OFF*/ |
		#ifdef SPI_DMA_1BYTE_AT_TIME
	                   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
		#else
	                   0b10UL<<13 /*32-bit mem*/ | 0b10UL<<11 /*32-bit periph*/ |
		#endif
	                   1UL<<10 /*mem increment*/ | 0b01UL<<6 /*mem-to-periph*/;

	#ifdef SPI_DMA_1BYTE_AT_TIME
		DMA1_Stream0->NDTR = B2S_MAX_LEN;
	#else
		DMA1_Stream0->NDTR = B2S_MAX_LEN/4;
	#endif
	DMAMUX1_Channel0->CCR = 38;

	// RX DMA

	DMA1_Stream1->PAR = (uint32_t)&(SPI1->RXDR);
	DMA1_Stream1->M0AR = DMA1_Stream1->M0AR = (uint32_t)rx_fifo[rx_fifo_spi];
	DMA1_Stream1->CR = 0b01UL<<16 /*med prio*/ | 0UL<<8 /*circular OFF*/ |
		#ifdef SPI_DMA_1BYTE_AT_TIME
	                   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
		#else
	                   0b10UL<<13 /*32-bit mem*/ | 0b10UL<<11 /*32-bit periph*/ |
		#endif
	                   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/;

	DMA1_Stream1->NDTR = RX_DMA_NDTR;

	DMAMUX1_Channel1->CCR = 37;
	DMA_CLEAR_INTFLAGS(DMA1, 1);
	DMA1_Stream1->CR |= 1; // Enable RX DMA
	__DSB();
	SPI1->CR1 = 1UL; // Enable in slave mode


	// Chip select is hardware managed for rx start - but ending must be handled by software. We use EXTI for that.
	// Maybe we won't use nCS to delimit the packet end, since it clearly isn't supported properly in the HW
	// For now, let's not use EXTI. The code is here, anyway:

	if(sbc_iface == ODROID)
	{
		// nCS is PA15, so EXTI15 must be used.
		REG_WRITE_PART(SYSCFG->EXTICR[/*refman idx:*/4   -1], 12, 0b1111UL, 0b0000UL);
		EXTI->RTSR1 |= 1UL<<15; // Get the rising edge interrupt
		EXTI_D1->IMR1 |= 1UL<<15; // Enable CPU interrupt
	}
	else // RASPI
	{
		// nCS is PG10, so EXTI10 must be used.
		REG_WRITE_PART(SYSCFG->EXTICR[/*refman idx:*/3   -1], 8, 0b1111UL, 0b0110UL);
		EXTI->RTSR1 |= 1UL<<10; // Get the rising edge interrupt
		EXTI_D1->IMR1 |= 1UL<<10; // Enable CPU interrupt
	}

	// The interrupt priority must be fairly high, to quickly reconfigure the DMA so that if the master pulls CSn low
	// again very quickly to make a new transaction, we have the DMA up and running for that before the super small 4-byte
	// FIFO in the SPI is exhausted.
	// Both RASPI (EXTI10) and ODROID (EXTI15) happen to coincide to the same multiplexed EXTI interrupt group (15,14,13,12,11,10).
	NVIC_SetPriority(EXTI15_10_IRQn, INTPRIO_SBC_COMM);
	NVIC_EnableIRQ(EXTI15_10_IRQn);

}

void deinit_sbc_comm()
{
	IO_TO_GPI(GPIOB,5);
	IO_TO_GPI(GPIOB,4);
	IO_TO_GPI(GPIOB,3);
	IO_TO_GPI(GPIOA,15);
	IO_TO_GPI(GPIOG,9);
	IO_TO_GPI(GPIOD,7);
	IO_TO_GPI(GPIOG,11);
	IO_TO_GPI(GPIOG,10);

	EXTI->RTSR1 &= ~(1UL<<15 | 1UL<<10);
	EXTI_D1->IMR1 &= ~(1UL<<15 | 1UL<<10);

}
