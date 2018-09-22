#include <stdint.h>
#include <string.h>
#include "ext_include/stm32h7xx.h"
#include "stm32_cmsis_extension.h"
#include "misc.h"
#include "flash.h"
#include "own_std.h"

#include "../robotsoft/api_board_to_soft.h"

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


*/

#define TX_FIFO_DEPTH  4
#define RX_FIFO_DEPTH  6
#define TX_MAX_LEN 55000
#define RX_MAX_LEN 6000

#if (TX_MAX_LEN%4 != 0 || RX_MAX_LEN%4 != 0)
	#error "SPI max transfer lengths must be multiples of 4"
#endif

static volatile uint8_t tx_fifo[TX_FIFO_DEPTH][TX_MAX_LEN] __attribute__((aligned(4)));
static volatile uint8_t rx_fifo[RX_FIFO_DEPTH][RX_MAX_LEN] __attribute__((aligned(4)));

static volatile int tx_fifo_cpu = 0;
static volatile int tx_fifo_spi = 0;

static volatile int rx_fifo_cpu = 0;
static volatile int rx_fifo_spi = 0;


test_msg1_t* test_msg1;
test_msg2_t* test_msg2;
test_msg3_t* test_msg3;

void * * const p_p_tx_msgs[256]  =
{
	0,
	(void**)&test_msg1,
	(void**)&test_msg2,
	(void**)&test_msg3,
	0
};

uint16_t const tx_msg_sizes[256] =
{
	0,
	sizeof(test_msg1_t),
	sizeof(test_msg2_t),
	sizeof(test_msg3_t),
	0
};


uint64_t subs[4]; // Enabled subscriptions, 1 bit per message ID, [0] LSb = id0, [0] MSb = id63, [1] LSb = id64, and so on

#define TX_SUBS_START_OFFSET 16
#define TX_FOOTER_LEN 4

void update_subs(uint64_t *subs_vector)
{
	for(int i=0; i<256; i++)
	{
		if(p_p_tx_msgs[i])
			*p_p_tx_msgs[i] = 0;
	}

	int offs = TX_SUBS_START_OFFSET;

	for(int i=0; i<4; i++)
	{
		subs[i] = 0;
		uint64_t t = subs_vector[i];
		for(int s=i*64; s<(i+1)*64; s++)
		{
			if(t & 1)
			{
				// id #s is enabled
				if(offs + tx_msg_sizes[s] > TX_MAX_LEN-TX_FOOTER_LEN)
				{
					// requested subscription doesn't fit: stop adding subscriptions.
					break;
				}

				subs[i] |= 1ULL<<(s-i*64);
				*p_p_tx_msgs[s] = tx_fifo[tx_fifo_cpu] + offs;
				offs += tx_msg_sizes[s];
			}
			t >>= 1;
		}
	}

	// Write the footer, which stays permanently on the buffers, until new update_subs()
	for(int i=0; i<TX_FIFO_DEPTH; i++)
	{
		for(int o=0; o<TX_FOOTER_LEN; o++)
		{
			tx_fifo[i][offs+o] = 0xee;
		}
	}
}

int is_tx_overrun()
{
	int next_tx_fifo_cpu = tx_fifo_cpu+1; if(next_tx_fifo_cpu >= TX_FIFO_DEPTH) next_tx_fifo_cpu = 0;

	return (next_tx_fifo_cpu == tx_fifo_spi);
}

void tx_fifo_push()
{
	tx_fifo_cpu++; if(tx_fifo_cpu >= TX_FIFO_DEPTH) tx_fifo_cpu = 0;

	// Increased tx_fifo_cpu tells the SPI procedure that the data won't change anymore, and is OK to send.
	// Having an SPI interrupt at this point is OK (and optimal).

	// Now, let's make the active data pointers to point to the next TX FIFO block.
	// This is basically the same offset calculation as in update_subs, but no need to check if the requests fit,
	// because it's already checked (and subs item zeroed out if necessary).

	int offs = TX_SUBS_START_OFFSET;

	for(int i=0; i<4; i++)
	{
		uint64_t t = subs[i];
		for(int s=i*64; s<(i+1)*64; s++)
		{
			if(t & 1)
			{
				// id #s is enabled
				*p_p_tx_msgs[s] = tx_fifo[tx_fifo_cpu] + offs;
				offs += tx_msg_sizes[s];
			}
			t >>= 1;
		}
	}
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

volatile int spi_test_cnt2;

/*
void sbc_spi_eot_inthandler()
{
	SPI1->IFCR = 1UL<<3; // clear the intflag
	__DSB();
	spi_test_cnt2++;
}
*/

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
	


	When the SPI is enabled with DMA, it always generates DMA requests right away to fill its TX fifo. When the
	nSS is asserted half a year later, the first data are from half a year ago, rest is DMA'ed from the memory at that point.
	With our communication FIFO scheme, this isn't a problem, but do remember this.

*/

volatile int spi_test_cnt;
volatile int new_rx;
volatile int new_rx_len;

volatile int spi_dbg1, spi_dbg2;

void sbc_spi_cs_end_inthandler()
{
//	spi_dbg1 = DMA1_Stream0->NDTR;
//	spi_dbg2 = DMA1_Stream1->NDTR;
	// Triggered when cs goes high

	// Clear the EXTI interrupt pending bit:
	if(sbc_iface == ODROID)
		EXTI_D1->PR1 = 1UL<<15;
	else // RASPI
		EXTI_D1->PR1 = 1UL<<10;

	spi_test_cnt++;

	SPI1->CR1 = 0; // Disable and reset the SPI - FIFO flush happens

	// Turn TX DMA bit off (by rewriting the whole register to save time):
#ifdef SPI_DMA_1BYTE_AT_TIME
	SPI1->CFG1 = 1UL<<14 /*RX DMA EN*/ |(1UL -1UL)<<5 /*FIFO threshold*/ | 0b00111UL /*8-bit data*/;  // don't set TX DMA EN yet 
#else
	SPI1->CFG1 = 1UL<<14 /*RX DMA EN*/ |(4UL -1UL)<<5 /*FIFO threshold*/ | 0b00111UL /*8-bit data*/;  // don't set TX DMA EN yet 
#endif
	__DSB();

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
	int len = RX_MAX_LEN - DMA1_Stream1->NDTR;
	#else
	int len = RX_MAX_LEN - DMA1_Stream1->NDTR*4;
	#endif


	// Check if there is the maintenance magic code:
	if(*((volatile uint32_t*)&rx_fifo[rx_fifo_spi]) == 0x9876fedb)
	{
		run_flasher();
	}

	// TX DMA
	if(tx_fifo_cpu == tx_fifo_spi)
	{
		// No data to send.
		// Don't enable TX DMA, let the SPI send the content of the "underrun register"
	}
	else
	{
		if(len >= 16)
		{
			tx_fifo_spi++;
			if(tx_fifo_spi >= TX_FIFO_DEPTH) tx_fifo_spi = 0;
		}

		DMA1_Stream0->M0AR = (uint32_t)tx_fifo[tx_fifo_spi];

		DMA_CLEAR_INTFLAGS(DMA1, 0);
		DMA1_Stream0->CR |= 1; // Enable TX DMA
		SPI1->CFG1 |= 1UL<<15 /*TX DMA ena*/; // not earlier, if you want to follow the refman advice
	}


	// RX DMA

	if(len >= 16)
	{
		int next_rx_fifo_spi = rx_fifo_spi+1;
		if(next_rx_fifo_spi >= RX_FIFO_DEPTH)
			next_rx_fifo_spi = 0;

		if(next_rx_fifo_spi == rx_fifo_cpu)
		{
			// RX fifo full - overrun. Considered catastrophic condition.
			error(4);
		}
		else
		{
			rx_fifo_spi = next_rx_fifo_spi;
		}
	}
	else
	{
		// Just re-enable the DMA so the next operation writes to the same place.
	}	

	DMA1_Stream1->M0AR = (uint32_t)rx_fifo[rx_fifo_spi];

	__DSB();
	DMA_CLEAR_INTFLAGS(DMA1, 1);
	DMA1_Stream1->CR |= 1; // Enable RX DMA
	__DSB();


	SPI1->CR1 = 1UL; // Enable in slave mode
	__DSB();
}

void check_rx_test()
{
	char printbuf[128];

	if(rx_fifo_spi != rx_fifo_cpu)
	{
		uart_print_string_blocking("rx pop:"); o_utoa32_hex(*(uint32_t*)rx_fifo[rx_fifo_cpu], printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("...\r\n");

		rx_fifo_cpu++; if(rx_fifo_cpu >= RX_FIFO_DEPTH) rx_fifo_cpu = 0;
	}
	__DSB();
}

void sbc_comm_test()
{
	char printbuf[128];
	uint8_t data = 0x42;
	int cnt = 0;

	int delays[13] = {50, 20, 70, 4, 17, 200, 40, 4, 15, 45, 4, 4, 4};
	while(1)
	{
//		delay_ms(delays[cnt]);
		for(int i = 0; i<1; i++)
		{
			delay_ms(1000);
			check_rx_test();
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


uint64_t subs_test1[4] = {0b1110,0,0,0};
uint64_t subs_test2[4] = {0b1010,0,0,0};
uint64_t subs_test3[4] = {0b0000,0,0,0};
uint64_t subs_test4[4] = {0b0100,0,0,0};

static void gen_some_data1()
{
	char printbuf[128];
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
	char printbuf[128];
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
	char printbuf[128];
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
	char printbuf[128];
	int cnt=0;
	while(1)
	{
		for(int i=0; i<50; i++)
		{
			uart_print_string_blocking("!");
			delay_ms(100);
			check_rx_test();
		}

		if(is_tx_overrun())
		{
			uart_print_string_blocking("\r\nTX buffer overrun! Skipping data generation.\r\n"); 
		}
		else
		{
			gen_some_data1();
			gen_some_data2();
			gen_some_data3();


			uart_print_string_blocking("\r\nPush!\r\n"); 

			tx_fifo_push();
			if(cnt == 5)
				update_subs(subs_test1);
			if(cnt == 10)
				update_subs(subs_test2);
			if(cnt == 15)
				update_subs(subs_test3);
			if(cnt == 20)
				update_subs(subs_test4);
		}

		cnt++;
	}


}

void init_sbc_comm()
{

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


	// Initialization order from reference manual:
#ifdef SPI_DMA_1BYTE_AT_TIME
	SPI1->CFG1 = 1UL<<14 /*RX DMA EN*/ |(1UL -1UL)<<5 /*FIFO threshold*/ | 0b00111UL /*8-bit data*/;  // don't set TX DMA EN yet 
#else
	SPI1->CFG1 = 1UL<<14 /*RX DMA EN*/ |(4UL -1UL)<<5 /*FIFO threshold*/ | 0b00111UL /*8-bit data*/;  // don't set TX DMA EN yet 
#endif

	SPI1->UDRDR = 0;
	// SPI1->CFG2 defaults good

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
		DMA1_Stream0->NDTR = TX_MAX_LEN;
	#else
		DMA1_Stream0->NDTR = TX_MAX_LEN/4;
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

	#ifdef SPI_DMA_1BYTE_AT_TIME
		DMA1_Stream1->NDTR = RX_MAX_LEN;
	#else
		DMA1_Stream1->NDTR = RX_MAX_LEN/4;
	#endif
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
	NVIC_SetPriority(EXTI15_10_IRQn, 2);
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
