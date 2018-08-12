#include <stdint.h>

#include "ext_include/stm32h7xx.h"

void stm32init();
void nmi_handler();
void invalid_handler();
void hardfault_handler();

extern void error(int code);
extern void main();

extern unsigned int _STACKTOP;

// Vector table on page 730 on the Reference Manual RM0433
unsigned int * the_nvic_vector[126] __attribute__ ((section(".nvic_vector"))) =
{
/* 0x0000                    */ (unsigned int *) &_STACKTOP,
/* 0x0004 RESET              */ (unsigned int *) stm32init,
/* 0x0008 NMI                */ (unsigned int *) nmi_handler,
/* 0x000C HARDFAULT          */ (unsigned int *) hardfault_handler,
/* 0x0010 MemManage          */ (unsigned int *) invalid_handler,
/* 0x0014 BusFault           */ (unsigned int *) invalid_handler,
/* 0x0018 UsageFault         */ (unsigned int *) invalid_handler,
/* 0x001C                    */ (unsigned int *) invalid_handler,
/* 0x0020                    */ (unsigned int *) invalid_handler,
/* 0x0024                    */ (unsigned int *) invalid_handler,
/* 0x0028                    */ (unsigned int *) invalid_handler,
/* 0x002C SVcall             */ (unsigned int *) invalid_handler,
/* 0x0030 DebugMonitor       */ (unsigned int *) invalid_handler,
/* 0x0034                    */ (unsigned int *) invalid_handler,
/* 0x0038 PendSV             */ (unsigned int *) invalid_handler,
/* 0x003C SysTick            */ (unsigned int *) invalid_handler,
/* 0x0040 WWDG1              */ (unsigned int *) invalid_handler,
/* 0x0044 PVD (volt detector)*/ (unsigned int *) invalid_handler,
/* 0x0048                    */ (unsigned int *) invalid_handler,
/* 0x004C                    */ (unsigned int *) invalid_handler,
/* 0x0050                    */ (unsigned int *) invalid_handler,
/* 0x0054                    */ (unsigned int *) invalid_handler,
/* 0x0058                    */ (unsigned int *) invalid_handler,
/* 0x005C                    */ (unsigned int *) invalid_handler,
/* 0x0060                    */ (unsigned int *) invalid_handler,
/* 0x0064                    */ (unsigned int *) invalid_handler,
/* 0x0068                    */ (unsigned int *) invalid_handler,
/* 0x006C                    */ (unsigned int *) invalid_handler,
/* 0x0070                    */ (unsigned int *) invalid_handler,
/* 0x0074                    */ (unsigned int *) invalid_handler,
/* 0x0078                    */ (unsigned int *) invalid_handler,
/* 0x007C                    */ (unsigned int *) invalid_handler,
/* 0x0080                    */ (unsigned int *) invalid_handler,
/* 0x0084                    */ (unsigned int *) invalid_handler,
/* 0x0088                    */ (unsigned int *) invalid_handler,
/* 0x008C                    */ (unsigned int *) invalid_handler,
/* 0x0090                    */ (unsigned int *) invalid_handler,
/* 0x0094                    */ (unsigned int *) invalid_handler,
/* 0x0098                    */ (unsigned int *) invalid_handler,
/* 0x009C                    */ (unsigned int *) invalid_handler,
/* 0x00A0                    */ (unsigned int *) invalid_handler,
/* 0x00A4                    */ (unsigned int *) invalid_handler,
/* 0x00A8                    */ (unsigned int *) invalid_handler,
/* 0x00AC                    */ (unsigned int *) invalid_handler,
/* 0x00B0                    */ (unsigned int *) invalid_handler,
/* 0x00B4                    */ (unsigned int *) invalid_handler,
/* 0x00B8                    */ (unsigned int *) invalid_handler,
/* 0x00BC                    */ (unsigned int *) invalid_handler,
/* 0x00C0                    */ (unsigned int *) invalid_handler,
/* 0x00C4                    */ (unsigned int *) invalid_handler,
/* 0x00C8                    */ (unsigned int *) invalid_handler,
/* 0x00CC                    */ (unsigned int *) invalid_handler,
/* 0x00D0                    */ (unsigned int *) invalid_handler,
/* 0x00D4                    */ (unsigned int *) invalid_handler,
/* 0x00D8                    */ (unsigned int *) invalid_handler,
/* 0x00DC                    */ (unsigned int *) invalid_handler,
/* 0x00E0                    */ (unsigned int *) invalid_handler,
/* 0x00E4                    */ (unsigned int *) invalid_handler,
/* 0x00E8                    */ (unsigned int *) invalid_handler,
/* 0x00EC                    */ (unsigned int *) invalid_handler,
/* 0x00F0                    */ (unsigned int *) invalid_handler,
/* 0x00F4                    */ (unsigned int *) invalid_handler,
/* 0x00F8                    */ (unsigned int *) invalid_handler,
/* 0x00FC                    */ (unsigned int *) invalid_handler,
/* 0x0100                    */ (unsigned int *) invalid_handler,
/* 0x0104                    */ (unsigned int *) invalid_handler,
/* 0x0108                    */ (unsigned int *) invalid_handler,
/* 0x010C                    */ (unsigned int *) invalid_handler,
/* 0x0110                    */ (unsigned int *) invalid_handler,
/* 0x0114                    */ (unsigned int *) invalid_handler,
/* 0x0118                    */ (unsigned int *) invalid_handler,
/* 0x011C                    */ (unsigned int *) invalid_handler,
/* 0x0120                    */ (unsigned int *) invalid_handler,
/* 0x0124                    */ (unsigned int *) invalid_handler,
/* 0x0128                    */ (unsigned int *) invalid_handler,
/* 0x012C                    */ (unsigned int *) invalid_handler,
/* 0x0130                    */ (unsigned int *) invalid_handler,
/* 0x0134                    */ (unsigned int *) invalid_handler,
/* 0x0138                    */ (unsigned int *) invalid_handler,
/* 0x013C                    */ (unsigned int *) invalid_handler,
/* 0x0140                    */ (unsigned int *) invalid_handler,
/* 0x0144                    */ (unsigned int *) invalid_handler,
/* 0x0148                    */ (unsigned int *) invalid_handler,
/* 0x014C                    */ (unsigned int *) invalid_handler,
/* 0x0150                    */ (unsigned int *) invalid_handler,
/* 0x0154                    */ (unsigned int *) invalid_handler,
/* 0x0158                    */ (unsigned int *) invalid_handler,
/* 0x015C                    */ (unsigned int *) invalid_handler,
/* 0x0160                    */ (unsigned int *) invalid_handler,
/* 0x0164                    */ (unsigned int *) invalid_handler,
/* 0x0168                    */ (unsigned int *) invalid_handler,
/* 0x016C                    */ (unsigned int *) invalid_handler,
/* 0x0170                    */ (unsigned int *) invalid_handler,
/* 0x0174                    */ (unsigned int *) invalid_handler,
/* 0x0178                    */ (unsigned int *) invalid_handler,
/* 0x017C                    */ (unsigned int *) invalid_handler,
/* 0x0180                    */ (unsigned int *) invalid_handler,
/* 0x0184                    */ (unsigned int *) invalid_handler,
/* 0x0188                    */ (unsigned int *) invalid_handler,
/* 0x018C                    */ (unsigned int *) invalid_handler,
/* 0x0190                    */ (unsigned int *) invalid_handler,
/* 0x0194                    */ (unsigned int *) invalid_handler,
/* 0x0198                    */ (unsigned int *) invalid_handler,
/* 0x019C                    */ (unsigned int *) invalid_handler,
/* 0x01A0                    */ (unsigned int *) invalid_handler,
/* 0x01A4                    */ (unsigned int *) invalid_handler,
/* 0x01A8                    */ (unsigned int *) invalid_handler,
/* 0x01AC                    */ (unsigned int *) invalid_handler,
/* 0x01B0                    */ (unsigned int *) invalid_handler,
/* 0x01B4                    */ (unsigned int *) invalid_handler,
/* 0x01B8                    */ (unsigned int *) invalid_handler,
/* 0x01BC                    */ (unsigned int *) invalid_handler,
/* 0x01C0                    */ (unsigned int *) invalid_handler,
/* 0x01C4                    */ (unsigned int *) invalid_handler,
/* 0x01C8                    */ (unsigned int *) invalid_handler,
/* 0x01CC                    */ (unsigned int *) invalid_handler,
/* 0x01D0                    */ (unsigned int *) invalid_handler,
/* 0x01D4                    */ (unsigned int *) invalid_handler,
/* 0x01D8                    */ (unsigned int *) invalid_handler,
/* 0x01DC                    */ (unsigned int *) invalid_handler,
/* 0x01E0                    */ (unsigned int *) invalid_handler,
/* 0x01E4                    */ (unsigned int *) invalid_handler,
/* 0x01E8                    */ (unsigned int *) invalid_handler,
/* 0x01EC                    */ (unsigned int *) invalid_handler,
/* 0x01F0                    */ (unsigned int *) invalid_handler,
/* 0x01F4                    */ (unsigned int *) invalid_handler,
/* 0x01F8                    */ (unsigned int *) invalid_handler,
/* 0x01FC                    */ (unsigned int *) invalid_handler,
/* 0x0200                    */ (unsigned int *) invalid_handler,
/* 0x0204                    */ (unsigned int *) invalid_handler,
/* 0x0208                    */ (unsigned int *) invalid_handler,
/* 0x020C                    */ (unsigned int *) invalid_handler,
/* 0x0210                    */ (unsigned int *) invalid_handler,
/* 0x0214                    */ (unsigned int *) invalid_handler,
/* 0x0218                    */ (unsigned int *) invalid_handler,
/* 0x021C                    */ (unsigned int *) invalid_handler,
/* 0x0220                    */ (unsigned int *) invalid_handler,
/* 0x0224                    */ (unsigned int *) invalid_handler,
/* 0x0228                    */ (unsigned int *) invalid_handler,
/* 0x022C                    */ (unsigned int *) invalid_handler,
/* 0x0230                    */ (unsigned int *) invalid_handler,
/* 0x0234                    */ (unsigned int *) invalid_handler,
/* 0x0238                    */ (unsigned int *) invalid_handler,
/* 0x023C                    */ (unsigned int *) invalid_handler,
/* 0x0240                    */ (unsigned int *) invalid_handler,
/* 0x0244                    */ (unsigned int *) invalid_handler,
/* 0x0248                    */ (unsigned int *) invalid_handler,
/* 0x024C                    */ (unsigned int *) invalid_handler,
/* 0x0250                    */ (unsigned int *) invalid_handler,
/* 0x0254                    */ (unsigned int *) invalid_handler,
/* 0x0258                    */ (unsigned int *) invalid_handler,
/* 0x025C                    */ (unsigned int *) invalid_handler,
/* 0x0260                    */ (unsigned int *) invalid_handler,
/* 0x0264                    */ (unsigned int *) invalid_handler,
/* 0x0268                    */ (unsigned int *) invalid_handler,
/* 0x026C                    */ (unsigned int *) invalid_handler,
/* 0x0270                    */ (unsigned int *) invalid_handler,
/* 0x0274                    */ (unsigned int *) invalid_handler,
/* 0x0278                    */ (unsigned int *) invalid_handler,
/* 0x027C                    */ (unsigned int *) invalid_handler,
/* 0x0280                    */ (unsigned int *) invalid_handler,
/* 0x0284                    */ (unsigned int *) invalid_handler,
/* 0x0288                    */ (unsigned int *) invalid_handler,
/* 0x028C                    */ (unsigned int *) invalid_handler,
/* 0x0290                    */ (unsigned int *) invalid_handler,
/* 0x0294                    */ (unsigned int *) invalid_handler

};

extern unsigned int _BSS_BEGIN;
extern unsigned int _BSS_END;

extern unsigned int _DATA_BEGIN;
extern unsigned int _DATA_END;
extern unsigned int _DATAI_BEGIN;

extern unsigned int _DTCM_DATA_BEGIN;
extern unsigned int _DTCM_DATA_END;
extern unsigned int _DTCM_DATA_I_BEGIN;

extern unsigned int _SETTINGS_BEGIN;
extern unsigned int _SETTINGS_END;
extern unsigned int _SETTINGS_I_BEGIN;

extern unsigned int _DTCM_BSS_BEGIN;
extern unsigned int _DTCM_BSS_END;

extern unsigned int _TEXT_ITCM_BEGIN;
extern unsigned int _TEXT_ITCM_END;
extern unsigned int _TEXT_ITCM_I_BEGIN;


extern void hwtest_main();

void refresh_settings()
{
	volatile uint32_t* settings_begin  = (volatile uint32_t*)&_SETTINGS_BEGIN;
	volatile uint32_t* settings_end    = (volatile uint32_t*)&_SETTINGS_END;
	volatile uint32_t* settings_i_begin = (volatile uint32_t*)&_SETTINGS_I_BEGIN;

	while(settings_begin < settings_end)
	{
		*settings_begin = *settings_i_begin;
		settings_begin++;
		settings_i_begin++;
	}
	__DSB(); __ISB();
}

void stm32init(void)
{
	RCC->AHB1ENR |= 1UL<<20; // Enable DTCM RAM...

	uint32_t* bss_begin = (uint32_t*)&_BSS_BEGIN;
	uint32_t* bss_end   = (uint32_t*)&_BSS_END;
	while(bss_begin < bss_end)
	{
		*bss_begin = 0;
		bss_begin++;
	}

	uint32_t* dtcm_bss_begin = (uint32_t*)&_DTCM_BSS_BEGIN;
	uint32_t* dtcm_bss_end   = (uint32_t*)&_DTCM_BSS_END;
	while(dtcm_bss_begin < dtcm_bss_end)
	{
		*dtcm_bss_begin = 0;
		dtcm_bss_begin++;
	}


	uint32_t* data_begin  = (uint32_t*)&_DATA_BEGIN;
	uint32_t* data_end    = (uint32_t*)&_DATA_END;
	uint32_t* datai_begin = (uint32_t*)&_DATAI_BEGIN;

	while(data_begin < data_end)
	{
		*data_begin = *datai_begin;
		data_begin++;
		datai_begin++;
	}

	uint32_t* dtcm_data_begin  = (uint32_t*)&_DTCM_DATA_BEGIN;
	uint32_t* dtcm_data_end    = (uint32_t*)&_DTCM_DATA_END;
	uint32_t* dtcm_data_i_begin = (uint32_t*)&_DTCM_DATA_I_BEGIN;

	while(dtcm_data_begin < dtcm_data_end)
	{
		*dtcm_data_begin = *dtcm_data_i_begin;
		dtcm_data_begin++;
		dtcm_data_i_begin++;
	}

	refresh_settings();

	uint32_t* text_itcm_begin  = (uint32_t*)&_TEXT_ITCM_BEGIN;
	uint32_t* text_itcm_end    = (uint32_t*)&_TEXT_ITCM_END;
	uint32_t* text_itcm_i_begin = (uint32_t*)&_TEXT_ITCM_I_BEGIN;

	while(text_itcm_begin < text_itcm_end)
	{
		*text_itcm_begin = *text_itcm_i_begin;
		text_itcm_begin++;
		text_itcm_i_begin++;
	}

	main();
}


void nmi_handler(void)
{
	error(1);
}

void hardfault_handler(void)
{
	error(2);
}

void invalid_handler(void)
{
	error(3);
}
