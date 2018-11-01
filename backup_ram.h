#pragma once
#include <stdint.h>

typedef struct __attribute__((packed))
{
	uint32_t boot_cnt;
	uint32_t dummy;

	uint32_t  immediate_5v;

} backup_ram_t;

extern volatile backup_ram_t backup_ram __attribute__((section(".ram_backup_data")));
