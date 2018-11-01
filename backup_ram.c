#include <stdint.h>
#include "misc.h"
#include "backup_ram.h"

volatile backup_ram_t backup_ram __attribute__((section(".ram_backup_data"))) =
{
	.boot_cnt = 0,
	.dummy = 42
};
