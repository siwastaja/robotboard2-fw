#pragma once
#include <stdint.h>

void init_ext_vacuum_boost();

void ext_vacuum_nozzle_rise();
void ext_vacuum_nozzle_lower();
void ext_vacuum_power(int percent);
// Automatically uses the previous three functions, if changed from the previous values:
void ext_vacuum_cmd(int power, int nozzle); // power 0 - 100. Nozzle: 0 down, 1 up


void ext_vacuum_fsm();
