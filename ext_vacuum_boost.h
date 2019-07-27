#pragma once
#include <stdint.h>

void init_ext_vacuum_boost();

// Automatically uses the previous three functions, if changed from the previous values:
void ext_vacuum_cmd(int power, int nozzle); // power 0 - 100. Nozzle: 0 down, 1 up


void ext_vacuum_fsm();

void ext_vacuum_freerunning_fsm(int temporary_rise);

