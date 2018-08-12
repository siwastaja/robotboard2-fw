#pragma once
#include <stdint.h>

void tof_mux_init(); // expects enabled GPIO port clocks
void tof_mux_select(int idx);

