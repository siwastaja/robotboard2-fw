#pragma once
#include <stdint.h>

extern volatile uint32_t cnt_100us;

void timebase_inthandler() __attribute__((section(".text_itcm")));
void init_timebase();
void deinit_timebase();


