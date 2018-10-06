#pragma once
#include <stdint.h>

extern volatile uint32_t ms_cnt;

void timebase_inthandler() __attribute__((section(".text_itcm")));
void init_timebase();
void deinit_timebase();


