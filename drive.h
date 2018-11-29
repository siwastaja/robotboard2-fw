#pragma once
#include <stdint.h>

#define ANG_180_DEG 2147483648UL
#define ANG_90_DEG  1073741824
#define ANG_2_5_DEG   29826162
#define ANG_1_DEG     11930465
#define ANG_0_5_DEG    5965232
#define ANG_0_25_DEG   2982616
#define ANG_0_125_DEG  1491308
#define ANG_0_1_DEG    1193047
#define ANG_0_05_DEG    596523
#define ANG_0_01_DEG    119305
#define ANG_0_001_DEG    11930

#define ANG_1PER16_DEG  745654  // cumulated full circle rounding error: 0.000006%


typedef struct
{
	uint32_t ang; // uint32_t range --> 0..+360 deg; let it overflow freely. 1 unit = 83.81903171539 ndeg
	int64_t x;   // 1/65536 mm
	int64_t y;   // 1/65536 mm
} hires_pos_t;

extern volatile hires_pos_t hires_cur_pos;

