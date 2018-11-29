#ifndef SIN_LUT_H
#define SIN_LUT_H

#include <inttypes.h>

#define SIN_LUT_POINTS 4096
#define SIN_LUT_BITS 12
#define SIN_LUT_SHIFT (32-SIN_LUT_BITS)
#define SIN_LUT_RESULT_SHIFT 15

extern const int16_t sin_lut[SIN_LUT_POINTS] __attribute__((section(".sin_lut")));

static inline int16_t lut_sin_from_u32(uint32_t ang)
{
	return sin_lut[ang>>(32-SIN_LUT_BITS)]; // Won't overindex; ang=0 -> idx = 0, ang=2^32-1 -> idx = 4095
}

static inline int16_t lut_cos_from_u32(uint32_t ang)
{
	ang += 1073741824; // Will wrap around properly
	return sin_lut[ang>>(32-SIN_LUT_BITS)]; // Won't overindex; ang=0 -> idx = 0, ang=2^32-1 -> idx = 4095
}

#endif
