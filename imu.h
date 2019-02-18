/*
	RobotBoard firmware project
	
	Intertial measurement unit - integrates sensor data

	(c) 2017-2018 Pulu Robotics and other contributors
	Maintainer: Antti Alhonen <antti.alhonen@iki.fi>

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License version 2, as 
	published by the Free Software Foundation.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	GNU General Public License version 2 is supplied in file LICENSING.
*/

#pragma once
#include <stdint.h>

void init_imu() __attribute__((section(".text_itcm")));
void init_imu();

int16_t m_compensate_x(int16_t x_in, uint16_t rhall_in, int idx);
int16_t m_compensate_y(int16_t y_in, uint16_t rhall_in, int idx);
int16_t m_compensate_z(int16_t z_in, uint16_t rhall_in, int idx);



typedef struct __attribute__((packed))
{
	int16_t x;
	int16_t y;
	int16_t z;
} xyz_i16_packed_t;

// n field:
// Works as a "read fifo" command (0x3f register address) on the TX packet
// Comes inevitably back on the RX packet, containing random, do not care data
// Is reused as a "number of samples" field, after DMA is done :-).
typedef struct __attribute__((packed))
{
	uint8_t n;
	xyz_i16_packed_t xyz[8];
} g_dma_packet_t;

typedef struct __attribute__((packed))
{
	uint8_t n;
	xyz_i16_packed_t xyz[8];
} a_dma_packet_t;

typedef union __attribute__((packed))
{
	struct __attribute__((packed))
	{
		uint8_t dummy;
		int16_t x;
		int16_t y;
		int16_t z;
		uint16_t rhall;
		uint16_t align1;
		uint8_t  align2;
	} coords;

	struct __attribute__((packed))
	{
		uint8_t dummy;
		uint32_t first;
		uint32_t second;
		uint16_t align1;
		uint8_t  align2;
	} blocks;
} m_dma_packet_t;


extern volatile a_dma_packet_t * const imu_a[6];
extern volatile g_dma_packet_t * const imu_g[6];
extern volatile m_dma_packet_t * const imu_m[6];

