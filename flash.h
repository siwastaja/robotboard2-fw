/*
	Originally from: PULUROBOT RN1-BRAIN RobotBoard main microcontroller firmware project

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

#ifndef _FLASHER_H
#define _FLASHER_H

void run_flasher();

extern void refresh_settings(); // in stm32init.c
void save_flash_settings(); // don't do too often


typedef struct __attribute__((packed)) __attribute__((aligned(4)))
{
	int32_t offsets[4][4];
	int32_t offsets_at_temps[4];
} settings_t;

extern volatile settings_t settings;


#endif
