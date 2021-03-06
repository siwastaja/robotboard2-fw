/*
	RobotBoard firmware project
	
	Audio module - drives on-chip DAC to produce sounds

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

void init_audio();

void beep_blocking(int pulses, int us, int volume);

void beep(int len_ms, int hz_start, int sweep, int volume); // len milliseconds, hz initial freq, sweep: positive sweeps down, negative sweeps up, volume 0-100


void audio_10khz();
