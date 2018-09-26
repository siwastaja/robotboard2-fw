/*
	RobotBoard firmware project
	
	3DTOF muxing control: up to 10 3DTOF sensor modules are connected to the single MCU interface. This
	module controls the multiplexer tree to select one at a time.

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

void tof_mux_init(); // expects enabled GPIO port clocks
void tof_mux_all_off();
void tof_mux_select(int idx);

