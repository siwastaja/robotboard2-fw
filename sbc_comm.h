/*
	RobotBoard firmware project
	Module for communicating with the single-board computer (Raspberry Pi 3, Odroid XU4, etc.)
	through SPI and/or UART


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

void init_sbc_comm();
void deinit_sbc_comm();

