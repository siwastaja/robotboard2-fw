/*
	RobotBoard firmware project
	
	Charger module: implements a software-defined dual phase synchronous buck
	converter using on-chip resources

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

void init_charger();
void deinit_charger();


void charger_10khz();
void charger_1khz();
void charger_freerunning_fsm();

int charger_get_latest_cur_pha();
int charger_get_latest_cur_phb();
int charger_is_running();
int charger_is_full();
int charger_is_mounted();
uint8_t conv_bat_percent(int mv);

