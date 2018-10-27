/*
	RobotBoard firmware project
	
	Power switch / watchdog module: drives the charge pumps to control platfrom and
	application power switch on the battery module

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


void pwrswitch_chargepump_init();
void init_pwrswitch_and_led();

void chargepump_initial_pulsetrain();
void chargepump_replenish_pulsetrain();
void chargepump_pulsetrain_low_power(uint32_t del_ms);

void pwrswitch_1khz();
void shutdown();


#define PLAT_CP_LO() do{LO(GPIOB, 2);}while(0)
#define PLAT_CP_HI() do{HI(GPIOB, 2);}while(0)

#define PWRLED_ON()  do{LO(GPIOF, 2);}while(0)
#define PWRLED_OFF() do{HI(GPIOF, 2);}while(0)

#define PWRSWITCH_PRESSED (!IN(GPIOE,2))
