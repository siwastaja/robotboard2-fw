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

void app_power_on();
void app_power_off();



#define PWRLED_ON()  do{LO(GPIOF, 2);}while(0)
#define PWRLED_OFF() do{HI(GPIOF, 2);}while(0)

#define PWRSWITCH_PRESSED (!IN(GPIOE,2))


#define PLAT_CP_LO() do{LO(GPIOB, 2);}while(0)
#define PLAT_CP_HI() do{HI(GPIOB, 2);}while(0)

#define APP_CP_LO() do{LO(GPIOF, 12);}while(0)
#define APP_CP_HI() do{HI(GPIOF, 12);}while(0)


// Disable application switch desat protection very carefully, know what you are doing before using
// APP_DIS_DESAT_PROT().
// 10 us pulse is acceptable at below 1% duty to maintain the FETs 700A rating.
// Total resistance in battery (10mOhm), MOSFETs (2*1mOhm), fuse (1.6mOhm), PCB traces(0.5mOhm), connectors(1mOhm),
// cables(1mOhm), application internal (1mOhm) is around 17mOhm,
// With Vds=25V, maximum resistance-limited current is 1470A.
// With stray inductance of 100nH, the current rise rate is di/dt = V/L = 25V/100nH = 250A/ns.
// During a 10us pulse, the initial current is 0, and the final current is around ~1000A worst case.

#define APP_EN_DESAT_PROT()  do{LO(GPIOE, 15);}while(0)
#define APP_DIS_DESAT_PROT() do{HI(GPIOE, 15);}while(0)
