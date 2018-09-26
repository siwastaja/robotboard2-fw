/*
	RobotBoard firmware project

	3DTOF control module: configures the Espros 3DTOF chips and reads out the data, controls
	the data processing flow	


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


#define EPC_XS 160
#define EPC_YS 60

typedef struct __attribute__((packed))
{
	uint16_t start_pad[2];
	uint16_t img[EPC_XS*EPC_YS];
} epc_img_t;

typedef struct __attribute__((packed))
{
	epc_img_t dcs[4];
} epc_4dcs_t;

typedef struct __attribute__((packed))
{
	epc_img_t dcs[2];
} epc_2dcs_t;

#define SIZEOF_MONO (sizeof(epc_img_t))  //(EPC_XS*EPC_YS*2)
#define SIZEOF_2DCS (sizeof(epc_2dcs_t)) //(EPC_XS*EPC_YS*2*2)
#define SIZEOF_4DCS (sizeof(epc_4dcs_t)) //(EPC_XS*EPC_YS*2*4)


void epc_safety_shutdown();
void tof_ctrl_init();
