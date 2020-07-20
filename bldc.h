/*
	RobotBoard firmware project
	
	BLDC motor controller module.

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

void init_bldc();
void init_bldc_dummy(); // Generates ADC triggers required by other systems; otherwise doesn't init BLDC.

void enable_motors();
void disable_motors();

// Sets the motor current limit, torque between 0-100%
void motor_torque_lim(int m, int percent);

// Returns measured current converted to torque 0-100%
// Filtered (response time a few milliseconds) absolute value
int get_motor_torque(int m);

// Returns wheel oscillation/jerking percentage 0-100%. 100% is the level of automatic (cannot be disabled) instant shutdown (error)
// The user the bldc module is responsible for stopping motion way before 100% if errors leading to shutdown are undesired.
int get_jerk_status(int m);


// In arbitrary speed unit, 150*256 is quite a typical travel speed.
void set_motor_velocities(int v1, int v2);


// Wheel positions.
// 256 steps equals one hall sensor step. Full revolution is 60 hall steps or 15360 digital numbers.
// Right now steps are just counted 256 at once. The factor 256 is there to account for
// future changes (more accurate encoders or interpolation tricks)

// Starts at 0 in the boot.
// Designed to wrap around as specified in the C standard; difference (pos_earlier - pos) casted into signed int32_t is valid.

#define SUBSTEPS 256
#define SUBSTEP_SHIFT 8
extern uint32_t bldc_pos[2];

