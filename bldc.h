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

// Puts the motor to the state of actively correcting location so that bldc_pos_set matches actual position (readable from bldc_pos):
void motor_run(int m);

// Like motor_run, but commands the bldc_pos_set to the call-time bldc_pos plus amount (can be positive or negative).
// Also, sets wanna_stop to 1, meaning after this amount is corrected, the motor stops.
// Calling this continuously makes the motor run continuously
void motor_run_a_bit(int m, int amount);


// Changes the state so that the motor goes into stop state once the position error has been corrected
void motor_let_stop(int m);

// Instantly stops the motor, and sets the error to zero by changing bldc_pos_set
void motor_stop_now(int m);

// Sets the motor current limit, torque between 0-100%
void motor_torque_lim(int m, int percent);

// Returns measured current converted to torque 0-100%
int get_motor_torque(int m);

int get_jerk_status(int m);


// Completely frees the motor by floating the windings. Also does what motor_stop_now does.
void motor_release(int m);


// Setpoints and actual positions.
// 256 steps equals one hall sensor step. Full revolution is 60 hall steps or 15360 digital numbers.

// Starts at 0 in the boot.
// Designed to wrap around as specified in the C standard; difference (pos_set - pos) casted into signed int32_t is valid.

#define SUBSTEPS 256
#define SUBSTEP_SHIFT 8
extern uint32_t bldc_pos_set[2];
extern uint32_t bldc_pos[2];

