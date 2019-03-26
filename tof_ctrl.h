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


void epc_safety_shutdown();
void epc_shutdown();
void tof_ctrl_init();


void epc_trig();
void epc_clk_div(int div);
void epc_dis_leds();
void epc_ena_wide_leds();
void epc_ena_narrow_leds();
void epc_ena_wide_and_narrow_leds();
void epc_greyread_single_ended();
void epc_greyread_differential();

void epc_greyscale(); // OK to do while acquiring: shadow registered: applied to next trigger.
void epc_2dcs(); // OK to do while acquiring: shadow registered: applied to next trigger.
void epc_4dcs(); // OK to do while acquiring: shadow registered: applied to next trigger.
void epc_4dcs_dualint(); // OK to do while acquiring: shadow registered: applied to next trigger.
void epc_dualphase_or_int(); // OK to do while acquiring: shadow registered: applied to next trigger.
void epc_normalphase_or_int(); // OK to do while acquiring: shadow registered: applied to next trigger.
void epc_intlen(uint8_t multiplier, uint16_t time); // OK to do while acquiring: shadow registered: applied to next trigger.
// time1 = odd rows, time2 = even rows
// epc_4dcs_dualint() and epc_dualphase_or_int() must be called first
void epc_intlen_dual(uint8_t multiplier, uint16_t time1, uint16_t time2); // OK to do while acquiring: shadow registered: applied to next trigger.
// Do the magic stuff specified in the datasheet to enable temperature sensor conversion:
void epc_temperature_magic_mode(int idx);
// Do the magic stuff specified in the datasheet to disable temperature sensor conversion, back to normal operation
void epc_temperature_magic_mode_off(int idx);

void epc_enable_dll();
void epc_disable_dll();
void epc_coarse_dll_steps(int steps);
void epc_fine_dll_steps(int steps);
void epc_pll_steps(int steps);

uint8_t epc_reg_read(uint8_t addr);

void epc_start_read_eeprom(uint8_t addr);
uint8_t epc_read_eeprom_byte();

int epc_i2c_is_busy();
void block_epc_i2c(int err_idx);

void dcmi_start_dma(void *data, int size);
int poll_capt_with_timeout();
int poll_capt_with_timeout_complete();
int poll_data_transfer_start_with_timeout(int size);


void dcmi_crop_narrow();
void dcmi_crop_wide();
int32_t epc_read_temperature(int idx);

#define N_SENSORS 10
extern const uint8_t sensors_in_use[N_SENSORS];

void rgb_update(uint32_t val);

extern uint32_t sensor_silicon_ids[N_SENSORS];

#define INTUS(x_) (((x_)*10)/6)

