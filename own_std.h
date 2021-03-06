/*
	RobotBoard firmware project
	
	Some helper functions from my earlier projects

	(c) 2014-2018 Antti Alhonen, Pulu Robotics and other contributors
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

// Find terminating 0 from a string, or terminate after n characters,
// whichever comes first. Returns the length of a null-terminated string,
// not including the null character, or at most n.
int o_strnlen(const char* str, int n);
int o_pow(int b, int e);
char* o_utoa16(uint16_t val, char* str);
char* o_utoa16_fixed(uint16_t val, char* str);
char* o_utoa8_fixed(uint8_t val, char* str);
char* o_utoa32(uint32_t val, char* str);
char* o_utoa32_fixed(uint32_t val, char* str);
char* o_itoa16(int16_t val, char* str);
char* o_itoa16_fixed(int16_t val, char* str);
char* o_itoa8_fixed(int8_t val, char* str);
char* o_itoa32(int32_t val, char* str);
char* o_str_append(char* str1, char* str2);
char* o_str_cmp(char* str1, char* str2);
char* o_atoi_append(char* str, int* val_out);

char* o_utoa8_hex(uint8_t val, char* str);
char* o_utoa16_hex(uint16_t val, char* str);
char* o_utoa32_hex(uint32_t val, char* str);

char* o_btoa8_fixed(uint8_t val, char* str);
char* o_btoa16_fixed(uint16_t val, char* str);
char* o_btoa32_fixed(uint32_t val, char* str);


