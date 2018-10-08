/*
	Magnetometer data compensation algorithms & trim value register map
	To be included in imu.c. Not a compilation unit. 



	The magnetometer in BOSCH BMX055 does not provide accurate-enough data as is (in raw form).

	Hence, BOSCH runs a certain kind of factory calibration, and stores a set of resulting calibration parameters
	in the non-volatile memory of the BMX055 chip.

	Very simple algorithms are then used to compensate the data, using the calibration parameters. These algorithms
	don't run on the BMX055, but on the customer's (our) hardware. Hence, they need to be implemented in our code.

	Both the memory locations of said calibration parameters, and the algorithms to compensate the data,
	are not documented in the device datasheet. Instead, BOSCH provides a fully working example code
	for interfacing with the sensor. This code contains the memory map for the calibration registers (not
	available on the device datasheet), and the very simple algorithms to utilize said registers for compensation.

	This is a typical calibration dataset (read out from one example sensor):
		x1=0
		y1=0
		z4=0
		x2=28
		y2=28
		z2=736
		z1=22792
		xyz1=6806
		z3=0
		xy2=-3
		xy1=29



	BOSCH allows redistribution of their program, in both source and binary forms, with a simple license, which
	does not prevent using in a GPLv2 licensed application.

	Nevertheless, we don't use their code, not because there's anything wrong with the code itself, or the
	license, but simply because our own use case is very custom and tightly integrated, so we need our specific
	data structures, and our own ways to implement the algorithm.

	We have looked at the BOSCH's code as an example while writing our own implementation. Hence, we think that
	our code is (C) Pulu Robotics Oy, and we publish it under GNU GPL V2.

	Nevertheless, we see that somebody might disagree with our viewpoint. If, in some jurisdiction, this code
	is seen to be a modified version of BOSCH's code (which it in our opinion isn't), or if in some jurisdiction,
	algorithms are subject to copyright, to comply with BOSCH's requirements, the following copyright notice is
	reproduced here thusly. If this is the case, we see there are no issues still further licensing the code
	under GNU GPL V2, given the permissitivity of the license:


		 * [Possibly] Copyright (C) 2015 - 2016 Bosch Sensortec GmbH
		 *
		 * Redistribution and use in source and binary forms, with or without
		 * modification, are permitted provided that the following conditions are met:
		 *
		 * Redistributions of source code must retain the above copyright
		 * notice, this list of conditions and the following disclaimer.
		 *
		 * Redistributions in binary form must reproduce the above copyright
		 * notice, this list of conditions and the following disclaimer in the
		 * documentation and/or other materials provided with the distribution.
		 *
		 * Neither the name of the copyright holder nor the names of the
		 * contributors may be used to endorse or promote products derived from
		 * this software without specific prior written permission.
		 *
		 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
		 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
		 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
		 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
		 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
		 * OR CONTRIBUTORS BE LIABLE FOR ANY
		 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
		 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
		 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
		 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
		 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
		 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
		 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
		 * ANY WAY OUT OF THE USE OF THIS
		 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
		 *
		 * The information provided is believed to be accurate and reliable.
		 * The copyright holder assumes no responsibility
		 * for the consequences of use
		 * of such information nor for any infringement of patents or
		 * other rights of third parties which may result from its use.
		 * No license is granted by implication or otherwise under any patent or
		 * patent rights of the copyright holder.

	This applies to this file (imu_m_compensation.c) only.
*/

typedef struct __attribute__((packed))
{
	int8_t   x1;   // at 0x5d
	int8_t   y1;   // at 0x5e
	int8_t   dummy1;
	int16_t  dummy2;
	int16_t  z4;   // at 0x62
	int8_t   x2;   // at 0x64
	int8_t   y2;   // at 0x65
	int16_t  dummy3;
	int16_t  z2;   // at 0x68
	uint16_t z1;   // at 0x6a
	uint16_t xyz1; // at 0x6c
	int16_t  z3;   // at 0x6e
	int8_t   xy2;  // at 0x70
	uint8_t  xy1;  // at 0x71
} m_calib_t;

m_calib_t m_calib[6];

#define M_CALIB_START_ADDR 0x5d

#define XY_OVERFLOW (-4096)
#define Z_OVERFLOW (-16384)
#define OVERFLOW_OUTPUT	(-32768)

/*
	Note on the BMM150 compensation algorithms:
	The example code from BOSCH is obfuscated. Simple operations, such as interpreting two
	bytes uint8_t[2] into int16_t, is difficult to follow. Algorithms do a lot of nested if-else-if-else-if-...
	checks that end up giving the same "overflow" error output for both saturated data (strong magnetic field)
	or sensor malfunction (missing required calibration data). Similarly, in case of another type of sensor malfunction -
	Rhall measure fials and is returned as 0 - the algorithm silently picks some "typical" value out of the
	calibration data.

	This complexity and uncertainty is not reproduced.

	Following calibration parameters must be non-zero: z2, z1, xyz1.

	TODO: sanity check (not only against non-zero) all calibration parameters when reading them out initially.

	The example algorithm divides the results by 16, claiming the results are in "micro-teslas". Unfortunately,
	resolution seems poor when we do this, typical magnetic values range around 20-30 LSB - too much quantization.
	Thus, we don't reproduce this /16 operation, so our results should be in 1/16th microteslas.

	Currently, the formulas are exactly reproduced (except the final /16), including the exact type casts.
	TODO: clean them up to remove leftover obfuscation.

*/

static int16_t m_compensate_x(int16_t x_in, uint16_t rhall_in, int idx)
{
	// Remove any status/don't care bits and shift to actual width
	x_in >>= 3;
	rhall_in >>= 2;

	if(rhall_in == 0 || x_in == XY_OVERFLOW)
		return OVERFLOW_OUTPUT;

	int16_t ret;
	int32_t tmp1;
	uint16_t tmp2;
	int32_t tmp3;
	int32_t tmp4;
	int32_t tmp5;
	int32_t tmp6;
	int32_t tmp7;
	int32_t tmp8;
	int32_t tmp9;
	int32_t tmp10;

	tmp1 = ((int32_t)m_calib[idx].xyz1) * 16384;
	tmp2 = ((uint16_t)(tmp1 / rhall_in)) - ((uint16_t)0x4000);
	ret = ((int16_t)tmp2);
	tmp3 = (((int32_t)ret) * ((int32_t)ret));
	tmp4 = (((int32_t)m_calib[idx].xy2) * (tmp3 / 128));
	tmp5 = (int32_t)(((int16_t)m_calib[idx].xy1) * 128);
	tmp6 = ((int32_t)ret) * tmp5;
	tmp7 = (((tmp4 + tmp6) / 512) + ((int32_t)0x100000));
	tmp8 = ((int32_t)(((int16_t)m_calib[idx].x2) + ((int16_t)0xA0)));
	tmp9 = ((tmp7 * tmp8) / 4096);
	tmp10 = ((int32_t)x_in) * tmp9;
	ret = ((int16_t)(tmp10 / 8192));
	ret = (ret + (((int16_t)m_calib[idx].x1) * 8)); // / 16;

	return ret;
}



static int16_t m_compensate_y(int16_t y_in, uint16_t rhall_in, int idx)
{
	// Remove any status/don't care bits and shift to actual width
	y_in >>= 3;
	rhall_in >>= 2;

	if(rhall_in == 0 || y_in == XY_OVERFLOW)
		return OVERFLOW_OUTPUT;

	int16_t ret;
	uint16_t tmp0 = 0;
	int32_t tmp1;
	uint16_t tmp2;
	int32_t tmp3;
	int32_t tmp4;
	int32_t tmp5;
	int32_t tmp6;
	int32_t tmp7;
	int32_t tmp8;
	int32_t tmp9;

	tmp1 = (((int32_t)m_calib[idx].xyz1) * 16384) / tmp0;
	tmp2 = ((uint16_t)tmp1) - ((uint16_t)0x4000);
	ret = ((int16_t)tmp2);
	tmp3 = ((int32_t) ret) * ((int32_t)ret);
	tmp4 = ((int32_t)m_calib[idx].xy2) * (tmp3 / 128);
	tmp5 = ((int32_t)(((int16_t)m_calib[idx].xy1) * 128));
	tmp6 = ((tmp4 + (((int32_t)ret) * tmp5)) / 512);
	tmp7 = ((int32_t)(((int16_t)m_calib[idx].y2) + ((int16_t)0xA0)));
	tmp8 = (((tmp6 + ((int32_t)0x100000)) * tmp7) / 4096);
	tmp9 = (((int32_t)y_in) * tmp8);
	ret = (int16_t)(tmp9 / 8192);
	ret = (ret + (((int16_t)m_calib[idx].y1) * 8)); // / 16;

	return ret;
}

#define BMM150_NEGATIVE_SATURATION_Z -32767
#define BMM150_POSITIVE_SATURATION_Z 32767

static int16_t m_compensate_z(int16_t z_in, uint16_t rhall_in, int idx)
{
	// Remove any status/don't care bits and shift to actual width
	z_in >>= 1;
	rhall_in >>= 2;

	if(rhall_in == 0 || z_in == Z_OVERFLOW)
		return OVERFLOW_OUTPUT;

	int32_t ret;
	int16_t tmp0;
	int32_t tmp1;
	int32_t tmp2;
	int32_t tmp3;
	int16_t tmp4;

	tmp0 = ((int16_t)rhall_in) - ((int16_t) m_calib[idx].xyz1);
	tmp1 = (((int32_t)m_calib[idx].z3) * ((int32_t)(tmp0))) / 4;
	tmp2 = (((int32_t)(z_in - m_calib[idx].z4)) * 32768);
	tmp3 = ((int32_t)m_calib[idx].z1) * (((int16_t)rhall_in) * 2);
	tmp4 = (int16_t)((tmp3 + (32768)) / 65536);
	ret = ((tmp2 - tmp1) / (m_calib[idx].z2 + tmp4));

	if (ret > 32767)
	{
		ret =  32767;
	}
	else if(ret < -32768)
	{
		ret = -32768;
	}

	return ret;
}



