#include <stdint.h>
#include <string.h>
#include <math.h>

#include "ext_include/stm32h7xx.h"
#include "stm32_cmsis_extension.h"
#include "tof_process.h"
#include "tof_table.h"
#include "misc.h"


void tof_calc_ampl_hdr(uint8_t *ampl_out, uint8_t* long_in, uint8_t* short_in)
{
	for(int i=0; i<160*60; i++)
	{
		uint8_t out;
		if(long_in[i] == 255)
			out = 128+(short_in[i]>>1);
		else
			out = long_in[i]>>1;
		ampl_out[i] = out;
	}
}


/*
Process four DCS images, with offset_mm in millimeters. With clk_div=1, does the calculation at fled=20MHz (unamb range = 7.5m). Larger clk_div
multiplies the result.
*/

void tof_calc_dist_ampl(uint8_t *ampl_out, uint16_t *dist_out, epc_4dcs_t *in, int offset_mm, int clk_div) __attribute__((section(".text_itcm")));
void tof_calc_dist_ampl(uint8_t *ampl_out, uint16_t *dist_out, epc_4dcs_t *in, int offset_mm, int clk_div)
{
	int16_t offset = offset_mm/clk_div;

	for(int i=0; i < 160*60; i++)
	{
		uint16_t dist;
		int ampl;

		int16_t dcs0 = ((in->dcs[0].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs1 = ((in->dcs[1].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs2 = ((in->dcs[2].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs3 = ((in->dcs[3].img[i]&0b0011111111111100)>>2)-2048;

		if(dcs0 < -2047 || dcs0 > 2046 || dcs1 < -2047 || dcs1 > 2046 || dcs2 < -2047 || dcs2 > 2046 || dcs3 < -2047 || dcs3 > 2046)
		{
			ampl = 255;
			dist = 0;
		}
		else
		{
			int16_t dcs31 = dcs3-dcs1;
			int16_t dcs20 = dcs2-dcs0;

			// Use the lookup table to perform atan:

			int16_t dcs31_mod, dcs20_mod;

			if(dcs31<0)
				dcs31_mod = -dcs31;
			else
				dcs31_mod = dcs31;

			if(dcs20<0)
				dcs20_mod = -dcs20;
			else
				dcs20_mod = dcs20;

			int swapped = 0;
			if(dcs20_mod<dcs31_mod)
			{
				swapped = 1;
				int16_t tmp = dcs20_mod;
				dcs20_mod = dcs31_mod;
				dcs31_mod = tmp;
			}

			if(dcs20_mod == 0)
			{
				dist = 65534;
				ampl = 0;
			}
			else
			{

				int idx = (dcs31_mod*(TOF_TBL_LEN-1))/dcs20_mod;

				int32_t dist_i = tof_tbl[idx];
				if(swapped) dist_i = TOF_TBL_QUART_PERIOD - dist_i;
				if(dcs20<0) dist_i = TOF_TBL_HALF_PERIOD - dist_i;
				if(dcs31<0) dist_i = -dist_i;

				dist_i += offset;
				
				dist_i *= clk_div;

				if(dist_i < 1) dist_i = 1; else if(dist_i>6000) dist_i=6000;
//				if(dist_i < 1) dist_i += 3000; 
//				if(dist_i < 1) dist_i = 1;

				if(dist_i>6000) dist_i=6000;

#ifdef FAST_APPROX_AMPLITUDE
				ampl = (abso(dcs20)+abso(dcs31))/30; if(ampl > 255) ampl = 255;
#else
				ampl = sqrt(sq(dcs20)+sq(dcs31))/23;
#endif

				if(ampl<3)
					dist = 65534;
				else
					dist = dist_i;

			}

		}
		ampl_out[i] = ampl;
		dist_out[i] = dist;
	}
}


void tof_calc_dist_ampl_narrow(uint8_t *ampl_out, uint16_t *dist_out, epc_4dcs_narrow_t *in, int offset_mm, int clk_div) __attribute__((section(".text_itcm")));
void tof_calc_dist_ampl_narrow(uint8_t *ampl_out, uint16_t *dist_out, epc_4dcs_narrow_t *in, int offset_mm, int clk_div)
{
	int16_t offset = offset_mm/clk_div;

	for(int i=0; i < TOF_XS_NARROW*TOF_YS_NARROW; i++)
	{
		uint16_t dist;
		int ampl;

		int16_t dcs0 = ((in->dcs[0].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs1 = ((in->dcs[1].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs2 = ((in->dcs[2].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs3 = ((in->dcs[3].img[i]&0b0011111111111100)>>2)-2048;

		if(dcs0 < -2047 || dcs0 > 2046 || dcs1 < -2047 || dcs1 > 2046 || dcs2 < -2047 || dcs2 > 2046 || dcs3 < -2047 || dcs3 > 2046)
		{
			ampl = 255;
			dist = 0;
		}
		else
		{
			int16_t dcs31 = dcs3-dcs1;
			int16_t dcs20 = dcs2-dcs0;

			// Use the lookup table to perform atan:

			int16_t dcs31_mod, dcs20_mod;

			if(dcs31<0)
				dcs31_mod = -dcs31;
			else
				dcs31_mod = dcs31;

			if(dcs20<0)
				dcs20_mod = -dcs20;
			else
				dcs20_mod = dcs20;

			int swapped = 0;
			if(dcs20_mod<dcs31_mod)
			{
				swapped = 1;
				int16_t tmp = dcs20_mod;
				dcs20_mod = dcs31_mod;
				dcs31_mod = tmp;
			}

			if(dcs20_mod == 0)
			{
				dist = 65534;
				ampl = 0;
			}
			else
			{

				int idx = (dcs31_mod*(TOF_TBL_LEN-1))/dcs20_mod;

				int32_t dist_i = tof_tbl[idx];
				if(swapped) dist_i = TOF_TBL_QUART_PERIOD - dist_i;
				if(dcs20<0) dist_i = TOF_TBL_HALF_PERIOD - dist_i;
				if(dcs31<0) dist_i = -dist_i;

				dist_i += offset;
				
				dist_i *= clk_div;

				if(dist_i < 1) dist_i = 1; else if(dist_i>6000) dist_i=6000;
//				if(dist_i < 1) dist_i += 3000; 
//				if(dist_i < 1) dist_i = 1;

				if(dist_i>6000) dist_i=6000;

#ifdef FAST_APPROX_AMPLITUDE
				ampl = (abso(dcs20)+abso(dcs31))/30; if(ampl > 255) ampl = 255;
#else
				ampl = sqrt(sq(dcs20)+sq(dcs31))/23;
#endif

				if(ampl<3)
					dist = 65534;
				else
					dist = dist_i;

			}

		}
		ampl_out[i] = ampl;
		dist_out[i] = dist;
	}
}


void calc_toofar_ignore_from_2dcs(uint8_t *ignore_out, epc_4dcs_t *in, int threshold /*mm*/, int offset_mm, int clk_div)
{
	int16_t offset = offset_mm/clk_div;

	for(int yy = 0; yy < TOF_YS; yy++)
	{
		for(int xx = 0; xx < TOF_XS; xx++)
		{
			int i = yy*TOF_XS+xx;
			int16_t dcs0 = ((in->dcs[0].img[i]&0b0011111111111100)>>2)-2048;
			int16_t dcs1 = ((in->dcs[1].img[i]&0b0011111111111100)>>2)-2048;

			if(dcs0 < -2047 || dcs0 > 2046 || dcs1 < -2047 || dcs1 > 2046)
			{
				// Overexp - it's ok, probably not too far.
			}
			else
			{
				// for 2dcs mode to get the quadrant correct:
				dcs0 *= -1;
				dcs1 *= -1;

				// Use the lookup table to perform atan:

				int16_t dcs1_mod, dcs0_mod;

				if(dcs1<0)
					dcs1_mod = -dcs1;
				else
					dcs1_mod = dcs1;

				if(dcs0<0)
					dcs0_mod = -dcs0;
				else
					dcs0_mod = dcs0;

				int swapped = 0;
				if(dcs0_mod<dcs1_mod)
				{
					swapped = 1;
					int16_t tmp = dcs0_mod;
					dcs0_mod = dcs1_mod;
					dcs1_mod = tmp;
				}

				if(sq(dcs0)+sq(dcs1) < sq(75) || dcs0_mod == 0 /* should always be true if the first one is: todo: prove*/)
				{
					// amplitude too low
					ignore_out[yy*TOF_XS+xx] = 1;
				}
				else
				{
					int idx = (dcs1_mod*(TOF_TBL_LEN-1))/dcs0_mod;

					int32_t dist_i = tof_tbl[idx];
					if(swapped) dist_i = TOF_TBL_QUART_PERIOD - dist_i;
					if(dcs0<0) dist_i = TOF_TBL_HALF_PERIOD - dist_i;
					if(dcs1<0) dist_i = -dist_i;

					dist_i += offset;

					//if(dist_i < 0) dist_i += TOF_TBL_PERIOD;
					//else if(dist_i > TOF_TBL_PERIOD) dist_i -= TOF_TBL_PERIOD;
				
					dist_i *= clk_div;

					if(dist_i > threshold)
						ignore_out[yy*TOF_XS+xx] = 1;
				}

			}



		}


	}
}



void calc_interference_ignore_from_2dcs(uint8_t *ignore_out, epc_4dcs_t *in, int threshold)
{
	for(int yy = 0; yy < TOF_YS; yy++)
	{
		for(int xx = 0; xx < TOF_XS; xx++)
		{
			int16_t dcs0 = ((in->dcs[0].img[yy*TOF_XS+xx]&0b0011111111111100)>>2)-2048;
			int16_t dcs1 = ((in->dcs[1].img[yy*TOF_XS+xx]&0b0011111111111100)>>2)-2048;

			if( dcs0 < -1*threshold || dcs0 > threshold || dcs1 < -1*threshold || dcs1 > threshold)
			{
				ignore_out[yy*TOF_XS+xx] = 1;
			}
		}
	}
}


/*
	Suitability vector.

	We know that low amplitudes aren't good: there is more sensor noise - also small
	amplitudes are swept over with modulated stray light in the lens.

	In reality, high amplitudes aren't that good either: they suggest excessive exposure,
	which increases multipath and stray light!

	When combining HDR images, we want to prevent abrupt changes. So we are going to take
	the combination of suitable exposures and combine them based on how "good" we think
	they are based on amplitude.

*/
static uint8_t ampl_suitability[256] __attribute__((section(".dtcm_data"))) =
{
	0,	0,	0,	5,	10,	15,	20,	25,	30,	35,
	40,	50,	60,	80,	100,	120,	140,	160,	180,	200,
	220,	240,	255,	255,	255,	255,	255,	255,	255,	255,	
	255,	255,	255,	255,	255,	255,	255,	255,	255,	255,	
	255,	255,	255,	255,	255,	255,	255,	255,	255,	255,	
	255,	255,	255,	255,	255,	255,	255,	255,	255,	255,	
	255,	255,	255,	255,	255,	255,	255,	255,	255,	255,	
	255,	255,	255,	255,	255,	255,	255,	255,	255,	255,	
	249,	248,	247,	246,	245,	244,	243,	242,	241,	240,	
	239,	238,	237,	236,	235,	234,	233,	232,	231,	230,	


	229,	228,	227,	226,	225,	224,	223,	222,	221,	220,	
	219,	218,	217,	216,	215,	214,	213,	212,	211,	210,	
	209,	208,	207,	206,	205,	204,	203,	202,	201,	200,	
	199,	198,	197,	196,	195,	194,	193,	192,	191,	190,	
	189,	188,	187,	186,	185,	184,	183,	182,	181,	180,	
	179,	178,	177,	176,	175,	174,	173,	172,	171,	170,	
	169,	168,	167,	166,	165,	164,	163,	162,	161,	160,	
	159,	158,	157,	156,	155,	154,	153,	152,	151,	150,	
	149,	148,	147,	146,	145,	144,	143,	142,	141,	140,	
	139,	138,	137,	136,	135,	134,	133,	132,	131,	130,	

	129,	128,	127,	126,	125,	124,	123,	122,	121,	120,	
	119,	118,	117,	116,	115,	114,	113,	112,	111,	110,	
	109,	108,	107,	106,	105,	104,	103,	102,	101,	100,	
	99,	98,	97,	96,	95,	94,	93,	92,	91,	90,	
	89,	88,	87,	86,	85,	84,	83,	82,	81,	80,	
	79,	78,	77,	76,	75,	0 /*overexp*/
};


/*
	ampl and dist are expected to contain 3*TOF_XS*TOF_YS elements.
*/
void tof_calc_dist_3hdr_with_ignore(uint16_t* dist_out, uint8_t* ampl, uint16_t* dist, uint8_t* ignore_in)
{
	for(int yy=0; yy < TOF_YS; yy++)
	{
		for(int xx=0; xx < TOF_XS; xx++)
		{
			int pxidx = yy*TOF_XS+xx;
			if(     // On ignore list: either directly, or on any 8 neighbors.
				ignore_in[pxidx] ||
				( yy>0        &&    ( ignore_in[(yy-1)*TOF_XS+xx] || (xx>0 && ignore_in[(yy-1)*TOF_XS+xx-1]) || (xx<TOF_XS-1 && ignore_in[(yy-1)*TOF_XS+xx+1]) ) ) ||
				( yy<TOF_YS-1 &&    ( ignore_in[(yy+1)*TOF_XS+xx] || (xx>0 && ignore_in[(yy+1)*TOF_XS+xx-1]) || (xx<TOF_XS-1 && ignore_in[(yy+1)*TOF_XS+xx+1]) ) ) ||
				( xx>0        &&    ignore_in[yy*TOF_XS+xx-1] ) ||
				( xx<TOF_XS-1 &&    ignore_in[yy*TOF_XS+xx+1] )
			  )
			{
				dist_out[pxidx] = 0;
			}
			else if(ampl[0*TOF_XS*TOF_YS+pxidx] == 255) // Shortest exposure overexposed
			{
				dist_out[pxidx] = 1;
			}
			else
			{
				int32_t suit0 = ampl_suitability[ampl[0*TOF_XS*TOF_YS+pxidx]];
				int32_t suit1 = ampl_suitability[ampl[1*TOF_XS*TOF_YS+pxidx]];
				int32_t suit2 = ampl_suitability[ampl[2*TOF_XS*TOF_YS+pxidx]];

				int32_t dist0 = dist[0*TOF_XS*TOF_YS+pxidx];
				int32_t dist1 = dist[1*TOF_XS*TOF_YS+pxidx];
				int32_t dist2 = dist[2*TOF_XS*TOF_YS+pxidx];

				int32_t suit_sum = suit0 + suit1 + suit2;

				if(suit_sum < 10)
					dist_out[pxidx] = 0;
				else
				{
					dist_out[pxidx] = (dist0*suit0 + dist1*suit1 + dist2*suit2)/suit_sum;
				}				

			}

		}
	}
}


#define STRAY_CORR_LEVEL 1024 // smaller -> more correction. 2000 for a long time
#define STRAY_BLANKING_LVL 40 // smaller -> more easily ignored, was 40 for a long time

#define STRAY_CORR_FACTOR 64 // bigger -> do more correction - has been 16 for a long time

/*
	ampl and dist are expected to contain 3*TOF_XS*TOF_YS elements: three images at different exposures that vary by HDR_EXP_MULTIPLIER
*/
void tof_calc_dist_3hdr_with_ignore_with_straycomp(uint16_t* dist_out, uint8_t* ampl, uint16_t* dist, uint8_t* ignore_in, uint16_t stray_ampl, uint16_t stray_dist)
{
	int hdr_multiplier = 5;
	int stray_ignore = stray_ampl/STRAY_BLANKING_LVL;
	if(stray_ignore < 5) stray_ignore = 5;
	for(int yy=0; yy < TOF_YS; yy++)
	{
		for(int xx=0; xx < TOF_XS; xx++)
		{
			int pxidx          = yy*TOF_XS+xx;
			if(     // On ignore list: either directly, or on any 8 neighbors.
				ignore_in[pxidx] ||
				( yy>0        &&    ( ignore_in[(yy-1)*TOF_XS+xx] || (xx>0 && ignore_in[(yy-1)*TOF_XS+xx-1]) || (xx<TOF_XS-1 && ignore_in[(yy-1)*TOF_XS+xx+1]) ) ) ||
				( yy<TOF_YS-1 &&    ( ignore_in[(yy+1)*TOF_XS+xx] || (xx>0 && ignore_in[(yy+1)*TOF_XS+xx-1]) || (xx<TOF_XS-1 && ignore_in[(yy+1)*TOF_XS+xx+1]) ) ) ||
				( xx>0        &&    ignore_in[yy*TOF_XS+xx-1] ) ||
				( xx<TOF_XS-1 &&    ignore_in[yy*TOF_XS+xx+1] )
			  )
			{
				dist_out[pxidx] = 0;
			}
			else if(ampl[0*TOF_XS*TOF_YS+pxidx] == 255) // Shortest exposure overexposed
			{
				dist_out[pxidx] = 3;
			}
			else
			{
				int32_t suit0 = ampl_suitability[ampl[0*TOF_XS*TOF_YS+pxidx]];
				int32_t suit1 = ampl_suitability[ampl[1*TOF_XS*TOF_YS+pxidx]];
				int32_t suit2 = ampl_suitability[ampl[2*TOF_XS*TOF_YS+pxidx]];

				int32_t dist0 = dist[0*TOF_XS*TOF_YS+pxidx];
				int32_t dist1 = dist[1*TOF_XS*TOF_YS+pxidx];
				int32_t dist2 = dist[2*TOF_XS*TOF_YS+pxidx];

//				int32_t ampl0 = ampl[0*TOF_XS*TOF_YS+pxidx]*HDR_EXP_MULTIPLIER*HDR_EXP_MULTIPLIER;
//				int32_t ampl1 = ampl[1*TOF_XS*TOF_YS+pxidx]*HDR_EXP_MULTIPLIER;
				int32_t ampl0 = ampl[0*TOF_XS*TOF_YS+pxidx]*hdr_multiplier*hdr_multiplier;
				int32_t ampl1 = ampl[1*TOF_XS*TOF_YS+pxidx]*hdr_multiplier;
				int32_t ampl2 = ampl[2*TOF_XS*TOF_YS+pxidx];

				int32_t suit_sum = suit0 + suit1 + suit2;

				if(suit_sum < 10)
					dist_out[pxidx] = 0;
				else
				{
					int32_t combined_ampl = (ampl0*suit0 + ampl1*suit1 + ampl2*suit2)/suit_sum;
					int32_t combined_dist = (dist0*suit0 + dist1*suit1 + dist2*suit2)/suit_sum;

					int32_t corr_amount = ((STRAY_CORR_FACTOR*(int32_t)stray_ampl)/(int32_t)combined_ampl);

					// Expect higher amplitude from "close" pixels. If the amplitude is low, they are artefacts most likely.
					int32_t expected_ampl;
					if(combined_dist > 2000)
						expected_ampl = stray_ignore;
					else
						expected_ampl = (2000*stray_ignore)/combined_dist;

					if(corr_amount > 1000 || corr_amount < -300 || combined_ampl < expected_ampl)
					{
						dist_out[pxidx] = 0;
					}
					else
					{
						combined_dist -= stray_dist;

						combined_dist *= STRAY_CORR_LEVEL + corr_amount;
						combined_dist /= STRAY_CORR_LEVEL;

						combined_dist += stray_dist;

						if(combined_dist < 1) combined_dist = 1;
						else if(combined_dist > 6000) combined_dist = 6000;

						dist_out[pxidx] = combined_dist;
					}
				}				

			}

		}
	}
}

/*
	threshold = 250mm
	Starts to eat some areas away, but would be perfectly usable if good filtration is important

	threshold = 500mm
	Doesn't filter all real-world midliers
*/
#define MIDLIER_THRESHOLD 350
void tof_remove_midliers(uint16_t* out, uint16_t* in)
{
	memset(out, 0, 2*TOF_XS*TOF_YS);
	for(int xx=0; xx<TOF_XS; xx++)
	{
		for(int yy=0; yy<TOF_YS; yy++)
		{
			int pxval = in[yy*TOF_XS+xx];

			int min = pxval-MIDLIER_THRESHOLD;
			int max = pxval+MIDLIER_THRESHOLD;

			for(int xxx=xx-1; xxx<=xx+1; xxx++)
			{
				for(int yyy=yy-1; yyy<=yy+1; yyy++)
				{
					if((xxx>=0 && xxx<TOF_XS && yyy>=0 && yyy<TOF_YS) && (in[yyy*TOF_XS+xxx] < min || in[yyy*TOF_XS+xxx] > max))
						goto MIDLIER_FOUND;
				}
			}

			out[yy*TOF_XS+xx] = pxval;
			MIDLIER_FOUND: continue;
		}
	}
}

void process_bw(uint8_t *out, epc_img_t *in)  __attribute__((section(".text_itcm")));
void process_bw(uint8_t *out, epc_img_t *in)
{
	for(int i=0; i<TOF_XS*TOF_YS; i++)
	{
		int lum = ((in->img[i]&0b0011111111111100)>>2)-2048;
		if(lum < 0) lum = 0; else if(lum > 2047) lum = 2047;
		out[i] = lum>>3;
	}
}

void process_dcs(int16_t *out, epc_img_t *in)  __attribute__((section(".text_itcm")));
void process_dcs(int16_t *out, epc_img_t *in)
{
	for(int i=0; i<TOF_XS*TOF_YS; i++)
	{
		out[i] = ((in->img[i]&0b0011111111111100)>>2)-2048;
	}
}

void process_dcs_narrow(int16_t *out, epc_img_narrow_t *in)  __attribute__((section(".text_itcm")));
void process_dcs_narrow(int16_t *out, epc_img_narrow_t *in)
{
	for(int i=0; i<TOF_XS_NARROW*TOF_YS_NARROW; i++)
	{
		out[i] = ((in->img[i]&0b0011111111111100)>>2)-2048;
	}
}

