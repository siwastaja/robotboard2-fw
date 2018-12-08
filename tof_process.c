#include <stdint.h>
#include <string.h>
#include <math.h>

#include "ext_include/stm32h7xx.h"
#include "stm32_cmsis_extension.h"
#include "tof_process.h"
#include "tof_table.h"
#include "misc.h"

#include "own_std.h"

#include "flash.h" // for offset address

#include "sin_lut.h"
#include "geotables.h"

#include "drive.h"

#include "../robotsoft/api_board_to_soft.h"

#define FLASH_SENSOR0  (FLASH_OFFSET + (0*8+2)*128*1024)  // bank1 sector2
#define FLASH_SENSOR5  (FLASH_OFFSET + (1*8+2)*128*1024)  // bank2 sector2



static char printbuf[128];


// Sensors 0..4 fill flash bank1 sector2..sector7 completely
// Sensors 5..9 fill flash bank2 sector2..sector7 completely
const tof_calib_t * const tof_calibs[N_SENSORS] =
{
	(tof_calib_t*)(FLASH_SENSOR0 + 0*TOFCAL_SIZE),
	(tof_calib_t*)(FLASH_SENSOR0 + 1*TOFCAL_SIZE),
	(tof_calib_t*)(FLASH_SENSOR0 + 2*TOFCAL_SIZE),
	(tof_calib_t*)(FLASH_SENSOR0 + 3*TOFCAL_SIZE),
	(tof_calib_t*)(FLASH_SENSOR0 + 4*TOFCAL_SIZE),
	(tof_calib_t*)(FLASH_SENSOR5 + 0*TOFCAL_SIZE),
	(tof_calib_t*)(FLASH_SENSOR5 + 1*TOFCAL_SIZE),
	(tof_calib_t*)(FLASH_SENSOR5 + 2*TOFCAL_SIZE),
	(tof_calib_t*)(FLASH_SENSOR5 + 3*TOFCAL_SIZE),
	(tof_calib_t*)(FLASH_SENSOR5 + 4*TOFCAL_SIZE)
};

// Transferred from flash by DMA. Can share the same memory, used alternately
// For once, I'm using union for what is was originally meant in the C standard!! :).
union
{
	chipcal_hifreq_t hif;
	chipcal_lofreq_t lof;
} shadow_luts __attribute__((section(".dtcm_bss")));

// To be replaced with DMA
void copy_cal_to_shadow(int sid, int f)
{
	if(f<0 || f>3) error(123);
	if(f<2)
	{
//		DBG_PR_VAR_U32_HEX((uint32_t)&tof_calibs[sid]->hif[f]);
//		DBG_PR_VAR_U32_HEX((uint32_t)&shadow_luts.hif);
//		DBG_PR_VAR_U32((uint32_t)sizeof tof_calibs[0]->hif[0]);
		memcpy(&shadow_luts.hif, &tof_calibs[sid]->hif[f], sizeof shadow_luts.hif);
	}
	else
	{
		memcpy(&shadow_luts.lof, &tof_calibs[sid]->lof[f-2], sizeof shadow_luts.lof);
	}
}

/*
	lookup_dist:
	Gives the complete precalculated distance which includes full non-linearity and offset calibration.
	Inputs: g = pixel group index, dcs3-dcs1, and dcs2-dcs0.
	

	beam must be compile-time constant to optimize out. This "if" thing is needed for wide/narrow lookup, they need to look at
	a different types of tables because N_PIXGROUP is different.
*/

// beam must be compile-time constants to optimize out
static inline uint16_t lookup_dist(int beam, int g, int16_t d31, int16_t d20) __attribute__((always_inline));
static inline uint16_t lookup_dist(int beam, int g, int16_t d31, int16_t d20)
{
	int s;
	int i;
	if(d31 >= 0)
	{
		if(d20 >= 0)
		{
			if(d31 > d20) {s=5; i=((TOF_TBL_SEG_LEN-1)*d20)/d31;}
			else          {s=4; i=((TOF_TBL_SEG_LEN-1)*d31)/d20;}
		}
		else
		{
			d20 *= -1;
			if(d31 > d20) {s=6; i=((TOF_TBL_SEG_LEN-1)*d20)/d31;}
			else          {s=7; i=((TOF_TBL_SEG_LEN-1)*d31)/d20;}
		}		
	}
	else
	{
		d31 *= -1;
		if(d20 >= 0)
		{
			if(d31 > d20) {s=2; i=((TOF_TBL_SEG_LEN-1)*d20)/d31;}
			else          {s=3; i=((TOF_TBL_SEG_LEN-1)*d31)/d20;}
		}
		else
		{
			d20 *= -1;
			if(d31 > d20) {s=1; i=((TOF_TBL_SEG_LEN-1)*d20)/d31;}
			else          {s=0; i=((TOF_TBL_SEG_LEN-1)*d31)/d20;}
		}		

	}

	if(beam == 0)
		return shadow_luts.hif.wid_luts[g][s][i];
	else
		return shadow_luts.hif.nar_luts[g][s][i];
}


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


#define FAST_APPROX_AMPLITUDE
//#define DO_AMB_CORR

//#define DBGPR
//#define PIX (30*160+80)

void conv_4dcs_to_2dcs(int16_t *dcs20_out, int16_t *dcs31_out, epc_4dcs_t *in, epc_img_t *bwimg)  __attribute__((section(".text_itcm")));
void conv_4dcs_to_2dcs(int16_t *dcs20_out, int16_t *dcs31_out, epc_4dcs_t *in, epc_img_t *bwimg)
{
	for(int i=0; i < TOF_XS*TOF_YS; i++)
	{
		uint16_t dist;
		int ampl;

		int16_t dcs0 = ((in->dcs[0].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs1 = ((in->dcs[1].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs2 = ((in->dcs[2].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs3 = ((in->dcs[3].img[i]&0b0011111111111100)>>2)-2048;

		int16_t dcs31 = dcs3-dcs1;
		int16_t dcs20 = dcs2-dcs0;

		dcs20_out[i] = dcs20;
		dcs31_out[i] = dcs31;
	}
}

#define HDR_FACTOR 8
void compensated_hdr_tof_calc_dist_ampl(uint8_t *ampl_out, uint16_t *dist_out, int16_t* dcs20_lo_in, int16_t* dcs31_lo_in, int16_t* dcs20_hi_in, int16_t* dcs31_hi_in)   __attribute__((section(".text_itcm")));
void compensated_hdr_tof_calc_dist_ampl(uint8_t *ampl_out, uint16_t *dist_out, int16_t* dcs20_lo_in, int16_t* dcs31_lo_in, int16_t* dcs20_hi_in, int16_t* dcs31_hi_in)
{
	for(int i=0; i < TOF_XS*TOF_YS; i++)
	{
		uint16_t dist;
		int ampl;

		int16_t dcs20_lo = dcs20_lo_in[i];
		int16_t dcs31_lo = dcs31_lo_in[i];
		int16_t dcs20_hi = dcs20_hi_in[i];
		int16_t dcs31_hi = dcs31_hi_in[i];


		if(dcs20_lo < -4000 || dcs20_lo > 4000 || dcs31_lo < -4000 || dcs31_lo > 4000)
		{
			ampl = 255;
			dist = 0;
		}
		else
		{
			int16_t dcs20, dcs31;

			if(dcs20_hi < -3500 || dcs20_hi > 3500 || dcs31_hi < -3500 || dcs31_hi > 3500)
			{
				dcs20 = dcs20_lo*HDR_FACTOR;
				dcs31 = dcs31_lo*HDR_FACTOR;
			}
			else
			{
				dcs20 = dcs20_hi;
				dcs31 = dcs31_hi;
			}

			#ifdef FAST_APPROX_AMPLITUDE
				ampl = (abso(dcs20)+abso(dcs31))/(23);// if(ampl > 255) ampl = 255;
			#else
				ampl = sqrt(sq(dcs20)+sq(dcs31))/(17);// if(ampl > 255) ampl = 255;
			#endif

			if(ampl<4)
			{
				dist=65534;
			}
			else
			{
				int pixgroup;
				if(!(i&1)) // even
					pixgroup = shadow_luts.hif.wid_lut_group_ids[i/2] & 0x0f;
				else
					pixgroup = shadow_luts.hif.wid_lut_group_ids[i/2]>>4;

				dist = lookup_dist(0, pixgroup, dcs31, dcs20);
			}
		}
		ampl /= HDR_FACTOR; if(ampl>255) ampl=255;
		ampl_out[i] = ampl;
		dist_out[i] = dist;
	}
}

int flare_factor = 11;

void adjust()
{
	uint8_t cmd = uart_input();

	if(cmd == 'a')
	{
		flare_factor++;
	}
	else if(cmd == 'z')
	{
		flare_factor--;
	}
	else if(cmd == 's')
	{
		flare_factor+=10;
	}
	else if(cmd == 'x')
	{
		flare_factor-=10;
	}

}

void compensated_hdr_tof_calc_dist_ampl_flarecomp(uint8_t *ampl_out, uint16_t *dist_out, int16_t* dcs20_lo_in, int16_t* dcs31_lo_in, int16_t* dcs20_hi_in, int16_t* dcs31_hi_in)   __attribute__((section(".text_itcm")));
void compensated_hdr_tof_calc_dist_ampl_flarecomp(uint8_t *ampl_out, uint16_t *dist_out, int16_t* dcs20_lo_in, int16_t* dcs31_lo_in, int16_t* dcs20_hi_in, int16_t* dcs31_hi_in)
{
	int dcs20_accum = 0;
	int dcs31_accum = 0;
	for(int i=0; i < TOF_XS*TOF_YS; i++)
	{
		int16_t dcs20_lo = dcs20_lo_in[i];
		int16_t dcs31_lo = dcs31_lo_in[i];
		int16_t dcs20_hi = dcs20_hi_in[i];
		int16_t dcs31_hi = dcs31_hi_in[i];

		int32_t dcs20, dcs31;

		if(dcs20_hi < -3500 || dcs20_hi > 3500 || dcs31_hi < -3500 || dcs31_hi > 3500)
		{
			dcs20 = dcs20_lo*HDR_FACTOR;
			dcs31 = dcs31_lo*HDR_FACTOR;
		}
		else
		{
			dcs20 = dcs20_hi;
			dcs31 = dcs31_hi;
		}

		dcs20_accum += dcs20;
		dcs31_accum += dcs31;
	}

	dcs20_accum /= TOF_XS*TOF_YS;
	dcs31_accum /= TOF_XS*TOF_YS;

	DBG_PR_VAR_I32(flare_factor);
	DBG_PR_VAR_I32(dcs20_accum);
	DBG_PR_VAR_I32(dcs31_accum);

	for(int i=0; i < TOF_XS*TOF_YS; i++)
	{
		uint16_t dist;
		int ampl;

		int16_t dcs20_lo = dcs20_lo_in[i];
		int16_t dcs31_lo = dcs31_lo_in[i];
		int16_t dcs20_hi = dcs20_hi_in[i];
		int16_t dcs31_hi = dcs31_hi_in[i];


		if(dcs20_lo < -4000 || dcs20_lo > 4000 || dcs31_lo < -4000 || dcs31_lo > 4000)
		{
			ampl = 255;
			dist = 0;
		}
		else
		{
			int16_t dcs20, dcs31;

			if(dcs20_hi < -3500 || dcs20_hi > 3500 || dcs31_hi < -3500 || dcs31_hi > 3500)
			{
				dcs20 = dcs20_lo*HDR_FACTOR;
				dcs31 = dcs31_lo*HDR_FACTOR;
			}
			else
			{
				dcs20 = dcs20_hi;
				dcs31 = dcs31_hi;
			}

			dcs31 -= ((int64_t)dcs31_accum*(int64_t)flare_factor)>>8;
			dcs20 -= ((int64_t)dcs20_accum*(int64_t)flare_factor)>>8;
			

			#ifdef FAST_APPROX_AMPLITUDE
				ampl = (abso(dcs20)+abso(dcs31))/(23);// if(ampl > 255) ampl = 255;
			#else
				ampl = sqrt(sq(dcs20)+sq(dcs31))/(17);// if(ampl > 255) ampl = 255;
			#endif

			if(ampl<4)
			{
				dist = 65534;
			}
			else
			{
				int pixgroup;
				if(!(i&1)) // even
					pixgroup = shadow_luts.hif.wid_lut_group_ids[i/2] & 0x0f;
				else
					pixgroup = shadow_luts.hif.wid_lut_group_ids[i/2]>>4;

				dist = lookup_dist(0, pixgroup, dcs31, dcs20);
			}
		}
		ampl /= HDR_FACTOR; if(ampl>255) ampl=255;
		ampl_out[i] = ampl;
		dist_out[i] = dist;
	}
}


void compensated_tof_calc_dist_ampl(uint8_t *max_ampl_out, uint8_t *ampl_out, uint16_t *dist_out, epc_4dcs_t *in, epc_img_t *bwimg) __attribute__((section(".text_itcm")));
void compensated_tof_calc_dist_ampl(uint8_t *max_ampl_out, uint8_t *ampl_out, uint16_t *dist_out, epc_4dcs_t *in, epc_img_t *bwimg)
{
//	uint32_t avg_ampl = 0;
	uint8_t max_ampl = 0;
	for(int i=0; i < TOF_XS*TOF_YS; i++)
	{
		uint16_t dist;
		int ampl;

		int16_t dcs0 = ((in->dcs[0].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs1 = ((in->dcs[1].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs2 = ((in->dcs[2].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs3 = ((in->dcs[3].img[i]&0b0011111111111100)>>2)-2048;

		#ifdef DBGPR
			if(i == PIX)
			{
				DBG_PR_VAR_I16(dcs0);
				DBG_PR_VAR_I16(dcs1);
				DBG_PR_VAR_I16(dcs2);
				DBG_PR_VAR_I16(dcs3);
			}
		#endif

		if(dcs0 < -2047 || dcs0 > 2046 || dcs1 < -2047 || dcs1 > 2046 || dcs2 < -2047 || dcs2 > 2046 || dcs3 < -2047 || dcs3 > 2046)
		{
			ampl = 255;
			dist = 0;
		}
		else
		{
			int16_t dcs31 = dcs3-dcs1;
			int16_t dcs20 = dcs2-dcs0;

			#ifdef DBGPR
				if(i == PIX)
				{
					DBG_PR_VAR_I16(dcs31);
					DBG_PR_VAR_I16(dcs20);
				}
			#endif
			#ifdef DO_AMB_CORR
				int16_t bw = ((bwimg->img[i]&0b0011111111111100)>>2)-2048;
				if(bw<0) bw=0;
				int corr_factor;
				if(!(i&1)) // even
					corr_factor = shadow_luts.hif.amb_corr[i/2] & 0x0f;
				else
					corr_factor = shadow_luts.hif.amb_corr[i/2]>>4;
				int16_t comp = (bw*corr_factor)>>4;

				#ifdef DBGPR
						if(i == PIX)
						{
							DBG_PR_VAR_I16(bw);
							DBG_PR_VAR_I16(corr_factor);
							DBG_PR_VAR_I16(comp);
						}
				#endif

				dcs31 -= comp;
				dcs20 -= comp;
			#endif


			#ifdef FAST_APPROX_AMPLITUDE
				ampl = (abso(dcs20)+abso(dcs31))/23; if(ampl > 255) ampl = 255;
			#else
				ampl = sqrt(sq(dcs20)+sq(dcs31))/17; if(ampl > 255) ampl = 255;
			#endif

			if(ampl<4)
			{
				dist = 65534;
			}
			else
			{
				int pixgroup;
				if(!(i&1)) // even
					pixgroup = shadow_luts.hif.wid_lut_group_ids[i/2] & 0x0f;
				else
					pixgroup = shadow_luts.hif.wid_lut_group_ids[i/2]>>4;

				dist = lookup_dist(0, pixgroup, dcs31, dcs20);

				#ifdef DBGPR

					if(i == PIX)
					{
						DBG_PR_VAR_I16(pixgroup);
						DBG_PR_VAR_U16(dist);
					}

				#endif

			}
		}
		ampl_out[i] = ampl;
		dist_out[i] = dist;
//		avg_ampl += ampl;
		if(ampl > max_ampl) max_ampl = ampl;
	}

//	if(avg_ampl_out != NULL)
//		*avg_ampl_out = avg_ampl/(TOF_XS*TOF_YS);

	if(max_ampl_out != NULL)
		*max_ampl_out = max_ampl;
}

#if 0
void compensated_tof_calc_dist_ampl_2dcs(uint8_t *ampl_out, uint16_t *dist_out, epc_2dcs_t *in, epc_img_t *bwimg) __attribute__((section(".text_itcm")));
void compensated_tof_calc_dist_ampl_2dcs(uint8_t *ampl_out, uint16_t *dist_out, epc_2dcs_t *in, epc_img_t *bwimg)
{
//	uint32_t avg_ampl = 0;
	uint8_t max_ampl = 0;
	for(int i=0; i < TOF_XS*TOF_YS; i++)
	{
		uint16_t dist;
		int ampl;

		int16_t dcs0 = ((in->dcs[0].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs1 = ((in->dcs[1].img[i]&0b0011111111111100)>>2)-2048;

		#ifdef DBGPR
			if(i == PIX)
			{
				DBG_PR_VAR_I16(dcs0);
				DBG_PR_VAR_I16(dcs1);
				DBG_PR_VAR_I16(dcs2);
				DBG_PR_VAR_I16(dcs3);
			}
		#endif

		if(dcs0 < -2047 || dcs0 > 2046 || dcs1 < -2047 || dcs1 > 2046)
		{
			ampl = 255;
			dist = 0;
		}
		else
		{
			int16_t dcs31 = -1*dcs1;
			int16_t dcs20 = -1*dcs0;

			#ifdef DBGPR
				if(i == PIX)
				{
					DBG_PR_VAR_I16(dcs31);
					DBG_PR_VAR_I16(dcs20);
				}
			#endif
			#ifdef DO_AMB_CORR
				int16_t bw = ((bwimg->img[i]&0b0011111111111100)>>2)-2048;
				if(bw<0) bw=0;
				int corr_factor;
				if(!(i&1)) // even
					corr_factor = shadow_luts.hif.amb_corr[i/2] & 0x0f;
				else
					corr_factor = shadow_luts.hif.amb_corr[i/2]>>4;
				int16_t comp = (bw*corr_factor)>>4;

				#ifdef DBGPR
						if(i == PIX)
						{
							DBG_PR_VAR_I16(bw);
							DBG_PR_VAR_I16(corr_factor);
							DBG_PR_VAR_I16(comp);
						}
				#endif

				dcs31 -= comp;
				dcs20 -= comp;
			#endif


			#ifdef FAST_APPROX_AMPLITUDE
				ampl = (abso(dcs20)+abso(dcs31))/23; if(ampl > 255) ampl = 255;
			#else
				ampl = sqrt(sq(dcs20)+sq(dcs31))/17; if(ampl > 255) ampl = 255;
			#endif

//			if(ampl<4)
//			{
//				dist = 65534;
//			}
//			else
			{
				int pixgroup;
				if(!(i&1)) // even
					pixgroup = shadow_luts.hif.wid_lut_group_ids[i/2] & 0x0f;
				else
					pixgroup = shadow_luts.hif.wid_lut_group_ids[i/2]>>4;

				dist = lookup_dist(0, pixgroup, dcs31, dcs20);

				#ifdef DBGPR

					if(i == PIX)
					{
						DBG_PR_VAR_I16(pixgroup);
						DBG_PR_VAR_U16(dist);
					}

				#endif

			}
		}
		ampl_out[i] = ampl;
		dist_out[i] = dist;
//		avg_ampl += ampl;
		if(ampl > max_ampl) max_ampl = ampl;
	}

//	if(avg_ampl_out != NULL)
//		*avg_ampl_out = avg_ampl/(TOF_XS*TOF_YS);

	if(max_ampl_out != NULL)
		*max_ampl_out = max_ampl;
}
#endif


void compensated_tof_calc_dist_ampl_narrow(uint8_t *max_ampl_out, uint8_t *ampl_out, uint16_t *dist_out, epc_4dcs_narrow_t *in, epc_img_t *bwimg) __attribute__((section(".text_itcm")));
void compensated_tof_calc_dist_ampl_narrow(uint8_t *max_ampl_out, uint8_t *ampl_out, uint16_t *dist_out, epc_4dcs_narrow_t *in, epc_img_t *bwimg)
{
//	uint32_t avg_ampl = 0;
	uint8_t max_ampl = 0;
	for(int i=0; i < TOF_XS_NARROW*TOF_YS_NARROW; i++)
	{
		uint16_t dist;
		int ampl;

		int16_t dcs0 = ((in->dcs[0].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs1 = ((in->dcs[1].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs2 = ((in->dcs[2].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs3 = ((in->dcs[3].img[i]&0b0011111111111100)>>2)-2048;

		#ifdef DBGPR
			if(i == PIX)
			{
				DBG_PR_VAR_I16(dcs0);
				DBG_PR_VAR_I16(dcs1);
				DBG_PR_VAR_I16(dcs2);
				DBG_PR_VAR_I16(dcs3);
			}
		#endif

		if(dcs0 < -2047 || dcs0 > 2046 || dcs1 < -2047 || dcs1 > 2046 || dcs2 < -2047 || dcs2 > 2046 || dcs3 < -2047 || dcs3 > 2046)
		{
			ampl = 255;
			dist = 0;
		}
		else
		{
			int16_t dcs31 = dcs3-dcs1;
			int16_t dcs20 = dcs2-dcs0;

			#ifdef DBGPR
				if(i == PIX)
				{
					DBG_PR_VAR_I16(dcs31);
					DBG_PR_VAR_I16(dcs20);
				}
			#endif
			#ifdef DO_AMB_CORR
				int16_t bw = ((bwimg->img[i]&0b0011111111111100)>>2)-2048;
				if(bw<0) bw=0;
				int corr_factor;
				if(!(i&1)) // even
					corr_factor = shadow_luts.hif.amb_corr[i/2] & 0x0f;
				else
					corr_factor = shadow_luts.hif.amb_corr[i/2]>>4;
				int16_t comp = (bw*corr_factor)>>4;

				#ifdef DBGPR
						if(i == PIX)
						{
							DBG_PR_VAR_I16(bw);
							DBG_PR_VAR_I16(corr_factor);
							DBG_PR_VAR_I16(comp);
						}
				#endif

				dcs31 -= comp;
				dcs20 -= comp;
			#endif


			#ifdef FAST_APPROX_AMPLITUDE
				ampl = (abso(dcs20)+abso(dcs31))/23; if(ampl > 255) ampl = 255;
			#else
				ampl = sqrt(sq(dcs20)+sq(dcs31))/17; if(ampl > 255) ampl = 255;
			#endif

//			if(ampl<4)
//			{
//				dist = 65534;
//			}
//			else
			{
				int pixgroup;
				if(!(i&1)) // even
					pixgroup = shadow_luts.hif.nar_lut_group_ids[i/2] & 0x0f;
				else
					pixgroup = shadow_luts.hif.nar_lut_group_ids[i/2]>>4;

				dist = lookup_dist(1, pixgroup, dcs31, dcs20);

				#ifdef DBGPR

					if(i == PIX)
					{
						DBG_PR_VAR_I16(pixgroup);
						DBG_PR_VAR_U16(dist);
					}

				#endif

			}
		}
		ampl_out[i] = ampl;
		dist_out[i] = dist;
		//avg_ampl += ampl;
		if(ampl > max_ampl) max_ampl = ampl;

	}

//	if(avg_ampl_out != NULL)
//		*avg_ampl_out = avg_ampl/(TOF_XS*TOF_YS);

	if(max_ampl_out != NULL)
		*max_ampl_out = max_ampl;
}

// Positive *corr: narrow is longer - wide is too short.
int32_t calc_widnar_correction(int32_t* corr, uint8_t *wid_ampl, uint16_t *wid_dist, uint8_t *nar_ampl, uint16_t *nar_dist)  __attribute__((section(".text_itcm")));
int32_t calc_widnar_correction(int32_t* corr, uint8_t *wid_ampl, uint16_t *wid_dist, uint8_t *nar_ampl, uint16_t *nar_dist)
{
	if(corr == NULL)
		return -999999;

	int cnt = 0;
	int32_t wid_dist_acc = 0;
	int32_t nar_dist_acc = 0;
	for(int nyy=0; nyy<TOF_YS_NARROW; nyy++)
	{
		for(int nxx=0; nxx<TOF_XS_NARROW; nxx++)
		{
			int wxx = nxx + TOF_NARROW_X_START;
			int wyy = nyy + TOF_NARROW_Y_START;

			uint16_t wampl = wid_ampl[wyy*TOF_XS+wxx];
			uint16_t nampl = nar_ampl[nyy*TOF_XS_NARROW+nxx];
			if(wampl > 12 && nampl > 12 && wampl < 255 && nampl < 255 && nampl < (wampl<<1) && nampl > (wampl>>1))
			{
				cnt++;
				wid_dist_acc += wid_dist[wyy*TOF_XS+wxx];
				nar_dist_acc += nar_dist[nyy*TOF_XS_NARROW+nxx];
			}
		}
	}

	if(cnt > 0)
		*corr = (nar_dist_acc-wid_dist_acc)/cnt;

	return cnt;
}


/*
typedef struct
{
	int32_t mount_mode;             // mount position 1,2,3 or 4
	int32_t x_rel_robot;          // zero = robot origin. Positive = robot front (forward)
	int32_t y_rel_robot;          // zero = robot origin. Positive = to the right of the robot
	uint16_t ang_rel_robot;        // zero = robot forward direction. positive = ccw
	uint16_t vert_ang_rel_ground;  // zero = looks directly forward. positive = looks up. negative = looks down
	int32_t z_rel_ground;         // sensor height from the ground	
} sensor_mount_t;
*/

/*
	Sensor mount position 1:
	 _ _
	| | |
	| |L|
	|O|L|
	| |L|
	|_|_|  (front view)

	Sensor mount position 2:
	 _ _
	| | |
	|L| |
	|L|O|
	|L| |
	|_|_|  (front view)

	Sensor mount position 3:

	-------------
	|  L  L  L  |
	-------------
	|     O     |
	-------------

	Sensor mount position 4:

	-------------
	|     O     |
	-------------
	|  L  L  L  |
	-------------
*/



// const
sensor_mount_t sensor_mounts[N_SENSORS] =
{          //      mountmode    x     y       hor ang           ver ang      height    
 /*0:                */ { 0,     0,     0, DEGTOANG16(       0), DEGTOANG16( 3),   0 },
 /*1:                */ { 1,   130,   103, DEGTOANG16(      23),    728+500,         300 },
 /*2:                */ { 2,  -235,   215, DEGTOANG16(   90-23),    273+500,         300  },
 /*3:                */ { 1,  -380,   215, DEGTOANG16(      90),    273+500,         300  },
 /*4:                */ { 2,  -522,   103, DEGTOANG16(  180-23),    546+500,         300  },
 /*5:                */ { 1,  -522,    35, DEGTOANG16(    180 ),   1092+500,         310  },
 /*6:                */ { 1,  -522,  -103, DEGTOANG16(  180+23),    728+500,         300  },
 /*7:                */ { 2,  -380,  -215, DEGTOANG16(   270  ),    273+500,         290  },
 /*8:                */ { 1,  -235,  -215, DEGTOANG16(  270+23),    910+500,         280  },
 /*9:                */ { 2,   130,  -103, DEGTOANG16(  360-23),   135,        300  }
};

void recalc_sensor_mounts(int idx, int d_hor_ang, int d_ver_ang, int d_z)
{
	sensor_mounts[idx].ang_rel_robot += d_hor_ang;
	sensor_mounts[idx].vert_ang_rel_ground += d_ver_ang;
	sensor_mounts[idx].z_rel_ground += d_z;

	DBG_PR_VAR_I32(idx);
	DBG_PR_VAR_U16(sensor_mounts[idx].ang_rel_robot);
	DBG_PR_VAR_U16(sensor_mounts[idx].vert_ang_rel_ground);
	DBG_PR_VAR_I32(sensor_mounts[idx].z_rel_ground);
}

int adjustings = 0;
#if 0
void adjust()
{
	uint8_t cmd = uart_input();

	if(cmd >= '0' && cmd <= '9')
		adjustings = cmd-'0';
	else if(cmd == 'a')
	{
		recalc_sensor_mounts(adjustings, 0, 91, 0);
	}
	else if(cmd == 'z')
	{
		recalc_sensor_mounts(adjustings, 0, -91, 0);
	}
	else if(cmd == 's')
	{
		recalc_sensor_mounts(adjustings, 0, 0, 10);
	}
	else if(cmd == 'x')
	{
		recalc_sensor_mounts(adjustings, 0, 0, -10);
	}
	else if(cmd == 'q')
	{
		recalc_sensor_mounts(adjustings, 91, 0, 0);
	}
	else if(cmd == 'w')
	{
		recalc_sensor_mounts(adjustings, -91, 0, 0);
	}

}
#endif
#define VOX_SEG_XS 100
#define VOX_SEG_YS 100
#define VOX_HIRES_UNIT 50 // mm
#define VOX_LORES_UNIT 100 // mm

full_voxel_map_t voxmap;

typedef struct
{
	int xmin;
	int xmax;
	int ymin;
	int ymax;
	int reso;
} seg_limits_t;

const seg_limits_t seg_lims[12] =
{
	{	// Seg 0
		0, VOX_SEG_XS*VOX_HIRES_UNIT-1,
		0, VOX_SEG_YS*VOX_HIRES_UNIT-1,
		VOX_HIRES_UNIT
	},

	{	// Seg 1
		-VOX_SEG_XS*VOX_HIRES_UNIT, -1,
		0, VOX_SEG_YS*VOX_HIRES_UNIT-1,
		VOX_HIRES_UNIT
	},

	{	// Seg 2
		-VOX_SEG_XS*VOX_HIRES_UNIT, -1,
		-VOX_SEG_YS*VOX_HIRES_UNIT, -1,
		VOX_HIRES_UNIT
	},

	{	// Seg 3
		0, VOX_SEG_XS*VOX_HIRES_UNIT-1,
		-VOX_SEG_YS*VOX_HIRES_UNIT, -1,
		VOX_HIRES_UNIT
	},

	{	// Seg 4
		VOX_SEG_XS*VOX_HIRES_UNIT, VOX_SEG_XS*VOX_HIRES_UNIT + VOX_SEG_XS*VOX_LORES_UNIT-1,
		-VOX_SEG_YS*VOX_HIRES_UNIT, VOX_SEG_YS*VOX_HIRES_UNIT-1,
		VOX_LORES_UNIT
	},

	{	// Seg 5
		VOX_SEG_XS*VOX_HIRES_UNIT, VOX_SEG_XS*VOX_HIRES_UNIT + VOX_SEG_XS*VOX_LORES_UNIT-1,
		VOX_SEG_YS*VOX_HIRES_UNIT, VOX_SEG_YS*VOX_HIRES_UNIT + VOX_SEG_YS*VOX_LORES_UNIT-1,
		VOX_LORES_UNIT
	},

	{	// Seg 6
		-VOX_SEG_XS*VOX_HIRES_UNIT, VOX_SEG_XS*VOX_HIRES_UNIT-1,
		VOX_SEG_YS*VOX_HIRES_UNIT, VOX_SEG_YS*VOX_HIRES_UNIT + VOX_SEG_YS*VOX_LORES_UNIT-1,
		VOX_LORES_UNIT
	},

	{	// Seg 7
		-VOX_SEG_XS*VOX_HIRES_UNIT - VOX_SEG_XS*VOX_LORES_UNIT, -VOX_SEG_XS*VOX_HIRES_UNIT-1,
		VOX_SEG_YS*VOX_HIRES_UNIT, VOX_SEG_YS*VOX_HIRES_UNIT + VOX_SEG_YS*VOX_LORES_UNIT-1,
		VOX_LORES_UNIT
	},

	{	// Seg 8
		-VOX_SEG_XS*VOX_HIRES_UNIT - VOX_SEG_XS*VOX_LORES_UNIT, -VOX_SEG_XS*VOX_HIRES_UNIT-1,
		-VOX_SEG_YS*VOX_HIRES_UNIT, VOX_SEG_YS*VOX_HIRES_UNIT-1,
		VOX_LORES_UNIT
	},

	{	// Seg 9
		-VOX_SEG_XS*VOX_HIRES_UNIT - VOX_SEG_XS*VOX_LORES_UNIT, -VOX_SEG_XS*VOX_HIRES_UNIT-1,
		-VOX_SEG_YS*VOX_HIRES_UNIT-VOX_SEG_YS*VOX_LORES_UNIT, -VOX_SEG_YS*VOX_HIRES_UNIT-1,
		VOX_LORES_UNIT
	},

	{	// Seg 10
		-VOX_SEG_XS*VOX_HIRES_UNIT, VOX_SEG_XS*VOX_HIRES_UNIT-1,
		-VOX_SEG_YS*VOX_HIRES_UNIT-VOX_SEG_YS*VOX_LORES_UNIT, -VOX_SEG_YS*VOX_HIRES_UNIT-1,
		VOX_LORES_UNIT
	},

	{	// Seg 11
		VOX_SEG_XS*VOX_HIRES_UNIT, VOX_SEG_XS*VOX_HIRES_UNIT + VOX_SEG_XS*VOX_LORES_UNIT-1,
		-VOX_SEG_YS*VOX_HIRES_UNIT-VOX_SEG_YS*VOX_LORES_UNIT, -VOX_SEG_YS*VOX_HIRES_UNIT-1,
		VOX_LORES_UNIT
	}

};


void tof_to_voxmap(uint8_t *wid_ampl, uint16_t *wid_dist, int32_t widnar_corr, int sidx, uint8_t ampl_accept_min, uint8_t ampl_accept_max, int32_t ref_x, int32_t ref_y) __attribute__((section(".text_itcm")));
void tof_to_voxmap(uint8_t *wid_ampl, uint16_t *wid_dist, int32_t widnar_corr, int sidx, uint8_t ampl_accept_min, uint8_t ampl_accept_max, int32_t ref_x, int32_t ref_y)
{
	if(sidx < 0 || sidx >= N_SENSORS) error(150);

	int32_t robot_x = cur_pos.x>>16;
	int32_t robot_y = cur_pos.y>>16;

/*
	DBG_PR_VAR_I32(ref_x);
	DBG_PR_VAR_I32(ref_y);

	DBG_PR_VAR_I32(robot_x);
	DBG_PR_VAR_I32(robot_y);
*/
	if(abso(robot_x-ref_x) > 3000 || abso(robot_y-ref_y) > 3000) error(151);

	uint16_t robot_ang = cur_pos.ang>>16;

	// Rotation: xr = x*cos(a) + y*sin(a)
	//           yr = -x*sin(a) + y*cos(a)
	// It seems to me this widely touted formula has inverted y axis, don't understand why, so it should be:
	// Rotation: xr = x*cos(a) - y*sin(a)
	//           yr = x*sin(a) + y*cos(a)


	uint16_t global_sensor_hor_ang = sensor_mounts[sidx].ang_rel_robot + robot_ang;
	uint16_t global_sensor_ver_ang = sensor_mounts[sidx].vert_ang_rel_ground;

	uint16_t local_sensor_hor_ang = sensor_mounts[sidx].ang_rel_robot;
	uint16_t local_sensor_ver_ang = sensor_mounts[sidx].vert_ang_rel_ground;

	int32_t  global_sensor_x = robot_x - ref_x +
			((lut_cos_from_u16(robot_ang)*sensor_mounts[sidx].x_rel_robot)>>SIN_LUT_RESULT_SHIFT) +
			((lut_sin_from_u16(robot_ang)*-1*sensor_mounts[sidx].y_rel_robot)>>SIN_LUT_RESULT_SHIFT);

	int32_t  global_sensor_y = robot_y - ref_y + 
			((lut_sin_from_u16(robot_ang)*sensor_mounts[sidx].x_rel_robot)>>SIN_LUT_RESULT_SHIFT) +
			((lut_cos_from_u16(robot_ang)*sensor_mounts[sidx].y_rel_robot)>>SIN_LUT_RESULT_SHIFT);
	int32_t  global_sensor_z = sensor_mounts[sidx].z_rel_ground;


	int32_t  local_sensor_x = sensor_mounts[sidx].x_rel_robot;
	int32_t  local_sensor_y = sensor_mounts[sidx].y_rel_robot;
	int32_t  local_sensor_z = sensor_mounts[sidx].z_rel_ground;



	#ifdef DBGPR

		DBG_PR_VAR_I32(robot_ang);
		DBG_PR_VAR_I32(robot_x);
		DBG_PR_VAR_I32(robot_y);

		DBG_PR_VAR_I32(global_sensor_hor_ang);
		DBG_PR_VAR_I32(global_sensor_ver_ang);
		DBG_PR_VAR_I32(global_sensor_x);
		DBG_PR_VAR_I32(global_sensor_y);
		DBG_PR_VAR_I32(global_sensor_z);
	#endif


	extern int obstacle_front, obstacle_back, obstacle_left, obstacle_right;

	int insertion_cnt = 0;
	for(int py=1; py<TOF_YS-1; py++)
//	for(int py=29; py<32; py++)
	{
		for(int px=1; px<TOF_XS-1; px++)
//		for(int px=75; px<85; px++)
//		for(int px=79; px<82; px++)
		{
			int32_t dists[5];

			dists[0] = wid_dist[(py+0)*TOF_XS+(px+0)] + widnar_corr;
			dists[1] = wid_dist[(py-1)*TOF_XS+(px+0)] + widnar_corr;
			dists[2] = wid_dist[(py+1)*TOF_XS+(px+0)] + widnar_corr;
			dists[3] = wid_dist[(py+0)*TOF_XS+(px+1)] + widnar_corr;
			dists[4] = wid_dist[(py+0)*TOF_XS+(px-1)] + widnar_corr;

			uint8_t ampls[5];
			ampls[0] = wid_ampl[(py+0)*TOF_XS+(px+0)];
			ampls[1] = wid_ampl[(py-1)*TOF_XS+(px+0)];
			ampls[2] = wid_ampl[(py+1)*TOF_XS+(px+0)];
			ampls[3] = wid_ampl[(py+0)*TOF_XS+(px+1)];
			ampls[4] = wid_ampl[(py+0)*TOF_XS+(px-1)];

			int32_t avg = (dists[0]+dists[1]+dists[2]+dists[3]+dists[4])/5;

			int n_conform = 0;
			int32_t conform_avg = 0;
			for(int i=0; i<5; i++)
			{
				if(ampls[i] >= ampl_accept_min && ampls[i] <= ampl_accept_max && dists[i] > avg-100 && dists[i] < avg+100)
				{
					n_conform++;
					conform_avg += dists[i];
				}
			}

			if(n_conform >= 3)
			{
				int32_t d = conform_avg / n_conform;

				uint16_t hor_ang, ver_ang;

				#ifdef DBGPR

					DBG_PR_VAR_I32(d);
				#endif

				// TODO: This optimizes out once we have sensor-by-sensor geometric tables;
				// they can be pre-built to the actual mount_mode.
				switch(sensor_mounts[sidx].mount_mode)
				{
					case 1: 
					hor_ang = -1*geocoords[py*TOF_XS+px].yang;
					ver_ang = geocoords[py*TOF_XS+px].xang;
					break;

					case 2: 
					hor_ang = geocoords[py*TOF_XS+px].yang;
					ver_ang = -1*geocoords[py*TOF_XS+px].xang;
					break;

					case 3:
					hor_ang = -1*geocoords[py*TOF_XS+px].xang;
					ver_ang = geocoords[py*TOF_XS+px].yang;
					break;

					case 4:
					hor_ang = geocoords[py*TOF_XS+px].xang;
					ver_ang = -1*geocoords[py*TOF_XS+px].yang;
					break;

					default: error(145); while(1); // to tell the compiler we always set hor_ang, ver_ang
				}

				#ifdef DBGPR

					DBG_PR_VAR_U16(hor_ang);
					DBG_PR_VAR_U16(ver_ang);

				#endif

				uint16_t comb_hor_ang = hor_ang + global_sensor_hor_ang;
				uint16_t comb_ver_ang = ver_ang + global_sensor_ver_ang;

				#ifdef DBGPR

					DBG_PR_VAR_U16(comb_hor_ang);
					DBG_PR_VAR_U16(comb_ver_ang);

				#endif

				int32_t x = (((int64_t)d * (int64_t)lut_cos_from_u16(comb_ver_ang) * (int64_t)lut_cos_from_u16(comb_hor_ang))>>(2*SIN_LUT_RESULT_SHIFT)) + global_sensor_x;
				int32_t y = (((int64_t)d * (int64_t)lut_cos_from_u16(comb_ver_ang) * (int64_t)lut_sin_from_u16(comb_hor_ang))>>(2*SIN_LUT_RESULT_SHIFT)) + global_sensor_y;
				int32_t z = (((int64_t)d * (int64_t)lut_sin_from_u16(comb_ver_ang))>>SIN_LUT_RESULT_SHIFT) + global_sensor_z;

				uint16_t local_comb_hor_ang = hor_ang + local_sensor_hor_ang;
				uint16_t local_comb_ver_ang = ver_ang + local_sensor_ver_ang;


				int32_t local_x = (((int64_t)d * (int64_t)lut_cos_from_u16(local_comb_ver_ang) * (int64_t)lut_cos_from_u16(local_comb_hor_ang))>>(2*SIN_LUT_RESULT_SHIFT)) + local_sensor_x;
				int32_t local_y = (((int64_t)d * (int64_t)lut_cos_from_u16(local_comb_ver_ang) * (int64_t)lut_sin_from_u16(local_comb_hor_ang))>>(2*SIN_LUT_RESULT_SHIFT)) + local_sensor_y;
				int32_t local_z = (((int64_t)d * (int64_t)lut_sin_from_u16(local_comb_ver_ang))>>SIN_LUT_RESULT_SHIFT) + local_sensor_z;


				#ifdef DBGPR

					DBG_PR_VAR_I32(x);
					DBG_PR_VAR_I32(y);
					DBG_PR_VAR_I32(z);
				#endif

				#define OBST_AVOID_WIDTH 500

				if(local_z > 250 && local_z < 1600)
				{
					if(local_y > -(OBST_AVOID_WIDTH/2) && local_y < (OBST_AVOID_WIDTH/2))
					{
						if(local_x > 300 && local_x < 500)
							obstacle_front++;

						if(local_x > -750 && local_x < -500)
							obstacle_back++;
					}

					if(local_x > -500 && local_x < -200)
					{
						if(local_y > 250 && local_y < 400)
							obstacle_left++;

						if(local_y < -250 && local_y > -400)
							obstacle_right++;

					}
				}

				if(z > BASE_Z && z < MAX_Z)
				{
					uint16_t new_z = 1<<((z-BASE_Z)/Z_STEP);
					
					for(int seg=0; seg<12; seg++)
					{
						int xmin = seg_lims[seg].xmin;
						int xmax = seg_lims[seg].xmax;
						int ymin = seg_lims[seg].ymin;
						int ymax = seg_lims[seg].ymax;
						int reso = seg_lims[seg].reso;
						if(x >= xmin && x <= xmax && y >= ymin && y <= ymax)
						{
							x -= xmin;
							y -= ymin;

							x /= reso;
							y /= reso;

							#ifdef DBGPR

								DBG_PR_VAR_I32(seg);
								DBG_PR_VAR_I32(x);
								DBG_PR_VAR_I32(y);
							#endif

							if(x<0 || x >= VOX_SEG_XS || y<0 || y>= VOX_SEG_YS)
							{
								DBG_PR_VAR_I32(px);
								DBG_PR_VAR_I32(py);
								DBG_PR_VAR_I32(seg);
								DBG_PR_VAR_I32(x);
								DBG_PR_VAR_I32(y);
								error(155);
							}
							voxmap.segs[seg][y*VOX_SEG_XS+x] |= new_z;
							insertion_cnt++;
							break;
						}
					}
				}

			}
		}
	}

	#ifdef DBGPR
		DBG_PR_VAR_I32(insertion_cnt);
	#endif
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

				if(ampl<4)
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

				if(ampl<4)
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


static uint8_t stray_weight[15*40] __attribute__((section(".dtcm_data"))) =
{                                             /* LEDS ON THIS SIDE */
 20,20,30,39,45,60,70,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,70,60,45,39,30,20,20,
 10,10,15,20,25,32,37,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,37,32,25,20,15,10,10,
 9, 9,13,18,23,29,34,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,34,29,23,18,13, 9, 9,
 8, 8,12,16,20,26,30,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,30,26,20,16,12, 8, 8,
 6, 6,10,13,17,22,25,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,25,22,17,13,10, 6, 6,
 6, 6, 9,12,15,19,22,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,22,19,15,12, 9, 6, 6,
 5, 5, 8,11,14,18,21,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,21,18,14,11, 8, 5, 5,
 5, 5, 7,10,13,16,19,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,19,16,13,10, 7, 5, 5,
 4, 4, 7, 9,12,15,18,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,18,15,12, 9, 7, 4, 4,
 4, 4, 6, 8,11,14,16,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,16,14,11, 8, 6, 4, 4,
 4, 4, 6, 8,10,13,15,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,15,13,10, 8, 6, 4, 4,
 4, 4, 6, 8,10,13,15,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,15,13,10, 8, 6, 4, 4,
 3, 3, 5, 7, 9,12,14,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,14,12, 9, 7, 5, 3, 3,
 3, 3, 5, 7, 9,11,13,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,13,11, 9, 7, 5, 3, 3,
 3, 3, 5, 7, 9,11,13,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,13,11, 9, 7, 5, 3, 3
};                            /* NON-LED SIDE: items here don't seem to cause any issue */

void calc_stray_estimate(uint8_t *ampl_in, uint16_t *dist_in, uint16_t *stray_ampl, uint16_t *stray_dist)  __attribute__((section(".text_itcm")));
void calc_stray_estimate(uint8_t *ampl_in, uint16_t *dist_in, uint16_t *stray_ampl, uint16_t *stray_dist)
{
	int32_t ampl_acc = 0;
	int64_t dist_acc = 0;
	// To save time, we only process 1/16 of the pixels
	for(int yy=0; yy<15; yy++)
	{
		for(int xx=0; xx<40; xx++)
		{
			int weigh_i = yy*40+xx;
			int pix_i = (yy<<2)*160+(xx<<2);

			int32_t ampl_causing_stray = (int32_t)ampl_in[pix_i] * (int32_t)stray_weight[weigh_i]; // Biggest = 255*99 = 25245

			ampl_acc += ampl_causing_stray;
			dist_acc += dist_in[pix_i] * ampl_causing_stray; // Weigh the distances based on the amplitude. Biggest: 65535 * 25245
		}
	}

	*stray_dist = dist_acc / ampl_acc; // Average distance of weighed stray-causing light (short distances dominate)
	*stray_ampl = ampl_acc / (15*40); // Biggest = 25245
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

