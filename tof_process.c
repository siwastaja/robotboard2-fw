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
#include "micronavi.h"

#include "../robotsoft/api_board_to_soft.h"


#define FLASH_SENSOR0  (FLASH_OFFSET + (0*8+2)*128*1024)  // bank1 sector2
#define FLASH_SENSOR5  (FLASH_OFFSET + (1*8+2)*128*1024)  // bank2 sector2




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
	Gives the complete precalculated distance (in mm) which includes full non-linearity and offset calibration.
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
			if(d31 > d20) {s=5; i=((TOF_TBL_SEG_LEN)*d20)/d31;}
			else          {s=4; i=((TOF_TBL_SEG_LEN)*d31)/d20;}
		}
		else
		{
			d20 *= -1;
			if(d31 > d20) {s=6; i=((TOF_TBL_SEG_LEN)*d20)/d31;}
			else          {s=7; i=((TOF_TBL_SEG_LEN)*d31)/d20;}
		}		
	}
	else
	{
		d31 *= -1;
		if(d20 >= 0)
		{
			if(d31 > d20) {s=2; i=((TOF_TBL_SEG_LEN)*d20)/d31;}
			else          {s=3; i=((TOF_TBL_SEG_LEN)*d31)/d20;}
		}
		else
		{
			d20 *= -1;
			if(d31 > d20) {s=1; i=((TOF_TBL_SEG_LEN)*d20)/d31;}
			else          {s=0; i=((TOF_TBL_SEG_LEN)*d31)/d20;}
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

#ifdef FAST_APPROX_AMPLITUDE
	#define AMPL(dcs20_, dcs31_) ((abso((dcs20_))+abso((dcs31_)))/(23))
#else
	#define AMPL(dcs20_, dcs31_) (sqrt(sq((dcs20_))+sq((dcs31_)))/(17))
#endif			


//#define DO_AMB_CORR

//#define DBGPR

//#define DBGPRVOX
#define PIX (30*160+80)
//#define PIX (15*160+60)
#define DBGPRSIDX 5
#define IN_01_DEG (ANG_0_1_DEG/65536)


void conv_4dcs_to_2dcs(int16_t *dcs20_out, int16_t *dcs31_out, epc_4dcs_t *in, epc_img_t *bwimg)  __attribute__((section(".text_itcm")));
void conv_4dcs_to_2dcs(int16_t *dcs20_out, int16_t *dcs31_out, epc_4dcs_t *in, epc_img_t *bwimg)
{
	for(int i=0; i < TOF_XS*TOF_YS; i++)
	{
		int16_t dcs0 = ((in->dcs[0].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs1 = ((in->dcs[1].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs2 = ((in->dcs[2].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs3 = ((in->dcs[3].img[i]&0b0011111111111100)>>2)-2048;

		int16_t dcs31 = dcs3-dcs1;
		int16_t dcs20 = dcs2-dcs0;


		#ifdef DO_AMB_CORR
			int16_t bw = ((bwimg->img[i]&0b0011111111111100)>>2)-2048;
			if(bw<0) bw=0;
			int corr_factor;
			if(!(i&1)) // even
				corr_factor = shadow_luts.hif.amb_corr[i/2] & 0x0f;
			else
				corr_factor = shadow_luts.hif.amb_corr[i/2]>>4;
			int16_t comp = (bw*corr_factor)>>4;

			dcs31 -= comp;
			dcs20 -= comp;
		#endif


		dcs20_out[i] = dcs20;
		dcs31_out[i] = dcs31;
	}
}

void conv_4dcs_to_2dcs_narrow(int16_t *dcs20_out, int16_t *dcs31_out, epc_4dcs_narrow_t *in, epc_img_t *bwimg)  __attribute__((section(".text_itcm")));
void conv_4dcs_to_2dcs_narrow(int16_t *dcs20_out, int16_t *dcs31_out, epc_4dcs_narrow_t *in, epc_img_t *bwimg)
{
	for(int yy=0; yy < TOF_YS_NARROW; yy++)
	{
		for(int xx=0; xx < TOF_XS_NARROW; xx++)
		{
			int i = yy*TOF_XS_NARROW+xx;

			int16_t dcs0 = ((in->dcs[0].img[i]&0b0011111111111100)>>2)-2048;
			int16_t dcs1 = ((in->dcs[1].img[i]&0b0011111111111100)>>2)-2048;
			int16_t dcs2 = ((in->dcs[2].img[i]&0b0011111111111100)>>2)-2048;
			int16_t dcs3 = ((in->dcs[3].img[i]&0b0011111111111100)>>2)-2048;

			int16_t dcs31 = dcs3-dcs1;
			int16_t dcs20 = dcs2-dcs0;


			#ifdef DO_AMB_CORR
				int i_bw = (yy+TOF_NARROW_Y_START)*TOF_XS+(xx+TOF_NARROW_X_START);
				int16_t bw = ((bwimg->img[i_bw]&0b0011111111111100)>>2)-2048;
				if(bw<0) bw=0;
				int corr_factor;
				if(!(i&1)) // even
					corr_factor = shadow_luts.hif.amb_corr[i_bw/2] & 0x0f;
				else
					corr_factor = shadow_luts.hif.amb_corr[i_bw/2]>>4;
				int16_t comp = (bw*corr_factor)>>4;

				dcs31 -= comp;
				dcs20 -= comp;
			#endif


			dcs20_out[i] = dcs20;
			dcs31_out[i] = dcs31;
		}
	}
}

int calc_avg_ampl_x256(int16_t* dcs20_in, int16_t* dcs31_in)
{
	uint32_t accum = 0;
	for(int i=0; i < TOF_XS*TOF_YS; i++)
	{
		int16_t dcs20 = dcs20_in[i];
		int16_t dcs31 = dcs31_in[i];
		int ampl;
		ampl = AMPL(dcs20, dcs31);
		if(ampl > 255) ampl = 255;

		accum += ampl;
	}
	uint32_t avg = (256*accum)/(TOF_XS*TOF_YS);
	return avg;
}

int calc_avg_ampl_x256_narrow(int16_t* dcs20_in, int16_t* dcs31_in)
{
	uint32_t accum = 0;
	for(int i=0; i < TOF_XS_NARROW*TOF_YS_NARROW; i++)
	{
		int16_t dcs20 = dcs20_in[i];
		int16_t dcs31 = dcs31_in[i];
		int ampl;
		ampl = AMPL(dcs20, dcs31);
		if(ampl > 255) ampl = 255;

		accum += ampl;
	}
	uint32_t avg = (256*accum)/(TOF_XS_NARROW*TOF_YS_NARROW);
	return avg;
}

int calc_avg_ampl_x256_nar_region_on_wide(int16_t* dcs20_in, int16_t* dcs31_in) __attribute__((section(".text_itcm")));
int calc_avg_ampl_x256_nar_region_on_wide(int16_t* dcs20_in, int16_t* dcs31_in)
{
	uint32_t accum = 0;
	for(int yy=0; yy < TOF_YS_NARROW; yy++)
	{
		for(int xx=0; xx < TOF_XS_NARROW; xx++)
		{
			// Index on wide data:
			int i = (yy+TOF_NARROW_Y_START)*TOF_XS+(xx+TOF_NARROW_X_START);

			int16_t dcs20 = dcs20_in[i];
			int16_t dcs31 = dcs31_in[i];
			int ampl;
			ampl = AMPL(dcs20, dcs31);
			if(ampl > 255) ampl = 255;

			accum += ampl;
		}
	}
	uint32_t avg = (256*accum)/(TOF_XS_NARROW*TOF_YS_NARROW);
	return avg;
}

void lens_model(int16_t* dcs20_blur_out, int16_t* dcs31_blur_out, int16_t* dcs20_in, int16_t* dcs31_in) __attribute__((section(".text_itcm")));
void lens_model(int16_t* dcs20_blur_out, int16_t* dcs31_blur_out, int16_t* dcs20_in, int16_t* dcs31_in)
{
	int32_t dcs20_avg = 0, dcs31_avg = 0;

	for(int i=0; i < TOF_XS*TOF_YS; i++)
	{
		dcs20_avg += dcs20_in[i];
		dcs31_avg += dcs31_in[i];
	}

	dcs20_avg /= TOF_XS*TOF_YS;
	dcs31_avg /= TOF_XS*TOF_YS;


	int16_t evenfield_flare20 = (dcs20_avg*11)>>8;
	int16_t evenfield_flare31 = (dcs31_avg*11)>>8;

	for(int i=0; i < TOF_XS*TOF_YS; i++)
	{
		dcs20_blur_out[i] = evenfield_flare20;
		dcs31_blur_out[i] = evenfield_flare31;
	}
}

void lens_model_narrow(int16_t* dcs20_blur_out, int16_t* dcs31_blur_out, int16_t* dcs20_in, int16_t* dcs31_in) __attribute__((section(".text_itcm")));
void lens_model_narrow(int16_t* dcs20_blur_out, int16_t* dcs31_blur_out, int16_t* dcs20_in, int16_t* dcs31_in)
{
	int32_t dcs20_avg = 0, dcs31_avg = 0;

	for(int i=0; i < TOF_XS_NARROW*TOF_YS_NARROW; i++)
	{
		dcs20_avg += dcs20_in[i];
		dcs31_avg += dcs31_in[i];
	}

	dcs20_avg /= TOF_XS_NARROW*TOF_YS_NARROW;
	dcs31_avg /= TOF_XS_NARROW*TOF_YS_NARROW;


	int16_t evenfield_flare20 = (dcs20_avg*8)>>8;
	int16_t evenfield_flare31 = (dcs31_avg*8)>>8;

	for(int i=0; i < TOF_XS_NARROW*TOF_YS_NARROW; i++)
	{
		dcs20_blur_out[i] = evenfield_flare20;
		dcs31_blur_out[i] = evenfield_flare31;
	}
}

int flare_factor = 17;
int flare_factor_narrow = 7;

#if 0
void adjust()
{
	uint8_t cmd = uart_input();

	if(cmd == 'a')
	{
		flare_factor++;
		DBG_PR_VAR_I32(flare_factor);
	}
	else if(cmd == 'z')
	{
		flare_factor--;
		DBG_PR_VAR_I32(flare_factor);
	}
	else if(cmd == 's')
	{
		flare_factor_narrow++;
		DBG_PR_VAR_I32(flare_factor_narrow);
	}
	else if(cmd == 'x')
	{
		flare_factor_narrow--;
		DBG_PR_VAR_I32(flare_factor_narrow);
	}



}
#endif

void dealias_20mhz(uint16_t *hf_dist, uint16_t *lf_dist) __attribute__((section(".text_itcm")));
void dealias_20mhz(uint16_t *hf_dist, uint16_t *lf_dist)
{
	#define DEALIAS_THRESHOLD 2000
	for(int i=0; i < TOF_XS*TOF_YS; i++)
	{
		if(lf_dist[i] == 65535)
		{
			// Lf overexposed - keep the hf
		}
		else if(hf_dist[i] < 65534) // Only touch a valid value
		{
			int hf_wrap0 = hf_dist[i];
			int hf_wrap1 = hf_dist[i] + 7495;
			int hf_wrap2 = hf_dist[i] + 2*7495;

			int err0 = abso(hf_wrap0 - (int)lf_dist[i]);
			int err1 = abso(hf_wrap1 - (int)lf_dist[i]);
			int err2 = abso(hf_wrap2 - (int)lf_dist[i]);

			if(err0 < DEALIAS_THRESHOLD)
			{
				// Keep hf as is
			}
			else if(err1 < DEALIAS_THRESHOLD)
			{
				hf_dist[i] = hf_wrap1;
			}
			else if(err2 < DEALIAS_THRESHOLD)
			{
				hf_dist[i] = hf_wrap2;
			}
			else
			{
				hf_dist[i] = 65534;
			}
		}
	}
	#undef DEALIAS_THRESHOLD
}

void dealias_20mhz_narrow(uint16_t *hf_dist, uint16_t *lf_dist) __attribute__((section(".text_itcm")));
void dealias_20mhz_narrow(uint16_t *hf_dist, uint16_t *lf_dist)
{
	#define DEALIAS_THRESHOLD 2000
	for(int i=0; i < TOF_XS_NARROW*TOF_YS_NARROW; i++)
	{
		if(lf_dist[i] == 65535)
		{
			// Lf overexposed - keep the hf
		}
		else if(hf_dist[i] < 65534) // Only touch a valid value
		{
			int hf_wrap0 = hf_dist[i];
			int hf_wrap1 = hf_dist[i] + 7495;
			int hf_wrap2 = hf_dist[i] + 2*7495;

			int err0 = abso(hf_wrap0 - (int)lf_dist[i]);
			int err1 = abso(hf_wrap1 - (int)lf_dist[i]);
			int err2 = abso(hf_wrap2 - (int)lf_dist[i]);

			if(err0 < DEALIAS_THRESHOLD)
			{
				// Keep hf as is
			}
			else if(err1 < DEALIAS_THRESHOLD)
			{
				hf_dist[i] = hf_wrap1;
			}
			else if(err2 < DEALIAS_THRESHOLD)
			{
				hf_dist[i] = hf_wrap2;
			}
			else
			{
				hf_dist[i] = 65534;
			}
		}
	}
	#undef DEALIAS_THRESHOLD
}

void dealias_10mhz(uint16_t *hf_dist, uint16_t *lf_dist) __attribute__((section(".text_itcm")));
void dealias_10mhz(uint16_t *hf_dist, uint16_t *lf_dist)
{
	#define DEALIAS_THRESHOLD 2000
	for(int i=0; i < TOF_XS*TOF_YS; i++)
	{
		if(lf_dist[i] == 65535)
		{
			// Lf overexposed - keep the hf
		}
		else if(hf_dist[i] < 65534) // Only touch a valid value
		{
			int hf_wrap0 = hf_dist[i];
			int hf_wrap1 = hf_dist[i] + 14990;
			int hf_wrap2 = hf_dist[i] + 2*14990;

			int err0 = abso(hf_wrap0 - (int)lf_dist[i]);
			int err1 = abso(hf_wrap1 - (int)lf_dist[i]);
			int err2 = abso(hf_wrap2 - (int)lf_dist[i]);

			if(err0 < DEALIAS_THRESHOLD)
			{
				// Keep hf as is
			}
			else if(err1 < DEALIAS_THRESHOLD)
			{
				hf_dist[i] = hf_wrap1;
			}
			else if(err2 < DEALIAS_THRESHOLD)
			{
				hf_dist[i] = hf_wrap2;
			}
			else
			{
				hf_dist[i] = 65534;
			}
		}
	}
	#undef DEALIAS_THRESHOLD
}

void dealias_10mhz_narrow(uint16_t *hf_dist, uint16_t *lf_dist) __attribute__((section(".text_itcm")));
void dealias_10mhz_narrow(uint16_t *hf_dist, uint16_t *lf_dist)
{
	#define DEALIAS_THRESHOLD 2000
	for(int i=0; i < TOF_XS_NARROW*TOF_YS_NARROW; i++)
	{
		if(lf_dist[i] == 65535)
		{
			// Lf overexposed - keep the hf
		}
		else if(hf_dist[i] < 65534) // Only touch a valid value
		{
			int hf_wrap0 = hf_dist[i];
			int hf_wrap1 = hf_dist[i] + 14990;
			int hf_wrap2 = hf_dist[i] + 2*14990;

			int err0 = abso(hf_wrap0 - (int)lf_dist[i]);
			int err1 = abso(hf_wrap1 - (int)lf_dist[i]);
			int err2 = abso(hf_wrap2 - (int)lf_dist[i]);

			if(err0 < DEALIAS_THRESHOLD)
			{
				// Keep hf as is
			}
			else if(err1 < DEALIAS_THRESHOLD)
			{
				hf_dist[i] = hf_wrap1;
			}
			else if(err2 < DEALIAS_THRESHOLD)
			{
				hf_dist[i] = hf_wrap2;
			}
			else
			{
				hf_dist[i] = 65534;
			}
		}
	}
	#undef DEALIAS_THRESHOLD
}


// Narrow version with lf_dist is expected as wide. Useful when exposing the narrow to give similar ranges to the wide,
// to save time, by using single wide low-freq image for dealiasing both wide and narrow main acquisitions.
void dealias_20mhz_narrow_from_wide_lf(uint16_t *hf_dist, uint16_t *lf_dist) __attribute__((section(".text_itcm")));
void dealias_20mhz_narrow_from_wide_lf(uint16_t *hf_dist, uint16_t *lf_dist)
{
	#define DEALIAS_THRESHOLD 2000
	for(int yy=0; yy<TOF_YS_NARROW; yy++)
	{
		for(int xx=0; xx<TOF_XS_NARROW; xx++)
		{
			int i_hf = yy*TOF_XS_NARROW+xx;
			int i_lf = (yy+TOF_NARROW_Y_START)*TOF_XS+(xx+TOF_NARROW_X_START);

			if(lf_dist[i_lf] == 65535)
			{
				// Lf overexposed - keep the hf
			}
			else if(hf_dist[i_hf] < 65534) // Only touch a valid value
			{
				int hf_wrap0 = hf_dist[i_hf];
				int hf_wrap1 = hf_dist[i_hf] + 7495;
				int hf_wrap2 = hf_dist[i_hf] + 2*7495;

				int err0 = abso(hf_wrap0 - (int)lf_dist[i_lf]);
				int err1 = abso(hf_wrap1 - (int)lf_dist[i_lf]);
				int err2 = abso(hf_wrap2 - (int)lf_dist[i_lf]);

				if(err0 < DEALIAS_THRESHOLD)
				{
					// Keep hf as is
				}
				else if(err1 < DEALIAS_THRESHOLD)
				{
					hf_dist[i_hf] = hf_wrap1;
				}
				else if(err2 < DEALIAS_THRESHOLD)
				{
					hf_dist[i_hf] = hf_wrap2;
				}
				else
				{
					hf_dist[i_hf] = 65534;
				}
			}

		}
	}
	#undef DEALIAS_THRESHOLD
}

void compensated_nonhdr_tof_calc_dist_ampl_flarecomp(uint8_t *ampl_out, uint16_t *dist_out, int16_t* dcs20_hi_in, int16_t* dcs31_hi_in, int kludge_corr)   __attribute__((section(".text_itcm")));
void compensated_nonhdr_tof_calc_dist_ampl_flarecomp(uint8_t *ampl_out, uint16_t *dist_out, int16_t* dcs20_hi_in, int16_t* dcs31_hi_in, int kludge_corr)
{
	int dcs20_accum = 0;
	int dcs31_accum = 0;
	for(int i=0; i < TOF_XS*TOF_YS; i++)
	{
		int32_t dcs20, dcs31;
		dcs20 = dcs20_hi_in[i];
		dcs31 = dcs31_hi_in[i];
		dcs20_accum += dcs20;
		dcs31_accum += dcs31;
	}

	dcs20_accum /= TOF_XS*TOF_YS;
	dcs31_accum /= TOF_XS*TOF_YS;

//	DBG_PR_VAR_I32(flare_factor);
//	DBG_PR_VAR_I32(dcs20_accum);
//	DBG_PR_VAR_I32(dcs31_accum);

	for(int i=0; i < TOF_XS*TOF_YS; i++)
	{
		uint16_t dist;
		int ampl;

		int16_t dcs20 = dcs20_hi_in[i];
		int16_t dcs31 = dcs31_hi_in[i];


		if(dcs20 < -4090 || dcs20 > 4090 || dcs31 < -4090 || dcs31 > 4090)
		{
			ampl = 255;
			dist = 0;
		}
		else
		{
			dcs31 -= ((int64_t)dcs31_accum*(int64_t)flare_factor)>>8;
			dcs20 -= ((int64_t)dcs20_accum*(int64_t)flare_factor)>>8;
			

			ampl = AMPL(dcs20, dcs31);
			if(ampl > 255) ampl = 255;

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
				dist += kludge_corr;
			}

		}
		ampl_out[i] = ampl;
		dist_out[i] = dist;
	}
}
void compensated_nonhdr_tof_calc_dist_ampl_flarecomp_narrow(uint8_t *ampl_out, uint16_t *dist_out, int16_t* dcs20_hi_in, int16_t* dcs31_hi_in, int kludge_corr)   __attribute__((section(".text_itcm")));
void compensated_nonhdr_tof_calc_dist_ampl_flarecomp_narrow(uint8_t *ampl_out, uint16_t *dist_out, int16_t* dcs20_hi_in, int16_t* dcs31_hi_in, int kludge_corr)
{
	int dcs20_accum = 0;
	int dcs31_accum = 0;
	for(int i=0; i < TOF_XS_NARROW*TOF_YS_NARROW; i++)
	{
		int32_t dcs20, dcs31;
		dcs20 = dcs20_hi_in[i];
		dcs31 = dcs31_hi_in[i];
		dcs20_accum += dcs20;
		dcs31_accum += dcs31;
	}

	dcs20_accum /= TOF_XS_NARROW*TOF_YS_NARROW;
	dcs31_accum /= TOF_XS_NARROW*TOF_YS_NARROW;

//	DBG_PR_VAR_I32(flare_factor_narrow);
//	DBG_PR_VAR_I32(dcs20_accum);
//	DBG_PR_VAR_I32(dcs31_accum);

	for(int i=0; i < TOF_XS_NARROW*TOF_YS_NARROW; i++)
	{
		uint16_t dist;
		int ampl;

		int16_t dcs20 = dcs20_hi_in[i];
		int16_t dcs31 = dcs31_hi_in[i];


		if(dcs20 < -4090 || dcs20 > 4090 || dcs31 < -4090 || dcs31 > 4090)
		{
			ampl = 255;
			dist = 0;
		}
		else
		{
			dcs31 -= ((int64_t)dcs31_accum*(int64_t)flare_factor_narrow)>>8;
			dcs20 -= ((int64_t)dcs20_accum*(int64_t)flare_factor_narrow)>>8;
			
			ampl = AMPL(dcs20, dcs31);
			if(ampl > 255) ampl = 255;

			if(ampl<4)
			{
				dist = 65534;
			}
			else
			{
				int pixgroup;
				if(!(i&1)) // even
					pixgroup = shadow_luts.hif.nar_lut_group_ids[i/2] & 0x0f;
				else
					pixgroup = shadow_luts.hif.nar_lut_group_ids[i/2]>>4;

				dist = lookup_dist(1, pixgroup, dcs31, dcs20);
				dist += kludge_corr;
			}

		}
		ampl_out[i] = ampl;
		dist_out[i] = dist;
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
			dcs20 = (int32_t)dcs20_lo*HDR_FACTOR;
			dcs31 = (int32_t)dcs31_lo*HDR_FACTOR;
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

//	DBG_PR_VAR_I32(flare_factor);
//	DBG_PR_VAR_I32(dcs20_accum);
//	DBG_PR_VAR_I32(dcs31_accum);

	for(int i=0; i < TOF_XS*TOF_YS; i++)
	{
		uint16_t dist;
		int ampl;

		int16_t dcs20_lo = dcs20_lo_in[i];
		int16_t dcs31_lo = dcs31_lo_in[i];
		int16_t dcs20_hi = dcs20_hi_in[i];
		int16_t dcs31_hi = dcs31_hi_in[i];


		if(dcs20_lo < -4090 || dcs20_lo > 4090 || dcs31_lo < -4090 || dcs31_lo > 4090)
		{
			ampl = 255;
			dist = 0;
		}
		else
		{
			int32_t dcs20, dcs31;
			int kludge_corr;

			int used_lo = 0;

			if(dcs20_hi < -3500 || dcs20_hi > 3500 || dcs31_hi < -3500 || dcs31_hi > 3500)
			{
				dcs20 = (int32_t)dcs20_lo*HDR_FACTOR;
				dcs31 = (int32_t)dcs31_lo*HDR_FACTOR;
				kludge_corr = -40;
				used_lo = 1;
			}
			else
			{
				dcs20 = dcs20_hi;
				dcs31 = dcs31_hi;
				kludge_corr = 0;
			}

			dcs31 -= ((int64_t)dcs31_accum*(int64_t)flare_factor)>>8;
			dcs20 -= ((int64_t)dcs20_accum*(int64_t)flare_factor)>>8;

			ampl = AMPL(dcs20, dcs31);
			if(used_lo)
				ampl /= HDR_FACTOR;

			if((used_lo && ampl < 3) || ampl<4)
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
				dist += kludge_corr; // Typical short exposure shows around 50mm too long results
			}

			// ampl /= HDR_FACTOR; 
			if(ampl>255) ampl=255;

		}
		ampl_out[i] = ampl;
		dist_out[i] = dist;
	}
}
void compensated_hdr_tof_calc_dist_ampl_flarecomp_narrow(uint8_t *ampl_out, uint16_t *dist_out, int16_t* dcs20_lo_in, int16_t* dcs31_lo_in, int16_t* dcs20_hi_in, int16_t* dcs31_hi_in)   __attribute__((section(".text_itcm")));
void compensated_hdr_tof_calc_dist_ampl_flarecomp_narrow(uint8_t *ampl_out, uint16_t *dist_out, int16_t* dcs20_lo_in, int16_t* dcs31_lo_in, int16_t* dcs20_hi_in, int16_t* dcs31_hi_in)
{
	int dcs20_accum = 0;
	int dcs31_accum = 0;
	for(int i=0; i < TOF_XS_NARROW*TOF_YS_NARROW; i++)
	{
		int16_t dcs20_lo = dcs20_lo_in[i];
		int16_t dcs31_lo = dcs31_lo_in[i];
		int16_t dcs20_hi = dcs20_hi_in[i];
		int16_t dcs31_hi = dcs31_hi_in[i];

		int32_t dcs20, dcs31;

		if(dcs20_hi < -3500 || dcs20_hi > 3500 || dcs31_hi < -3500 || dcs31_hi > 3500)
		{
			dcs20 = (int32_t)dcs20_lo*HDR_FACTOR;
			dcs31 = (int32_t)dcs31_lo*HDR_FACTOR;
		}
		else
		{
			dcs20 = dcs20_hi;
			dcs31 = dcs31_hi;
		}

		dcs20_accum += dcs20;
		dcs31_accum += dcs31;
	}

	dcs20_accum /= TOF_XS_NARROW*TOF_YS_NARROW;
	dcs31_accum /= TOF_XS_NARROW*TOF_YS_NARROW;

//	DBG_PR_VAR_I32(flare_factor_narrow);
//	DBG_PR_VAR_I32(dcs20_accum);
//	DBG_PR_VAR_I32(dcs31_accum);

	for(int i=0; i < TOF_XS_NARROW*TOF_YS_NARROW; i++)
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
			int32_t dcs20, dcs31;

			int used_lo = 0;
			if(dcs20_hi < -3500 || dcs20_hi > 3500 || dcs31_hi < -3500 || dcs31_hi > 3500)
			{
				dcs20 = (int32_t)dcs20_lo*HDR_FACTOR;
				dcs31 = (int32_t)dcs31_lo*HDR_FACTOR;
				used_lo = 1;
			}
			else
			{
				dcs20 = dcs20_hi;
				dcs31 = dcs31_hi;
			}

			dcs31 -= ((int64_t)dcs31_accum*(int64_t)flare_factor_narrow)>>8;
			dcs20 -= ((int64_t)dcs20_accum*(int64_t)flare_factor_narrow)>>8;
			
			ampl = AMPL(dcs20, dcs31);

			if(used_lo)
				ampl /= HDR_FACTOR;

			if((used_lo && ampl < 3) || ampl<4)
			{
				dist = 65534;
			}
			else
			{
				int pixgroup;
				if(!(i&1)) // even
					pixgroup = shadow_luts.hif.nar_lut_group_ids[i/2] & 0x0f;
				else
					pixgroup = shadow_luts.hif.nar_lut_group_ids[i/2]>>4;

				dist = lookup_dist(1, pixgroup, dcs31, dcs20);
			}

			//ampl /= HDR_FACTOR;
			if(ampl>255) ampl=255;

		}
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


			ampl = AMPL(dcs20, dcs31);

			if(ampl > 255) ampl = 255;

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


			ampl = AMPL(dcs20, dcs31);
			if(ampl > 255) ampl = 255;

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
 /*0:                */ { 0,     0,     0, DEGTOANG16(       0), DEGTOANG16( 2),         300 },

 /*1:                */ { 1,   130,   103, DEGTOANG16(    24.4), DEGTOANG16( 4.4),       310  }, // -1
 /*2:                */ { 2,  -235,   215, DEGTOANG16(    66.4), DEGTOANG16( 1.4),       310  }, // -1
 /*3:                */ { 2,  -415,   215, DEGTOANG16(    93.5), DEGTOANG16( 1.9),       310  }, // -1
 /*4:                */ { 2,  -522,   103, DEGTOANG16(   157.4), DEGTOANG16( 3.9),       280  }, // -1
 /*5:                */ { 2,  -522,   -35, DEGTOANG16(   176.0), DEGTOANG16( 4.9),       290  }, // -1
 /*6:                */ { 1,  -522,  -103, DEGTOANG16(   206.0), DEGTOANG16( 4.4),       290  }, // -1
 /*7:                */ { 1,  -415,  -215, DEGTOANG16(   271.5), DEGTOANG16( 2.4),       280  }, // -1
 /*8:                */ { 1,  -235,  -215, DEGTOANG16(   294.9), DEGTOANG16( 4.4),       300  }, // -1
 /*9:                */ { 2,   130,  -103, DEGTOANG16(   334.9), DEGTOANG16( -0.9),      320  }  // 0
};

void recalc_sensor_mounts(int idx, int d_hor_ang, int d_ver_ang, int d_z)
{
	sensor_mounts[idx].ang_rel_robot += d_hor_ang;
	sensor_mounts[idx].vert_ang_rel_ground += d_ver_ang;
	sensor_mounts[idx].z_rel_ground += d_z;

	DBG_PR_VAR_I32(idx);
	uint32_t ang_rel_robot = (uint32_t)sensor_mounts[idx].ang_rel_robot*65536;
	int32_t vert_ang_rel_robot = (int32_t)((int16_t)sensor_mounts[idx].vert_ang_rel_ground)*65536;
	int z_rel_ground = sensor_mounts[idx].z_rel_ground;

	DBG_PR_VAR_U32(ang_rel_robot/ANG_0_1_DEG);
	DBG_PR_VAR_I32(vert_ang_rel_robot/ANG_0_1_DEG);
	DBG_PR_VAR_I32(z_rel_ground);
}

int adjustings = 0;
#if 1
void adjust()
{
	uint8_t cmd = uart_input();

	if(cmd >= '0' && cmd <= '9')
	{
		adjustings = cmd-'0';
		recalc_sensor_mounts(adjustings, 0, 0, 0);
	}
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

int chafind_enabled = 0;

void tof_enable_chafind_datapoints()
{
	chafind_enabled = 1;
}

void tof_disable_chafind_datapoints()
{
	chafind_enabled = 0;
}


void tof_to_voxmap(uint8_t *wid_ampl, uint16_t *wid_dist, int sidx, uint8_t ampl_accept_min, uint8_t ampl_accept_max, int32_t ref_x, int32_t ref_y) __attribute__((section(".text_itcm")));
void tof_to_voxmap(uint8_t *wid_ampl, uint16_t *wid_dist, int sidx, uint8_t ampl_accept_min, uint8_t ampl_accept_max, int32_t ref_x, int32_t ref_y)
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
	if(abso(robot_x-ref_x) > 4000 || abso(robot_y-ref_y) > 4000) error(151);

	uint16_t robot_ang = cur_pos.ang>>16;

	// Rotation: xr = x*cos(a) + y*sin(a)
	//           yr = -x*sin(a) + y*cos(a)
	// It seems to me this widely touted formula has inverted y axis, don't understand why, so it should be:
	// Rotation: xr = x*cos(a) - y*sin(a)
	//           yr = x*sin(a) + y*cos(a)


	uint16_t global_sensor_hor_ang = sensor_mounts[sidx].ang_rel_robot + robot_ang;
//	uint16_t global_sensor_ver_ang = sensor_mounts[sidx].vert_ang_rel_ground;

	/*
		TODO: Implement proper full 3D transformation to each pixel, taking into account the robot pitch and roll.

		For now, we only correct the sensor vertical angle by robot pitch and roll, which works well for the usual
		case of small pitch/roll angles, which mostly causes massive shifts in sensor vertical angle. Sensors do
		rotate as well, and this isn't currently taken into account.

		Positive pitch = nose goes up:
			sensor with ang=0deg   increments directly by pitch
			sensor with ang=180deg increments directly by -1*pitch
			sensors with ang=90deg, 270 deg are not affected

			-->
			ver_ang += cos(sensor_mount_ang)*pitch

		Positive roll = "left wing" rises:
			sensor with ang=90deg, increments directly by roll
			sensor with ang=270deg, increments directly by -1*roll
			0 deg, 180 deg not affected

			-->
			ver_ang += sin(sensor_mount_ang)*roll
	*/

	int16_t pitch_ang = cur_pos.pitch>>16;
	int16_t roll_ang = cur_pos.roll>>16;

	uint16_t global_sensor_ver_ang = 
		(int32_t)((int16_t)sensor_mounts[sidx].vert_ang_rel_ground) +
		((lut_cos_from_u16(sensor_mounts[sidx].ang_rel_robot)*pitch_ang)>>SIN_LUT_RESULT_SHIFT) +
		((lut_sin_from_u16(sensor_mounts[sidx].ang_rel_robot)*roll_ang)>>SIN_LUT_RESULT_SHIFT);

	#ifdef DBGPRVOX
		if(sidx == DBGPRSIDX)
		{
			DBG_PR_VAR_U16(sidx);
			DBG_PR_VAR_I16((int16_t)sensor_mounts[sidx].vert_ang_rel_ground/IN_01_DEG);
			DBG_PR_VAR_I16(pitch_ang/IN_01_DEG);
			DBG_PR_VAR_I16(roll_ang/IN_01_DEG);
			DBG_PR_VAR_I16((int16_t)global_sensor_ver_ang/IN_01_DEG);
		}
	#endif



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



	#ifdef DBGPRVOX
		if(sidx == DBGPRSIDX)
		{

			DBG_PR_VAR_I32(robot_ang/IN_01_DEG);
			DBG_PR_VAR_I32(robot_x);
			DBG_PR_VAR_I32(robot_y);

			DBG_PR_VAR_I32(global_sensor_hor_ang/IN_01_DEG);
			DBG_PR_VAR_I32(global_sensor_ver_ang/IN_01_DEG);
			DBG_PR_VAR_I32(global_sensor_x);
			DBG_PR_VAR_I32(global_sensor_y);
			DBG_PR_VAR_I32(global_sensor_z);
		}
	#endif


	extern int obstacle_front_near, obstacle_back_near, obstacle_left_near, obstacle_right_near;
	extern int obstacle_front_far, obstacle_back_far, obstacle_left_far, obstacle_right_far;

	int insertion_cnt = 0;
	for(int py=1; py<TOF_YS-1; py++)
//	for(int py=29; py<32; py++)
	{
		for(int px=1; px<TOF_XS-1; px++)
//		for(int px=75; px<85; px++)
//		for(int px=79; px<82; px++)
		{
			int32_t dists[5];

			dists[0] = wid_dist[(py+0)*TOF_XS+(px+0)];
			dists[1] = wid_dist[(py-1)*TOF_XS+(px+0)];
			dists[2] = wid_dist[(py+1)*TOF_XS+(px+0)];
			dists[3] = wid_dist[(py+0)*TOF_XS+(px+1)];
			dists[4] = wid_dist[(py+0)*TOF_XS+(px-1)];

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

				#ifdef DBGPRVOX
					if(sidx == DBGPRSIDX  && py*TOF_XS+px == PIX)
					{

						o_itoa32(dists[i], printbuf); uart_print_string_blocking(printbuf); 
						uart_print_string_blocking("mm ("); 
						o_itoa32(ampls[i], printbuf); uart_print_string_blocking(printbuf); 
						uart_print_string_blocking(")  "); 
					}
				#endif

				// Typical dataset maximum variance (spatial noise) on flat surface is around +/-20mm (40mm p-p)
				// Accept +/- 70mm
				if(ampls[i] >= ampl_accept_min && ampls[i] <= ampl_accept_max && dists[i] > avg-70 && dists[i] < avg+70)
				{
					n_conform++;
					conform_avg += dists[i];
				}
			}

			#ifdef DBGPRVOX
				if(sidx == DBGPRSIDX  && py*TOF_XS+px == PIX)
				{

					uart_print_string_blocking("\r\n");
					DBG_PR_VAR_I32(avg);
					DBG_PR_VAR_I32(n_conform);
				}
			#endif

			if(n_conform >= 5)
			{
				int32_t d = conform_avg / n_conform;

				uint16_t hor_ang, ver_ang;

				#ifdef DBGPRVOX
					if(sidx == DBGPRSIDX && py*TOF_XS+px == PIX)
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

				#ifdef DBGPRVOX

					if(sidx == DBGPRSIDX && py*TOF_XS+px == PIX)
					{
						DBG_PR_VAR_U16(hor_ang/IN_01_DEG);
						DBG_PR_VAR_U16(ver_ang/IN_01_DEG);
					}
				#endif

				uint16_t comb_hor_ang = hor_ang + global_sensor_hor_ang;
				uint16_t comb_ver_ang = ver_ang + global_sensor_ver_ang;

				#ifdef DBGPRVOX

					if(sidx == DBGPRSIDX && py*TOF_XS+px == PIX)
					{
						DBG_PR_VAR_U16(comb_hor_ang/IN_01_DEG);
						DBG_PR_VAR_U16(comb_ver_ang/IN_01_DEG);
					}
				#endif

				int32_t x = (((int64_t)d * (int64_t)lut_cos_from_u16(comb_ver_ang) * (int64_t)lut_cos_from_u16(comb_hor_ang))>>(2*SIN_LUT_RESULT_SHIFT)) + global_sensor_x;
				int32_t y = (((int64_t)d * (int64_t)lut_cos_from_u16(comb_ver_ang) * (int64_t)lut_sin_from_u16(comb_hor_ang))>>(2*SIN_LUT_RESULT_SHIFT)) + global_sensor_y;
				int32_t z = (((int64_t)d * (int64_t)lut_sin_from_u16(comb_ver_ang))>>SIN_LUT_RESULT_SHIFT) + global_sensor_z;

				uint16_t local_comb_hor_ang = hor_ang + local_sensor_hor_ang;
				uint16_t local_comb_ver_ang = ver_ang + local_sensor_ver_ang;


				int32_t local_x = (((int64_t)d * (int64_t)lut_cos_from_u16(local_comb_ver_ang) * (int64_t)lut_cos_from_u16(local_comb_hor_ang))>>(2*SIN_LUT_RESULT_SHIFT)) + local_sensor_x;
				int32_t local_y = (((int64_t)d * (int64_t)lut_cos_from_u16(local_comb_ver_ang) * (int64_t)lut_sin_from_u16(local_comb_hor_ang))>>(2*SIN_LUT_RESULT_SHIFT)) + local_sensor_y;
				int32_t local_z = (((int64_t)d * (int64_t)lut_sin_from_u16(local_comb_ver_ang))>>SIN_LUT_RESULT_SHIFT) + local_sensor_z;

				#ifdef DBGPRVOX

					if(sidx == DBGPRSIDX && py*TOF_XS+px == PIX)
					{
						DBG_PR_VAR_I32(x);
						DBG_PR_VAR_I32(y);
						DBG_PR_VAR_I32(z);

						DBG_PR_VAR_U16(local_sensor_hor_ang/IN_01_DEG);
						DBG_PR_VAR_U16(local_sensor_ver_ang/IN_01_DEG);

						DBG_PR_VAR_I32(local_x);
						DBG_PR_VAR_I32(local_y);
						DBG_PR_VAR_I32(local_z);

					}
				#endif


				// VACUUM APP: Ignore the nozzle
				#define NOZZLE_WIDTH 760
				if(local_z < 200 && local_x < 520 && local_x > 120 && local_y > -(NOZZLE_WIDTH/2) && local_y < (NOZZLE_WIDTH/2))
					continue;

				#define OBST_MARGIN (50)

				#define OBST_AVOID_WIDTH (600+OBST_MARGIN)

				if(local_z > 120 && local_z < 1200)
				{
					if(local_y > -(OBST_AVOID_WIDTH/2) && local_y < (OBST_AVOID_WIDTH/2))
					{
						if(local_x >= 100 && local_x < 450+OBST_MARGIN)
							obstacle_front_near++;
						if(local_x >= 450+OBST_MARGIN && local_x < 650+OBST_MARGIN)
							obstacle_front_far++;

						if(local_x <= -450 && local_x > -700-OBST_MARGIN)
							obstacle_back_near++;
						if(local_x <= -700-OBST_MARGIN && local_x > -900-OBST_MARGIN)
							obstacle_back_far++;
					}

					if(local_x > -490 && local_x < -200)
					{
						if(local_y >= 200+OBST_MARGIN && local_y < 400+OBST_MARGIN)
							obstacle_left_near++;

						if(local_y <= -200 && local_y > -400-OBST_MARGIN)
							obstacle_right_near++;
					}


					if(local_x > -440 && local_x < -200)
					{

						if(local_y <= -400-OBST_MARGIN && local_y > -500-OBST_MARGIN)
							obstacle_right_far++;

						if(local_y >= 400+OBST_MARGIN && local_y < 500+OBST_MARGIN)
							obstacle_left_far++;

					}
				}

				// Completely ignore nozzle area obstacles for mapping, but give the floor if visible!
				if(local_z > 100 && local_x < 520 && local_x > 120 && local_y > -(NOZZLE_WIDTH/2) && local_y < (NOZZLE_WIDTH/2))
					continue;

				if(chafind_enabled && local_z > 170 && local_z < 230) // was 170..220
				{
					micronavi_point_in_chafind(local_x, local_y, local_z, 0, 0);
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

							#ifdef DBGPRVOX
								if(sidx == DBGPRSIDX && py*TOF_XS+px == PIX)
								{
									DBG_PR_VAR_I32(seg);
									DBG_PR_VAR_I32(x);
									DBG_PR_VAR_I32(y);
								}
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

	#ifdef DBGPRVOX
		if(sidx == DBGPRSIDX)
		{
			DBG_PR_VAR_I32(insertion_cnt);
		}

	#endif
}
void tof_to_voxmap_narrow(uint8_t *nar_ampl, uint16_t *nar_dist, int sidx, uint8_t ampl_accept_min, uint8_t ampl_accept_max, int32_t ref_x, int32_t ref_y) __attribute__((section(".text_itcm")));
void tof_to_voxmap_narrow(uint8_t *nar_ampl, uint16_t *nar_dist, int sidx, uint8_t ampl_accept_min, uint8_t ampl_accept_max, int32_t ref_x, int32_t ref_y)
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
	if(abso(robot_x-ref_x) > 4000 || abso(robot_y-ref_y) > 4000) error(151);

	uint16_t robot_ang = cur_pos.ang>>16;

	// Rotation: xr = x*cos(a) + y*sin(a)
	//           yr = -x*sin(a) + y*cos(a)
	// It seems to me this widely touted formula has inverted y axis, don't understand why, so it should be:
	// Rotation: xr = x*cos(a) - y*sin(a)
	//           yr = x*sin(a) + y*cos(a)


	uint16_t global_sensor_hor_ang = sensor_mounts[sidx].ang_rel_robot + robot_ang;
//	uint16_t global_sensor_ver_ang = sensor_mounts[sidx].vert_ang_rel_ground;

	/*
		TODO: Implement proper full 3D transformation to each pixel, taking into account the robot pitch and roll.

		For now, we only correct the sensor vertical angle by robot pitch and roll, which works well for the usual
		case of small pitch/roll angles, which mostly causes massive shifts in sensor vertical angle. Sensors do
		rotate as well, and this isn't currently taken into account.

		Positive pitch = nose goes up:
			sensor with ang=0deg   increments directly by pitch
			sensor with ang=180deg increments directly by -1*pitch
			sensors with ang=90deg, 270 deg are not affected

			-->
			ver_ang += cos(sensor_mount_ang)*pitch

		Positive roll = "left wing" rises:
			sensor with ang=90deg, increments directly by roll
			sensor with ang=270deg, increments directly by -1*roll
			0 deg, 180 deg not affected

			-->
			ver_ang += sin(sensor_mount_ang)*roll
	*/

	int16_t pitch_ang = cur_pos.pitch>>16;
	int16_t roll_ang = cur_pos.roll>>16;

	uint16_t global_sensor_ver_ang = 
		(int32_t)((int16_t)sensor_mounts[sidx].vert_ang_rel_ground) +
		((lut_cos_from_u16(sensor_mounts[sidx].ang_rel_robot)*pitch_ang)>>SIN_LUT_RESULT_SHIFT) +
		((lut_sin_from_u16(sensor_mounts[sidx].ang_rel_robot)*roll_ang)>>SIN_LUT_RESULT_SHIFT);

	#ifdef DBGPRVOX_NARROW

		#define IN_01_DEG (ANG_0_1_DEG/65536)
		DBG_PR_VAR_U16(sidx);
		DBG_PR_VAR_I16((int16_t)sensor_mounts[sidx].vert_ang_rel_ground/IN_01_DEG);
		DBG_PR_VAR_I16(pitch_ang/IN_01_DEG);
		DBG_PR_VAR_I16(roll_ang/IN_01_DEG);
		DBG_PR_VAR_I16((int16_t)global_sensor_ver_ang/IN_01_DEG);
	#endif



	int32_t  global_sensor_x = robot_x - ref_x +
			((lut_cos_from_u16(robot_ang)*sensor_mounts[sidx].x_rel_robot)>>SIN_LUT_RESULT_SHIFT) +
			((lut_sin_from_u16(robot_ang)*-1*sensor_mounts[sidx].y_rel_robot)>>SIN_LUT_RESULT_SHIFT);

	int32_t  global_sensor_y = robot_y - ref_y + 
			((lut_sin_from_u16(robot_ang)*sensor_mounts[sidx].x_rel_robot)>>SIN_LUT_RESULT_SHIFT) +
			((lut_cos_from_u16(robot_ang)*sensor_mounts[sidx].y_rel_robot)>>SIN_LUT_RESULT_SHIFT);
	int32_t  global_sensor_z = sensor_mounts[sidx].z_rel_ground;



	#ifdef DBGPRVOX_NARROW

		DBG_PR_VAR_I32(robot_ang);
		DBG_PR_VAR_I32(robot_x);
		DBG_PR_VAR_I32(robot_y);

		DBG_PR_VAR_I32(global_sensor_hor_ang);
		DBG_PR_VAR_I32(global_sensor_ver_ang);
		DBG_PR_VAR_I32(global_sensor_x);
		DBG_PR_VAR_I32(global_sensor_y);
		DBG_PR_VAR_I32(global_sensor_z);
	#endif


	int insertion_cnt = 0;
	for(int py=1; py<TOF_YS_NARROW-1; py++)
	{
		for(int px=1; px<TOF_XS_NARROW-1; px++)
		{
			int32_t dists[5];

			dists[0] = nar_dist[(py+0)*TOF_XS_NARROW+(px+0)];
			dists[1] = nar_dist[(py-1)*TOF_XS_NARROW+(px+0)];
			dists[2] = nar_dist[(py+1)*TOF_XS_NARROW+(px+0)];
			dists[3] = nar_dist[(py+0)*TOF_XS_NARROW+(px+1)];
			dists[4] = nar_dist[(py+0)*TOF_XS_NARROW+(px-1)];

			uint8_t ampls[5];
			ampls[0] = nar_ampl[(py+0)*TOF_XS_NARROW+(px+0)];
			ampls[1] = nar_ampl[(py-1)*TOF_XS_NARROW+(px+0)];
			ampls[2] = nar_ampl[(py+1)*TOF_XS_NARROW+(px+0)];
			ampls[3] = nar_ampl[(py+0)*TOF_XS_NARROW+(px+1)];
			ampls[4] = nar_ampl[(py+0)*TOF_XS_NARROW+(px-1)];

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

			if(n_conform >= 5)
			{
				int32_t d = conform_avg / n_conform;

				uint16_t hor_ang, ver_ang;

				#ifdef DBGPRVOX_NARROW

					if(py*TOF_XS_NARROW+px == PIX_NARROW)
						DBG_PR_VAR_I32(d);
				#endif

				// TODO: This optimizes out once we have sensor-by-sensor geometric tables;
				// they can be pre-built to the actual mount_mode.

				int i_geo = (py+TOF_NARROW_Y_START)*TOF_XS+(px+TOF_NARROW_X_START);

				switch(sensor_mounts[sidx].mount_mode)
				{
					case 1: 
					hor_ang = -1*geocoords[i_geo].yang;
					ver_ang = geocoords[i_geo].xang;
					break;

					case 2: 
					hor_ang = geocoords[i_geo].yang;
					ver_ang = -1*geocoords[i_geo].xang;
					break;

					case 3:
					hor_ang = -1*geocoords[i_geo].xang;
					ver_ang = geocoords[i_geo].yang;
					break;

					case 4:
					hor_ang = geocoords[i_geo].xang;
					ver_ang = -1*geocoords[i_geo].yang;
					break;

					default: error(145); while(1); // to tell the compiler we always set hor_ang, ver_ang
				}

				#ifdef DBGPRVOX_NARROW

					if(py*TOF_XS_NARROW+px == PIX_NARROW)
					{
						DBG_PR_VAR_U16(hor_ang);
						DBG_PR_VAR_U16(ver_ang);
					}
				#endif

				uint16_t comb_hor_ang = hor_ang + global_sensor_hor_ang;
				uint16_t comb_ver_ang = ver_ang + global_sensor_ver_ang;

				#ifdef DBGPRVOX_NARROW

					if(py*TOF_XS_NARROW+px == PIX_NARROW)
					{
						DBG_PR_VAR_U16(comb_hor_ang);
						DBG_PR_VAR_U16(comb_ver_ang);
					}
				#endif

				int32_t x = (((int64_t)d * (int64_t)lut_cos_from_u16(comb_ver_ang) * (int64_t)lut_cos_from_u16(comb_hor_ang))>>(2*SIN_LUT_RESULT_SHIFT)) + global_sensor_x;
				int32_t y = (((int64_t)d * (int64_t)lut_cos_from_u16(comb_ver_ang) * (int64_t)lut_sin_from_u16(comb_hor_ang))>>(2*SIN_LUT_RESULT_SHIFT)) + global_sensor_y;
				int32_t z = (((int64_t)d * (int64_t)lut_sin_from_u16(comb_ver_ang))>>SIN_LUT_RESULT_SHIFT) + global_sensor_z;

				#ifdef DBGPRVOX_NARROW

					if(py*TOF_XS_NARROW+px == PIX_NARROW)
					{
						DBG_PR_VAR_I32(x);
						DBG_PR_VAR_I32(y);
						DBG_PR_VAR_I32(z);
					}
				#endif


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

							#ifdef DBGPRVOX_NARROW

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

	#ifdef DBGPRVOX_NARROW
		DBG_PR_VAR_I32(insertion_cnt);
	#endif
}

void tof_to_obstacle_avoidance(uint8_t *wid_ampl, uint16_t *wid_dist, int sidx, uint8_t ampl_accept_min, uint8_t ampl_accept_max) __attribute__((section(".text_itcm")));
void tof_to_obstacle_avoidance(uint8_t *wid_ampl, uint16_t *wid_dist, int sidx, uint8_t ampl_accept_min, uint8_t ampl_accept_max)
{
	if(sidx < 0 || sidx >= N_SENSORS) error(150);



	uint16_t local_sensor_hor_ang = sensor_mounts[sidx].ang_rel_robot;
	uint16_t local_sensor_ver_ang = sensor_mounts[sidx].vert_ang_rel_ground;


	int32_t  local_sensor_x = sensor_mounts[sidx].x_rel_robot;
	int32_t  local_sensor_y = sensor_mounts[sidx].y_rel_robot;
	int32_t  local_sensor_z = sensor_mounts[sidx].z_rel_ground;


	extern int obstacle_front_near, obstacle_back_near, obstacle_left_near, obstacle_right_near;
	extern int obstacle_front_far, obstacle_back_far, obstacle_left_far, obstacle_right_far;

	for(int py=1; py<TOF_YS-1; py++)
	{
		for(int px=1; px<TOF_XS-1; px++)
		{
			int32_t dists[5];

			dists[0] = wid_dist[(py+0)*TOF_XS+(px+0)];
			dists[1] = wid_dist[(py-1)*TOF_XS+(px+0)];
			dists[2] = wid_dist[(py+1)*TOF_XS+(px+0)];
			dists[3] = wid_dist[(py+0)*TOF_XS+(px+1)];
			dists[4] = wid_dist[(py+0)*TOF_XS+(px-1)];

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

			if(n_conform >= 5)
			{
				int32_t d = conform_avg / n_conform;

				uint16_t hor_ang, ver_ang;

				#ifdef DBGPRVOX_AVOIDANCE

					if(py*TOF_XS+px == PIX)
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

				#ifdef DBGPRVOX_AVOIDANCE

					if(py*TOF_XS+px == PIX)
					{
						DBG_PR_VAR_U16(hor_ang);
						DBG_PR_VAR_U16(ver_ang);
					}
				#endif


				uint16_t local_comb_hor_ang = hor_ang + local_sensor_hor_ang;
				uint16_t local_comb_ver_ang = ver_ang + local_sensor_ver_ang;


				int32_t local_x = (((int64_t)d * (int64_t)lut_cos_from_u16(local_comb_ver_ang) * (int64_t)lut_cos_from_u16(local_comb_hor_ang))>>(2*SIN_LUT_RESULT_SHIFT)) + local_sensor_x;
				int32_t local_y = (((int64_t)d * (int64_t)lut_cos_from_u16(local_comb_ver_ang) * (int64_t)lut_sin_from_u16(local_comb_hor_ang))>>(2*SIN_LUT_RESULT_SHIFT)) + local_sensor_y;
				int32_t local_z = (((int64_t)d * (int64_t)lut_sin_from_u16(local_comb_ver_ang))>>SIN_LUT_RESULT_SHIFT) + local_sensor_z;


				// VACUUM APP: Ignore the nozzle
				if(local_z < 200 && local_x < 520 && local_x > 120 && local_y > -(NOZZLE_WIDTH/2) && local_y < (NOZZLE_WIDTH/2))
					continue;

				if(local_z > 120 && local_z < 1200)
				{
					if(local_y > -(OBST_AVOID_WIDTH/2) && local_y < (OBST_AVOID_WIDTH/2))
					{
						if(local_x >= 100 && local_x < 450+OBST_MARGIN)
							obstacle_front_near++;
						if(local_x >= 450+OBST_MARGIN && local_x < 650+OBST_MARGIN)
							obstacle_front_far++;

						if(local_x <= -450 && local_x > -700-OBST_MARGIN)
							obstacle_back_near++;
						if(local_x <= -700-OBST_MARGIN && local_x > -900-OBST_MARGIN)
							obstacle_back_far++;
					}

					if(local_x > -490 && local_x < -200)
					{
						if(local_y >= 200+OBST_MARGIN && local_y < 400+OBST_MARGIN)
							obstacle_left_near++;

						if(local_y <= -200 && local_y > -400-OBST_MARGIN)
							obstacle_right_near++;
					}


					if(local_x > -440 && local_x < -200)
					{

						if(local_y <= -400-OBST_MARGIN && local_y > -500-OBST_MARGIN)
							obstacle_right_far++;

						if(local_y >= 400+OBST_MARGIN && local_y < 500+OBST_MARGIN)
							obstacle_left_far++;

					}
				}
			}
		}
	}

	#ifdef DBGPRVOX_AVOIDANCE
		DBG_PR_VAR_I32(insertion_cnt);
	#endif
}


/*
	2DCS -> amplitude&distance conversion for 6.67MHz modulation frequency.
	Does ambient correction, and pixel-by-pixel correction. There is no curve shape (nonlinearity) correction.
*/
void compensated_2dcs_6mhz_ampl_dist(uint8_t *ampl_out, uint16_t *dist_out, epc_2dcs_t *in, epc_img_t *bwimg) __attribute__((section(".text_itcm")));
void compensated_2dcs_6mhz_ampl_dist(uint8_t *ampl_out, uint16_t *dist_out, epc_2dcs_t *in, epc_img_t *bwimg)
{
	int32_t base_offs = shadow_luts.lof.wid_base_offset_2dcs;
	for(int i=0; i < TOF_XS*TOF_YS; i++)
	{
		uint16_t dist;
		int ampl;

		int16_t dcs0 = ((in->dcs[0].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs1 = ((in->dcs[1].img[i]&0b0011111111111100)>>2)-2048;

		if(dcs0 < -2047 || dcs0 > 2046 || dcs1 < -2047 || dcs1 > 2046)
		{
			ampl = 255;
			dist = 65535;
		}
		else
		{
			int16_t dcs31 = -dcs1;
			int16_t dcs20 = -dcs0;


			#ifdef DO_AMB_CORR
				int16_t bw = ((bwimg->img[i]&0b0011111111111100)>>2)-2048;
				if(bw<0) bw=0;
				int corr_factor;
				if(!(i&1)) // even
					corr_factor = shadow_luts.lof.amb_corr[i/2] & 0x0f;
				else
					corr_factor = shadow_luts.lof.amb_corr[i/2]>>4;
				int16_t comp = (bw*corr_factor)>>4;

				dcs31 -= comp;
				dcs20 -= comp;
			#endif

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

				dist_i += base_offs;

				int32_t pix_offs;
				if(!(i&1)) // even
					pix_offs = shadow_luts.lof.wid_offsets_2dcs[i/2] & 0x0f;
				else
					pix_offs = shadow_luts.lof.wid_offsets_2dcs[i/2]>>4;

				pix_offs <<= 5; // pixel offset unit is 32mm

				dist_i += pix_offs;

				dist_i += TOF_TBL_HALF_PERIOD;


				if(dist_i < 0) dist_i += TOF_TBL_PERIOD;				
				else if(dist_i >= TOF_TBL_PERIOD) dist_i -= TOF_TBL_PERIOD;

				ampl = AMPL(dcs20, dcs31);
				if(ampl > 255) ampl = 255;

				dist = dist_i;
			}

		}
		ampl_out[i] = ampl;
		dist_out[i] = dist;
	}
}
void compensated_2dcs_6mhz_ampl_dist_narrow(uint8_t *ampl_out, uint16_t *dist_out, epc_2dcs_narrow_t *in, epc_img_t *bwimg) __attribute__((section(".text_itcm")));
void compensated_2dcs_6mhz_ampl_dist_narrow(uint8_t *ampl_out, uint16_t *dist_out, epc_2dcs_narrow_t *in, epc_img_t *bwimg)
{
	int32_t base_offs = shadow_luts.lof.nar_base_offset_2dcs;
	for(int yy=0; yy < TOF_YS_NARROW; yy++)
	{
		for(int xx=0; xx < TOF_XS_NARROW; xx++)
		{
			int i = yy*TOF_XS_NARROW+xx;
			uint16_t dist;
			int ampl;

			int16_t dcs0 = ((in->dcs[0].img[i]&0b0011111111111100)>>2)-2048;
			int16_t dcs1 = ((in->dcs[1].img[i]&0b0011111111111100)>>2)-2048;

			if(dcs0 < -2047 || dcs0 > 2046 || dcs1 < -2047 || dcs1 > 2046)
			{
				ampl = 255;
				dist = 0;
			}
			else
			{
				int16_t dcs31 = -dcs1;
				int16_t dcs20 = -dcs0;


				#ifdef DO_AMB_CORR
					int i_bw = (yy+TOF_NARROW_Y_START)*TOF_XS+(xx+TOF_NARROW_X_START);
					int16_t bw = ((bwimg->img[i_bw]&0b0011111111111100)>>2)-2048;
					if(bw<0) bw=0;
					int corr_factor;
					if(!(i&1)) // even
						corr_factor = shadow_luts.lof.amb_corr[i_bw/2] & 0x0f;
					else
						corr_factor = shadow_luts.lof.amb_corr[i_bw/2]>>4;
					int16_t comp = (bw*corr_factor)>>4;

					dcs31 -= comp;
					dcs20 -= comp;
				#endif

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

					dist_i += base_offs;

					int32_t pix_offs;
					if(!(i&1)) // even
						pix_offs = shadow_luts.lof.nar_offsets_2dcs[i/2] & 0x0f;
					else
						pix_offs = shadow_luts.lof.nar_offsets_2dcs[i/2]>>4;

					pix_offs <<= 5; // pixel offset unit is 32mm

					dist_i += pix_offs;

					dist_i += TOF_TBL_HALF_PERIOD;
					if(dist_i < 0) dist_i += TOF_TBL_PERIOD;				
					else if(dist_i >= TOF_TBL_PERIOD) dist_i -= TOF_TBL_PERIOD;

					ampl = AMPL(dcs20, dcs31);
					if(ampl > 255) ampl = 255;

					dist = dist_i;
				}

			}
			ampl_out[i] = ampl;
			dist_out[i] = dist;
		}
	}
}


// This version outputs distance only, not amplitude; instead, it internally masks low-amplitude (dist=65534) and overexposed (dist=65535) parts away.
void compensated_2dcs_6mhz_dist_masked(uint16_t *dist_out, epc_2dcs_t *in, epc_img_t *bwimg) __attribute__((section(".text_itcm")));
void compensated_2dcs_6mhz_dist_masked(uint16_t *dist_out, epc_2dcs_t *in, epc_img_t *bwimg)
{
	int32_t base_offs = shadow_luts.lof.wid_base_offset_2dcs;
	for(int i=0; i < TOF_XS*TOF_YS; i++)
	{
		uint16_t dist;
		int ampl;

		int16_t dcs0 = ((in->dcs[0].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs1 = ((in->dcs[1].img[i]&0b0011111111111100)>>2)-2048;

		if(dcs0 < -2047 || dcs0 > 2046 || dcs1 < -2047 || dcs1 > 2046)
		{
			ampl = 255;
			dist = 65535;
		}
		else
		{
			int16_t dcs31 = -dcs1;
			int16_t dcs20 = -dcs0;


			#ifdef DO_AMB_CORR
				int16_t bw = ((bwimg->img[i]&0b0011111111111100)>>2)-2048;
				if(bw<0) bw=0;
				int corr_factor;
				if(!(i&1)) // even
					corr_factor = shadow_luts.lof.amb_corr[i/2] & 0x0f;
				else
					corr_factor = shadow_luts.lof.amb_corr[i/2]>>4;
				int16_t comp = (bw*corr_factor)>>4;

				dcs31 -= comp;
				dcs20 -= comp;
			#endif

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

				dist_i += base_offs;

				int32_t pix_offs;
				if(!(i&1)) // even
					pix_offs = shadow_luts.lof.wid_offsets_2dcs[i/2] & 0x0f;
				else
					pix_offs = shadow_luts.lof.wid_offsets_2dcs[i/2]>>4;

				pix_offs <<= 5; // pixel offset unit is 32mm

				dist_i += pix_offs;

				dist_i += TOF_TBL_HALF_PERIOD;


				if(dist_i < 0) dist_i += TOF_TBL_PERIOD;				
				else if(dist_i >= TOF_TBL_PERIOD) dist_i -= TOF_TBL_PERIOD;

				ampl = AMPL(dcs20, dcs31);
				if(ampl > 255) ampl = 255;

				if(ampl < 3)
					dist = 65534;
				else
					dist = dist_i;
			}

		}
		dist_out[i] = dist;
	}
}
void compensated_2dcs_6mhz_dist_masked_narrow(uint16_t *dist_out, epc_2dcs_narrow_t *in, epc_img_t *bwimg) __attribute__((section(".text_itcm")));
void compensated_2dcs_6mhz_dist_masked_narrow(uint16_t *dist_out, epc_2dcs_narrow_t *in, epc_img_t *bwimg)
{
	int32_t base_offs = shadow_luts.lof.nar_base_offset_2dcs;
	for(int yy=0; yy < TOF_YS_NARROW; yy++)
	{
		for(int xx=0; xx < TOF_XS_NARROW; xx++)
		{
			int i = yy*TOF_XS_NARROW+xx;
			uint16_t dist;
			int ampl;

			int16_t dcs0 = ((in->dcs[0].img[i]&0b0011111111111100)>>2)-2048;
			int16_t dcs1 = ((in->dcs[1].img[i]&0b0011111111111100)>>2)-2048;

			if(dcs0 < -2047 || dcs0 > 2046 || dcs1 < -2047 || dcs1 > 2046)
			{
				ampl = 255;
				dist = 65535;
			}
			else
			{
				int16_t dcs31 = -dcs1;
				int16_t dcs20 = -dcs0;


				#ifdef DO_AMB_CORR
					int i_bw = (yy+TOF_NARROW_Y_START)*TOF_XS+(xx+TOF_NARROW_X_START);
					int16_t bw = ((bwimg->img[i_bw]&0b0011111111111100)>>2)-2048;
					if(bw<0) bw=0;
					int corr_factor;
					if(!(i&1)) // even
						corr_factor = shadow_luts.lof.amb_corr[i_bw/2] & 0x0f;
					else
						corr_factor = shadow_luts.lof.amb_corr[i_bw/2]>>4;
					int16_t comp = (bw*corr_factor)>>4;

					dcs31 -= comp;
					dcs20 -= comp;
				#endif

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

					dist_i += base_offs;

					int32_t pix_offs;
					if(!(i&1)) // even
						pix_offs = shadow_luts.lof.nar_offsets_2dcs[i/2] & 0x0f;
					else
						pix_offs = shadow_luts.lof.nar_offsets_2dcs[i/2]>>4;

					pix_offs <<= 5; // pixel offset unit is 32mm

					dist_i += pix_offs;

					dist_i += TOF_TBL_HALF_PERIOD;


					if(dist_i < 0) dist_i += TOF_TBL_PERIOD;				
					else if(dist_i >= TOF_TBL_PERIOD) dist_i -= TOF_TBL_PERIOD;

					ampl = AMPL(dcs20, dcs31);
					if(ampl > 255) ampl = 255;

					if(ampl < 3)
						dist = 65534;
					else
						dist = dist_i;
				}

			}
			dist_out[i] = dist;
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

