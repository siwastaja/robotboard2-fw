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

void verify_calibration()
{
	for(int s=0; s<N_SENSORS; s++)
	{
		if(!sensors_in_use[s])
			continue;

		if(tof_calibs[s]->chip_id != sensor_silicon_ids[s])
		{
			SAFETY_SHUTDOWN();
			DBG_PR_VAR_U16(s);
			DBG_PR_VAR_U32_HEX(tof_calibs[s]->chip_id);
			DBG_PR_VAR_U32_HEX(sensor_silicon_ids[s]);
			error(393);
		}
	}
}

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
	if(f<0 || f>2) error(123);
	if(f<2)
	{
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
// d20, d31 must be small enough so that multiplying by TOF_TBL_SEG_LEN_MULT (max. 257 by design) doesn't overflow (easily satisfied with 16-bit data)
static inline uint16_t lookup_dist(int beam, int g, int32_t d31, int32_t d20) __attribute__((always_inline));
static inline uint16_t lookup_dist(int beam, int g, int32_t d31, int32_t d20)
{
	int s;
	int i;
	if(d31 >= 0)
	{
		if(d20 >= 0)
		{
			if(d31 > d20) {s=5; i=(TOF_TBL_SEG_LEN_MULT*d20)/d31;}
			else          {s=4; i=(TOF_TBL_SEG_LEN_MULT*d31)/d20;}
		}
		else
		{
			d20 *= -1;
			if(d31 > d20) {s=6; i=(TOF_TBL_SEG_LEN_MULT*d20)/d31;}
			else          {s=7; i=(TOF_TBL_SEG_LEN_MULT*d31)/d20;}
		}		
	}
	else
	{
		d31 *= -1;
		if(d20 >= 0)
		{
			if(d31 > d20) {s=2; i=(TOF_TBL_SEG_LEN_MULT*d20)/d31;}
			else          {s=3; i=(TOF_TBL_SEG_LEN_MULT*d31)/d20;}
		}
		else
		{
			d20 *= -1;
			if(d31 > d20) {s=1; i=(TOF_TBL_SEG_LEN_MULT*d20)/d31;}
			else          {s=0; i=(TOF_TBL_SEG_LEN_MULT*d31)/d20;}
		}		

	}

	if(beam == 0)
		return shadow_luts.hif.wid_luts[g][s][i];
	else
		return shadow_luts.hif.nar_luts[g][s][i];
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

#if 0
#define BLUR_R 6
#define BLUR_DIV (2*BLUR_R+1)
static void boxblur_h(int16_t* in, int16_t* out)
{
	for(int i=0; i<TOF_YS; i++)
	{
		int ti = i*TOF_XS;
		int li = ti;
		int ri = ti+BLUR_R;

		int fv = 0; //in[ti];
		int lv = 0; //in[ti+TOF_XS-1];
		int val = (BLUR_R+1)*fv;
		for(int j=0; j<BLUR_R; j++) val += in[ti+j];
		for(int j=0  ; j<=BLUR_R ; j++) { val += in[ri++] - fv      ;   out[ti++] = val/BLUR_DIV; }
		for(int j=BLUR_R+1; j<TOF_XS-BLUR_R; j++) { val += in[ri++] - in[li++];   out[ti++] = val/BLUR_DIV; }
		for(int j=TOF_XS-BLUR_R; j<TOF_XS  ; j++) { val += lv       - in[li++];   out[ti++] = val/BLUR_DIV; }
	}
}

static void boxblur_t(int16_t* in, int16_t* out)
{
	for(int i=0; i<TOF_XS; i++)
	{
		int ti = i;
		int li = ti;
		int ri = ti+BLUR_R*TOF_XS;

		int fv = 0; //in[ti];
		int lv = 0; //in[ti+TOF_XS*(TOF_YS-1)];
		int val = (BLUR_R+1)*fv;
		for(int j=0; j<BLUR_R; j++) val += in[ti+j*TOF_XS];
		for(int j=0  ; j<=BLUR_R ; j++) { val += in[ri] - fv     ;  out[ti] = val/BLUR_DIV;  ri+=TOF_XS; ti+=TOF_XS; }
		for(int j=BLUR_R+1; j<TOF_YS-BLUR_R; j++) { val += in[ri] - in[li];  out[ti] = val/BLUR_DIV;  li+=TOF_XS; ri+=TOF_XS; ti+=TOF_XS; }
		for(int j=TOF_YS-BLUR_R; j<TOF_YS  ; j++) { val += lv      - in[li];  out[ti] = val/BLUR_DIV;  li+=TOF_XS; ti+=TOF_XS; }
	}
}

static void boxblur(int16_t* in, int16_t* out)
{
	static int16_t tmp[TOF_XS*TOF_YS];

	boxblur_h(in, tmp);
	boxblur_t(tmp, out);
}

static void blur_5_convol_biased(int16_t* in, int16_t* out, blur_params_t* blur_params)
{
/*
	Convolution matrix:

	        b+c  
	b-c      a      b-c
	        b+c

*/
	for(int yy=1; yy<TOF_YS-1; yy++)
	{
		for(int xx=1; xx<TOF_XS-1; xx++)
		{
			int bx = xx/OUTPUT_BLUR_PARAMS_RATIO_X;			
			int by = yy/OUTPUT_BLUR_PARAMS_RATIO_Y;
			int a = blur_params[OBPC(bx,by)].a;
			int b = blur_params[OBPC(bx,by)].b;
			int c = blur_params[OBPC(bx,by)].c;

			int32_t val = 
				(int32_t)in[TC(xx  , yy-1)] * (int32_t)(b+c) +
				(int32_t)in[TC(xx-1, yy  )] * (int32_t)(b-c) +
				(int32_t)in[TC(xx  , yy  )] * (int32_t)a +
				(int32_t)in[TC(xx+1, yy  )] * (int32_t)(b-c) +
				(int32_t)in[TC(xx  , yy+1)] * (int32_t)(b+c);

			val /= 256;

			out[TC(xx, yy)] = val;
		}
	}
}


static void run_blur_model(int16_t* in, int16_t* out, blur_params_t* blur_params)
{
	static int16_t tmp[TOF_XS*TOF_YS];

	// Temporarily use "out" as a buffer
	boxblur(in, out);
	boxblur(out, tmp);

	// Mix input ("in") and blurred input ("tmp") together - store result to "in"
	for(int yy=0; yy<TOF_YS; yy++)
	{
		for(int xx=0; xx<TOF_XS; xx++)
		{
			int bx = xx/OUTPUT_BLUR_PARAMS_RATIO_X;			
			int by = yy/OUTPUT_BLUR_PARAMS_RATIO_Y;
			int d = blur_params[OBPC(bx,by)].d;
			in[TC(xx,yy)] -= (d*tmp[TC(xx,yy)])/1024;
		}
	}


	blur_5_convol_biased(in, out, blur_params);
}
#endif


int flare_factors[2] = {17, 7}; // wide, narrow

#if 0
void adjust()
{
	uint8_t cmd = uart_input();

	if(cmd == 'a')
	{
		flare_factors[0]++;
		DBG_PR_VAR_I32(flare_factors[0]);
	}
	else if(cmd == 'z')
	{
		flare_factors[0]--;
		DBG_PR_VAR_I32(flare_factors[0]);
	}
	else if(cmd == 's')
	{
		flare_factors[1]++;
		DBG_PR_VAR_I32(flare_factors[1]);
	}
	else if(cmd == 'x')
	{
		flare_factors[1]--;
		DBG_PR_VAR_I32(flare_factors[1]);
	}



}
#endif

#define OVEREXP_LIMIT 4090
#define HDR_RANGESWITCH 3500
#define DEALIAS_THRESHOLD (2300)


void compensated_2hdr_tof_calc_ampldist_flarecomp(int is_narrow, uint16_t *ampldist_out, int16_t* dcs20_lo_in, int16_t* dcs31_lo_in, int16_t* dcs20_hi_in, int16_t* dcs31_hi_in, int hdr_factor, uint8_t* dealias_dist, int freq)   __attribute__((section(".text_itcm")));
void compensated_2hdr_tof_calc_ampldist_flarecomp(int is_narrow, uint16_t *ampldist_out, int16_t* dcs20_lo_in, int16_t* dcs31_lo_in, int16_t* dcs20_hi_in, int16_t* dcs31_hi_in, int hdr_factor, uint8_t* dealias_dist, int freq)
{
	int wrap_mm = 0;
	if(dealias_dist != NULL)
	{
		if(freq==0)
			wrap_mm = 7495;
		else if(freq==1)
			wrap_mm = 14990;
		else
		{
			error(777);
			while(1);
		}
	}

	int n_pix = is_narrow?(TOF_XS_NARROW*TOF_YS_NARROW):(TOF_XS*TOF_YS);
	int dcs20_accum = 0;
	int dcs31_accum = 0;
	for(int i=0; i < n_pix; i++)
	{
		int16_t dcs20_lo = dcs20_lo_in[i];
		int16_t dcs31_lo = dcs31_lo_in[i];
		int16_t dcs20_hi = dcs20_hi_in[i];
		int16_t dcs31_hi = dcs31_hi_in[i];

		int32_t dcs20, dcs31;

		if(dcs20_hi < -HDR_RANGESWITCH || dcs20_hi > HDR_RANGESWITCH || dcs31_hi < -HDR_RANGESWITCH || dcs31_hi > HDR_RANGESWITCH)
		{
			dcs20 = (int32_t)dcs20_lo*hdr_factor;
			dcs31 = (int32_t)dcs31_lo*hdr_factor;
		}
		else
		{
			dcs20 = dcs20_hi;
			dcs31 = dcs31_hi;
		}

		dcs20_accum += dcs20;
		dcs31_accum += dcs31;
	}

	dcs20_accum /= n_pix;
	dcs31_accum /= n_pix;

//	DBG_PR_VAR_I32(flare_factors[0]);
//	DBG_PR_VAR_I32(dcs20_accum);
//	DBG_PR_VAR_I32(dcs31_accum);

	for(int i=0; i < n_pix; i++)
	{
		uint16_t dist;
		int ampl;

		int16_t dcs20_lo = dcs20_lo_in[i];
		int16_t dcs31_lo = dcs31_lo_in[i];
		int16_t dcs20_hi = dcs20_hi_in[i];
		int16_t dcs31_hi = dcs31_hi_in[i];


		if(dcs20_lo < -OVEREXP_LIMIT || dcs20_lo > OVEREXP_LIMIT || dcs31_lo < -OVEREXP_LIMIT || dcs31_lo > OVEREXP_LIMIT)
		{
			ampl = 15;
			dist = DIST_OVEREXP;
		}
		else
		{

			int32_t dcs20, dcs31;

			if(dcs20_hi < -HDR_RANGESWITCH || dcs20_hi > HDR_RANGESWITCH || dcs31_hi < -HDR_RANGESWITCH || dcs31_hi > HDR_RANGESWITCH)
			{
				dcs20 = (int32_t)dcs20_lo*hdr_factor;
				dcs31 = (int32_t)dcs31_lo*hdr_factor;
			}
			else
			{
				dcs20 = dcs20_hi;
				dcs31 = dcs31_hi;
			}

			dcs31 -= ((int64_t)dcs31_accum*(int64_t)flare_factors[is_narrow])>>8;
			dcs20 -= ((int64_t)dcs20_accum*(int64_t)flare_factors[is_narrow])>>8;

			ampl = AMPL(dcs20, dcs31);

			if(ampl<4)
			{
				dist = DIST_UNDEREXP;
			}
			else
			{
				int pixgroup;
				if(!is_narrow) // is wide
				{
					if(!(i&1)) // even
						pixgroup = shadow_luts.hif.wid_lut_group_ids[i/2] & 0x0f;
					else
						pixgroup = shadow_luts.hif.wid_lut_group_ids[i/2]>>4;

					dist = lookup_dist(0, pixgroup, dcs31, dcs20);
					if(dist < 2) dist = 2;

				}
				else // is narrow
				{
					if(!(i&1)) // even
						pixgroup = shadow_luts.hif.nar_lut_group_ids[i/2] & 0x0f;
					else
						pixgroup = shadow_luts.hif.nar_lut_group_ids[i/2]>>4;

					dist = lookup_dist(1, pixgroup, dcs31, dcs20);
					if(dist < 2) dist = 2;

				}

				if(dealias_dist != NULL)
				{
					int lfdist_mm = dealias_dist[i]<<7;
					int hf_wrap0 = dist;
					int hf_wrap1 = dist + wrap_mm;
					int hf_wrap2 = dist + 2*wrap_mm;

					int err0 = abso(hf_wrap0 - lfdist_mm);
					int err1 = abso(hf_wrap1 - lfdist_mm);
					int err2 = abso(hf_wrap2 - lfdist_mm);

					if(dealias_dist[i] == DIST_UNDEREXP)
						dist = DIST_UNDEREXP;
					else if(dealias_dist[i] == DIST_OVEREXP || err0 < DEALIAS_THRESHOLD)
						dist = dist>>DIST_SHIFT; // Keep hf as is
					else if(err1 < DEALIAS_THRESHOLD)
						dist = hf_wrap1>>DIST_SHIFT;
					else if(err2 < DEALIAS_THRESHOLD)
						dist = hf_wrap2>>DIST_SHIFT;
					else
						dist = DIST_UNDEREXP;

				}
				else
					dist>>=DIST_SHIFT;

				if(dist < 0) dist = 0;
				else if(dist > MAX_DISTVAL) dist = DIST_UNDEREXP;
			}

			ampl/=16;
			if(ampl>15) ampl=15;

		}
		ampldist_out[i] = (ampl<<12) | dist;
	}
}

// val1 = short exp, val3 = long exp, ratio1 = short to mid, ratio2 = mid to long:
static inline int32_t hdrize(int16_t val1, int16_t val2, int16_t val3, int ratio1, int ratio2) __attribute__((always_inline));
static inline int32_t hdrize(int16_t val1, int16_t val2, int16_t val3, int ratio1, int ratio2)
{
	int32_t val1_extended = val1 * ratio1 * ratio2;
	int32_t val2_extended = val2 * ratio2;
	int32_t val3_extended = val3;

	int32_t val_out;

	if(val2 > HDR_RANGESWITCH || val2 < -HDR_RANGESWITCH)
		val_out = val1_extended;
	else if(val3 > HDR_RANGESWITCH || val3 < -HDR_RANGESWITCH)
		val_out = val2_extended;
	else
		val_out = val3_extended;

	return val_out;
}


void compensated_3hdr_tof_calc_ampldist_flarecomp(int is_narrow, uint16_t *ampldist_out, int16_t* dcs20_lo_in, int16_t* dcs31_lo_in, int16_t* dcs20_mid_in, int16_t* dcs31_mid_in, int16_t* dcs20_hi_in, int16_t* dcs31_hi_in, int hdr_factor_lomid, int hdr_factor_midhi, uint8_t* dealias_dist, int freq)   __attribute__((section(".text_itcm")));
void compensated_3hdr_tof_calc_ampldist_flarecomp(int is_narrow, uint16_t *ampldist_out, int16_t* dcs20_lo_in, int16_t* dcs31_lo_in, int16_t* dcs20_mid_in, int16_t* dcs31_mid_in, int16_t* dcs20_hi_in, int16_t* dcs31_hi_in, int hdr_factor_lomid, int hdr_factor_midhi, uint8_t* dealias_dist, int freq)
{
	int wrap_mm;
	if(dealias_dist != NULL)
	{
		if(freq==0)
			wrap_mm = 7495;
		else if(freq==1)
			wrap_mm = 14990;
		else
		{
			error(777);
			while(1);
		}
	}

	int n_pix = is_narrow?(TOF_XS_NARROW*TOF_YS_NARROW):(TOF_XS*TOF_YS);
	int64_t dcs20_accum = 0;
	int64_t dcs31_accum = 0;
	for(int i=0; i < n_pix; i++)
	{
		dcs20_accum += hdrize(dcs20_lo_in[i], dcs20_mid_in[i], dcs20_hi_in[i], hdr_factor_lomid, hdr_factor_midhi);
		dcs20_accum += hdrize(dcs31_lo_in[i], dcs31_mid_in[i], dcs31_hi_in[i], hdr_factor_lomid, hdr_factor_midhi);
	}

	dcs20_accum /= n_pix;
	dcs31_accum /= n_pix;

//	DBG_PR_VAR_I32(flare_factors[0]);
//	DBG_PR_VAR_I32(dcs20_accum);
//	DBG_PR_VAR_I32(dcs31_accum);

	for(int i=0; i < n_pix; i++)
	{
		uint16_t dist;
		int ampl;

		if(dcs20_lo_in[i] < -OVEREXP_LIMIT || dcs20_lo_in[i] > OVEREXP_LIMIT || 
		   dcs31_lo_in[i] < -OVEREXP_LIMIT || dcs31_lo_in[i] > OVEREXP_LIMIT)
		{
			ampl = 15;
			dist = DIST_OVEREXP;

		}
		else
		{
			int32_t dcs20, dcs31;

			dcs20 = hdrize(dcs20_lo_in[i], dcs20_mid_in[i], dcs20_hi_in[i], hdr_factor_lomid, hdr_factor_midhi);
			dcs31 = hdrize(dcs31_lo_in[i], dcs31_mid_in[i], dcs31_hi_in[i], hdr_factor_lomid, hdr_factor_midhi);

#if 0
			{
				DBG_PR_VAR_I32(dcs20_lo_in[i]);
				DBG_PR_VAR_I32(dcs31_lo_in[i]);

				DBG_PR_VAR_I32(dcs20_mid_in[i]);
				DBG_PR_VAR_I32(dcs31_mid_in[i]);

				DBG_PR_VAR_I32(dcs20_hi_in[i]);
				DBG_PR_VAR_I32(dcs31_hi_in[i]);

				DBG_PR_VAR_I32(dcs20);
				DBG_PR_VAR_I32(dcs31);

			}
#endif

			dcs20 -= ((int64_t)dcs20_accum*(int64_t)flare_factors[is_narrow])>>8;
			dcs31 -= ((int64_t)dcs31_accum*(int64_t)flare_factors[is_narrow])>>8;

			ampl = AMPL(dcs20_hi_in[i], dcs31_hi_in[i]);

#if 0
			{
				DBG_PR_VAR_I32(ampl);

				DBG_PR_VAR_I32(dcs20);
				DBG_PR_VAR_I32(dcs31);

			}
#endif

			if(ampl<4)
			{
				dist = DIST_UNDEREXP;

			}
			else
			{
				int pixgroup;
				if(!is_narrow)
				{
					if(!(i&1)) // even
						pixgroup = shadow_luts.hif.wid_lut_group_ids[i/2] & 0x0f;
					else
						pixgroup = shadow_luts.hif.wid_lut_group_ids[i/2]>>4;

					dist = lookup_dist(0, pixgroup, dcs31, dcs20);
					if(dist < 2) dist = 2;

				}
				else // is_narrow
				{
					if(!(i&1)) // even
						pixgroup = shadow_luts.hif.nar_lut_group_ids[i/2] & 0x0f;
					else
						pixgroup = shadow_luts.hif.nar_lut_group_ids[i/2]>>4;

					dist = lookup_dist(1, pixgroup, dcs31, dcs20);
					if(dist < 2) dist = 2;

				}

#if 0
				{
					DBG_PR_VAR_I32(dist);
				}
#endif

				if(dealias_dist != NULL)
				{
					int lfdist_mm = dealias_dist[i]<<7;
					int hf_wrap0 = dist;
					int hf_wrap1 = dist + wrap_mm;
					int hf_wrap2 = dist + 2*wrap_mm;

					int err0 = abso(hf_wrap0 - lfdist_mm);
					int err1 = abso(hf_wrap1 - lfdist_mm);
					int err2 = abso(hf_wrap2 - lfdist_mm);


					if(dealias_dist[i] == DIST_UNDEREXP)
						dist = DIST_UNDEREXP;
					else if(dealias_dist[i] == DIST_OVEREXP || err0 < DEALIAS_THRESHOLD)
						dist = dist>>DIST_SHIFT; // Keep hf as is
					else if(err1 < DEALIAS_THRESHOLD)
						dist = hf_wrap1>>DIST_SHIFT;
					else if(err2 < DEALIAS_THRESHOLD)
						dist = hf_wrap2>>DIST_SHIFT;
					else
						dist = DIST_UNDEREXP;

				}
				else
					dist>>=DIST_SHIFT;

				if(dist < 0) dist = 0;
				else if(dist > MAX_DISTVAL) dist = DIST_UNDEREXP;


			}

			ampl/=16;
			if(ampl>15) ampl=15;

		}
		ampldist_out[i] = (ampl<<12) | dist;
	}
}


int chafind_enabled = 0;

void tof_enable_chafind_datapoints()
{
	chafind_enabled = 1;
}

void tof_disable_chafind_datapoints()
{
	chafind_enabled = 0;
}

#ifdef REV2A

	#define NOZZLE_WIDTH 760
	#define OBST_MARGIN (50)
	#define OBST_AVOID_WIDTH (600+OBST_MARGIN)

	void tof_to_obstacle_avoidance(uint16_t* ampldist, int sidx) __attribute__((section(".text_itcm")));
	void tof_to_obstacle_avoidance(uint16_t* ampldist, int sidx)
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

				dists[0] = (ampldist[(py+0)*TOF_XS+(px+0)]&DIST_MASK)<<DIST_SHIFT;
				dists[1] = (ampldist[(py-1)*TOF_XS+(px+0)]&DIST_MASK)<<DIST_SHIFT;
				dists[2] = (ampldist[(py+1)*TOF_XS+(px+0)]&DIST_MASK)<<DIST_SHIFT;
				dists[3] = (ampldist[(py+0)*TOF_XS+(px+1)]&DIST_MASK)<<DIST_SHIFT;
				dists[4] = (ampldist[(py+0)*TOF_XS+(px-1)]&DIST_MASK)<<DIST_SHIFT;

				int32_t avg = (dists[0]+dists[1]+dists[2]+dists[3]+dists[4])/5;

				int n_conform = 0;
				int32_t conform_avg = 0;
				for(int i=0; i<5; i++)
				{
					if(dists[i] != DIST_UNDEREXP && dists[i] > avg-120 && dists[i] < avg+120)
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
							if(local_x >= 100 && local_x < 300+OBST_MARGIN)
							{
								obstacle_front_near++;
							/*	o_itoa32(local_x, printbuf); uart_print_string_blocking(printbuf); 
								uart_print_string_blocking("  ");
								o_itoa32(local_y, printbuf); uart_print_string_blocking(printbuf); 
								uart_print_string_blocking("  ");
								o_itoa32(local_z, printbuf); uart_print_string_blocking(printbuf); 
								uart_print_string_blocking("   FN\r\n");*/
							}
							if(local_x >= 300+OBST_MARGIN && local_x < 450+OBST_MARGIN)
							{
								obstacle_front_far++;
							/*	o_itoa32(local_x, printbuf); uart_print_string_blocking(printbuf); 
								uart_print_string_blocking("  ");
								o_itoa32(local_y, printbuf); uart_print_string_blocking(printbuf); 
								uart_print_string_blocking("  ");
								o_itoa32(local_z, printbuf); uart_print_string_blocking(printbuf); 
								uart_print_string_blocking("   FF\r\n");*/
							}

							if(local_x <= -450 && local_x > -650-OBST_MARGIN)
							{
								obstacle_back_near++;

							}
							if(local_x <= -650-OBST_MARGIN && local_x > -800-OBST_MARGIN)
							{
								obstacle_back_far++;

							}
						}

						if(local_x > -490 && local_x < -200)
						{
							if(local_y >= 200+OBST_MARGIN && local_y < 350+OBST_MARGIN)
							{
								obstacle_left_near++;
							}

							if(local_y <= -200 && local_y > -350-OBST_MARGIN)
							{
								obstacle_right_near++;
							}
						}


						if(local_x > -440 && local_x < -200)
						{

							if(local_y <= -350-OBST_MARGIN && local_y > -450-OBST_MARGIN)
							{
								obstacle_right_far++;
							}

							if(local_y >= 350+OBST_MARGIN && local_y < 450+OBST_MARGIN)
							{
								obstacle_left_far++;
							}

						}
					}
				}
			}
		}

		#ifdef DBGPRVOX_AVOIDANCE
			DBG_PR_VAR_I32(insertion_cnt);
		#endif
	}
#else // individually calibrated sensors:

	#define OBST_MARGIN (0)
	#define OBST_AVOID_WIDTH (480+OBST_MARGIN)

	void tof_to_obstacle_avoidance(uint16_t* ampldist, int sidx) __attribute__((section(".text_itcm")));
	void tof_to_obstacle_avoidance(uint16_t* ampldist, int sidx)
	{
		if(sidx < 0 || sidx >= N_SENSORS) error(150);

//		uart_print_string_blocking("\r\n");

		//if(sidx != 0) return;

		uint16_t local_sensor_hor_ang = tof_calibs[sidx]->mount.ang_rel_robot;
		uint16_t local_sensor_ver_ang = tof_calibs[sidx]->mount.vert_ang_rel_ground;


		int32_t  local_sensor_x = tof_calibs[sidx]->mount.x_rel_robot;
		int32_t  local_sensor_y = tof_calibs[sidx]->mount.y_rel_robot;
		int32_t  local_sensor_z = tof_calibs[sidx]->mount.z_rel_ground;


		extern int obstacle_front_near, obstacle_back_near, obstacle_left_near, obstacle_right_near;
		extern int obstacle_front_far, obstacle_back_far, obstacle_left_far, obstacle_right_far;

		for(int py=2; py<TOF_YS-2; py+=2)
		{
			for(int px=2; px<TOF_XS-2; px+=2)
			{
				int32_t dists[5];

				dists[0] = (ampldist[(py+0)*TOF_XS+(px+0)]&DIST_MASK)<<DIST_SHIFT;
				dists[1] = (ampldist[(py-1)*TOF_XS+(px+0)]&DIST_MASK)<<DIST_SHIFT;
				dists[2] = (ampldist[(py+1)*TOF_XS+(px+0)]&DIST_MASK)<<DIST_SHIFT;
				dists[3] = (ampldist[(py+0)*TOF_XS+(px+1)]&DIST_MASK)<<DIST_SHIFT;
				dists[4] = (ampldist[(py+0)*TOF_XS+(px-1)]&DIST_MASK)<<DIST_SHIFT;

				int32_t avg = (dists[0]+dists[1]+dists[2]+dists[3]+dists[4])/5;

				int n_conform = 0;
				int32_t conform_avg = 0;
				for(int i=0; i<5; i++)
				{
					if(dists[i] != DIST_UNDEREXP && dists[i] > avg-120 && dists[i] < avg+120)
					{
						n_conform++;
						conform_avg += dists[i];
					}
				}

				if(n_conform >= 4)
				{
					int32_t d = conform_avg / n_conform;

					d+=70; // fuck

					uint16_t hor_ang, ver_ang;

					hor_ang = -tof_calibs[sidx]->hor_angs[(py/2)*(TOF_XS/2)+(px/2)];
					ver_ang = tof_calibs[sidx]->ver_angs[(py/2)*(TOF_XS/2)+(px/2)];

					#ifdef DBGPRVOX_AVOIDANCE

						if(py*TOF_XS+px == PIX)
							DBG_PR_VAR_I32(d);

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


					#ifdef EXT_VACUUM
						// VACUUM APP: Ignore the nozzle
						if(local_z < 200 && local_x < 520 && local_x > 120 && local_y > -(NOZZLE_WIDTH/2) && local_y < (NOZZLE_WIDTH/2))
							continue;
					#endif

					if(local_z > 120 && local_z < 1200)
					{
						if(local_y > -(OBST_AVOID_WIDTH/2) && local_y < (OBST_AVOID_WIDTH/2))
						{
							if(local_x >= 100 && local_x < 300+OBST_MARGIN)
							{
								obstacle_front_near++;
								led_status(sidx, RED, LED_MODE_FADE);
								//DBG_PR_VAR_U16(sidx);
/*								o_itoa32(local_x, printbuf); uart_print_string_blocking(printbuf); 
								uart_print_string_blocking("  ");
								o_itoa32(local_y, printbuf); uart_print_string_blocking(printbuf); 
								uart_print_string_blocking("  ");
								o_itoa32(local_z, printbuf); uart_print_string_blocking(printbuf); 
								uart_print_string_blocking("   FN\r\n");*/
							}
							if(local_x >= 300+OBST_MARGIN && local_x < 500+OBST_MARGIN)
							{
								obstacle_front_far++;
/*
								o_itoa32(local_x, printbuf); uart_print_string_blocking(printbuf); 
								uart_print_string_blocking("  ");
								o_itoa32(local_y, printbuf); uart_print_string_blocking(printbuf); 
								uart_print_string_blocking("  ");
								o_itoa32(local_z, printbuf); uart_print_string_blocking(printbuf); 
								uart_print_string_blocking("   FF\r\n");*/
							}

							if(local_x <= -450 && local_x > -700-OBST_MARGIN)
							{
								//DBG_PR_VAR_U16(sidx);
								led_status(sidx, RED, LED_MODE_FADE);
								obstacle_back_near++;

							}
							if(local_x <= -700-OBST_MARGIN && local_x > -900-OBST_MARGIN)
							{
								obstacle_back_far++;

							}
						}

						if(local_x > -490 && local_x < -200)
						{
							if(local_y >= 200+OBST_MARGIN && local_y < 350+OBST_MARGIN)
							{
								//DBG_PR_VAR_U16(sidx);
								led_status(sidx, RED, LED_MODE_FADE);

								obstacle_left_near++;
							}

							if(local_y <= -200 && local_y > -350-OBST_MARGIN)
							{
								//DBG_PR_VAR_U16(sidx);
								led_status(sidx, RED, LED_MODE_FADE);

								obstacle_right_near++;
							}
						}


						if(local_x > -440 && local_x < -200)
						{

							if(local_y <= -350-OBST_MARGIN && local_y > -450-OBST_MARGIN)
							{
								obstacle_right_far++;
							}

							if(local_y >= 350+OBST_MARGIN && local_y < 450+OBST_MARGIN)
							{
								obstacle_left_far++;
							}

						}
					}
				}
			}
		}

		#ifdef DBGPRVOX_AVOIDANCE
			DBG_PR_VAR_I32(insertion_cnt);
		#endif
	}
#endif

/*
	2DCS -> amplitude&distance conversion for 6.67MHz modulation frequency.
	Does ambient correction, and pixel-by-pixel correction. There is no curve shape (nonlinearity) correction.
	Output is 8-bit distance, step=128mm
	0=invalid (low ampl)
	1=overexp
	2=256mm
	...
	255=32640mm
*/

void compensated_2dcs_6mhz_dist_masked(uint8_t *dist_out, epc_2dcs_t *in, epc_img_t *bwimg) __attribute__((section(".text_itcm")));
void compensated_2dcs_6mhz_dist_masked(uint8_t *dist_out, epc_2dcs_t *in, epc_img_t *bwimg)
{
	int32_t base_offs = shadow_luts.lof.wid_base_offset_2dcs;
	for(int i=0; i < TOF_XS*TOF_YS; i++)
	{
		int dist;
		int ampl;

		int16_t dcs0 = ((in->dcs[0].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs1 = ((in->dcs[1].img[i]&0b0011111111111100)>>2)-2048;

		if(dcs0 < -2047 || dcs0 > 2046 || dcs1 < -2047 || dcs1 > 2046)
		{
			ampl = 255;
			dist = DIST_OVEREXP;
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
				dist = DIST_UNDEREXP;
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
					dist = DIST_UNDEREXP;
				else
				{
					dist = dist_i>>7;
					if(dist < 2) dist = 2;
				}
			}

		}
		if(dist < 0 || dist > 255) error(781);
		dist_out[i] = dist;
	}
}
void compensated_2dcs_6mhz_dist_masked_narrow(uint8_t *dist_out, epc_2dcs_narrow_t *in, epc_img_t *bwimg) __attribute__((section(".text_itcm")));
void compensated_2dcs_6mhz_dist_masked_narrow(uint8_t *dist_out, epc_2dcs_narrow_t *in, epc_img_t *bwimg)
{
	int32_t base_offs = shadow_luts.lof.nar_base_offset_2dcs;
	for(int yy=0; yy < TOF_YS_NARROW; yy++)
	{
		for(int xx=0; xx < TOF_XS_NARROW; xx++)
		{
			int i = yy*TOF_XS_NARROW+xx;
			int dist;
			int ampl;

			int16_t dcs0 = ((in->dcs[0].img[i]&0b0011111111111100)>>2)-2048;
			int16_t dcs1 = ((in->dcs[1].img[i]&0b0011111111111100)>>2)-2048;

			if(dcs0 < -2047 || dcs0 > 2046 || dcs1 < -2047 || dcs1 > 2046)
			{
				ampl = 255;
				dist = DIST_OVEREXP;
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
					dist = DIST_UNDEREXP;
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
						dist = DIST_UNDEREXP;
					else
					{
						dist = dist_i>>7;
						if(dist < 2) dist = 2;
					}
				}

			}
			if(dist < 0 || dist > 255) error(782);
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

