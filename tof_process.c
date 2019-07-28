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
void ITCM copy_cal_to_shadow(int sid, int f)
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
	Copy everything except the amb_corr table, which is the last element.
	Then, reproduce the amb table so the data can be indexed in order.
*/
void ITCM copy_cal_to_shadow_narrow(int sid, int f)
{
	if(f<0 || f>2) error(123);
	if(f<2)
	{
		memcpy(&shadow_luts.hif, &tof_calibs[sid]->hif[f], sizeof(shadow_luts.hif) - sizeof(shadow_luts.hif.amb_corr));
		int i=0;
		for(int yy=TOF_NARROW_Y_START; yy<TOF_NARROW_Y_START+TOF_YS_NARROW; yy++)
			for(int xx=TOF_NARROW_X_START;xx<TOF_NARROW_X_START+TOF_XS_NARROW; xx+=2) // xx+=2, because data is 4 bits, two records per []
				shadow_luts.hif.amb_corr[i] = tof_calibs[sid]->hif[f].amb_corr[(yy*TOF_XS+xx)/2];
	}
	else
	{
		memcpy(&shadow_luts.lof, &tof_calibs[sid]->lof[f-2], sizeof(shadow_luts.lof) - sizeof(shadow_luts.hif.amb_corr));
		int i=0;
		for(int yy=TOF_NARROW_Y_START; yy<TOF_NARROW_Y_START+TOF_YS_NARROW; yy++)
			for(int xx=TOF_NARROW_X_START;xx<TOF_NARROW_X_START+TOF_XS_NARROW; xx+=2) // xx+=2, because data is 4 bits, two records per []
				shadow_luts.lof.amb_corr[i] = tof_calibs[sid]->hif[f-2].amb_corr[(yy*TOF_XS+xx)/2];
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
	// Calculate amplitude (sum vector length of the orthogonal components) as is
	// Fast version gets it close enough.
	// For example:
	// sin(pi/4) + cos(pi/4) = 1.414
	// sin(0) + cos(0) = 1.0
	// divide the sum by 8/6 = 1.333
	#define AMPL_SAME_UNITS(dcs20_, dcs31_)    ((6*((abso((dcs20_))+abso((dcs31_)))))>>3)
	// Outputs 0..255 from -4095..+4096 DCS values:
	#define AMPL(dcs20_, dcs31_)    ((abso((dcs20_))+abso((dcs31_)))/(23))
	// Outputs 0..255 from HDR_FACTOR*-4095..+4096 DCS values:
	#define AMPLHDR(dcs20_, dcs31_) ((abso((dcs20_))+abso((dcs31_)))/(23*HDR_FACTOR))
#else
	// Calculate amplitude (sum vector length of the orthogonal components) as is
	#define AMPL_SAME_UNITS(dcs20_, dcs31_)    (sqrt(sq((dcs20_))+sq((dcs31_))))
	// Outputs 0..255 from -4095..+4096 DCS values:
	#define AMPL(dcs20_, dcs31_)    (sqrt(sq((dcs20_))+sq((dcs31_)))/(17))
	// Outputs 0..255 from HDR_FACTOR*-4095..+4096 DCS values:
	#define AMPLHDR(dcs20_, dcs31_) (sqrt(sq((dcs20_))+sq((dcs31_)))/(17*HDR_FACTOR))
#endif			


#define DO_AMB_CORR

//#define DBGPR

//#define DBGPRVOX
#define PIX (30*160+80)
//#define PIX (15*160+60)
#define DBGPRSIDX 5
#define IN_01_DEG (ANG_0_1_DEG/65536)


void ITCM conv_4dcs_to_2dcs_wide(int16_t* restrict dcs20_out, int16_t* restrict dcs31_out, epc_4dcs_t* restrict in, uint16_t* restrict bwimg)
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
			int16_t bw = ((bwimg[i]&0b0011111111111100)>>2)-2048;
			if(bw<0) bw=0;
			int corr_factor;
			if(!(i&1)) // even
				corr_factor = shadow_luts.hif.amb_corr[i/2] & 0x0f;
			else
				corr_factor = shadow_luts.hif.amb_corr[i/2]>>4;
			int16_t comp = (bw*corr_factor)>>4;

			dcs31 += comp;
			dcs20 += comp;
		#endif


		dcs20_out[i] = dcs20;
		dcs31_out[i] = dcs31;
	}
}

void ITCM conv_4dcs_to_2dcs_hdr0_wide(int16_t* restrict dcs20_out, int16_t* restrict dcs31_out, epc_4dcs_t* restrict in, uint16_t* restrict bwimg)
{
	for(int i=0; i < TOF_XS*TOF_YS; i++)
	{
		int16_t dcs0 = ((in->dcs[0].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs1 = ((in->dcs[1].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs2 = ((in->dcs[2].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs3 = ((in->dcs[3].img[i]&0b0011111111111100)>>2)-2048;

		if(dcs0 >= 2047 || dcs0 <= -2047 || dcs1 >= 2047 || dcs1 <= -2047 ||
		   dcs2 >= 2047 || dcs2 <= -2047 || dcs3 >= 2047 || dcs3 <= -2047)
		{
			dcs20_out[i] = (dcs2>0)?INT16_MAX:INT16_MIN;
			dcs31_out[i] = (dcs3>0)?INT16_MAX:INT16_MIN;
		}
		else
		{
			int16_t dcs31 = dcs3-dcs1;
			int16_t dcs20 = dcs2-dcs0;

			if(bwimg)
			{
				int16_t bw = ((bwimg[i]&0b0011111111111100)>>2)-2048;
				if(bw<0) bw=0;
				int corr_factor;
				if(!(i&1)) // even
					corr_factor = shadow_luts.hif.amb_corr[i/2] & 0x0f;
				else
					corr_factor = shadow_luts.hif.amb_corr[i/2]>>4;
				int16_t comp = (bw*corr_factor)>>4;

				dcs31 += comp;
				dcs20 += comp;
			}

			/*
				If the latter hdr1 is overexposed (and thus, this value here is kept),
				it usually follows that the amplitude of hdr0 should be high enough (unless HDR_FACTOR is set way
				too high). In rare corner cases, there is a risk that the amplitude for hdr1 is too much,
				but amplitude for hdr0 is still too little. Because we multiply by HDR_FACTOR here,
				it's impossible to tell later what the original amplitude was (to reject values near sensor noise floor).
				Hence, we do the amplitude check (for sensor noise strictly) here.

				Amplitude checks at later point cater for optical issues as well.
			*/
			int ampl = AMPL(dcs20, dcs31);
			if(ampl < 3)
			{
				dcs20_out[i] = 0;
				dcs31_out[i] = 0;
			}
			else
			{
				dcs20_out[i] = dcs20 * HDR_FACTOR;
				dcs31_out[i] = dcs31 * HDR_FACTOR;
			}
		}
	}
}

void ITCM conv_4dcs_to_2dcs_hdr1_wide(int16_t* restrict dcs20_out, int16_t* restrict dcs31_out, epc_4dcs_t* restrict in, uint16_t* restrict bwimg)
{
	for(int i=0; i < TOF_XS*TOF_YS; i++)
	{
		int16_t dcs0 = ((in->dcs[0].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs1 = ((in->dcs[1].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs2 = ((in->dcs[2].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs3 = ((in->dcs[3].img[i]&0b0011111111111100)>>2)-2048;

		if(dcs0 >= 2047 || dcs0 <= -2047 || dcs1 >= 2047 || dcs1 <= -2047 ||
		   dcs2 >= 2047 || dcs2 <= -2047 || dcs3 >= 2047 || dcs3 <= -2047)
		{
			// Don't do anything, keep the old (short-exp) value
		}
		else
		{
			int16_t dcs31 = dcs3-dcs1;
			int16_t dcs20 = dcs2-dcs0;

			if(bwimg)
			{
				int16_t bw = ((bwimg[i]&0b0011111111111100)>>2)-2048;
				if(bw<0) bw=0;
				int corr_factor;
				if(!(i&1)) // even
					corr_factor = shadow_luts.hif.amb_corr[i/2] & 0x0f;
				else
					corr_factor = shadow_luts.hif.amb_corr[i/2]>>4;
				int16_t comp = (bw*corr_factor)>>4;

				dcs31 += comp;
				dcs20 += comp;
			}

			dcs20_out[i] = dcs20;
			dcs31_out[i] = dcs31;
		}
	}
}



void ITCM conv_4dcs_to_2dcs_narrow(int16_t* restrict dcs20_out, int16_t* restrict dcs31_out, epc_4dcs_narrow_t* restrict in, uint16_t* restrict bwimg)
{
	for(int i=0; i < TOF_XS_NARROW*TOF_YS_NARROW; i++)
	{
		int16_t dcs0 = ((in->dcs[0].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs1 = ((in->dcs[1].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs2 = ((in->dcs[2].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs3 = ((in->dcs[3].img[i]&0b0011111111111100)>>2)-2048;

		int16_t dcs31 = dcs3-dcs1;
		int16_t dcs20 = dcs2-dcs0;

		// The shadow lut amb_corr has been rebuilt to run linearly at the start of the array
		#ifdef DO_AMB_CORR
			int16_t bw = ((bwimg[i]&0b0011111111111100)>>2)-2048;
			if(bw<0) bw=0;
			int corr_factor;
			if(!(i&1)) // even
				corr_factor = shadow_luts.hif.amb_corr[i/2] & 0x0f;
			else
				corr_factor = shadow_luts.hif.amb_corr[i/2]>>4;
			int16_t comp = (bw*corr_factor)>>4;

			dcs31 += comp;
			dcs20 += comp;
		#endif


		dcs20_out[i] = dcs20;
		dcs31_out[i] = dcs31;
	}
}

void ITCM conv_4dcs_to_2dcs_hdr0_narrow(int16_t* restrict dcs20_out, int16_t* restrict dcs31_out, epc_4dcs_narrow_t* restrict in, uint16_t* restrict bwimg)
{
	for(int i=0; i < TOF_XS_NARROW*TOF_YS_NARROW; i++)
	{
		int16_t dcs0 = ((in->dcs[0].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs1 = ((in->dcs[1].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs2 = ((in->dcs[2].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs3 = ((in->dcs[3].img[i]&0b0011111111111100)>>2)-2048;

		if(dcs0 >= 2047 || dcs0 <= -2047 || dcs1 >= 2047 || dcs1 <= -2047 ||
		   dcs2 >= 2047 || dcs2 <= -2047 || dcs3 >= 2047 || dcs3 <= -2047)
		{
			dcs20_out[i] = (dcs2>0)?INT16_MAX:INT16_MIN;
			dcs31_out[i] = (dcs3>0)?INT16_MAX:INT16_MIN;
		}
		else
		{
			int16_t dcs31 = dcs3-dcs1;
			int16_t dcs20 = dcs2-dcs0;

			// The shadow lut amb_corr has been rebuilt to run linearly at the start of the array
			#ifdef DO_AMB_CORR
				int16_t bw = ((bwimg[i]&0b0011111111111100)>>2)-2048;
				if(bw<0) bw=0;
				int corr_factor;
				if(!(i&1)) // even
					corr_factor = shadow_luts.hif.amb_corr[i/2] & 0x0f;
				else
					corr_factor = shadow_luts.hif.amb_corr[i/2]>>4;
				int16_t comp = (bw*corr_factor)>>4;

				dcs31 += comp;
				dcs20 += comp;
			#endif

			/*
				If the latter hdr1 is overexposed (and thus, this value here is kept),
				it usually follows that the amplitude of hdr0 should be high enough (unless HDR_FACTOR is set way
				too high). In rare corner cases, there is a risk that the amplitude for hdr1 is too much,
				but amplitude for hdr0 is still too little. Because we multiply by HDR_FACTOR here,
				it's impossible to tell later what the original amplitude was (to reject values near sensor noise floor).
				Hence, we do the amplitude check (for sensor noise strictly) here.

				Amplitude checks at later point cater for optical issues as well.
			*/
			int ampl = AMPL(dcs20, dcs31);
			if(ampl < 3)
			{
				dcs20_out[i] = 0;
				dcs31_out[i] = 0;
			}
			else
			{
				dcs20_out[i] = dcs20 * HDR_FACTOR;
				dcs31_out[i] = dcs31 * HDR_FACTOR;
			}
		}
	}
}

void ITCM conv_4dcs_to_2dcs_hdr1_narrow(int16_t* restrict dcs20_out, int16_t* restrict dcs31_out, epc_4dcs_narrow_t* restrict in, uint16_t* restrict bwimg)
{
	for(int i=0; i < TOF_XS_NARROW*TOF_YS_NARROW; i++)
	{
		int16_t dcs0 = ((in->dcs[0].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs1 = ((in->dcs[1].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs2 = ((in->dcs[2].img[i]&0b0011111111111100)>>2)-2048;
		int16_t dcs3 = ((in->dcs[3].img[i]&0b0011111111111100)>>2)-2048;

		if(dcs0 >= 2047 || dcs0 <= -2047 || dcs1 >= 2047 || dcs1 <= -2047 ||
		   dcs2 >= 2047 || dcs2 <= -2047 || dcs3 >= 2047 || dcs3 <= -2047)
		{
			// Don't do anything, keep the old (short-exp) value
		}
		else
		{
			int16_t dcs31 = dcs3-dcs1;
			int16_t dcs20 = dcs2-dcs0;

			// The shadow lut amb_corr has been rebuilt to run linearly at the start of the array
			#ifdef DO_AMB_CORR
				int16_t bw = ((bwimg[i]&0b0011111111111100)>>2)-2048;
				if(bw<0) bw=0;
				int corr_factor;
				if(!(i&1)) // even
					corr_factor = shadow_luts.hif.amb_corr[i/2] & 0x0f;
				else
					corr_factor = shadow_luts.hif.amb_corr[i/2]>>4;
				int16_t comp = (bw*corr_factor)>>4;

				dcs31 += comp;
				dcs20 += comp;
			#endif

			dcs20_out[i] = dcs20;
			dcs31_out[i] = dcs31;
		}
	}
}

int calc_avg_ampl_x256(int16_t* restrict dcs20_in, int16_t* restrict dcs31_in)
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

int calc_avg_ampl_x256_narrow(int16_t* restrict dcs20_in, int16_t* restrict dcs31_in)
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

int ITCM calc_avg_ampl_x256_nar_region_on_wide(int16_t* restrict dcs20_in, int16_t* restrict dcs31_in)
{
	#if 0
	// Use a slightly smaller area actually.
	uint32_t accum = 0;
	for(int yy=2; yy < TOF_YS_NARROW-2; yy++)
	{
		for(int xx=4; xx < TOF_XS_NARROW-4; xx++)
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
	uint32_t avg = (256*accum)/((TOF_XS_NARROW-8)*(TOF_YS_NARROW-4));
	return avg;

	#endif

	// Use a filtered maximum instead
	int32_t max_ampl = INT32_MIN;
	for(int yy=TOF_NARROW_Y_START+4; yy<TOF_NARROW_Y_START+TOF_YS_NARROW-5; yy+=3)
	{
		for(int xx=TOF_NARROW_X_START+6; xx<TOF_NARROW_X_START+TOF_XS_NARROW-6; xx+=2)
		{
			int32_t ampl = 0;
			for(int iy=-2; iy<=+2; iy++)
			{
				for(int ix=-1; ix<=+1; ix++)
				{
					int idx = TC(xx+ix,yy+iy);
					ampl += AMPL(dcs31_in[idx], dcs20_in[idx]);
				}
			}

			ampl /= 5*3;

			if(ampl > max_ampl)
			{
				max_ampl = ampl;
			}
		}
	}

	return max_ampl;
}

#define BLUR_R 6
#define BLUR_DIV (2*BLUR_R+1)
static void ITCM boxblur_h(int16_t* restrict in, int16_t* restrict out)
{
	for(int i=0; i<TOF_YS; i++)
	{
		int ti = i*TOF_XS;
		int li = ti;
		int ri = ti+BLUR_R;

		int fv = 0;
		int lv = 0;
		int val = (BLUR_R+1)*fv;
		for(int j=0; j<BLUR_R; j++) val += in[ti+j];
		for(int j=0  ; j<=BLUR_R ; j++) { val += in[ri++] - fv      ;   out[ti++] = val/BLUR_DIV; }
		for(int j=BLUR_R+1; j<TOF_XS-BLUR_R; j++) { val += in[ri++] - in[li++];   out[ti++] = val/BLUR_DIV; }
		for(int j=TOF_XS-BLUR_R; j<TOF_XS  ; j++) { val += lv       - in[li++];   out[ti++] = val/BLUR_DIV; }
	}
}

static void ITCM boxblur_h_narrow(int16_t* restrict in, int16_t* restrict out)
{
	for(int i=0; i<TOF_YS_NARROW; i++)
	{
		int ti = i*TOF_XS_NARROW;
		int li = ti;
		int ri = ti+BLUR_R;

		int fv = 0;
		int lv = 0;
		int val = (BLUR_R+1)*fv;
		for(int j=0; j<BLUR_R; j++) val += in[ti+j];
		for(int j=0  ; j<=BLUR_R ; j++) { val += in[ri++] - fv      ;   out[ti++] = val/BLUR_DIV; }
		for(int j=BLUR_R+1; j<TOF_XS_NARROW-BLUR_R; j++) { val += in[ri++] - in[li++];   out[ti++] = val/BLUR_DIV; }
		for(int j=TOF_XS_NARROW-BLUR_R; j<TOF_XS_NARROW  ; j++) { val += lv       - in[li++];   out[ti++] = val/BLUR_DIV; }
	}
}

static void ITCM boxblur_t(int16_t* restrict in, int16_t* restrict out)
{
	for(int i=0; i<TOF_XS; i++)
	{
		int ti = i;
		int li = ti;
		int ri = ti+BLUR_R*TOF_XS;

		int fv = 0;
		int lv = 0;
		int val = (BLUR_R+1)*fv;
		for(int j=0; j<BLUR_R; j++) val += in[ti+j*TOF_XS];
		for(int j=0  ; j<=BLUR_R ; j++) { val += in[ri] - fv     ;  out[ti] = val/BLUR_DIV;  ri+=TOF_XS; ti+=TOF_XS; }
		for(int j=BLUR_R+1; j<TOF_YS-BLUR_R; j++) { val += in[ri] - in[li];  out[ti] = val/BLUR_DIV;  li+=TOF_XS; ri+=TOF_XS; ti+=TOF_XS; }
		for(int j=TOF_YS-BLUR_R; j<TOF_YS  ; j++) { val += lv      - in[li];  out[ti] = val/BLUR_DIV;  li+=TOF_XS; ti+=TOF_XS; }
	}
}

static void ITCM boxblur_t_narrow(int16_t* restrict in, int16_t* restrict out)
{
	for(int i=0; i<TOF_XS_NARROW; i++)
	{
		int ti = i;
		int li = ti;
		int ri = ti+BLUR_R*TOF_XS_NARROW;

		int fv = 0;
		int lv = 0;
		int val = (BLUR_R+1)*fv;
		for(int j=0; j<BLUR_R; j++) val += in[ti+j*TOF_XS_NARROW];
		for(int j=0  ; j<=BLUR_R ; j++) { val += in[ri] - fv     ;  out[ti] = val/BLUR_DIV;  ri+=TOF_XS_NARROW; ti+=TOF_XS_NARROW; }
		for(int j=BLUR_R+1; j<TOF_YS_NARROW-BLUR_R; j++) { val += in[ri] - in[li];  out[ti] = val/BLUR_DIV;  li+=TOF_XS_NARROW; ri+=TOF_XS_NARROW; ti+=TOF_XS_NARROW; }
		for(int j=TOF_YS_NARROW-BLUR_R; j<TOF_YS_NARROW  ; j++) { val += lv      - in[li];  out[ti] = val/BLUR_DIV;  li+=TOF_XS_NARROW; ti+=TOF_XS_NARROW; }
	}
}

static void ITCM boxblur(int16_t* restrict in, int16_t* restrict out)
{
	static int16_t tmp[TOF_XS*TOF_YS];

	boxblur_h(in, tmp);
	boxblur_t(tmp, out);
}

static void ITCM boxblur_narrow(int16_t* restrict in, int16_t* restrict out)
{
	static int16_t tmp[TOF_XS_NARROW*TOF_YS_NARROW];

	boxblur_h_narrow(in, tmp);
	boxblur_t_narrow(tmp, out);
}

#define OBPC(x_, y_) ((y_)*BLUR_PARAMS_XS+(x_))

static void ITCM blur_5_convol_biased(int16_t* restrict in, int16_t* restrict out, mcu_blur_params_t* blur_params)
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
			int bx = xx/BLUR_PARAMS_RATIO_X;			
			int by = yy/BLUR_PARAMS_RATIO_Y;
			int a = BLUR_PARAM_A(blur_params[OBPC(bx,by)]);
			int b = BLUR_PARAM_B(blur_params[OBPC(bx,by)]);
			int c = BLUR_PARAM_C(blur_params[OBPC(bx,by)]);

			int32_t val = 
				(int32_t)in[TC(xx  , yy-1)] * (int32_t)(b+c) +
				(int32_t)in[TC(xx-1, yy  )] * (int32_t)(b-c) +
				(int32_t)in[TC(xx  , yy  )] * (int32_t)a +
				(int32_t)in[TC(xx+1, yy  )] * (int32_t)(b-c) +
				(int32_t)in[TC(xx  , yy+1)] * (int32_t)(b+c);

			//val /= 256;
			val >>= 8;

			if(in[TC(xx, yy)] == INT16_MAX || in[TC(xx, yy)] == INT16_MIN)
				out[TC(xx,yy)] = in[TC(xx, yy)];
			else
				out[TC(xx, yy)] = val;
		}
	}
}

static void ITCM blur_5_convol_biased_narrow(int16_t* restrict in, int16_t* restrict out, mcu_blur_params_t* blur_params)
{
/*
	Convolution matrix:

	        b+c  
	b-c      a      b-c
	        b+c

*/
	for(int yy=TOF_NARROW_Y_START+1; yy<TOF_NARROW_Y_START+TOF_YS_NARROW-1; yy++)
	{
		for(int xx=TOF_NARROW_X_START+1; xx<TOF_NARROW_X_START+TOF_XS_NARROW-1; xx++)
		{
			int bx = xx/BLUR_PARAMS_RATIO_X;			
			int by = yy/BLUR_PARAMS_RATIO_Y;
			int a = BLUR_PARAM_A(blur_params[OBPC(bx,by)]);
			int b = BLUR_PARAM_B(blur_params[OBPC(bx,by)]);
			int c = BLUR_PARAM_C(blur_params[OBPC(bx,by)]);

			int32_t val = 
				(int32_t)in[TNC(xx  , yy-1)] * (int32_t)(b+c) +
				(int32_t)in[TNC(xx-1, yy  )] * (int32_t)(b-c) +
				(int32_t)in[TNC(xx  , yy  )] * (int32_t)a +
				(int32_t)in[TNC(xx+1, yy  )] * (int32_t)(b-c) +
				(int32_t)in[TNC(xx  , yy+1)] * (int32_t)(b+c);

			//val /= 256;
			val >>= 8;

			if(in[TNC(xx, yy)] == INT16_MAX || in[TNC(xx, yy)] == INT16_MIN)
				out[TNC(xx,yy)] = in[TNC(xx, yy)];
			else
				out[TNC(xx, yy)] = val;
		}
	}
}

#define RIC(x_, y_) ((y_)*RESICAL_IN_XS+(x_))
#define ROC(x_, y_) ((y_)*RESICAL_OUT_XS+(x_))

extern uint32_t cnt_100us;
void ITCM run_lens_model(int16_t* restrict in, int16_t* restrict out, tof_calib_t* p_tof_calib)
{
//	uint32_t time;
	static int16_t tmp[TOF_XS*TOF_YS];

//	mcu_blur_params_t* blur_params = p_tof_calib->blur_params;
	mcu_blur_params_t blur_params[BLUR_PARAMS_XS*BLUR_PARAMS_YS]; // DTCM tested SLOW! Local stack copy is good.
	memcpy(blur_params, p_tof_calib->blur_params, sizeof(blur_params));

	// BLUR MODEL:

//	time = cnt_100us;
	// Temporarily use "out" as a buffer
	// Two boxblur passes: 4.7ms --> ITCM 4.0ms
	boxblur(in, out);
	boxblur(out, tmp);

//	uart_print_string_blocking("BOXBLUR: ");
//	DBG_PR_VAR_I32(cnt_100us-time);
//	time = cnt_100us;

	// Mix input ("in") and blurred input ("tmp") together - store result to "out"
	// Mixing: 1.8ms --> ITCM 1.6ms --> div1024 to shift10 --> 1.5ms
	for(int yy=0; yy<TOF_YS; yy++)
	{
		for(int xx=0; xx<TOF_XS; xx++)
		{
			int bx = xx/BLUR_PARAMS_RATIO_X;			
			int by = yy/BLUR_PARAMS_RATIO_Y;
			int32_t d = BLUR_PARAM_D(blur_params[OBPC(bx,by)]);

			if(in[TC(xx,yy)] == INT16_MAX || in[TC(xx,yy)] == INT16_MIN) // Keep overexposed data as is
				out[TC(xx,yy)] = in[TC(xx,yy)];
			else
				out[TC(xx,yy)] = in[TC(xx,yy)] - ((d*(int32_t)tmp[TC(xx,yy)])>>10);
		}
	}

//	uart_print_string_blocking("MIX: ");
//	DBG_PR_VAR_I32(cnt_100us-time);
//	time = cnt_100us;

	// Then run convolution model: from "out" to "tmp"
	// 2.9ms --> ITCM 2.7ms --> /256 to shift --> 2.6ms
	blur_5_convol_biased(out, tmp, blur_params);

//	uart_print_string_blocking("CONVOL: ");
//	DBG_PR_VAR_I32(cnt_100us-time);

	// RESIDUAL MODEL:
	// Data in buffer: "tmp"
	// Measurement stage takes 1.9ms --> ITCM 1.3ms

//	time = cnt_100us;


	int16_t output_offsets[RESICAL_OUT_XS*RESICAL_OUT_YS] = {0}; // int32 -> same performance

	for(int riy=0; riy < RESICAL_IN_YS; riy++)
	{
		for(int rix=0; rix < RESICAL_IN_XS; rix++)
		{
			// Find input area coordinates

			int divline_x0 = p_tof_calib->resical_bounds_x[rix];
			int divline_x1 = p_tof_calib->resical_bounds_x[rix+1];
			int divline_y0 = p_tof_calib->resical_bounds_y[riy];
			int divline_y1 = p_tof_calib->resical_bounds_y[riy+1];

			// Average over the input area:

			int32_t avg_input;

			{
				int64_t avg_input_acc = 0;
				//int n = 0;

				for(int yy=divline_y0; yy<divline_y1; yy++)
				{
					for(int xx=divline_x0; xx<divline_x1; xx++)
					{
						avg_input_acc += tmp[TC(xx,yy)];
						//n++;
					}		
				}

				avg_input = avg_input_acc / ((divline_y1-divline_y0)*(divline_x1-divline_x0)); // / n;
			}

			for(int roy=0; roy < RESICAL_OUT_YS; roy++)
			{
				for(int rox=0; rox < RESICAL_OUT_XS; rox++)
				{
					#define GET_RESICAL_COEFF(cal_, ix_, iy_, ox_, oy_) ((cal_).resical_coeffs[(iy_)*RESICAL_IN_XS+(ix_)][(oy_)*RESICAL_OUT_XS+(ox_)])
					int32_t coeff = GET_RESICAL_COEFF(*p_tof_calib, rix, riy, rox, roy);

					output_offsets[ROC(rox,roy)] += (avg_input * coeff)/RESICAL_DIVIDER;
				}
			}

		}
	}

//	uart_print_string_blocking("RESIMEAS: ");

//	DBG_PR_VAR_I32(cnt_100us-time);
//	time = cnt_100us;

	// Compensate the output:
	// Compensation takes 1.0ms --> ITCM 1.0ms

	for(int roy=0; roy < RESICAL_OUT_YS; roy++)
	{
		for(int rox=0; rox < RESICAL_OUT_XS; rox++)
		{
			int32_t offset = output_offsets[ROC(rox,roy)];
			for(int suby=0; suby < RESICAL_OUT_RATIO; suby++)
			{
				for(int subx=0; subx < RESICAL_OUT_RATIO; subx++)
				{
					int tx = rox*RESICAL_OUT_RATIO + subx;
					int ty = roy*RESICAL_OUT_RATIO + suby;

					if(tmp[TC(tx,ty)] == INT16_MAX || tmp[TC(tx,ty)] == INT16_MIN)
						out[TC(tx,ty)] = tmp[TC(tx,ty)];
					else
					{
						int32_t outval = (int32_t)tmp[TC(tx,ty)] - offset;

						// Data saturated - was nearly overexposed, now is overexposed.
						// Mark as overexposed:
						if(outval < INT16_MIN) outval = INT16_MIN;
						else if(outval > INT16_MAX) outval = INT16_MAX;

						out[TC(tx,ty)] = outval;
					}
				}
			}
		}
	}
//	uart_print_string_blocking("RESICOMP: ");
//	DBG_PR_VAR_I32(cnt_100us-time);

}

void ITCM remove_narrow_edgevals(int16_t* restrict img20, int16_t* restrict img31)
{
	// The spot is oval in Y direction
	// (The exact position of the oval spot isn't always exactly at the middle)
	// Depends on sensor, but also on distance. Most reliable to find the maxima every time.
	// Average 5*3 rectangles, with a bit of overlap
	// Look for maximum of these averages
	// This way we find the brightest spot, but ignore very small things contributing little
	// to the ill effects. (for example, a bright chromed pipe reflection of a pixel or two)


	int32_t max_ampl = INT32_MIN;
	for(int yy=2; yy<TOF_YS_NARROW-2-2; yy+=2)
	{
		for(int xx=1; xx<TOF_XS_NARROW-1-2; xx+=2)
		{
			int32_t ampl = 0;
			for(int iy=-2; iy<=+2; iy++)
			{
				for(int ix=-1; ix<=+1; ix++)
				{
					int idx = (yy+iy)*TOF_XS_NARROW + (xx+ix);
					ampl += AMPL_SAME_UNITS(img31[idx], img20[idx]);
				}
			}

			ampl /= 5*3;

			if(ampl > max_ampl)
			{
				max_ampl = ampl;
			}
		}
	}


	// Now remove everything below certain percentage of that max val.

	for(int i=0; i< TOF_XS_NARROW*TOF_YS_NARROW; i++)
	{
		int32_t ampl = AMPL_SAME_UNITS(img31[i], img20[i]);

		if(6*ampl < max_ampl)
		{
			img20[i] = 0;
			img31[i] = 0;
		}
	}

	
}

void run_lens_model_narrow(int16_t* restrict in, int16_t* restrict out, tof_calib_t* p_tof_calib)
{
	static int16_t tmp[TOF_XS_NARROW*TOF_YS_NARROW];

//	mcu_blur_params_t* blur_params = p_tof_calib->blur_params;
	mcu_blur_params_t blur_params[BLUR_PARAMS_XS*BLUR_PARAMS_YS]; // DTCM tested SLOW! Local stack copy is good.
	memcpy(blur_params, p_tof_calib->blur_params, sizeof(blur_params));

	// BLUR MODEL:

	// Temporarily use "out" as a buffer
	boxblur_narrow(in, out);
	boxblur_narrow(out, tmp);

	// Mix input ("in") and blurred input ("tmp") together - store result to "out"
	int i = 0;
	for(int yy=TOF_NARROW_Y_START; yy<TOF_NARROW_Y_START+TOF_YS_NARROW; yy++)
	{
		for(int xx=TOF_NARROW_X_START; xx<TOF_NARROW_X_START+TOF_XS_NARROW; xx++)
		{
			int bx = xx/BLUR_PARAMS_RATIO_X;			
			int by = yy/BLUR_PARAMS_RATIO_Y;
			int32_t d = BLUR_PARAM_D(blur_params[OBPC(bx,by)]);

			if(in[i] == INT16_MAX || in[i] == INT16_MIN) // Keep overexposed data as is
				out[i] = in[i];
			else
				out[i] = in[i] - (d*(int32_t)tmp[i])/1024;

			i++;
		}
	}

	// Then run convolution model: from "out" to "tmp"
	blur_5_convol_biased_narrow(out, tmp, blur_params);

	// RESIDUAL MODEL:
	// Data in buffer: "tmp"

	int16_t output_offsets[RESICAL_OUT_XS*RESICAL_OUT_YS] = {0};

	for(int riy=0; riy < RESICAL_IN_YS; riy++)
	{
		for(int rix=0; rix < RESICAL_IN_XS; rix++)
		{
			// Find input area coordinates

			int divline_x0 = p_tof_calib->resical_bounds_x[rix];
			int divline_x1 = p_tof_calib->resical_bounds_x[rix+1];
			int divline_y0 = p_tof_calib->resical_bounds_y[riy];
			int divline_y1 = p_tof_calib->resical_bounds_y[riy+1];

			// Average over the input area:

			int32_t avg_input;

			{
				int64_t avg_input_acc = 0;
				int n = 0;

				for(int yy=divline_y0; yy<divline_y1; yy++)
				{
					for(int xx=divline_x0; xx<divline_x1; xx++)
					{
						if(yy >= TOF_NARROW_Y_START && yy < TOF_NARROW_Y_START+TOF_YS_NARROW &&
						   xx >= TOF_NARROW_X_START && xx < TOF_NARROW_X_START+TOF_XS_NARROW)
						{
							int narx = xx - TOF_NARROW_X_START;
							int nary = yy - TOF_NARROW_Y_START;
							avg_input_acc += tmp[nary*TOF_XS_NARROW+narx];
							n++;
						}
					}		
				}

				avg_input = (double)avg_input_acc / (double)n;
			}

			for(int roy=0; roy < RESICAL_OUT_YS; roy++)
			{
				for(int rox=0; rox < RESICAL_OUT_XS; rox++)
				{
					#define GET_RESICAL_COEFF(cal_, ix_, iy_, ox_, oy_) ((cal_).resical_coeffs[(iy_)*RESICAL_IN_XS+(ix_)][(oy_)*RESICAL_OUT_XS+(ox_)])
					int32_t coeff = GET_RESICAL_COEFF(*p_tof_calib, rix, riy, rox, roy);

					output_offsets[ROC(rox,roy)] += (avg_input * coeff)/RESICAL_DIVIDER;
				}
			}

		}
	}

	// Compensate the output:

	for(int roy=0; roy < RESICAL_OUT_YS; roy++)
	{
		for(int rox=0; rox < RESICAL_OUT_XS; rox++)
		{
			int32_t offset = output_offsets[ROC(rox,roy)];
			for(int suby=0; suby < RESICAL_OUT_RATIO; suby++)
			{
				for(int subx=0; subx < RESICAL_OUT_RATIO; subx++)
				{
					int tx = rox*RESICAL_OUT_RATIO + subx;
					int ty = roy*RESICAL_OUT_RATIO + suby;

					if(ty >= TOF_NARROW_Y_START && ty < TOF_NARROW_Y_START+TOF_YS_NARROW &&
					   tx >= TOF_NARROW_X_START && tx < TOF_NARROW_X_START+TOF_XS_NARROW)
					{
						int narx = tx - TOF_NARROW_X_START;
						int nary = ty - TOF_NARROW_Y_START;

						if(tmp[nary*TOF_XS_NARROW+narx] == INT16_MAX || tmp[nary*TOF_XS_NARROW+narx] == INT16_MIN)
						{
							out[nary*TOF_XS_NARROW+narx] = tmp[nary*TOF_XS_NARROW+narx];
						}
						else
						{
							int32_t outval = (int32_t)tmp[nary*TOF_XS_NARROW+narx] - offset;
							// Data saturated - was nearly overexposed, now is overexposed.
							// Mark as overexposed:
							if(outval < INT16_MIN) outval = INT16_MIN;
							else if(outval > INT16_MAX) outval = INT16_MAX;
							out[nary*TOF_XS_NARROW+narx] = outval;
						}
					}
				}
			}
		}
	}
}


int corr_mask_ratio = 50;
// Removes distance points that required too heavy correction - they are likely too far from the truth, despite correction.
void ITCM mask_highly_corrected(int is_narrow, int16_t* restrict img20_lenscorr, int16_t* restrict img31_lenscorr, int16_t* restrict img20, int16_t* restrict img31)
{
	int n_pix = is_narrow?(TOF_XS_NARROW*TOF_YS_NARROW):(TOF_XS*TOF_YS);

	for(int i=0; i<n_pix; i++)
	{
		int32_t img20corr = abso(img20_lenscorr[i] - img20[i]);
		int32_t img31corr = abso(img31_lenscorr[i] - img31[i]);

		int32_t ampl1 = abso(img20[i]) + abso(img31[i]);
		int32_t ampl2 = abso(img20_lenscorr[i]) + abso(img31_lenscorr[i]);

		int32_t totcorr = img20corr + img31corr;

		if(ampl1 == 0 || ampl2 == 0)
		{
			img20_lenscorr[i] = 0;
			img31_lenscorr[i] = 0;
		}
		else
		{
			int32_t corr_ratio1 = (256*totcorr)/ampl1;
			int32_t corr_ratio2 = (256*totcorr)/ampl2;

			if(corr_ratio1 > corr_mask_ratio || corr_ratio2 > corr_mask_ratio)
			{
				img20_lenscorr[i] = 0;
				img31_lenscorr[i] = 0;
			}
		}
	}
}


/*
	Example of dealias threshold:

	Actual, accurate 4DCS 20MHz measurement (wraps around every 7.5m) says: 4.5m

	The options for the actual 20MHz distance are:
	4.5m, of course - not wrapped
	4.5 + 1*7.5 = 12.0 m
	4.5 + 2*7.5 = 19.5 m
	4.5 + 3*7.5 = 27.0 m
	4.5 + 4*7.5 = 34.5 m
	4.5 + 5*7.5 = 42.0 m
	(45 m is the combined wrap-around for the dealiased system. We better not see anything big that far! So let's stop the list here.)

	Then we have the
	inaccurate, 2DCS 6.66MHz measurement (wraps around every 22.5m), which says: 5.0m. Let's say DEALIAS_THRESHOLD is set at 2.0m.

	This validifies the following ranges:
	5.0m +/- 2.0m = 3.0  ..  7.0 m   <-- A 20Mhz reading "4.5m" exists inside this range - it must be valid
	(5.0m + 1*22.5m) +/- 2.0m = 25.5  ..  29.5 m  <-- There is no 20MHz reading inside this range - this is invalid

	The final result is 4.5m.

	The threshold is needed simply due to the inaccuracy of the long-range 2DCS data. In an ideal world, we would find an
	exact distance match.
*/
// For the two frequencies:
static const int DEALIAS_THRESHOLD[2] = {2400, 5000};


void ITCM compensated_tof_calc_ampldist(int is_narrow, uint16_t* restrict ampldist_out, int16_t* restrict dcs20_in, int16_t* restrict dcs31_in, uint8_t* dealias_dist, int freq, int inttime_norm_number)
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

	for(int i=0; i < n_pix; i++)
	{
		uint16_t dist;
		uint32_t ampl;

		if(dcs20_in[i] == INT16_MAX || dcs31_in[i] == INT16_MAX || dcs20_in[i] == INT16_MIN || dcs31_in[i] == INT16_MIN)
		{
			ampl = 15;
			dist = DIST_OVEREXP;
		}
		else
		{
			int32_t dcs20 = dcs20_in[i];
			int32_t dcs31 = dcs31_in[i];

			// dcs run +/- 32767

			ampl = AMPL_SAME_UNITS(dcs20, dcs31);
			// ampl should run +/- 32767 as well (could go a bit over, theoretical max 46340)

			if(ampl<100)
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

				}
				else // is_narrow
				{
					if(!(i&1)) // even
						pixgroup = shadow_luts.hif.nar_lut_group_ids[i/2] & 0x0f;
					else
						pixgroup = shadow_luts.hif.nar_lut_group_ids[i/2]>>4;

					dist = lookup_dist(1, pixgroup, dcs31, dcs20);
				}

				if(dist < 2) dist = 2;

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
					else if(dealias_dist[i] == DIST_OVEREXP || err0 < DEALIAS_THRESHOLD[freq])
						dist = dist>>DIST_SHIFT; // Keep hf as is
					else if(err1 < DEALIAS_THRESHOLD[freq])
						dist = hf_wrap1>>DIST_SHIFT;
					else if(err2 < DEALIAS_THRESHOLD[freq])
						dist = hf_wrap2>>DIST_SHIFT;
					else
						dist = DIST_UNDEREXP;

				}
				else
					dist>>=DIST_SHIFT;

				if(dist < 0) dist = 0;
				else if(dist > MAX_DISTVAL) dist = DIST_UNDEREXP;


			}

// If defined, the 4-bit amplitude output pixel is the actual amplitude
// Otherwise, it's surface emissivity approximated by multiplying the amplitude with the square of distance, divided by integration time
// warning: if enabled, one mechanism of removing false close readings is out
//#define AMPL_OUTPUT_IS_AMPL

			#ifdef AMPL_OUTPUT_IS_AMPL
				// 0..approx 32768 --> 0..15
				ampl/=2048;
				if(ampl>15) ampl=15;
			#else
				// ampl is now surface emissivity approx. 0 - 255. Remove suspiciously low values.
				// Basically, if it looks like the material is some super extra black with <2% emissivity,
				// it likely doesn't exists, but the distance, instead, was calculated wrong.
				ampl = (sq((uint64_t)dist)*(uint64_t)ampl)/((uint64_t)inttime_norm_number*300ULL);
				if(ampl < 5)
					dist = DIST_UNDEREXP;

				// Convert to a 4-bit val
				ampl/=16;
				if(ampl>15) ampl=15;

			#endif


		}
		ampldist_out[i] = (ampl<<12) | dist;
	}
}

void ITCM compensated_tof_calc_ampldist_nodealias_noampl_nonarrow(uint16_t* restrict ampldist_out, int16_t* restrict dcs20_in, int16_t* restrict dcs31_in)
{
	for(int i=0; i < TOF_XS*TOF_YS; i++)
	{
		uint16_t dist;

		if(dcs20_in[i] == INT16_MAX || dcs31_in[i] == INT16_MAX || dcs20_in[i] == INT16_MIN || dcs31_in[i] == INT16_MIN)
		{
			dist = DIST_OVEREXP;
		}
		else
		{
			int32_t dcs20 = dcs20_in[i];
			int32_t dcs31 = dcs31_in[i];

			// dcs run +/- 32767

			int32_t ampl = AMPL_SAME_UNITS(dcs20, dcs31);
			// ampl should run +/- 32767 as well (could go a bit over, theoretical max 46340)

			if(ampl<100)
			{
				dist = DIST_UNDEREXP;
			}
			else
			{
				int pixgroup;

				if(!(i&1)) // even
					pixgroup = shadow_luts.hif.wid_lut_group_ids[i/2] & 0x0f;
				else
					pixgroup = shadow_luts.hif.wid_lut_group_ids[i/2]>>4;

				dist = lookup_dist(0, pixgroup, dcs31, dcs20);

				if(dist < 2) dist = 2;

				dist>>=DIST_SHIFT;
				if(dist < 0) dist = 0;
				else if(dist > MAX_DISTVAL) dist = DIST_UNDEREXP;

				ampl = (sq((uint64_t)dist)*(uint64_t)ampl)/((uint64_t)(OBSTACLE_LONG_US*2)*300ULL);
				if(ampl < 5)
					dist = DIST_UNDEREXP;

			}


		}
		ampldist_out[i] = dist;
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

extern int obstacle_front_near, obstacle_back_near, obstacle_left_near, obstacle_right_near;
extern int obstacle_front_far, obstacle_back_far, obstacle_left_far, obstacle_right_far;

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
	#define NOZZLE_WIDTH 760

	#define OBST_AVOID_WIDTH 650
	#define SIDE_LIMIT 360
	#define FRONT_LIMIT 500
	#define BACK_LIMIT -750
	#define BACK_TURN_LIMIT -540


	void ITCM tof_to_obstacle_avoidance(uint16_t* ampldist, int sidx)
	{
		if(sidx < 0 || sidx >= N_SENSORS) error(150);

//		uart_print_string_blocking("\r\n");

		//if(sidx != 0) return;

		uint16_t local_sensor_hor_ang = tof_calibs[sidx]->mount.ang_rel_robot;
		uint16_t local_sensor_ver_ang = tof_calibs[sidx]->mount.vert_ang_rel_ground;


		int32_t  local_sensor_x = tof_calibs[sidx]->mount.x_rel_robot;
		int32_t  local_sensor_y = tof_calibs[sidx]->mount.y_rel_robot;
		int32_t  local_sensor_z = tof_calibs[sidx]->mount.z_rel_ground;


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
						if(local_z < 240 && local_x < 520 && local_x > 120 && local_y > -(NOZZLE_WIDTH/2) && local_y < (NOZZLE_WIDTH/2))
							continue;
					#endif

					if(local_z > 120 && local_z < 1200)
					{
						if(local_y > -(OBST_AVOID_WIDTH/2) && local_y < (OBST_AVOID_WIDTH/2))
						{
							if(local_x >= 100 && local_x < FRONT_LIMIT)
							{
								obstacle_front_near++;
								led_status(sidx, RED, LED_MODE_FADE);
							}
							else if(local_x >= FRONT_LIMIT && local_x < FRONT_LIMIT+100)
							{
								if(obstacle_front_near==0) led_status(sidx, ORANGE, LED_MODE_FADE);
								obstacle_front_far++;
							}

							if(local_x <= -450 && local_x > BACK_LIMIT)
							{
								led_status(sidx, RED, LED_MODE_FADE);
								obstacle_back_near++;

							}
							else if(local_x <= BACK_LIMIT && local_x > BACK_LIMIT-100)
							{
								if(obstacle_back_near==0) led_status(sidx, ORANGE, LED_MODE_FADE);
								obstacle_back_far++;
							}
						}

						if(local_x > BACK_TURN_LIMIT && local_x < -100)
						{
							if(local_y >= 200 && local_y < SIDE_LIMIT)
							{
								led_status(sidx, RED, LED_MODE_FADE);
								obstacle_left_near++;
							}
							else if(local_y >= SIDE_LIMIT && local_y < SIDE_LIMIT+100)
							{
								if(obstacle_left_near==0) led_status(sidx, ORANGE, LED_MODE_FADE);
								obstacle_left_far++;
							}

							if(local_y <= -200 && local_y > -SIDE_LIMIT)
							{
								led_status(sidx, RED, LED_MODE_FADE);
								obstacle_right_near++;
							}
							else if(local_y <= -SIDE_LIMIT && local_y > -SIDE_LIMIT-100)
							{
								if(obstacle_right_near==0) led_status(sidx, ORANGE, LED_MODE_FADE);
								obstacle_right_far++;
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

void total_sensor_obstacle(int sidx)
{
	led_status(sidx, RED, LED_MODE_FADE);
	switch(sidx)
	{
		case 0:
		case 1:
		case 9:
		obstacle_front_near++;
		break;

		case 2:
		case 3:
		obstacle_left_near++;
		break;

		case 4:
		case 5:
		case 6:
		obstacle_back_near++;
		break;

		case 7:
		case 8:
		obstacle_right_near++;
		break;

		default:
		break;
	}
}


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

void ITCM compensated_2dcs_6mhz_dist_masked(uint8_t *dist_out, epc_2dcs_t* restrict in, uint16_t* restrict bwimg)
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
				int16_t bw = ((bwimg[i]&0b0011111111111100)>>2)-2048;
				if(bw<0) bw=0;
				int corr_factor;
				if(!(i&1)) // even
					corr_factor = shadow_luts.lof.amb_corr[i/2] & 0x0f;
				else
					corr_factor = shadow_luts.lof.amb_corr[i/2]>>4;
				int16_t comp = (bw*corr_factor)>>4;

				dcs31 += comp;
				dcs20 += comp;
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

void ITCM compensated_2dcs_6mhz_dist_masked_narrow(uint8_t *dist_out, epc_2dcs_narrow_t* restrict in, uint16_t* restrict bwimg)
{
	int32_t base_offs = shadow_luts.lof.nar_base_offset_2dcs;
	for(int i=0; i < TOF_XS_NARROW*TOF_YS_NARROW; i++)
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
				int16_t bw = ((bwimg[i]&0b0011111111111100)>>2)-2048;
				if(bw<0) bw=0;
				int corr_factor;
				if(!(i&1)) // even
					corr_factor = shadow_luts.lof.amb_corr[i/2] & 0x0f;
				else
					corr_factor = shadow_luts.lof.amb_corr[i/2]>>4;
				int16_t comp = (bw*corr_factor)>>4;

				dcs31 += comp;
				dcs20 += comp;
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



void ITCM process_bw(uint8_t *out, epc_img_t* restrict in)
{
	for(int i=0; i<TOF_XS*TOF_YS; i++)
	{
		int lum = ((in->img[i]&0b0011111111111100)>>2)-2048;
		if(lum < 0) lum = 0; else if(lum > 2047) lum = 2047;
		out[i] = lum>>3;
	}
}

void ITCM process_dcs(int16_t* restrict out, epc_img_t* restrict in)
{
	for(int i=0; i<TOF_XS*TOF_YS; i++)
	{
		out[i] = ((in->img[i]&0b0011111111111100)>>2)-2048;
	}
}

void process_dcs_narrow(int16_t* restrict out, epc_img_narrow_t* restrict in)
{
	for(int i=0; i<TOF_XS_NARROW*TOF_YS_NARROW; i++)
	{
		out[i] = ((in->img[i]&0b0011111111111100)>>2)-2048;
	}
}

