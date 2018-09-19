#include <stdint.h>
#include "ext_include/stm32h7xx.h"
#include "stm32_cmsis_extension.h"
#include "misc.h"

#define CONF_ADC_SEQ(adc_,len_,s1_,s2_,s3_,s4_,s5_,s6_,s7_,s8_,s9_,s10_,s11_,s12_,s13_,s14_,s15_,s16_) do{ \
	adc_->SQR1 = (s4_)<<24 | (s3_)<<18 | (s2_)<<12 | (s1_)<<6 | (len_-1); \
	adc_->SQR2 = (s9_)<<24 | (s8_)<<18 | (s7_)<<12 | (s6_)<<6 | (s5_); \
	adc_->SQR3 = (s14_)<<24 | (s13_)<<18 | (s12_)<<12 | (s11_)<<6 | (s10_); \
	adc_->SQR4 = (s16_)<<6 | (s15_); \
	} while(0)


// If differential measurements will be used, remember to add a calibration cycle for diff meas.
static void init_calib_adc(ADC_TypeDef *adc)
{
	adc->CR = 0; // DEEPPWD (deep power down) mode off (reset state is bit29 high).
	adc->CR = 1UL<<28; // Voltage regulator on
	delay_us(100); // minimum required 10us per datasheet, but let's be sure to get the calibration values accurate.
	adc->CR = 1UL<<28 /* Keep Vreg on*/ | 1UL<<16 /* Calibrate for nonlinearity as well */;
	adc->CR |= 1UL<<31; // Start calibration
	delay_us(1);
	while(adc->CR & (1UL<<31)) ; // Wait for calibration success
	adc->ISR = 1UL; // Clear ADRDY
	adc->CR |= 1UL; // Enable ADC
	while(adc->ISR & 1UL) ; // Wait for enable success
	adc->CR |= 1UL<<8 /*BOOST bit for fADC>20MHz*/;
}

#define TRIG_SW      0b00UL
#define TRIG_RISING  0b01UL
#define TRIG_FALLING 0b10UL
#define TRIG_BOTH    0b11UL

#define RESO_16B 0b000UL
#define RESO_14B 0b001UL
#define RESO_12B 0b010UL
#define RESO_10B 0b011UL
#define RESO_8B  0b100UL

void init_adcs()
{

	// Clear DEEPPWD
	// ADVREGEN = 1
	// Wait for TADCVREG_STUP (10 us min)
	
	// ADCALDIF = 0
	// ADCALLIN = 1
	// ADCAL = 1
	// Wait until ADCAL goes 0
	// ADC_ISR write 1 to ADRDY
	// ADEN = 1
	// Wait until ADRDY goes 1


	init_calib_adc(ADC1);
//	CONF_ADC_SEQ(ADC1, ADC1_SEQ_LEN, ADC1_SEQ);
	ADC1->CFGR = 1UL<<31 /*Injected queue disable*/ |

		     0UL<<26 /*AWD1 chan*/ | 0UL<<23 /*Enable AWD1?*/ | 1UL<<22 /*AWD1 on the single (configured) channel only?*/ |

		     1UL<<13 /*Continuous mode?*/ |
		     TRIG_SW<<10 /*Trigger type*/ |
		     0UL<<5 /*Trigger event number*/ |
		     RESO_16B<<2 /*Resolution*/ |
		     0b11UL /*11 = DMA circular, 01 = DMA one shot*/;

	// Analog watchdog 2 enabled on channels (bit index = channel number):
	ADC1->AWD2CR = 0; 
	ADC1->LTR2 = 0;
	ADC1->HTR2 = 65535;

	ADC1->AWD3CR = 0; 
	ADC1->LTR3 = 0;
	ADC1->HTR3 = 65535;

	init_calib_adc(ADC2);
//	CONF_ADC_SEQ(ADC2, ADC2_SEQ_LEN, ADC2_SEQ);

	init_calib_adc(ADC3);
//	CONF_ADC_SEQ(ADC3, ADC3_SEQ_LEN, ADC3_SEQ);

}
