#include <stdint.h>
#include "adcs.h"
#include "ext_include/stm32h7xx.h"
#include "stm32_cmsis_extension.h"
#include "misc.h"
#include "pwrswitch.h"
#include "timebase.h" // for cnt_100us

#include "adcs.h"
#include "own_std.h"

// QuickPrint86: 10mF elcap "battery" at 20V shorted at output,
// yellow = desat, pink = UV
// QuickPrint87:
// yellow = desat, pink = OV. Both UV and OV flicker spuriously during desat protection
// QuickPrint88:
// Desat reaction to the short in 30 us

// UV test:
// 15.5V (2.58V/cell) goes black
// 16.7V (2.78V/cell) allows restarting
// Theoretical design value was 15.60V

// QuickPrint92:
// Vbat=20V (still the elcap bank), 1 ohm load connected (I = 20A),
// UV trigs first (desat only triggers as a consequence 10us later)


// Trig tests:
// POS 5: UV trigs
// POS 4: UV trigs
// POS 3: UV trigs
// POS 2: QP93,QP94,QP95,QP96 around 600-1500 us  R = 213mV / 1A
// POS 1: QP97,QP98,QP99, around 100-300 us       R = 146mV / 1A
// POS 0: QP101,102,103,104, repeatably 30us      R = 111mV / 1A  (mechanical switch isn't bouncing that much anymore at this current level)
// "dead short": QP105, 106, 107: goes down to about 25us if the mechanical switch allows...

// Results:
// 93A: Trig in 500us
// 136A: Trig in 100us
// 180A: Trig in 30us
// ~300A+: Trig in 25us
// Excel calculated threshold at 25degC, Vbat=20V: 71.9A nominal
// Close!

// OV test:
// At 25.46V (4.243V/cell), some oscillating spikes start to appear in the OV signal, but not long enough to bring it down (QP108)
// At 25.50V (4.250V/cell), it shuts down
// Theoretical design value was 25.49V, so it's really close.


// TAP:
// Vin=25.0V
// TAPLO=11.90V
// TAPHI=13.10V
// --> 0.60V imbalance allowed - when off
// TAPHI=12.57V
// TAPLO=12.42V
// --> 0.07V imbalance allowed - when on

// Vin=19.0V
// TAPLO=8.57V
// TAPHI=9.43V
// --> 0.43V imbalance allowed - when off or on

// In-circuit test of the tap threshold change:
// 24.6V -> lowed thresholds


void pwrswitch_chargepump_init()
{
	#ifdef REV2A
		RCC->AHB4ENR |= 1UL<<1;
		LO(GPIOB, 2);
		IO_TO_GPO(GPIOB, 2);
	#endif

	#ifdef REV2B
		RCC->AHB4ENR |= 1UL<<7;
		LO(GPIOH, 4);
		IO_TO_GPO(GPIOH, 4);
	#endif

}

void init_pwrswitch_and_led()
{
	#ifdef REV2A
		RCC->AHB4ENR |= 1UL<<4 | 1UL<<5; // PE and PF

		IO_TO_GPI(GPIOE,2); // power switch sense

		IO_TO_GPO(GPIOF,2); // power switch LED
		IO_OPENDRAIN(GPIOF,2);

		APP_CP_LO();
		IO_TO_GPO(GPIOF,12); // APP charge pump

		APP_EN_DESAT_PROT();
		IO_TO_GPO(GPIOE,15); // App desat protection disable output
	#endif

	#ifdef REV2B
		RCC->AHB4ENR |= 1UL<<0 | 1UL<<5 | 1UL<<6 | 1UL<<8; // PA, PF, PG, and PI

		IO_TO_GPI(GPIOI,11); // power switch sense

		IO_TO_GPO(GPIOF,2); // power switch LED
		IO_OPENDRAIN(GPIOF,2);

		APP_CP_LO();
		IO_TO_GPO(GPIOG,3); // APP charge pump

		APP_EN_DESAT_PROT();
		IO_TO_GPO(GPIOA,2); // App desat protection disable output
	#endif

	
}


// Vgs decays below 7.0V (25.0V, Vbat=18.0V) in 9ms after the pulse generation stops.
// Before the proper timer is initialize, call this frequently enough.
void chargepump_initial_pulsetrain()
{

	// First scope trace: 100 pulses at 10kHz (delay value 8)
	// Scond: 50 pulses at 5kHz (delay value 16)
	// third: 25 pulses at 2.5kHz (delay value 32)
	// 4th: 13 pulses at 1.0kHz (delay value 80)

	// This runs at the low CPU clock (64MHz)
	// The delay functions are calculated for 400MHz CPU.

	// Steady state gate V ripple measurements for different pulses
	// Base voltage = 18.0V
	// Rise times measured to Vgs_min=8V (26.0V)
	// Ripple min&max:
	// 80,80 = 27.2V 29.4V  7.6ms
	// 40,80 = 27.9V 29.3V  7.9ms
	// 20,80 = 27.9V 29.2V  9.8ms
	// 8,80  = 27.9V 28.8V  11.2ms

	// 32,32 = 27.1V 28.5V  4.4ms
	// 16,32 = 27.6V 28.6V  5.3ms
	// 8,32  = 27.7V 28.6V  7.5ms
	// First number is HI-time (FET on, power consumed in pull-up resistor)
	// Second number is LO-time
	// Unit: delay_us argument: 8 units = 50us.

	// Low power consumption is important to minimize drop in precharge resistors.
	// Quick turn-on is preferable, but not paramount.
	// Maximized Vgs is unimportant; the actual timer takes over soon.
	// --> Use 16,32 (100us HI, 200us LO, period=300us, freq=3.33kHz)
	// Min. num pulses after Vgs_min=8V: 18 -> use 30

	// All info based on prototype measurements


	for(int i=0; i<30; i++)
	{
		PLAT_CP_HI();
		delay_us(16);
		PLAT_CP_LO();
		delay_us(32);
	}
}

// Shorter pulsetrain to keep the gate charged
// Calling this assumes that the gate is already charget over minimum usable threshold
// Fewer pulses are given than driving from zero.
void chargepump_replenish_pulsetrain()
{
	for(int i=0; i<18; i++)
	{
		PLAT_CP_HI();
		delay_us(16);
		PLAT_CP_LO();
		delay_us(32);
	}
}

void delay_us_local(uint32_t i) // from flash, running at slower clock
{
	i *= 15; // 100 for 400MHz -> 15 for 60MHz
	i -= 7;
	while(i--)
		__asm__ __volatile__ ("nop");
}

void chargepump_replenish_pulsetrain_before_itcm_copy()
{
	for(int i=0; i<18; i++)
	{
		PLAT_CP_HI();
		delay_us_local(16);
		PLAT_CP_LO();
		delay_us_local(32);
	}
}


// 10% duty at 1kHz; charges the gate slower, but keeps it charged with very little power.
// specify the delay length in ms
void chargepump_pulsetrain_low_power(uint32_t del_ms)
{
	while(del_ms--)
	{
		PLAT_CP_HI();
		delay_us(16);
		PLAT_CP_LO();
		delay_us(144);
	}
}

volatile int main_power_enabled = 1;
volatile int app_power_enabled = 0;


int app_precharge_pulsetrain;

/*

App pwr switch tests:
cyan = Vbat
yellow = App pwr out
pink = Vg
Blue = Vs

Test #1: QP115
	Output 5mF || 10.5 ohm,
	10 cycles of 100us CP_HI, 10000 us CP_LO,
	desat prot disabled during the cycles

Test #2: QP116
	100 us HI, desat disabled
	100 us LO, desat still disabled
	Desat enabled
	-> Vs goes negative in 10us

Test #3: QP117
	Same but 10us, 10us

Test #4: QP118
	Same, but 10 times back-to-back

Test #5: QP119
	20 times, 20us,40us

	20us,40us takes the same actual time (600us) as 10us,20us to reach Vs=0, where real gate charging starts.
	Exactly the same for 40,80; 40,40; or even 40,20.
	Input voltage doesn't matter.

Test #6: QP120
	Desat prot can be kept on during most of this period; Vs rises quickly after it's released.
	About 200us is still needed.

Test #7: QP121
	With 10  20+40us pulses DESAT ENABLED, then just 8  20+40us pulses DESAT DISABLED, the current already flows


Test #8: QP122
	Same, but only 7 DESAT ENABLED pulses. As we can see, 10 gives quicker current rise after we go.
	15 desat enabled pulses don't give any quicker rise anymore than 10.


Test #9: QP123
	Output voltage measurement decides the number of cycles needed to reach conducting Vg, trying to keep
	the number of actually current-increasing (conducting) cycles to a constant.
	QP124: Closeup of the second cycle (i=1): conducting time = 190us
	QP125: Closeup of the 29th cycle (i=28): conducting time = 380us

Test #10 QP126
	Fairly reproducible gate voltage pump-up driven by ADC, until the point just before conduction.

Test #11 QP127
	Proper precharge into 5mF | 10.5 ohm in 60ms

	00000 va:00223 vg:00995 n_cycs1:00000 co va:00103 vg:02481 n_cycs2:00007 done va:00161 vg:03928 vdiff:19622 cl_cnt:00006
	00001 va:01106 vg:01114 n_cycs1:00000 co va:00936 vg:02812 n_cycs2:00010 done va:01061 vg:04792 vdiff:18690 cl_cnt:00007
	00002 va:02095 vg:01175 n_cycs1:00001 co va:02209 vg:03599 n_cycs2:00011 done va:02120 vg:05755 vdiff:17621 cl_cnt:00008
	00003 va:03179 vg:01261 n_cycs1:00003 co va:03132 vg:04792 n_cycs2:00011 done va:03102 vg:06757 vdiff:16644 cl_cnt:00009
	00004 va:04234 vg:01323 n_cycs1:00005 co va:04290 vg:05828 n_cycs2:00011 done va:04163 vg:07734 vdiff:15582 cl_cnt:00010
	00005 va:05368 vg:01404 n_cycs1:00007 co va:05287 vg:06891 n_cycs2:00011 done va:05257 vg:08775 vdiff:14494 cl_cnt:00011
	00006 va:06546 vg:01401 n_cycs1:00009 co va:06521 vg:07809 n_cycs2:00012 done va:06210 vg:09822 vdiff:13548 cl_cnt:00012
	00007 va:07500 vg:01526 n_cycs1:00011 co va:07468 vg:08928 n_cycs2:00012 done va:07281 vg:10931 vdiff:12477 cl_cnt:00013
	00008 va:09202 vg:01771 n_cycs1:00014 co va:08855 vg:10167 n_cycs2:00013 done va:08915 vg:12434 vdiff:10846 cl_cnt:00015
	00009 va:10331 vg:01696 n_cycs1:00017 co va:10178 vg:11592 n_cycs2:00012 done va:09955 vg:13553 vdiff:09809 cl_cnt:00016
	00010 va:11456 vg:01784 n_cycs1:00019 co va:11239 vg:12552 n_cycs2:00012 done va:11009 vg:14558 vdiff:08757 cl_cnt:00017
	00011 va:12811 vg:01854 n_cycs1:00021 co va:12334 vg:13584 n_cycs2:00013 done va:12020 vg:15758 vdiff:07750 cl_cnt:00018
	00012 va:14114 vg:01898 n_cycs1:00024 co va:13450 vg:14858 n_cycs2:00011 done va:13214 vg:16737 vdiff:06545 cl_cnt:00019
	00013 va:15015 vg:01913 n_cycs1:00027 co va:14563 vg:16044 n_cycs2:00011 done va:14315 vg:17905 vdiff:05455 cl_cnt:00020
	00014 va:16157 vg:02044 n_cycs1:00029 co va:15431 vg:16956 n_cycs2:00012 done va:15329 vg:18935 vdiff:04445 cl_cnt:00021
	00015 va:17295 vg:02133 n_cycs1:00031 co va:16460 vg:17888 n_cycs2:00009 done va:16148 vg:19783 vdiff:03624 cl_cnt:00022
	00016 va:17396 vg:02116 n_cycs1:00031 co va:16679 vg:17928 n_cycs2:00012 done va:16821 vg:21130 vdiff:02953 cl_cnt:00023
	00017 va:19379 vg:27436 n_cycs1:00000 co va:19593 vg:27339 n_cycs2:00000 done va:19350 vg:27202 vdiff:00168 cl_cnt:03657


Test #12 QP128: Very successful precharge
	QP130: Driving into short
	QP131: Closeup at 2V/div. MOSFET is driven into short:
	QP132: Determining the max current from 10mF input: dt=65us, dV=212mV, 
	A = VF/s = 0.212V*10e-3F/65e-6s = 3.26A.
	Peak current = 3.26A, duration 65 us, at Vds=25V. SOA says 300A is OK for 65us at Vds=25V!

	QP133: Increased the precharge current.
	A = VF/s = 0.428V*10e-3F/180e-6s = 2.37A. (maximum peak might still be around 3.26A).
	Driving into the short lasts now about 250us.
	Per SOA, at Vds=25V, at 3A, 7-8ms is ok. Alternatively, for 250us, 70-80 A is ok

PCB revision changes:
* Add Vapp measurement, 47k + 6.8k like Vbat, but 470pF only
* Change Vg(app) measurement cap from 100n to 470pF

 (20.0V, 10mF elcap)
*/

int app_power_on()
{
	if(app_power_enabled)
		return -2;
	// RC time constant for measuring Vapp is 3us.
	// RC time constant for measuring Vg is 6us.
	// ADC sampling & conversion takes 0.3us. For two channels, 0.6us.
	// For two channels oversampled 4x, 2.4us

	APP_EN_DESAT_PROT();
	APP_CP_LO();
	delay_us(100);

#define N_PULSES 50

	#ifdef PWRSWITCH_DBG_PRINT
		int vas[N_PULSES];
		int vgs[N_PULSES];
		int changeover_vas[N_PULSES];
		int changeover_vgs[N_PULSES];
		int done_vas[N_PULSES];
		int done_vgs[N_PULSES];
		int n_cycs_1[N_PULSES];
		int n_cycs_2[N_PULSES];
		int vdiffs[N_PULSES];
		int currlim_cnts[N_PULSES];
	#endif

	int success = 0;
	int prev_va = -100;
	int prev_prev_va = -200;
	int short_cnt = 0;
	int boost = 0;

	// APP_CP_LO brings the gate actually higher

	for(int i=0; i<N_PULSES; i++)
	{

		// First, we give charge pump cycles that bring the gate out of negative voltage, still
		// against the desat circuit. Actually measured that 7 such pulses really decrease the
		// number of pulses needed in the next step (which will be protection disabled).
		// Perform 10 such pulses.
		for(int o=0; o<10; o++)
		{
			APP_CP_HI();
			delay_us(30);
			APP_CP_LO();
			delay_us(30);
		}

		APP_DIS_DESAT_PROT();
		// Now, we are unprotected, and rely on the charge pump generating proper amount of
		// gate charge, timed carefully.


		// Measure the application output voltage (Va) and the gate voltage (Vg)
		ADC3_CONV_INJECTED();
		delay_us(4);
		
		int va = VAPP_MEAS_TO_MV(ADC3_VAPP_DATAREG);
		#ifdef PWRSWITCH_DBG_PRINT
			int vg = VGAPP_MEAS_TO_MV(ADC3_VGAPP_DATAREG);
			vas[i] = va;
			vgs[i] = vg;
		#endif

		// See if the output voltage is actually rising at sufficient rate.
		// Require 200mV increase per precharge cycle
		// Measurement has a bit of noise, so we look two cycles back, and expect
		// 400mV per two cycles.
		// Don't bail out immediately, see if the problem persists for over 3 cycles.
		// The code later also looks at short_cnt, to react before the short detection hits.
		if(va < prev_prev_va+400)
		{
			if(++short_cnt > 3)
			{
				APP_EN_DESAT_PROT();
				#ifdef PWRSWITCH_DBG_PRINT
					uart_print_string_blocking("\r\nShort detection (non-rising): ");
					o_utoa16(i, printbuf); uart_print_string_blocking(printbuf);
					uart_print_string_blocking("\r\n");
				#endif
				break;
			}
		}
		else
			short_cnt = 0;

		prev_prev_va = prev_va;
		prev_va = va;

		// Now, as the desat protection was on (and active), the gate is always at 0V.
		// Bring it up to near conduction, which will be at Vapp + Vgs(th), so around
		// Vapp + 3V.
		// First do coarse pulses, pumping the gate up to Vapp + 1.2V.
		// Implement a safety limit on number of pulses:
		// Measured:
		// At 0V,  0 cycles needed
		// At 17000mV, 31 cycles needed.
		// 17000/438=38 cycles or 25% extra.
		int n_cyc = 6 + va/438; // sanity max

		int o;
		for(o = 0; o<n_cyc; o++)
		{
			APP_CP_HI();
			delay_us(26);
			ADC3_CONV_INJECTED();
			delay_us(4);
			int peak_va1 = ADC3_VAPP_DATAREG;
			APP_CP_LO();
			delay_us(26);
			ADC3_CONV_INJECTED();
			delay_us(4);
			int peak_va = VAPP_MEAS_TO_MV( (peak_va1 + ADC3_VAPP_DATAREG)>>1 );
			int peak_vg = VGAPP_MEAS_TO_MV(ADC3_VGAPP_DATAREG);

			if(peak_vg > peak_va+1200)
			{
				#ifdef PWRSWITCH_DBG_PRINT
					changeover_vas[i] = peak_va;
					changeover_vgs[i] = peak_vg;
				#endif
				goto SLOWER;
			}		
		}
		error(47);
		SLOWER:;

		#ifdef PWRSWITCH_DBG_PRINT
			n_cycs_1[i] = o;
		#endif

		// Then, do fine pulses, each pulse pumps the gate up considerably less than the
		// coarse pulses before. Bring the Vgs to 3.5V, which is just on the brink of conduction.
		// The MOSFETs might be conducting a little, but this is negligible regarding the safe
		// operating area. The most important bit is that this is fairly repeatable, regardless of
		// the actual operating conditions such as battery voltage or temperature.

		int latest_va;
		int e;
		for(e=0; e<20; e++) // Actually measured that 7..13 pulses are needed.
		{
			APP_CP_HI();
			delay_us(26);
			ADC3_CONV_INJECTED();
			delay_us(4);
			int peak_va1 = ADC3_VAPP_DATAREG;
			APP_CP_LO();
			delay_us(6);
			ADC3_CONV_INJECTED();
			delay_us(4);
			int peak_va = VAPP_MEAS_TO_MV( (peak_va1 + ADC3_VAPP_DATAREG)>>1 );
			int peak_vg = VGAPP_MEAS_TO_MV(ADC3_VGAPP_DATAREG);

			if(peak_vg > peak_va+3500)
			{
				latest_va = peak_va;
				#ifdef PWRSWITCH_DBG_PRINT
					done_vas[i] = peak_va;
					done_vgs[i] = peak_vg;
				#endif
				goto DONE;
			}
		}
		error(48);
		while(1); // to tell the compiler we don't come back from error(), to prevent Wmaybe-uninitialized
		DONE:;

		#ifdef PWRSWITCH_DBG_PRINT
			n_cycs_2[i] = e;
		#endif

		// Now, we are at conduction. Any pulses will basically immediately pump the MOSFET current
		// higher. Do a controlled amount of pulses. Constantly doing 1 pulse here is enough to give careful,
		// almost constant-current precharge into capacitive load. More pulses means bigger current, in a
		// somewhat quadratic way (at least steeper than linear).
		// Now, doing constant number of pulses (1-5 depending on how fast we want to precharge) is perfectly
		// fine for capacitive-only loads, but many loads may have resistive elements, or something is actually
		// starting up and consuming current way before the target voltage is reached. As such, more current
		// is needed, or the voltage rise stops. Luckily, the closer we are the target voltage, the less the
		// Vds over the MOSFETS, meaning wider SOA - so we can safely increase the current.

		// Calculate vdiff = voltage over the MOSFETs
		// If the difference is <1V, consider the precharge finished
		// If the difference is low, it's safe to use more pulses, to produce higher current.
		// If the output voltage is not rising, and the short detection is about to trigger, and
		// the precharge is almost complete (Vdiff<10V), we can safely increase the current even further,
		// the MOSFETs are fine.

		int vdiff = VBAT_MEAS_TO_MV(adc1.s.vbat_meas) - latest_va;

		#ifdef PWRSWITCH_DBG_PRINT
			vdiffs[i] = vdiff;
		#endif

		// At 25Vdiff, 2 pulses
		// At 22Vdiff, 3 pulses
		// At 20Vdiff, 5 pulses
		// At 15Vdiff, 10 pulses
		// At 10Vdiff, 15 pulses
		// At  5Vdiff, 20 pulses
		// At  5Vdiff, 30 pulses if the short detection is about to trigger
		// At  2Vdiff, 23 pulses
		// At  2Vdiff, 39 pulses if the short detection is about to trigger

		int currlim_cnt = (25-(vdiff>>10 /*/1024*/));
		if(currlim_cnt < 2) currlim_cnt=2;

		if(short_cnt > 0 && vdiff < 10000)
		{
			// Load has increased, this isn't just a capacitive precharge.
			// Vdiff is low enough (SOA has widened) so we can afford an extra
			// boost compared to the above linear equation.

			// This setting applies if the short detection is almost triggering near the
			// end of precharge. Even if this happens once, the boost setting keeps applied
			// till the end of precharge.
			boost = 2*(10-(vdiff>>10));
		}

		currlim_cnt += boost;

		#ifdef PWRSWITCH_DBG_PRINT
			currlim_cnts[i] = currlim_cnt;
		#endif

		for(int u=0; u<currlim_cnt; u++)
		{
			APP_CP_HI();
			delay_us(20);
			APP_CP_LO();
			delay_us(20);
		}

		// Cycle ended. Enable desat protection again, which brings the gate quickly down.

		APP_EN_DESAT_PROT();

		if(vdiff < 1000)
		{
			#ifdef PWRSWITCH_DBG_PRINT
				uart_print_string_blocking("\r\nDone precharging: ");
				o_utoa16(i, printbuf); uart_print_string_blocking(printbuf);
				uart_print_string_blocking("\r\n");
			#endif
			success = 1;
			break;
		}

		delay_us(1000);
	}
	APP_EN_DESAT_PROT();

	APP_CP_LO();
	delay_us(10);

	if(success)
	{
		// Start generating gate pump signals in the timebase ISR:
		app_power_enabled = 1;
	}
	else
	{
		#ifdef PWRSWITCH_DBG_PRINT
			uart_print_string_blocking("\r\n PRECHARGE PROBLEM \r\n");
		#endif
	}

	#ifdef PWRSWITCH_DBG_PRINT
		uart_print_string_blocking("\r\n PULSE TRAIN ENDED \r\n");
		for(int i=0; i<N_PULSES; i++)
		{
			o_utoa16_fixed(i, printbuf); uart_print_string_blocking(printbuf);
			uart_print_string_blocking(" va:");
			o_utoa16_fixed(vas[i], printbuf); uart_print_string_blocking(printbuf);
			uart_print_string_blocking(" vg:");
			o_utoa16_fixed(vgs[i], printbuf); uart_print_string_blocking(printbuf);
			uart_print_string_blocking(" n_cycs1:");
			o_utoa16_fixed(n_cycs_1[i], printbuf); uart_print_string_blocking(printbuf);
			uart_print_string_blocking(" co va:");
			o_utoa16_fixed(changeover_vas[i], printbuf); uart_print_string_blocking(printbuf);
			uart_print_string_blocking(" vg:");
			o_utoa16_fixed(changeover_vgs[i], printbuf); uart_print_string_blocking(printbuf);
			uart_print_string_blocking(" n_cycs2:");
			o_utoa16_fixed(n_cycs_2[i], printbuf); uart_print_string_blocking(printbuf);
			uart_print_string_blocking(" done va:");
			o_utoa16_fixed(done_vas[i], printbuf); uart_print_string_blocking(printbuf);
			uart_print_string_blocking(" vg:");
			o_utoa16_fixed(done_vgs[i], printbuf); uart_print_string_blocking(printbuf);
			uart_print_string_blocking(" vdiff:");
			o_utoa16_fixed(vdiffs[i], printbuf); uart_print_string_blocking(printbuf);
			uart_print_string_blocking(" cl_cnt:");
			o_utoa16_fixed(currlim_cnts[i], printbuf); uart_print_string_blocking(printbuf);
			uart_print_string_blocking("\r\n");
		}
		uart_print_string_blocking("\r\n");
	#endif	

	return success?0:-1;
}

void app_power_off()
{
	APP_EN_DESAT_PROT();
	DIS_IRQ();
	app_power_enabled = 0;
	APP_CP_LO();
	ENA_IRQ();
}

void pwrswitch_1khz() __attribute__((section(".text_itcm")));
void pwrswitch_1khz()
{
	static int pwrswitch_cnt;
	static int shutdown_cnt;
	static int ignore_pwrswitch = 2000; // for how many milliseconds the power button is ignored after bootup

	if(ignore_pwrswitch == 0 && PWRSWITCH_PRESSED)
	{
		pwrswitch_cnt++;
		if(main_power_enabled>1 && pwrswitch_cnt > 100)
		{
			main_power_enabled = 1;
		}
		else if(pwrswitch_cnt > 2000) // Milliseconds to keep the button pressed down to force power-off without extra waiting
		{
			main_power_enabled = 0;
		}
	}
	else
	{
		if(ignore_pwrswitch > 0)
			ignore_pwrswitch--;
		if(pwrswitch_cnt > 16)
			pwrswitch_cnt-= 16;
	}

	if(main_power_enabled > 1)
	{
		PWRLED_ON();
	}
	else if(main_power_enabled == 1)
	{
		if(++shutdown_cnt > 10000) // Milliseconds to give the SBC to shut down
		{
			main_power_enabled = 0;
		}
		if(cnt_100us & (1UL<<11))
			PWRLED_ON();
		else
			PWRLED_OFF();
	}
	else
	{
		shutdown();
	}
}

void pwrswitch_safety_shutdown() __attribute__((section(".text_itcm")));
void pwrswitch_safety_shutdown()
{
	APP_EN_DESAT_PROT();
//	main_power_enabled = 0;  // keep on for flashing
	app_power_enabled = 0;
	__DSB(); __ISB();
		
}

void shutdown()
{
	__disable_irq();
	main_power_enabled = 0;
	PWRLED_OFF();
	__DSB(); __ISB();
	SAFETY_SHUTDOWN();
	while(1);
}



