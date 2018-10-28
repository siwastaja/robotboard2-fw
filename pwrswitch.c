#include <stdint.h>
#include "adcs.h"
#include "ext_include/stm32h7xx.h"
#include "stm32_cmsis_extension.h"
#include "misc.h"
#include "pwrswitch.h"
#include "timebase.h" // for ms_cnt


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
	RCC->AHB4ENR |= 1UL<<1;
	LO(GPIOB, 2);
	IO_TO_GPO(GPIOB, 2);
}

void init_pwrswitch_and_led()
{
	RCC->AHB4ENR |= 1UL<<4 | 1UL<<5; // PE and PF

	IO_TO_GPI(GPIOE,2); // power switch sense

	IO_TO_GPO(GPIOF,2); // power switch LED
	IO_OPENDRAIN(GPIOF,2);

	APP_CP_LO();
	IO_TO_GPO(GPIOF,12); // APP charge pump

	APP_EN_DESAT_PROT();
	IO_TO_GPO(GPIOE,15); // App desat protection disable output
	
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

int main_power_enabled = 2;
int app_power_enabled = 0;


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

Test #3: Same but 10us, 10us




 (20.0V, 10mF elcap)
*/

void app_power_on()
{
//	app_power_enabled = 1;

	// Each 6us pulse, resistance and inductance limited to, say, 200A average, supplies
	// 1.2mC charge. If the app has 10mF of capacitance, each pulse precharges by about
	// 1.2mC/10mF = 120mV. Around 100 pulses are therefore required to properly precharge
	// such pessimistic load. At 1kHz, this takes 100ms.

	APP_EN_DESAT_PROT();
	APP_CP_LO();
	delay_us(10000);
	for(int i=0; i<1; i++)
	{
		APP_DIS_DESAT_PROT();
		APP_CP_HI();
		delay_us(10);
		APP_CP_LO();
		delay_us(10);
		APP_EN_DESAT_PROT();
		delay_us(10000);
	}

	APP_CP_LO();
//	app_precharge_pulsetrain = 100;
}

void app_power_off()
{
	APP_EN_DESAT_PROT();
	app_power_enabled = 0;
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
		if(++shutdown_cnt > 1000) // Milliseconds to give the SBC to shut down
		{
			main_power_enabled = 0;
		}
		if(ms_cnt & (1UL<<8))
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
	__DSB(); __ISB();
	SAFETY_SHUTDOWN();
	while(1);
}



