/*
 * ServoControl.cpp
 *
 *  Created on: 10 mai 2013
 *      Author: alienx
 */

#include "ServoControl.h"
#include <plib.h>

#include <wiring.h>
#include <wiring_private.h>

ServoControl::ServoControl() {
	// TODO Auto-generated constructor stub

}

void ServoControl::init()
{
	OpenOC1( OC_ON | OC_TIMER3_SRC | OC_TIMER_MODE16 | OC_PWM_FAULT_PIN_DISABLE, VAL_PPM_MIN, VAL_PPM_MIN);
	OpenOC2( OC_ON | OC_TIMER3_SRC | OC_TIMER_MODE16 | OC_PWM_FAULT_PIN_DISABLE, VAL_PPM_MIN, VAL_PPM_MIN);
	OpenOC3( OC_ON | OC_TIMER3_SRC | OC_TIMER_MODE16 | OC_PWM_FAULT_PIN_DISABLE, VAL_PPM_MIN, VAL_PPM_MIN);
	OpenOC4( OC_ON | OC_TIMER3_SRC | OC_TIMER_MODE16 | OC_PWM_FAULT_PIN_DISABLE, VAL_PPM_MIN, VAL_PPM_MIN);

	// init Timer2 mode and period (PR2)
	// Fpb = SYS_FREQ = 80Mhz (From configuration in bootloader code)
	// Timer Prescale = 8
	// PR2 = 0x9C3F = 39999
	// interrupts every 4 ms
	// 4 ms = (PR2 + 1) * TMR Prescale / Fpb = (39999 + 1) * 8 / 80000000
	CloseTimer3();
	OpenTimer3(T3_ON | T3_PS_1_64,T2_PERIOD);
	/*ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_7);

	mT2SetIntPriority( 7);    // set Timer2 Interrupt Priority
	mT2ClearIntFlag();       // clear interrupt flag
	mT2IntEnable(1);      // enable timer2 interrupts*/

	OC1RS = VAL_PPM_MIN;
	OC2RS = VAL_PPM_MIN;//VAL_PPM_MIN;
	OC3RS = VAL_PPM_MIN;//VAL_PPM_MIN;
	OC4RS = VAL_PPM_MIN;
}

void ServoControl::setMotor(uint8_t motor, uint16_t value)
{
	switch (motor) {
		case 0:
			OC1RS = value;
			break;
		case 1:
			OC2RS = value;
			break;
		case 2:
			OC3RS = value;
			break;
		case 3:
			OC4RS = value;
			break;
	}
}

extern "C" {

	void __ISR( _TIMER_2_VECTOR, ipl7) T2Interrupt( void)
	{
	   mT2ClearIntFlag();
	} // T2 Interrupt
}; // extern C
