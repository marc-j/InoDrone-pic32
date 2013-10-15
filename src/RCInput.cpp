/*
 * RCInput.cpp
 *
 *  Created on: 26 août 2013
 *      Author: alienx
 */

#include "RCInput.h"
#include <plib.h>

#include <wiring.h>
#include <wiring_private.h>

volatile uint16_t RCInput::_channels[INPUT_MAX_CHANNELS] = {0};
volatile uint8_t RCInput::_valid_channels = 0;

RCInput::RCInput() {}

void RCInput::init()
{
	mIC5ClearIntFlag();

	// Enable Input Capture Module 5
	// - Capture every rising edge
	// - Enable capture interupts
	// - use Timer 2 source
    ConfigIntCapture5( IC_INT_ON | IC_INT_PRIOR_1 | IC_INT_SUB_PRIOR_2);
    OpenCapture5( IC_EVERY_RISE_EDGE | IC_INT_1CAPTURE | IC_TIMER2_SRC | IC_ON);
}

uint16_t RCInput::read(uint8_t channel)
{
	uint16_t capture = RCInput::_channels[channel];

	return capture;
}

void RCInput::interrupt()
{
	static uint8_t selectedChannel;
	static uint16_t channelOld;

	const uint16_t current = IC5BUF;

	uint16_t pulseWidth = 0;

    if (current < channelOld) {
        pulseWidth = current + 0xFFFF - channelOld;
    } else {
        pulseWidth = current - channelOld;
    }

	if ( pulseWidth > VAL_PPM_3MS ) {
		selectedChannel = 0;
	} else {
		if (selectedChannel < INPUT_MAX_CHANNELS) {
			_channels[selectedChannel] = pulseWidth;
			selectedChannel++;
			if (selectedChannel == INPUT_MAX_CHANNELS) {
				_valid_channels = INPUT_MAX_CHANNELS;
			}
		}
	}

	channelOld = current;

}

extern "C" {

	void __ISR( _INPUT_CAPTURE_5_VECTOR, ipl1) IC5Interrupt( void)
	{
	   RCInput::interrupt();

	   mIC5ClearIntFlag();
	} // IC5 Interrupt
}; // extern C
