/*
 * Radio.cpp
 *
 *  Created on: 20 sept. 2013
 *      Author: alienx
 */

#include "Radio.h"

Radio::Radio() {

	// Init rc input reader
	_input.init();

	roll.setAngle(45);
	pitch.setAngle(45);
	yaw.setAngle(90);

	throttle.setRange(1300, 2500);

	armed.setSwitch();

}

void Radio::update()
{
	roll.setPwm( _input.read(RC_ROLL) );
	pitch.setPwm( _input.read(RC_PITCH) );
	yaw.setPwm( _input.read(RC_YAW) );
	throttle.setPwm( _input.read(RC_THROTTLE));
	armed.setPwm( _input.read(RC_AUX1));
}



