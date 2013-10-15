/*
 * RadioChannel.cpp
 *
 *  Created on: 20 sept. 2013
 *      Author: alienx
 */

#include "RadioChannel.h"

RadioChannel::RadioChannel() {
	// TODO Auto-generated constructor stub

}

void RadioChannel::setPwm(uint16_t pwm)
{
	_radioIn = pwm;
	isOn = 0;

	if (_type == RC_TYPE_RANGE) {
		value = getRange();
	} else if(_type == RC_TYPE_ANGLE) {
		value = getAngle();
	} else {
		isOn = getBoolean();
	}
}

void RadioChannel::setRange(uint16_t min, uint16_t max)
{
	_type = RC_TYPE_RANGE;
	_high = max;
	_low = min;
}

void RadioChannel::setAngle(uint16_t angle)
{
	_type = RC_TYPE_ANGLE;
	_angle = angle;
}

void RadioChannel::setSwitch()
{
	_type = RC_TYPE_SWITCH;
}

int16_t RadioChannel::getRange()
{
	int16_t rcIn = constrainInt16(_radioIn, RADIO_MIN, RADIO_MAX);

	if (rcIn > RADIO_MIN) {
		return (_low + ((long)(_high - _low) * (long)(rcIn - RADIO_MIN)) / (long)( RADIO_MAX - RADIO_MIN));
	} else {
		return _low;
	}
}

int16_t RadioChannel::getAngle()
{
	int16_t high_center = RADIO_CENTER + 15;
	int16_t low_center = RADIO_CENTER - 15;

	if ((low_center - RADIO_MIN) == 0 || (RADIO_MAX - high_center) ==0) {
		return 0;
	}

	if (_radioIn > high_center) {
		return ((long)_angle * (long)(_radioIn - high_center)) / (long)(RADIO_MAX - high_center);
	} else if (_radioIn < low_center) {
		return ((long)_angle * (long)(_radioIn - low_center)) / (long)(low_center - RADIO_MIN);
	} else {
		return 0;
	}
}

uint8_t RadioChannel::getBoolean()
{
	return _radioIn > RADIO_CENTER;
}

int16_t RadioChannel::constrainInt16(int16_t value, int16_t min, int16_t max)
{
	if (value > max) {
		return max;
	} else if (value < min) {
		return min;
	}

	return value;
}

