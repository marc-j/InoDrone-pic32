/*
 * RadioChannel.h
 *
 *  Created on: 20 sept. 2013
 *      Author: alienx
 */

#ifndef RADIOCHANNEL_H_
#define RADIOCHANNEL_H_

#include <inttypes.h>

#define RC_TYPE_RANGE 0
#define RC_TYPE_ANGLE 1
#define RC_TYPE_SWITCH 2

#define RADIO_MIN 1400
#define RADIO_MAX 2400
#define RADIO_CENTER 1880

class RadioChannel {
public:
	RadioChannel();

	void setPwm(uint16_t pwm);
	void setRange(uint16_t min, uint16_t max);
	void setAngle(uint16_t angle);
	void setSwitch();

	int16_t constrainInt16(int16_t value, int16_t min, int16_t max);

	int16_t getRange();
	int16_t getAngle();
	uint8_t getBoolean();

	int16_t value;
	int8_t isOn;

private:
	uint16_t _radioIn;
	uint16_t _radioMax;
	uint16_t _radioMin;
	uint16_t _high;
	uint16_t _low;
	uint16_t _angle;

	uint8_t _type;
};

#endif /* RADIOCHANNEL_H_ */
