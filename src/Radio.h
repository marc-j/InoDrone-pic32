/*
 * Radio.h
 *
 *  Created on: 20 sept. 2013
 *      Author: alienx
 */

#ifndef RADIO_H_
#define RADIO_H_

#include <inttypes.h>
#include "RCInput.h"
#include "RadioChannel.h"

#define RC_THROTTLE 0
#define RC_ROLL 1
#define RC_PITCH 2
#define RC_YAW 3
#define RC_GEAR 4
#define RC_AUX1 5

class Radio {
public:
	Radio();

	void update();

	RadioChannel roll;
	RadioChannel pitch;
	RadioChannel yaw;
	RadioChannel throttle;
	RadioChannel armed;

private:
	RCInput _input;
};

#endif /* RADIO_H_ */
