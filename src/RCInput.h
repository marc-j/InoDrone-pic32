/*
 * RCInput.h
 *
 *  Created on: 26 août 2013
 *      Author: alienx
 */

#ifndef RCINPUT_H_
#define RCINPUT_H_

#define INPUT_MAX_CHANNELS 8
#define VAL_PPM_3MS    (3*(__PIC32_pbClk/64))/1000


#include <inttypes.h>

class RCInput {

public:
	enum {
		CHANNEL_THROTTLE,
		CHANNEL_PITCH,
		CHANNEL_YAW,
		CHANNEL_ROLL
	};

	RCInput();
	void init();
	uint16_t read(uint8_t ch);

	static void interrupt(void);
private:
	static volatile uint16_t _channels[INPUT_MAX_CHANNELS];
	static volatile uint8_t _valid_channels;
};

//extern RCInput rc;

#endif /* RCINPUT_H_ */
