/*
 * UAV.h
 *
 *  Created on: 27 avr. 2013
 *      Author: alienx
 */

#ifndef UAV_H_
#define UAV_H_

enum {
	FLIGHTMODE_WAITING,
	FLIGHTMODE_VARIANCE,
	FLIGHTMODE_COMPASS_CALIBRATION,
	FLIGHTMODE_NORMAL
};

typedef struct _UAV {
	uint8_t flightmode;
	uint8_t landing;
	struct _CMD {
		int16_t roll;
		int16_t pitch;
		int16_t yaw;
		int16_t throttle;
	} CMD;
} UAV;

#endif /* UAV_H_ */
