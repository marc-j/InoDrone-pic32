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
	unsigned long safe_timer;
	uint8_t flightmode;
	uint8_t takeoff;
	struct _CMD {
		int16_t roll;
		int16_t pitch;
		int16_t yaw;
		int16_t throttle;
	} CMD;
	struct _MOTOR {
		uint16_t FL;
		uint16_t FR;
		uint16_t RL;
		uint16_t RR;
	} MOTOR;
} UAV;

#endif /* UAV_H_ */
