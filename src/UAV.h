/*
 * UAV.h
 *
 *  Created on: 27 avr. 2013
 *      Author: alienx
 */

#ifndef UAV_H_
#define UAV_H_

#include <plib.h>
#include "WProgram.h"
#include "PID.h"
#include "Radio.h"

#define _IMU_ARG
//#define USE_MAGNETOMETER
//#define _IMU_AHRS

#define FLIGHTMODE_RATE 0
#define FLIGHTMODE_ATTITUDE 1

enum {
	FLIGHTMODE_WAITING,
	FLIGHTMODE_VARIANCE,
	FLIGHTMODE_COMPASS_CALIBRATION,
	FLIGHTMODE_NORMAL,
	FLIGHTMODE_ESC_CALIBRATION
};

typedef struct _UAV {
	unsigned long safe_timer;
	uint8_t flightmode;
	uint8_t takeoff;
	struct _MOTOR {
		uint16_t FL;
		uint16_t FR;
		uint16_t RL;
		uint16_t RR;
	} MOTOR;
	struct _LED {
		unsigned long millis;
		uint8_t repeat;
		uint8_t time;
		void (*startBlink)(uint8_t, uint8_t);
		void (*updateBlink)();
	} LED;
	PID* rollPID;
	PID* rollAttRatePID;
	PID* pitchPID;
	PID* pitchAttRatePID;
	PID* yawPID;
	Radio* rc;
} UAV;

#endif /* UAV_H_ */
