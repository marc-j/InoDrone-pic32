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

#define TO_RAD(x) (x * 0.01745329252)
#define TO_DEG(x) (x * 57.2957795131)

/******************************************************************
 * ###################### USER CONFIG ############################
 ******************************************************************/
//#define _IMU_ARG
#define USE_MAGNETOMETER
#define _IMU_AHRS

#define FLIGHTMODE_RATE 0
#define FLIGHTMODE_ATTITUDE 1

//#define SONAR_SRF05
#define SONAR_MAX
//#define SONAR_TRIGGERPIN_TRIS TRISFbits.TRISF3

//#define DEBUG

/******************************************************************
 * SENSOR CALIBRATION
 ******************************************************************/

// Accelerometer
#define GRAVITY  8192.0f //MPU 6050 FS_SEL=1 (4g), 8192 LSB/g

#define ACC_X_MIN -8196.0f
#define ACC_X_MAX 8307.0f
#define ACC_Y_MIN -8103.0f
#define ACC_Y_MAX 8290.0f
#define ACC_Z_MIN -8358.0f
#define ACC_Z_MAX 8939.0f

#define ACC_X_OFFSET 55.5f  //((ACC_X_MIN + ACC_X_MAX) / 2.0f)
#define ACC_Y_OFFSET ((ACC_Y_MIN + ACC_Y_MAX) / 2.0f)
#define ACC_Z_OFFSET ((ACC_Z_MIN + ACC_Z_MAX) / 2.0f)
#define ACC_X_SCALE 0.99278919f //(GRAVITY / (ACC_X_MAX - ACC_X_OFFSET))
#define ACC_Y_SCALE (GRAVITY / (ACC_Y_MAX - ACC_Y_OFFSET))
#define ACC_Z_SCALE (GRAVITY / (ACC_Z_MAX - ACC_Z_OFFSET))

// Magnetometer
#define MAG_X_MAX 579
#define MAG_X_MIN -527
#define MAG_Y_MAX 492
#define MAG_Y_MIN -438
#define MAG_Z_MAX 424
#define MAG_Z_MIN -460

#define MAG_X_OFFSET ((MAG_X_MIN + MAG_X_MAX) / 2.0f)
#define MAG_Y_OFFSET ((MAG_Y_MIN + MAG_Y_MAX) / 2.0f)
#define MAG_Z_OFFSET ((MAG_Z_MIN + MAG_Z_MAX) / 2.0f)
#define MAG_X_SCALE (100.0f / (MAG_X_MAX - MAG_X_OFFSET))
#define MAG_Y_SCALE (100.0f / (MAG_Y_MAX - MAG_Y_OFFSET))
#define MAG_Z_SCALE (100.0f / (MAG_Z_MAX - MAG_Z_OFFSET))

// Gyroscopes
#define GYRO_GAIN 32.8f // 32.8f * toRad; // 32.8 LSB/°/s (+/- 1000 °/s)
#define GYRO_SCALED(x) (x * TO_RAD(GYRO_GAIN))

/******************************************************************
 * ################################################################
 ******************************************************************/

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
	struct _ATTITUDE {
		float roll;
		float pitch;
		float yaw;
	} ATTITUDE;
	PID* rollPID;
	PID* pitchPID;
	PID* yawPID;
	Radio* rc;
} UAV;

#endif /* UAV_H_ */
