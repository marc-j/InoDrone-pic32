/*
 * QuadCopter.h
 *
 *  Created on: 19 janv. 2013
 *      Author: marc
 */

#ifndef QUADCOPTER_H_
#define QUADCOPTER_H_

#include <plib.h>
#include "WProgram.h"
#include "IMU.h"
#include "RCInput.h"
#include "ServoControl.h"
#include "Protocol.h"
#include "UAVLink.h"
#include "PID.h"
#include "UAV.h"
#include "ISRRegistry.h"


#include "Kinematics.h"
#if defined(_IMU_ARG)
	#include "Kinematics_ARG.h"
#elif defined(_IMU_AHRS)
    #include "Kinematics_AHRS.h"
#else
	#error No kenamtics defined for IMU
#endif

#include "Sonar.h"
#ifdef SONAR_MAX
	#define SONAR
	#include "MaxSonar.h"
#endif

#define LEDRED_ON PORTFbits.RF3 = 1;
#define LEDRED_OFF PORTFbits.RF3 = 0;

#define CUTOFF STOP_MOTOR; uav.takeoff = 0; LEDRED_OFF;
#define ROLL 1
#define PITCH 2
#define YAW 3

/********************************
 MOTOR

 MFL   \   /  MFR
        \ /
         X
        / \
 MRL   /   \  MRR

*************************************/
#define STOP_MOTOR uav.MOTOR = {VAL_PPM_MIN, VAL_PPM_MIN, VAL_PPM_MIN, VAL_PPM_MIN}
#define mapMotorCmd(x)	map(x, 1300, 2500, VAL_PPM_MIN, VAL_PPM_MAX)
#define MIX(X,Y,Z) thrust + stabRoll*X + stabPitch*Y + stabYaw*Z
#define MAX_THROTTLE 2500

// Define OC(x)
#define MFL 0
#define MFR 1
#define MRL 2
#define MRR 3
/*****************************************/

IMU::attitude12f attitude;

unsigned long mainTimer=0;
unsigned long mainTimerOld=0;
unsigned long mainTimerEnd=0;
unsigned long mainTimerElapsed = 0;
unsigned long loopCounter = 0;

unsigned long task100HZtimer=0;
unsigned long task10HZtimer=0;

float G_Dt	= 0.01; //Integration time

float cpu_load=0;

IMU* imu;
UAV uav;
ServoControl motor;
ISRRegistry *isrRegistry;


/**
 * Async BLINK LED (No blocking function)
 */
void blinkLED(uint8_t t, uint8_t r)
{
	uav.LED.repeat = r;
	uav.LED.time = t;
	uav.LED.millis = 0;
	PORTCbits.RC3 = 0;
}

void blinkLEDUpdate()
{
	if (uav.LED.repeat == 0) return;

	if ( (millis()-uav.LED.millis) >= uav.LED.time ) {
		PORTCbits.RC3 = !PORTCbits.RC3;
		uav.LED.millis = millis();
		uav.LED.repeat--;
	}
}


//void receiveMessage(uavlink_message_t msg);


#endif /* QUADCOPTER_H_ */
