/*
 * QuadCopter.h
 *
 *  Created on: 19 janv. 2013
 *      Author: marc
 */

#ifndef QUADCOPTER_H_
#define QUADCOPTER_H_

#include "IMU.h"
#include "RCInput.h"
#include "ServoControl.h"
#include "Protocol.h"
#include "UAVLink.h"
#include "PID.h"
#include "UAV.h"

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
#define MIX(X,Y,Z) thrust + stabRoll*X + stabPitch*Y
#define MAX_THROTTLE 2500

// Define OC(x)
#define MFL 0
#define MFR 1
#define MRL 2
#define MRR 3
/*****************************************/

IMU::attitude12f attitude;

unsigned long timerMain=0;
unsigned long timerMain_old=0;
float G_Dt	= 0.01; //Integration time
float cpu_load=0;
unsigned long timer_end=0;
unsigned long telemetryTimer=0;
unsigned long safe_timer=0;

IMU imu;
UAV uav;
ServoControl motor;

void receiveMessage(uavlink_message_t msg);


#endif /* QUADCOPTER_H_ */
