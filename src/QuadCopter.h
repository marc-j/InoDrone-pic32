/*
 * QuadCopter.h
 *
 *  Created on: 19 janv. 2013
 *      Author: marc
 */

#ifndef QUADCOPTER_H_
#define QUADCOPTER_H_

#include "IMU.h"
#include "ServoControl.h"
#include "Protocol.h"
#include "UAVLink.h"
#include "PID.h"
#include "UAV.h"

#define CUTOFF STOP_MOTOR; uav.takeoff = 0;
#define ROLL 1
#define PITCH 2
#define YAW 3

/********************************
 MOTOR

 MLF   \   /  MFR
        \ /
         X
        / \
 MRL   /   \  MRR

*************************************/
#define STOP_MOTOR uav.MOTOR = {VAL_PPM_MIN, VAL_PPM_MIN, VAL_PPM_MIN, VAL_PPM_MIN}
#define mapMotorCmd(x)	map(x, 1000, 2000, VAL_PPM_MIN, VAL_PPM_MAX)

// Define OC(x)
#define MFL 0
#define MFR 3
#define MRL 2
#define MRR 1
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
