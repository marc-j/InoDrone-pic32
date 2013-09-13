/*
 * PID.cpp
 *
 *  Created on: 16 mai 2013
 *      Author: alienx
 */

#include "PID.h"

PID::PID(float kP, float kI, float kD) {
	this->kP = kP;
	this->kI = kI;
	this->kD = kD;

}

void PID::resetIntegral()
{
	integral = 0;
}

void PID::setKp(float kP)
{
	this->kP = kP;
}

void PID::setKi(float kI)
{
	this->kI = kI;
}

void PID::setKd(float kD)
{
	this->kD = kD;
}

float PID::calculate(float error, float G_Dt) {

	float P = error * kP;

	integral += (error*G_Dt);
	float I = integral*kI;

	float errorDiff = error - lastError;
	lastError = error;

	float D = errorDiff * kD;

	return P + I + D;
}

