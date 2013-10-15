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

	integral = 0;

}

void PID::resetIntegral()
{
	integral = 0;
}

void PID::setKp(float kP)
{
	this->kP = kP;
}

float PID::getKp()
{
	return this->kP;
}

void PID::setKi(float kI)
{
	this->kI = kI;
}

float PID::getKi()
{
	return this->kI;
}

void PID::setKd(float kD)
{
	this->kD = kD;
}

float PID::getKd()
{
	return this->kD;
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

