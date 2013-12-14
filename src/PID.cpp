/*
 * PID.cpp
 *
 *  Created on: 16 mai 2013
 *      Author: alienx
 */

#include "PID.h"

PID::PID(float kP, float kI, float kD, float kPStab, float kIStab)
{
	this->kP = kP;
	this->kI = kI;
	this->kD = kD;

	this->kPStab = kPStab;
	this->kIStab = kIStab;

	integral = 0;
	lastError = 0;

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

void PID::setKpStab(float kP)
{
    this->kPStab = kP;
}

float PID::getKpStab()
{
    return this->kPStab;
}

void PID::setKiStab(float kI)
{
    this->kIStab = kI;
}

float PID::getKiStab()
{
    return this->kIStab;
}

float PID::calculate(float target, float current, float G_Dt) {

	float error = target - current;

	integral += (error*G_Dt);
	if (integral > PID_I_GUARD) integral = PID_I_GUARD;
	if (integral < -PID_I_GUARD) integral = -PID_I_GUARD;

	float errorDiff = error - lastError;
	lastError = error;

	return (error * kP) + (integral * kI) + (errorDiff * kD);
}

float PID::calculate(float target, float angle, float gyro, float G_Dt) {

    float errorAngle = target - angle;

    integral += (errorAngle*G_Dt);
    if (integral > PID_I_GUARD) integral = PID_I_GUARD;
    if (integral < -PID_I_GUARD) integral = -PID_I_GUARD;


    float errorSum = gyro - lastError;
    lastError = gyro;


    return ((errorAngle * kPStab) - (gyro*kP)) + (integral * kIStab) - (errorSum * kD);
}

