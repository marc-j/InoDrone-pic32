/*
 * PID.h
 *
 *  Created on: 16 mai 2013
 *      Author: alienx
 */

#ifndef PID_H_
#define PID_H_

class PID {
public:
	PID(float kP, float kI, float kD);
	void resetIntegral();
	float calculate(float error, float G_Dt);

	void setKp(float kP);
	void setKi(float kI);
	void setKd(float kD);
private:
	float kP;
	float kI;
	float kD;

	float integral;
	float lastError;
};

#endif /* PID_H_ */
