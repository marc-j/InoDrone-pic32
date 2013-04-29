/*
 * Kalman.h
 *
 *  Created on: 24 avr. 2013
 *      Author: alienx
 */

#ifndef KALMAN_H_
#define KALMAN_H_

#include "Matrix.h"

class Kalman {
public:
	Kalman();
	void setAccValues(vector3f acc);
	void setGyroValues(vector3f gyro, float G_Dt);
	vector3f compute();

private:
	Matrix xk;		// Predicted state k at k-1
	Matrix xnew;    // New measured state
	Matrix uk;      // rate k

	Matrix I;		// Identity matrix
	Matrix Pk;		// Covariance matrix
	Matrix R;		// Measurement noise (linked to acc)
	Matrix Q;		// Processing noise (linked to gyroscope)

	Matrix S;
	Matrix K;		// Kalman Gain

};

#endif /* KALMAN_H_ */
