/*
 * Kalman.cpp
 *
 *  Created on: 24 avr. 2013
 *  Author: Marc Jacquier
 */

#include "Kalman.h"

Kalman::Kalman()
{
	xk(3,3);
	xnew(3,3);
	uk(3,3);

	I(3,3);
	Pk(3,3);
	R(3,3);
	Q(3,3);
	S(3,3);
	K(3,3);

	// Matrix initializations
	Pk.mat[0][0] = 1;
	Pk.mat[1][1] = 1;
	Pk.mat[2][2] = 1;
	// Identity Matrix
	I.mat[0][0] = 1;
	I.mat[1][1] = 1;
	I.mat[2][2] = 1;

	// COVARIANCE NOISE //
	// Accelerometer (see IMU::getAccelMeasurementNoise)
	R.mat[0][0] = 0.026f;
	R.mat[1][1] = 0.028f;
	R.mat[2][2] = 0.025f;
	// Gyroscope (see IMU::getGyroMeasurementNoise)
	Q.mat[0][0] = 0.005f;
	Q.mat[1][1] = 0.005f;
	Q.mat[2][2] = 0.0001f;
	/*
	Q.mat[0][0] = 0.00007;
	Q.mat[1][1] = 0.0001;
	Q.mat[2][2] = 0.0002;
	 */
}

void Kalman::setAccValues(vector3f acc)
{
	xnew.mat[0][0] = acc.x;
	xnew.mat[1][1] = acc.y;
	xnew.mat[2][2] = acc.z;
}

void Kalman::setGyroValues(vector3f gyro, float G_Dt)
{
	uk.mat[0][0] = gyro.x * G_Dt;
	uk.mat[1][1] = gyro.y * G_Dt;
	uk.mat[2][2] = gyro.z * G_Dt;
}

vector3f Kalman::compute()
{
	vector3f angles;

	// Observation
	Matrix yk(3,3);

	// Prediction
	xk = xk + uk; // state estimate
	Pk = Pk + Q;  // estimate covariance

	// Update
	yk = xnew - xk; // Innovation or measurement residual
	/*if (yk.mat[0][0]>320) { yk.mat[0][0] = yk.mat[0][0]-360; }
	if (yk.mat[0][0]<320) { yk.mat[0][0] = yk.mat[0][0]+360; }

	if (yk.mat[1][1]>320) { yk.mat[1][1] = yk.mat[1][1]-360; }
	if (yk.mat[1][1]<320) { yk.mat[1][1] = yk.mat[1][1]+360; }

	if (yk.mat[2][2]>320) { yk.mat[2][2] = yk.mat[2][2]-360; }
	if (yk.mat[2][2]<320) { yk.mat[2][2] = yk.mat[2][2]+360; }*/

	// Inovation covariance
	S = Pk + R;
	// Optimal Kalman Gain
	K = Pk * ( S.invert_3x3() );
	// Updated state estimated
	xk = xk + (K * yk);
	// Updated estimate covariance
	Pk = ( I - K ) * (Pk);

	angles.x = xk(0,0);
	angles.y = xk(1,1);
	angles.z = xk(2,2);

	return angles;
}
