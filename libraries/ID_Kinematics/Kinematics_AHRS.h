/*
 * Kinematics_AHRS.h
 *
 *  Created on: 21 oct. 2013
 *      Author: alienx
 */

#ifndef KINEMATICS_AHRS_H_
#define KINEMATICS_AHRS_H_

#include "Kinematics.h"
#include "IDMath.h"

	volatile float KpMag = 0.0;
	volatile float Kp = 0.0;                					// proportional gain governs rate of convergence to accelerometer
	volatile float Ki = 0.0;                					// integral gain governs rate of convergence of gyroscope biases
	               					// half the sample period
	volatile float q0 = 0.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;       // quaternion elements representing the estimated orientation
	volatile float exInt,  eyInt, ezInt;
	volatile float prevEx, prevEy, prevEz;

	volatile float beta = 0.1f;

	void initKinematics(){

		  q0 = 1.0;
		  q1 = 0;
		  q2 = 0;
		  q3 = 0;
		  exInt = 0.0f;
		  eyInt = 0.0f;
		  ezInt = 0.0f;

		  Kp = 2.0f;
		  Ki = 0.005f;

		  KpMag = 0.2f;

		  /*kpAcc = 0.2;
		  kiAcc = 0.0005;

		  kpMag = 0.2;//2.0;
		  kiMag = 0.0005;//0.005;*/
	};

	//=====================================================================================================
	// AHRS.c
	// S.O.H. Madgwick
	// 25th August 2010
	//=====================================================================================================
	// Description:
	//
	// Quaternion implementation of the 'DCM filter' [Mayhony et al].  Incorporates the magnetic distortion
	// compensation algorithms from my filter [Madgwick] which eliminates the need for a reference
	// direction of flux (bx bz) to be predefined and limits the effect of magnetic distortions to yaw
	// axis only.
	//
	// User must define 'halfT' as the (sample period / 2), and the filter gains 'Kp' and 'Ki'.
	//
	// Global variables 'q0', 'q1', 'q2', 'q3' are the quaternion elements representing the estimated
	// orientation.  See my report for an overview of the use of quaternions in this application.
	//
	// User must call 'AHRSupdate()' every sample period and parse calibrated gyroscope ('gx', 'gy', 'gz'),
	// accelerometer ('ax', 'ay', 'ay') and magnetometer ('mx', 'my', 'mz') data.  Gyroscope units are
	// radians/second, accelerometer and magnetometer units are irrelevant as the vector is normalised.
	//
	//=====================================================================================================

	/*void computeKinematics(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float G_Dt)
	{
		  float recipNorm;
		  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
		  float ex = 0.0f, ey = 0.0f, ez = 0.0f, ezMag = 0.0f;
		  float halfT = G_Dt/2;

		  // Auxiliary variables to avoid repeated arithmetic
		  q0q0 = q0 * q0;
		  q0q1 = q0 * q1;
		  q0q2 = q0 * q2;
		  q0q3 = q0 * q3;
		  q1q1 = q1 * q1;
		  q1q2 = q1 * q2;
		  q1q3 = q1 * q3;
		  q2q2 = q2 * q2;
		  q2q3 = q2 * q3;
		  q3q3 = q3 * q3;

		  // Use magnetometer measurement only when valid (avoids NaN in magnetometer normalisation)
		  if((mx != 0.0f) && (my != 0.0f) && (mz != 0.0f)) {
		    float hx, hy, hz, bx, bz;
		    float wx, wy, wz;

		    // Normalise magnetometer measurement
		    recipNorm = sqrtf(mx * mx + my * my + mz * mz);
		    mx /= recipNorm;
		    my /= recipNorm;
		    mz /= recipNorm;

		    // compute reference direction of flux
		    hx = mx * 2*(0.5 - q2q2 - q3q3) + my * 2*(q1q2 - q0q3)       + mz * 2*(q1q3 + q0q2);
		    hy = mx * 2*(q1q2 + q0q3)       + my * 2*(0.5 - q1q1 - q3q3) + mz * 2*(q2q3 - q0q1);
		    hz = mx * 2*(q1q3 - q0q2)       + my * 2*(q2q3 + q0q1)       + mz * 2*(0.5 - q1q1 - q2q2);
		    bx = sqrt((hx*hx) + (hy*hy));
		    bz = hz;

		    // Estimated direction of magnetic field
		    wx = bx * 2*(0.5 - q2q2 - q3q3) + bz * 2*(q1q3 - q0q2);
		    wy = bx * 2*(q1q2 - q0q3)       + bz * 2*(q0q1 + q2q3);
		    //wz = bx * 2*(q0q2 + q1q3)       + bz * 2*(0.5 - q1q1 - q2q2);

		    // Error is sum of cross product between estimated direction and measured direction of field vectors
		    //ex = (my * wz - mz * wy);
		    //ey = (mz * wx - mx * wz);
		    ezMag = (mx * wy - my * wx);
		  }

		  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		  if((ax != 0.0f) && (ay != 0.0f) && (az != 0.0f)) {
		    float vx, vy, vz;

		    // Normalise accelerometer measurement
		    recipNorm = sqrtf(ax * ax + ay * ay + az * az);
		    ax /= recipNorm;
		    ay /= recipNorm;
		    az /= recipNorm;

		    // Estimated direction of gravity
		    vx = 2 * (q1q3 - q0q2);
		    vy = 2 * (q0q1 + q2q3);
		    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

		    // Error is sum of cross product between estimated direction and measured direction of field vectors
			ex += (vy*az - vz*ay);
			ey += (vz*ax - vx*az);
			ez += (vx*ay - vy*ax);
		  }

		  // Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
		  if(ex != 0.0f && ey != 0.0f && ez != 0.0f) {
		    // Compute and apply integral feedback if enabled
			exInt = exInt + ex*Ki;
			if (isSwitched(prevEx, ex)) exInt = 0.0;
			prevEx = ex;

			eyInt = eyInt + ey*Ki;
			if (isSwitched(prevEy, ey)) eyInt = 0.0;
			prevEy = ey;

			ezInt = ezInt + ez*Ki;
			if (isSwitched(prevEz, ez)) ezInt = 0.0;
			prevEz = ez;

		    // Apply proportional feedback
		    gx += Kp * ex + exInt;
		    gy += Kp * ey + eyInt;
		    gz += Kp * ez + ezMag*KpMag + ezInt;
		  }

		  q0 += (-q1 * gx - q2 * gy - q3 * gz) * halfT;
		  q1 += (q0 * gx + q2 * gz - q3 * gy) * halfT;
		  q2 += (q0 * gy - q1 * gz + q3 * gx) * halfT;
		  q3 += (q0 * gz + q1 * gy - q2 * gx) * halfT;

		  // Normalise quaternion
		  recipNorm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
		  q0 /= recipNorm;
		  q1 /= recipNorm;
		  q2 /= recipNorm;
		  q3 /= recipNorm;

	}*/

	void computeKinematics(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float G_Dt)
	{
        float recipNorm;
        float s0, s1, s2, s3;
        float qDot1, qDot2, qDot3, qDot4;
        float hx, hy;
        float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

        // Rate of change of quaternion from gyroscope
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);


        if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

                // Normalise accelerometer measurement
                recipNorm = sqrtf(ax * ax + ay * ay + az * az);
                ax /= recipNorm;
                ay /= recipNorm;
                az /= recipNorm;

                // Auxiliary variables to avoid repeated arithmetic
                _2q0 = 2.0f * q0;
                _2q1 = 2.0f * q1;
                _2q2 = 2.0f * q2;
                _2q3 = 2.0f * q3;
                _4q0 = 4.0f * q0;
                _4q1 = 4.0f * q1;
                _4q2 = 4.0f * q2;
                _8q1 = 8.0f * q1;
                _8q2 = 8.0f * q2;
                q0q0 = q0 * q0;
                q1q1 = q1 * q1;
                q2q2 = q2 * q2;
                q3q3 = q3 * q3;

                // Gradient decent algorithm corrective step
                s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
                s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
                s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
                s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
                recipNorm = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
                s0 /= recipNorm;
                s1 /= recipNorm;
                s2 /= recipNorm;
                s3 /= recipNorm;

                // Apply feedback step
                qDot1 -= beta * s0;
                qDot2 -= beta * s1;
                qDot3 -= beta * s2;
                qDot4 -= beta * s3;
        }

        // Integrate rate of change of quaternion to yield quaternion
        q0 += qDot1 * G_Dt;
        q1 += qDot2 * G_Dt;
        q2 += qDot3 * G_Dt;
        q3 += qDot4 * G_Dt;

        // Normalise quaternion
        recipNorm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 /= recipNorm;
        q1 /= recipNorm;
        q2 /= recipNorm;
        q3 /= recipNorm;
	}

	vector3f getEulerAngles() {
		vector3f angles;

		angles.x = atan2(2 * (q0*q1 + q2*q3), 1 - 2 *(q1*q1 + q2*q2));
		angles.y = -asin(2 * (q1*q3 - q0*q2));//asin(2 * (q0*q2 - q1*q3));
		angles.z = atan2(2 * (q0*q3 + q1*q2), 1 - 2 *(q2*q2 + q3*q3));

        // to deg
       /* angles.x *= 57.29577f;
        angles.y *= 57.29577f;
        angles.z *= 57.29577f;*/

		return angles;
	}

#endif /* KINEMATICS_AHRS_H_ */
