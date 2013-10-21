/*
 * Kinematics.h
 *
 *  Created on: 19 oct. 2013
 *      Author: alienx
 */

#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include "Matrix.h"

void initKinematics();
void computeKinematics(float rollRate, float pitchRate, float yawRate,
						 float rollAccel, float pitchAccel, float zAccel,
						 float magX, float magY, float magZ,
						 float G_Dt);
vector3f getEulerAngles();

#endif /* KINEMATICS_H_ */
