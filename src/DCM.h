/*
 * DCM.h
 *
 *  Created on: 27 janv. 2013
 *      Author: marc
 */

#ifndef DCM_H_
#define DCM_H_

#include "wiring.h"
#include "math.h"
#include "QCMath.h"
#include "inttypes.h"

//#define HeadingMagHold

class DCM
{
public:
  DCM();
  void init(float hdgX, float hdgY);
  void calculate(float rollRate, float pitchRate, float yawRate,
                   float longAccel, float latAccel, float vertAccel,
                   float oneG, float magX, float magY,
                   float G_Dt);
  void getEuler(float *angles);

  enum AXIS {
    XAXIS,
    YAXIS,
    ZAXIS
  };

private:
  void matrixUpdate(float rollRate, float pitchRate, float yawRate, float G_Dt);
  void normalize();
  void driftCorrection(float longAccel, float latAccel, float vertAccel, float oneG, float magX, float magY);
  void eulerAngles();
  void earthAxisAccels(float longAccel, float latAccel, float vertAccel, float oneG);

  float dcmMatrix[9];
  float omegaP[3];
  float omegaI[3];
  float omega[3];
  float errorCourse;
  float kpRollPitch;
  float kiRollPitch;
  float kpYaw;
  float kiYaw;
  float correctedRateVector[3];
  float kinematicsAngle[3];
  float earthAccel[3];
};

#endif /* DCM_H_ */
