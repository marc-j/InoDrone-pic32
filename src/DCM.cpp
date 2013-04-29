/*
 * DCM.cpp
 *
 *  Created on: 27 janv. 2013
 *      Author: marc
 */

#include "DCM.h"

DCM::DCM()
{
  dcmMatrix = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
  omegaP = {0.0,0.0,0.0};
  omegaI = {0.0,0.0,0.0};
  omega = {0.0,0.0,0.0};
  errorCourse = 0.0;
  kpRollPitch = 0.0;
  kiRollPitch = 0.0;
  kpYaw = 0.0;
  kiYaw = 0.0;
  correctedRateVector = {0.0,0.0,0.0};
  kinematicsAngle = {0.0,0.0,0.0};
  earthAccel = {0.0,0.0,0.0};
}

void DCM::init(float hdgX, float hdgY)
{
  kinematicsAngle = {0.0,0.0,0.0};
  for (uint8_t i=0; i<3; i++) {
    omegaP[i] = 0;
    omegaI[i] = 0;
  }
  dcmMatrix[0] =  hdgX;
  dcmMatrix[1] = -hdgY;
  dcmMatrix[2] =  0;
  dcmMatrix[3] =  hdgY;
  dcmMatrix[4] =  hdgX;
  dcmMatrix[5] =  0;
  dcmMatrix[6] =  0;
  dcmMatrix[7] =  0;
  dcmMatrix[8] =  1;

  kpRollPitch = 0.1;        // alternate 0.05;
  kiRollPitch = 0.0002;     // alternate 0.0001;

  kpYaw = -0.1;             // alternate -0.05;
  kiYaw = -0.0002;          // alternate -0.0001;
}

void DCM::matrixUpdate(float rollRate, float pitchRate, float yawRate, float G_Dt)
{
  float rateGyroVector[3];
  float temporaryMatrix[9];
  float updateMatrix[9];

  rateGyroVector[XAXIS]  = rollRate;
  rateGyroVector[YAXIS]  = pitchRate;
  rateGyroVector[ZAXIS]  = yawRate;

  vectorSubtract(3, &omega[XAXIS], &rateGyroVector[XAXIS], &omegaI[XAXIS]);
  vectorSubtract(3, &correctedRateVector[XAXIS], &omega[XAXIS], &omegaP[XAXIS]);

  updateMatrix[0] =  0;
  updateMatrix[1] = -G_Dt * correctedRateVector[ZAXIS];    // -r
  updateMatrix[2] =  G_Dt * correctedRateVector[YAXIS];  //  q
  updateMatrix[3] =  G_Dt * correctedRateVector[ZAXIS];    //  r
  updateMatrix[4] =  0;
  updateMatrix[5] = -G_Dt * correctedRateVector[XAXIS];   // -p
  updateMatrix[6] = -G_Dt * correctedRateVector[YAXIS];  // -q
  updateMatrix[7] =  G_Dt * correctedRateVector[XAXIS];   //  p
  updateMatrix[8] =  0;

  matrixMultiply(3, 3, 3, temporaryMatrix, dcmMatrix, updateMatrix);
  matrixAdd(3, 3, dcmMatrix, dcmMatrix, temporaryMatrix);
}

void DCM::normalize()
{
  float error=0;
  float temporary[9];
  float renorm=0;

  error= -vectorDotProduct(3, &dcmMatrix[0], &dcmMatrix[3]) * 0.5;         // eq.18

  vectorScale(3, &temporary[0], &dcmMatrix[3], error);                     // eq.19
  vectorScale(3, &temporary[3], &dcmMatrix[0], error);                     // eq.19

  vectorAdd(6, &temporary[0], &temporary[0], &dcmMatrix[0]);               // eq.19

  vectorCrossProduct(&temporary[6],&temporary[0],&temporary[3]);           // eq.20

  for(uint8_t v=0; v<9; v+=3) {
    renorm = 0.5 *(3 - vectorDotProduct(3, &temporary[v],&temporary[v]));  // eq.21
    vectorScale(3, &dcmMatrix[v], &temporary[v], renorm);
  }
}

void DCM::driftCorrection(float longAccel, float latAccel, float vertAccel, float oneG, float magX, float magY)
{
  //  Compensation of the Roll, Pitch and Yaw drift.
  float accelMagnitude;
  float accelVector[3];
  float accelWeight;
  float errorRollPitch[3];
  #ifdef HeadingMagHold
    float errorCourse;
    float errorYaw[3];
    float scaledOmegaP[3];
  #endif
  float scaledOmegaI[3];

  //  Roll and Pitch Compensation
  accelVector[XAXIS] = longAccel;
  accelVector[YAXIS] = latAccel;
  accelVector[ZAXIS] = vertAccel;

  // Calculate the magnitude of the accelerometer vector
  accelMagnitude = (sqrt(accelVector[XAXIS] * accelVector[XAXIS] +
                         accelVector[YAXIS] * accelVector[YAXIS] +
                         accelVector[ZAXIS] * accelVector[ZAXIS])) / oneG;

  // Weight for accelerometer info (<0.75G = 0.0, 1G = 1.0 , >1.25G = 0.0)
  // accelWeight = constrain(1 - 4*abs(1 - accelMagnitude),0,1);

  // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
  accelWeight = constrain(1 - 2 * abs(1 - accelMagnitude), 0, 1);

  vectorCrossProduct(&errorRollPitch[0], &accelVector[0], &dcmMatrix[6]);
  vectorScale(3, &omegaP[0], &errorRollPitch[0], kpRollPitch * accelWeight);

  vectorScale(3, &scaledOmegaI[0], &errorRollPitch[0], kiRollPitch * accelWeight);
  vectorAdd(3, omegaI, omegaI, scaledOmegaI);

  //  Yaw Compensation
  #ifdef HeadingMagHold
    errorCourse = (dcmMatrix[0] * magY) - (dcmMatrix[3] * magX);
    vectorScale(3, errorYaw, &dcmMatrix[6], errorCourse);

    vectorScale(3, &scaledOmegaP[0], &errorYaw[0], kpYaw);
    vectorAdd(3, omegaP, omegaP, scaledOmegaP);

    vectorScale(3, &scaledOmegaI[0] ,&errorYaw[0], kiYaw);
    vectorAdd(3, omegaI, omegaI, scaledOmegaI);
  #else
    omegaP[ZAXIS] = 0.0;
    omegaI[ZAXIS] = 0.0;
  #endif
}

void DCM::eulerAngles()
{
  kinematicsAngle[XAXIS]  =  atan2(dcmMatrix[7], dcmMatrix[8]);
  kinematicsAngle[YAXIS] =  -asin(dcmMatrix[6]);
  kinematicsAngle[ZAXIS]   =  atan2(dcmMatrix[3], dcmMatrix[0]);
}

void DCM::earthAxisAccels(float ax, float ay, float az, float oneG)
{
  float accelVector[3];

  accelVector[XAXIS] = ax;
  accelVector[YAXIS] = ay;
  accelVector[ZAXIS] = az;

  earthAccel[XAXIS] = vectorDotProduct(3, &dcmMatrix[0], &accelVector[0]);
  earthAccel[YAXIS] = vectorDotProduct(3, &dcmMatrix[3], &accelVector[0]);
  earthAccel[ZAXIS] = vectorDotProduct(3, &dcmMatrix[6], &accelVector[0]) + oneG;
}

void DCM::calculate(float rollRate, float pitchRate, float yawRate,
                 float longAccel, float latAccel, float vertAccel,
                 float oneG, float magX, float magY,
                 float G_Dt)
{
    matrixUpdate(rollRate, pitchRate, yawRate, G_Dt);
    normalize();
    driftCorrection(longAccel, latAccel, vertAccel, oneG, magX, magY);
    eulerAngles();
    earthAxisAccels(longAccel, latAccel, vertAccel, oneG);
}

void DCM::getEuler(float *angles)
{
  angles[XAXIS] = kinematicsAngle[XAXIS];
  angles[YAXIS] = kinematicsAngle[YAXIS];
  angles[ZAXIS] = kinematicsAngle[ZAXIS];
}
