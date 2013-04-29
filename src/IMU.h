/*
 * IMU.h
 *
 *  Created on: 19 janv. 2013
 *  Author: Marc Jacquier
 */

#ifndef IMU_H_
#define IMU_H_

#include "MPU6050.h"
#include "DCM.h"
#include "Kalman.h"
#include "Matrix.h"

#define twoKpDef  (2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef  (2.0f * 0.1f) // 2 * integral gain

#define toDeg 57.29
#define toRad 0.0174532925
/*
 * [-612.33327827112589, 1679.4716140499254, 341.26608457168271]
 * [12749.228483890431, 16741.059282305599, 20494.275611442848]
 */
#define GRAVITY 1671.0f

#define ACC_RES 2
#define ACC_SCALE_X 0.0849344f
#define ACC_SCALE_Y 0.0889208f
#define ACC_SCALE_Z 0.0801823f

#define ACC_OFFSET_X 1806
#define ACC_OFFSET_Y -352
#define ACC_OFFSET_Z 1192

/*
    [12.952730780929429, 13.177076410003499, -51.21240188153007]
    [191.62254532125752, 163.31965459240323, 194.61254306359552]
 */
#define MAG_SCALE_X 191.62254532125752f
#define MAG_SCALE_Y 163.31965459240323f
#define MAG_SCALE_Z 194.61254306359552f

#define MAG_OFFSET_X 13
#define MAG_OFFSET_Y 13
#define MAG_OFFSET_Z -51

class IMU {
public:
  IMU();

  typedef struct __motion9f {
    vector3f ACC;
    vector3f GYRO;
    vector3f MAG;
  } motion9f;

  typedef struct __attitude12f {
    vector3f ACC;
    vector3f GYRO;
    vector3f MAG;
    struct {
      float roll;
      float pitch;
      float yaw;
    } EULER;
  } attitude12f;

  void init();
  void getMotion9(motion9f*);
  void getRawValues(int16_t*);

  void getAttitude(attitude12f*);
  void getMag(float roll, float pitch);

  vector3f getAccelMeasurementNoise();
  vector3f getGyroMeasurementNoise();

private:
  void zeroGyro();
  void zeroAccel();
  vector3f getAccelAngle(vector3f *acc);

  float getHeading(vector3f acc, vector3f mag);

  MPU6050 mpu6050;
#ifdef _WITH_DCM_
  DCM dcm;
#else
  Kalman* kalman;
#endif

  vector9 sensorsRaw;
  vector3 gyroOffset;
  vector3 accOffset;

  motion9f sensorsSens;

  volatile float q0, q1, q2, q3;
  float G_Dt;
  volatile float twoKp;      // 2 * proportional gain (Kp)
  volatile float twoKi;      // 2 * integral gain (Ki)
  volatile float integralFBx,  integralFBy, integralFBz;
  unsigned long lastUpdate, now; // sample period expressed in milliseconds
  float accelOneG;

  float hdgX, hdgY;
};

#endif /* IMU_H_ */
