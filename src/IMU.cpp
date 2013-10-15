/*
 * IMU.cpp
 *
 *  Created on: 19 janv. 2013
 *  Author: Marc Jacquier
 */

#include "IMU.h"
#include "math.h"

IMU::IMU()
{
  // initialize quaternion
  q0 = 1.0f;
  q1 = 0.0f;
  q2 = 0.0f;
  q3 = 0.0f;
  twoKp = twoKpDef;
  twoKi = twoKiDef;
  integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;
  lastUpdate = 0;
  now = 0;
  G_Dt = 0;

  // Sensors sens
  // ACC
  sensorsSens.ACC.x = 1;
  sensorsSens.ACC.y = 1;
  sensorsSens.ACC.z = 1;
  // GYRO
  sensorsSens.GYRO.x = -1;
  sensorsSens.GYRO.y = 1;
  sensorsSens.GYRO.z = -1;
  // MAG
  sensorsSens.MAG.x = -1;
  sensorsSens.MAG.y = -1;
  sensorsSens.MAG.z = -1;

  magLimit.x = { -200.0f , 200.0f };
  magLimit.y = { -200.0f , 200.0f };
  magLimit.z = { -200.0f , 200.0f };

  accelOneG = GRAVITY;
  movavg_i = 0;

  baro = MS561101BA();
}

void IMU::init()
{
  Wire.begin();
  mpu6050.initialize();
  delay(5);



  // Configure magneto
  I2C::writeByte(HMC5883_ADDRESS, 0x00, 0x70); //num samples: 8 ; output rate: 15Hz ; normal measurement mode
  I2C::writeByte(HMC5883_ADDRESS, 0x01, 0x20); //configuration gain 1.3Ga
  I2C::writeByte(HMC5883_ADDRESS, 0x02, 0x00); //continuous Conversion Mode

  mpu6050.enableI2CAux();

  if (mpu6050.testConnection()) {
	  PORTCbits.RC3 = 1;
  }

  baro.init(MS561101BA_ADDR_CSB_LOW);
  delay(100);
  // populate movavg_buff before starting loop
  for(int i=0; i<MOVAVG_SIZE; i++) {
    movavg_buff[i] = baro.getPressure(MS561101BA_OSR_4096);
  }

#ifdef _WITH_DCM_
  dcm = DCM();

  #ifdef HeadingMagHold
  getMag(0.0, 0.0);
  dcm.init(hdgX, hdgY);
  #else
  dcm.init(1.0, 0.0);
  #endif
#else
  kalman = new Kalman();
#endif


  zeroGyro();
  zeroAccel();
}

void IMU::zeroGyro()
{
  const int samples = 1000;
  long tmpOffsets[] = {0,0,0};

  for (int i=0; i < samples; i++) {
    mpu6050.getRotation(&sensorsRaw.GYRO.y,&sensorsRaw.GYRO.x,&sensorsRaw.GYRO.z);
    tmpOffsets[0] += sensorsRaw.GYRO.x;
    tmpOffsets[1] += sensorsRaw.GYRO.y;
    tmpOffsets[2] += sensorsRaw.GYRO.z;
  }

  gyroOffset.x = tmpOffsets[0] / samples;
  gyroOffset.y = tmpOffsets[1] / samples;
  gyroOffset.z = tmpOffsets[2] / samples;

}

void IMU::zeroAccel()
{
  const int samples = 100;
  long tmpOffsets[] = {0,0,0};

  for (int i=0; i < samples; i++) {
    mpu6050.getAcceleration(&sensorsRaw.ACC.x,&sensorsRaw.ACC.y,&sensorsRaw.ACC.z);
    tmpOffsets[0] += sensorsRaw.ACC.x;
    tmpOffsets[1] += sensorsRaw.ACC.y;
    tmpOffsets[2] += sensorsRaw.ACC.z;
  }

  accOffset.x = tmpOffsets[0] / samples;
  accOffset.y = tmpOffsets[1] / samples;
  accOffset.z = tmpOffsets[2] / samples;

  accelOneG = abs( (accOffset.z * ACC_SCALE_Z) - 0.5 );
}

void IMU::getMotion9(IMU::motion9f* sensors)
{

  mpu6050.getMotion9 (&sensorsRaw.ACC.x, &sensorsRaw.ACC.y, &sensorsRaw.ACC.z,
                      &sensorsRaw.GYRO.y, &sensorsRaw.GYRO.x, &sensorsRaw.GYRO.z,
                      &sensorsRaw.MAG.x, &sensorsRaw.MAG.z, &sensorsRaw.MAG.y);

  /*sensors->ACC.x = ( (sensorsRaw.ACC.x - ACC_OFFSET_X) / ACC_SCALE_X ) * sensorsSens.ACC.x;
  sensors->ACC.y = ( (sensorsRaw.ACC.y - ACC_OFFSET_Y) / ACC_SCALE_Y ) * sensorsSens.ACC.y;
  sensors->ACC.z = ( (sensorsRaw.ACC.z - ACC_OFFSET_Z) / ACC_SCALE_Z ) * sensorsSens.ACC.z;*/

  sensors->ACC.x = ( (sensorsRaw.ACC.x - accOffset.x) / 16384.0f ) * sensorsSens.ACC.x; // 16384 LSB/g (+/- 2g)
  sensors->ACC.y = ( (sensorsRaw.ACC.y - accOffset.y) / 16384.0f ) * sensorsSens.ACC.y; // 16384 LSB/g (+/- 2g)
  sensors->ACC.z = ( (sensorsRaw.ACC.z - accOffset.z) / 16384.0f ) * sensorsSens.ACC.z + 1; // 16384 LSB/g (+/- 2g)

  sensors->GYRO.x = ( (sensorsRaw.GYRO.x - gyroOffset.x) / 16.4f ) * sensorsSens.GYRO.x; // 16.4 LSB/°/s (+/- 2000 °/s)
  sensors->GYRO.y = ( (sensorsRaw.GYRO.y - gyroOffset.y) / 16.4f ) * sensorsSens.GYRO.y; // 16.4 LSB/°/s (+/- 2000 °/s)
  sensors->GYRO.z = ( (sensorsRaw.GYRO.z - gyroOffset.z) / 16.4f ) * sensorsSens.GYRO.z; // 16.4 LSB/°/s (+/- 2000 °/s)

  sensors->MAG.x = ((sensorsRaw.MAG.x * sensorsSens.MAG.x) - MAG_OFFSET_X) / 1090.0f;
  sensors->MAG.y = ((sensorsRaw.MAG.y * sensorsSens.MAG.y) - MAG_OFFSET_Y) / 1090.0f;
  sensors->MAG.z = ((sensorsRaw.MAG.z * sensorsSens.MAG.z) - MAG_OFFSET_Z) / 1090.0f; // removed scale * 0.92 (scale for 1.3Ga)

  // Centered MAG
  // Auto recalculate center
  /*magLimit.x.min = min(sensors->MAG.x, magLimit.x.min);
  magLimit.x.max = max(sensors->MAG.x, magLimit.x.max);
  magLimit.y.min = min(sensors->MAG.y, magLimit.y.min);
  magLimit.y.max = max(sensors->MAG.y, magLimit.y.max);
  magLimit.z.min = min(sensors->MAG.z, magLimit.z.min);
  magLimit.z.max = max(sensors->MAG.z, magLimit.z.max);

  sensors->MAG.x -= ((magLimit.x.max + magLimit.x.min) / 2);
  sensors->MAG.y -= ((magLimit.y.max + magLimit.y.min) / 2);
  sensors->MAG.z -= ((magLimit.z.max + magLimit.z.min) / 2);*/

}

void IMU::getRawValues(int16_t* values)
{
  mpu6050.getMotion9(&values[0], &values[1], &values[2],
                     &values[4], &values[3], &values[5],
                     &values[6], &values[8], &values[7]);

  values[0] *= sensorsSens.ACC.x;
  values[1] *= sensorsSens.ACC.y;
  values[2] *= sensorsSens.ACC.z;

  values[3] *= sensorsSens.GYRO.x;
  values[4] *= sensorsSens.GYRO.y;
  values[5] *= sensorsSens.GYRO.z;

  values[6] *= sensorsSens.MAG.x;
  values[7] *= sensorsSens.MAG.y;
  values[8] *= sensorsSens.MAG.z;
}

void IMU::getAttitude(IMU::attitude12f* attitude)
{
    now = micros();
    G_Dt = (now - lastUpdate) / 1000000.0;
    lastUpdate = now;

    motion9f values;
    getMotion9(&values);
    float angles[3];

#ifdef _WITH_DCM_

    #ifdef HeadingMagHold
    dcm.calculate( radians(values.GYRO.x), radians(values.GYRO.y), radians(values.GYRO.z),
                   values.ACC.x, values.ACC.y, values.ACC.z,
                   accelOneG, hdgX, hdgY,
                   G_Dt
                  );
    #else
    dcm.calculate( radians(values.GYRO.x), radians(values.GYRO.y), radians(values.GYRO.z),
                   values.ACC.x, values.ACC.y, values.ACC.z,
                   accelOneG, 0.0, 0.0,
                   G_Dt
                  );
    #endif

    dcm.getEuler(angles);
#else
    vector3f vAngles = getAccelAngle(&values.ACC);

    float heading = getHeading(vAngles, values.MAG);
    vAngles.z = heading;

    kalman->setAccValues(vAngles);
    kalman->setGyroValues(values.GYRO, G_Dt);

    vector3f kAngles = kalman->compute();

    angles[0] = kAngles.x;//kalmanRoll->innovate(roll, values.GYRO.y, G_Dt);
    angles[1] = kAngles.y;//kalmanPitch->innovate(pitch, values.GYRO.x, G_Dt);
    angles[2] = kAngles.z;
#endif

    attitude->ACC.x = values.ACC.x;
    attitude->ACC.y = values.ACC.y;
    attitude->ACC.z = values.ACC.z;

    attitude->GYRO.x = values.GYRO.x;
    attitude->GYRO.y = values.GYRO.y;
    attitude->GYRO.z = values.GYRO.z;

    attitude->MAG.x = values.MAG.x;
    attitude->MAG.y = values.MAG.y;
    attitude->MAG.z = values.MAG.z;

    attitude->EULER.roll = angles[0]; // roll
    attitude->EULER.pitch = angles[1]; // pitch
    attitude->EULER.yaw = angles[2]; // yaw
}

vector3f IMU::getAccelAngle(vector3f *acc)
{
	vector3f angles = {0,0,0};
	double R = sqrt((acc->x*acc->x) + (acc->y*acc->y) + (acc->z*acc->z));
	//angles.x = atan2(acc->x, acc->z) * toDeg;
	//angles.y = atan2(acc->y, acc->z) * toDeg;
	if (acc->z < 0) {
		angles.x = -acos(acc->x / R)*toDeg;
		angles.y = -acos(acc->y / R)*toDeg;
	} else {
		angles.x = acos(acc->x / R)*toDeg;
		angles.y = acos(acc->y / R)*toDeg;
	}

	angles.z = acos(acc->z / R)*toDeg;

	// Shifts the angles so as to be 0 on horizontal position

	angles.x -= 90;
	if (angles.x <-180) {
		angles.x +=360;
	}

	angles.y -= 90;
	if (angles.y <-180) {
		angles.y +=360;
	}

	angles.x = -angles.x;
	angles.y = -angles.y;

	// Ensures that Z gets negative if pointing downward
	/*if (angles.x<0)
	{
		angles.z = - angles.z;
	}*/

	return angles;
}

vector3f IMU::getAccelMeasurementNoise()
{
    motion9f values;

    vector3f store[20];
    vector3f variance = {0,0,0};

    vector3f sum = {0,0,0};
    vector3f sum2 = {0,0,0};

    for (int i=0; i<20; i++){
    	getMotion9(&values);
    	store[i] = getAccelAngle(&values.ACC);
    }

    for (int k=0; k<20; k++) {
    	sum.x = sum.x+store[k].x;
    	sum.y = sum.y+store[k].y;
    	sum.z = sum.z+store[k].z;
    }

    for (int k=0; k<20; k++) {
    	sum2.x = sum2.x+(store[k].x*store[k].x);
    	sum2.y = sum2.y+(store[k].y*store[k].y);
    	sum2.z = sum2.z+(store[k].z*store[k].z);
    }

    variance.x += (sum2.x - ((sum.x)*(sum.x))/20)/19;
    variance.y += (sum2.y - ((sum.y)*(sum.y))/20)/19;
    variance.z += (sum2.z - ((sum.z)*(sum.z))/20)/19;

    return variance;
}

vector3f IMU::getGyroMeasurementNoise()
{
    motion9f values;

    vector3f store[20];
    vector3f variance = {0,0,0};

    vector3f sum = {0,0,0};
    vector3f sum2 = {0,0,0};

    for (int i=0; i<20; i++){
    	getMotion9(&values);
    	store[i].x = values.GYRO.x;// * 0.01f; // 10ms
    	store[i].y = values.GYRO.y;// * 0.01f; // 10ms
    	store[i].z = values.GYRO.z;// * 0.01f; // 10ms
    }

    for (int k=0; k<20; k++) {
    	sum.x = sum.x+store[k].x;
    	sum.y = sum.y+store[k].y;
    	sum.z = sum.z+store[k].z;
    }

    for (int k=0; k<20; k++) {
    	sum2.x = sum2.x+(store[k].x*store[k].x);
    	sum2.y = sum2.y+(store[k].y*store[k].y);
    	sum2.z = sum2.z+(store[k].z*store[k].z);
    }

    variance.x += (sum2.x - ((sum.x)*(sum.x))/20)/19;
    variance.y += (sum2.y - ((sum.y)*(sum.y))/20)/19;
    variance.z += (sum2.z - ((sum.z)*(sum.z))/20)/19;

    return variance;
}

/**
 *
 */
float IMU::getHeading(vector3f acc, vector3f mag)
{

	float roll = acc.x * toRad;
	float pitch = acc.y * toRad;

	float cosRoll = cos(roll);
	float sinRoll = sin(roll);
	float cosPitch = cos(pitch);
	float sinPitch = sin(pitch);

    float Xh = (mag.x * cosRoll) + (mag.y * sinPitch * sinRoll) - (mag.z * cosPitch * sinRoll);
    float Yh = (mag.y * cosPitch) - (mag.z * sinPitch);

    float heading = atan2(Yh, Xh);

	return heading * toDeg;

	/*float declinationAngle = 1.454 / 1000.0; // Magnetic Declination 0° 5' West at Fontenay sous bois France
											 // http://www.wolframalpha.com/input/?i=%280%C2%B0+5%27%29+in+radians
	float heading = atan2(mag.y, mag.x);
	heading += declinationAngle;

	if (heading < 0)
		heading += 2*M_PI;

	if (heading > 2*M_PI)
		heading -= 2*M_PI;

	return heading * toDeg;*/
}

/**
 *
 */
float IMU::getAltitude()
{
	float temperature = baro.getTemperature(MS561101BA_OSR_4096);
	float press = baro.getPressure(MS561101BA_OSR_4096);
	if (press!=NULL) {
		  movavg_buff[movavg_i] = press;
		  movavg_i = (movavg_i + 1) % MOVAVG_SIZE;
	}

	float sum = 0.0;
	for (int i=0; i<MOVAVG_SIZE;i++) {
		sum += movavg_buff[i];
	}

	press = sum / MOVAVG_SIZE;

	return baro.getAltitude(press, temperature);

}
