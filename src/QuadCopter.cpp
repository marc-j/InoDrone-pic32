#include "WProgram.h"
#include "QuadCopter.h"


//#define _WITH_DCM_


uavlink_message_sensor_t sensor;
uavlink_message_sensor_raw_t sensorRaw;
uavlink_message_system_t systemInfo;

/**
 * Oups! Not working if in Protocol.cpp
 * Serial break
 *
 */
void sendMessage(uavlink_message_t msg)
{
	uint8_t crc_s  =0;
	protocol.write(255); 	// STX1
	protocol.write(255); 	// STX2
	protocol.write(msg.cmd); // CMD
	crc_s ^= msg.cmd;
	protocol.write(msg.len); // LEN
	crc_s ^= msg.len;
	for (uint8_t i=0; i< msg.len; i++) {
		crc_s ^= msg.datas[i];
		protocol.write(msg.datas[i]);
	}
	protocol.write(crc_s);
}

void setup()
{
	uav.flightmode = FLIGHTMODE_WAITING;
	uav.landing = 0;
	uav.CMD.roll = 0;
	uav.CMD.pitch = 0;
	uav.CMD.yaw = 0;
	uav.CMD.throttle = 0;

	protocol.start(&uav);

	imu = IMU();
	imu.init();

	//Serial.begin(9600);
}

void loop()
{
	// FailSAFE
	if(millis()-safe_timer >= 50){ // 50ms soit 20hz
		//TODO: Flight off
	}

	if (millis()-timerMain >= 10) { // 10ms -> 100hz
		timerMain_old = timerMain;
		timerMain = millis();
		cpu_load = float(timer_end-timerMain_old)/(timerMain-timerMain_old);
		G_Dt = (timerMain-timerMain_old)*0.001;

		imu.getAttitude(&attitude);

		if (uav.flightmode == FLIGHTMODE_WAITING) { //Waiting for instruction

		} else if (uav.flightmode == FLIGHTMODE_VARIANCE) { // get variance for Acc and Gyro
			vector3f accVariance = imu.getAccelMeasurementNoise();
			vector3f gyroVariance = imu.getGyroMeasurementNoise();


			uavlink_message_sensor_variance_t sensorVariance;
			sensorVariance.accX = (int32_t) (accVariance.x*10000.0f);
			sensorVariance.accY = (int32_t) (accVariance.y*10000.0f);
			sensorVariance.accZ = (int32_t) (accVariance.z*10000.0f);
			sensorVariance.gyroX = (int32_t) (gyroVariance.x*10000.0f);
			sensorVariance.gyroY = (int32_t) (gyroVariance.y*10000.0f);
			sensorVariance.gyroZ = (int32_t) (gyroVariance.z*10000.0f);

			uavlink_message_t msgSensorVariance;
			msgSensorVariance = uavlink_message_sensor_variance_encode(&sensorVariance);
			sendMessage(msgSensorVariance);

			uav.flightmode = 0;
		} else if (uav.flightmode == FLIGHTMODE_COMPASS_CALIBRATION) { // Compass calibration
			//TODO imu getCompassCalibration
		} else if (uav.flightmode == FLIGHTMODE_NORMAL) { // Normal mode waiting to landing

		}



		if (millis()-telemetryTimer >= 100 ) { // 100ms soit 10hz
		    imu.getMag(attitude.EULER.roll, attitude.EULER.pitch);

			//memcpy(&sensor, (const char *)&IMU, 12);
			sensor.accX = (int16_t) (attitude.ACC.x*1000.0f);
			sensor.accY = (int16_t) (attitude.ACC.y*1000.0f);
			sensor.accZ = (int16_t) (attitude.ACC.z*1000.0f);
			sensor.gyroX = (int16_t) (attitude.GYRO.x*10.0f);
			sensor.gyroY = (int16_t) (attitude.GYRO.y*10.0f);
			sensor.gyroZ = (int16_t) (attitude.GYRO.z*10.0f);
            sensor.magX = (int16_t) (attitude.MAG.x*10.0f);
            sensor.magY = (int16_t) (attitude.MAG.y*10.0f);
            sensor.magZ = (int16_t) (attitude.MAG.z*10.0f);
			/*sensor.pitch = (int16_t) (degrees(attitude.EULER.pitch)*10.0f);
			sensor.roll = (int16_t) (degrees(attitude.EULER.roll)*10.0f);
			sensor.yaw = (int16_t) (degrees(attitude.EULER.yaw)*10.0f);*/
			sensor.pitch = (int16_t) (attitude.EULER.pitch*10.0f);
			sensor.roll = (int16_t) (attitude.EULER.roll*10.0f);
			sensor.yaw = (int16_t) (attitude.EULER.yaw*10.0f);

			uavlink_message_t msgSensor;
			msgSensor = uavlink_message_sensor_encode(&sensor);
			sendMessage(msgSensor);

			// Send raw sensor values
			int16_t sraw[9];
			imu.getRawValues(sraw);
			sensorRaw.accX = sraw[0];
			sensorRaw.accY = sraw[1];
			sensorRaw.accZ = sraw[2];
			sensorRaw.gyroX = sraw[3];
			sensorRaw.gyroY = sraw[4];
			sensorRaw.gyroZ = sraw[5];
			sensorRaw.magX = sraw[6];
			sensorRaw.magY = sraw[7];
			sensorRaw.magZ = sraw[8];

			uavlink_message_t msgSensorRaw;
			msgSensorRaw = uavlink_message_sensor_raw_encode(&sensorRaw);
			sendMessage(msgSensorRaw);


			systemInfo.cpuLoad = (uint16_t) (cpu_load*1000.0f);
			systemInfo.flightMode = uav.flightmode;
			systemInfo.batteryVoltage = 0;
			systemInfo.mainLoopTime = (uint16_t) (timerMain-timerMain_old)*1000;

			uavlink_message_t msgSystem;
			msgSystem = uavlink_message_system_encode(&systemInfo);
			sendMessage(msgSystem);

			telemetryTimer = millis();
		}

		timer_end = millis();
	}

}
