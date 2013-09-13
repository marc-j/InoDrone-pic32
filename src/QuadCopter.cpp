#include "WProgram.h"
#include "QuadCopter.h"


//#define _WITH_DCM_


uavlink_message_sensor_t sensor;
uavlink_message_sensor_raw_t sensorRaw;
uavlink_message_system_t systemInfo;
uavlink_message_motor_t motorInfo;


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
	uav.safe_timer = 0;
	uav.flightmode = FLIGHTMODE_WAITING;
	uav.takeoff = 0;
	uav.CMD.roll = 0;
	uav.CMD.pitch = 0;
	uav.CMD.yaw = 0;
	uav.CMD.throttle = 0;

	uav.rollPID = new PID(0.0f,0.0f,0.0f);
	uav.pitchPID = new PID(0.0f,0.0f,0.0f);
	uav.yawPID = new PID(0.0f,0.0f,0.0f);

	protocol.start(&uav);

	imu = IMU();
	imu.init();

	motor = ServoControl();
	motor.init();


	//Serial.begin(57600);
}

void loop()
{

	/*while(Serial.read() != 0x20);
	int16_t sraw[9];
	imu.getRawValues(sraw);

	Serial.write(sraw[6]);
	Serial.write(sraw[6]>>8);
	Serial.write(sraw[7]);
	Serial.write(sraw[7]>>8);
	Serial.write(sraw[8]);
	Serial.write(sraw[8]>>8);
	Serial.write('\n');

	return;*/

	// FailSAFE
	if(millis()-uav.safe_timer >= 150){ // 50ms soit 20hz
		uav.flightmode = FLIGHTMODE_WAITING;
		//ERROR_RECV; BIPPPPPPPP
	}

	if (millis()-timerMain >= 10) { // 10ms -> 100hz
		timerMain_old = timerMain;
		timerMain = millis();
		cpu_load = float(timer_end-timerMain_old)/(timerMain-timerMain_old);
		G_Dt = (timerMain-timerMain_old)*0.001;

		imu.getAttitude(&attitude);

		if (uav.flightmode == FLIGHTMODE_WAITING) { //Waiting for instruction
			CUTOFF; // STOP MOTOR

		} else if (uav.flightmode == FLIGHTMODE_VARIANCE) { // get variance for Acc and Gyro
			CUTOFF;

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

			uav.flightmode = FLIGHTMODE_WAITING;
		} else if (uav.flightmode == FLIGHTMODE_NORMAL) { // Normal mode waiting to landing
			if (uav.takeoff == 0 ) {
				STOP_MOTOR; // Stop motor
			} else {
				// STOP MOTOR and takeoff = false if angle > |30|
				if ( abs(attitude.EULER.pitch) > 30 || abs(attitude.EULER.roll) > 30 ) {
					CUTOFF;
				} else {
					uint16_t thrust = mapMotorCmd(uav.CMD.throttle);

					float stabRoll = uav.rollPID->calculate(uav.CMD.roll - attitude.EULER.roll, G_Dt);
					float stabPitch = uav.pitchPID->calculate(uav.CMD.pitch - attitude.EULER.pitch, G_Dt);

					uav.MOTOR.FL = constrain(thrust - stabRoll - stabPitch, VAL_PPM_MIN, VAL_PPM_MAX);
					uav.MOTOR.FR = constrain(thrust + stabRoll - stabPitch, VAL_PPM_MIN, VAL_PPM_MAX);
					uav.MOTOR.RL = constrain(thrust - stabRoll + stabPitch, VAL_PPM_MIN, VAL_PPM_MAX);
					uav.MOTOR.RR = constrain(thrust + stabRoll + stabPitch, VAL_PPM_MIN, VAL_PPM_MAX);
				}
			}
		}

		// Send command to motor
		motor.setMotor(MFL, uav.MOTOR.FL);
		motor.setMotor(MFR, uav.MOTOR.FR);
		motor.setMotor(MRL, uav.MOTOR.RL);
		motor.setMotor(MRR, uav.MOTOR.RR);


		if (millis()-telemetryTimer >= 100 ) { // 100ms soit 10hz
			/*float alt = imu.getAltitude();
			Serial.println(alt);*/

			sensor.accX = (int16_t) (attitude.ACC.x*1000.0f);
			sensor.accY = (int16_t) (attitude.ACC.y*1000.0f);
			sensor.accZ = (int16_t) (attitude.ACC.z*1000.0f);
			sensor.gyroX = (int16_t) (attitude.GYRO.x*10.0f);
			sensor.gyroY = (int16_t) (attitude.GYRO.y*10.0f);
			sensor.gyroZ = (int16_t) (attitude.GYRO.z*10.0f);
            sensor.magX = (int16_t) (attitude.MAG.x*10.0f);
            sensor.magY = (int16_t) (attitude.MAG.y*10.0f);
            sensor.magZ = (int16_t) (attitude.MAG.z*10.0f);
			sensor.pitch = (int16_t) (attitude.EULER.pitch*10.0f);
			sensor.roll = (int16_t) (attitude.EULER.roll*10.0f);
			sensor.yaw = (int16_t) (attitude.EULER.yaw*10.0f);

			sendMessage(uavlink_message_sensor_encode(&sensor));

			motorInfo.motorFrontLeft = uav.MOTOR.FL;
			motorInfo.motorFrontRight = uav.MOTOR.FR;
			motorInfo.motorRearLeft = uav.MOTOR.RL;
			motorInfo.motorRearRight = uav.MOTOR.RR;
			sendMessage(uavlink_message_motor_encode(&motorInfo));

			// Send raw sensor values
			/*int16_t sraw[9];
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

			sendMessage(uavlink_message_sensor_raw_encode(&sensorRaw));*/


			systemInfo.cpuLoad = (uint16_t) (cpu_load*1000.0f);
			systemInfo.flightMode = uav.flightmode;
			systemInfo.batteryVoltage = 0;
			systemInfo.mainLoopTime = (uint16_t) (timerMain-timerMain_old)*1000;

			sendMessage(uavlink_message_system_encode(&systemInfo));

			telemetryTimer = millis();
		}

		timer_end = millis();
	}

}
