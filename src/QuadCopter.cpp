
/**
 *
 *
 * ############# TIMER USING ############
 * TIMER 3
 * --------------------------------------
 * Using for OC PWM to ESC motor
 * PWM Period = 10ms
 * Prescale = 64
 *
 * --------------------------------------
 * TIMER 2
 * --------------------------------------
 * Using for IC
 * - PPM input
 * - SRF05
 * Prescale = 64
 * Period = 0xFFFF (1/(P_CPU/PRE))*0xFFFF = (1/(80000000/64)) * 0xFFFF = ~52ms
 * Resolution per timer tick = 0.8µs
 *
 *
 *
 */


#include "QuadCopter.h"

#include "Kinematics.h"
#if defined(_IMU_ARG)
	#include "Kinematics_ARG.h"
#elif defined(_IMU_AHRS)
    #include "Kinematics_AHRS.h"
#else
	#error No kenamtics defined for IMU
#endif


uavlink_message_sensor_t sensor;
uavlink_message_sensor_raw_t sensorRaw;
uavlink_message_system_t systemInfo;
uavlink_message_motor_t motorInfo;
uavlink_message_command_t msgCommands;


/**
 * Oups! Not working if in Protocol.cpp
 * Serial break
 *
 */
void sendMessage(uavlink_message_t msg)
{
	if (!protocol.isConnected()) {
		return;
	}

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

	// SciLab serial
	Serial.begin(115200);

	TRISCbits.TRISC2 = 0; // RC2 to output blue
	TRISCbits.TRISC3 = 0; // RC3 to output yellow
	TRISFbits.TRISF3 = 0; // RF3 to output red

	PORTCbits.RC2 = 0;
	PORTCbits.RC3 = 0;
	PORTFbits.RF3 = 0;

	uav.LED.startBlink = &blinkLED;
	uav.LED.updateBlink = &blinkLEDUpdate;

	uav.safe_timer = 0;
	uav.flightmode = FLIGHTMODE_RATE;//FLIGHTMODE_WAITING;
	uav.takeoff = 0;

	uav.rollPID = new PID(0.8f,0.0f,0.0f);
	uav.rollAttRatePID = new PID(1.75f, 0.0f, -6.1f);
	uav.pitchPID = new PID(0.8f,0.0f,0.0f);
	uav.pitchAttRatePID = new PID(1.75f, 0.0f, -6.1f);

	uav.yawPID = new PID(1.0f,0.0f,0.0f);

	// Init timer2 for PPM and Sonar
	OpenTimer2( T2_ON | T2_PS_1_64, 0xFFFF);

	uav.rc = new Radio();

	imu = IMU();
	imu.init();

	motor = ServoControl();
	motor.init();

	protocol.start(&uav);

	// Init bluetooth Modem
	protocol.print("\r\n+INQ=1\r\n");
	delay(5000);

	uav.LED.startBlink(100, 20);

}

/**
 * 1000Hz loop (1ms)
 */
void Task1000Hz()
{
	imu.sensorsSum();
}

/**
 * 100Hz loop (10ms)
 */
void Task100Hz()
{
	G_Dt = (mainTimer-task100HZtimer)*0.000001;
	task100HZtimer = mainTimer;

	uav.rc->update();
	uav.takeoff = uav.rc->armed.isOn;
	uav.flightmode = uav.rc->mode.isOn;



	imu.getAttitude(&attitude, G_Dt);

	if (uav.takeoff == 0 ) {
		STOP_MOTOR; // Stop motor
		LEDRED_OFF;

		uav.rollPID->resetIntegral();
		uav.pitchPID->resetIntegral();
		uav.yawPID->resetIntegral();

	} else {

		LEDRED_ON;
		// STOP MOTOR and takeoff = false if angle > |45|
		if ( abs(attitude.EULER.pitch) > 60 || abs(attitude.EULER.roll) > 60 ) {
			CUTOFF;
		} else {
			uint16_t thrust = uav.rc->throttle.value;

			float stabRoll  = uav.rollPID->calculate(uav.rc->roll.value, -attitude.EULER.roll, G_Dt);
			float stabPitch = uav.pitchPID->calculate(uav.rc->pitch.value, attitude.EULER.pitch, G_Dt);
			float stabYaw   = uav.yawPID->calculate(uav.rc->yaw.value, attitude.GYRO.z, G_Dt);

			// Apply gyro rate PID
			stabRoll = uav.rollAttRatePID->calculate(stabRoll, -attitude.GYRO.x ,G_Dt);
			stabPitch = uav.pitchAttRatePID->calculate(stabPitch, attitude.GYRO.y ,G_Dt);

			uav.MOTOR.FL = MIX(+1,-1,-1); //constrain(thrust + stabRoll - stabPitch, VAL_PPM_MIN, VAL_PPM_MAX);
			uav.MOTOR.FR = MIX(-1,-1,+1); //constrain(thrust - stabRoll - stabPitch, VAL_PPM_MIN, VAL_PPM_MAX); //MIX(-1,-1,+1);
			uav.MOTOR.RL = MIX(+1,+1,+1); //constrain(thrust + stabRoll + stabPitch, VAL_PPM_MIN, VAL_PPM_MAX); //MIX(+1,+1,+1);
			uav.MOTOR.RR = MIX(-1,+1,-1); //constrain(thrust - stabRoll + stabPitch, VAL_PPM_MIN, VAL_PPM_MAX); //MIX(-1,+1,-1);

			uint16_t motorMax = uav.MOTOR.FL;
			if (uav.MOTOR.FR>motorMax) motorMax = uav.MOTOR.FR;
			if (uav.MOTOR.RL>motorMax) motorMax = uav.MOTOR.RL;
			if (uav.MOTOR.RR>motorMax) motorMax = uav.MOTOR.RR;

			// Decrease throttle if speed it's max
			if (motorMax > MAX_THROTTLE) {
				uav.MOTOR.FL -= motorMax - MAX_THROTTLE;
				uav.MOTOR.FR -= motorMax - MAX_THROTTLE;
				uav.MOTOR.RL -= motorMax - MAX_THROTTLE;
				uav.MOTOR.RR -= motorMax - MAX_THROTTLE;
			}

			if (thrust == VAL_PPM_MIN) {
				STOP_MOTOR;
			}

			uav.MOTOR.FL = constrain(uav.MOTOR.FL, VAL_PPM_MIN, VAL_PPM_MAX);
			uav.MOTOR.FR = constrain(uav.MOTOR.FR, VAL_PPM_MIN, VAL_PPM_MAX);
			uav.MOTOR.RL = constrain(uav.MOTOR.RL, VAL_PPM_MIN, VAL_PPM_MAX);
			uav.MOTOR.RR = constrain(uav.MOTOR.RR, VAL_PPM_MIN, VAL_PPM_MAX);
		}
	}

	// Send command to motor
	motor.setMotor(MFL, uav.MOTOR.FL);
	motor.setMotor(MFR, uav.MOTOR.FR);
	motor.setMotor(MRL, uav.MOTOR.RL);
	motor.setMotor(MRR, uav.MOTOR.RR);
}

/**
 * 10Hz loop (100ms)
 */
void Task10Hz()
{
	G_Dt = (mainTimer-task10HZtimer)*0.000001;
	task10HZtimer = mainTimer;

	// TODO calculate magnetometer
	imu.calculateHeading(attitude.EULER.roll, attitude.EULER.pitch, &attitude.EULER.yaw);


	/*int16_t sraw[9];
	imu.getRawValues(sraw);*/


	/*Serial.print(attitude.ACC.x);
	Serial.print(' ');
	Serial.print(attitude.ACC.y);
	Serial.print(' ');
	Serial.print(attitude.ACC.z);
	Serial.print(' ');
	Serial.print(attitude.GYRO.x);
	Serial.print(' ');
	Serial.print(attitude.GYRO.y);
	Serial.print(' ');
	Serial.print(attitude.GYRO.z);
	Serial.print(' ');
	Serial.print(attitude.EULER.roll);
	Serial.print(' ');
	Serial.print(attitude.EULER.pitch);
	Serial.print(' ');
	Serial.println(attitude.EULER.yaw);*/

	// TODO send telemetry
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
	systemInfo.mainLoopTime = (uint16_t) (mainTimer-mainTimerOld)*1000;

	sendMessage(uavlink_message_system_encode(&systemInfo));

	msgCommands.throttle = uav.rc->throttle.value;
	msgCommands.pitch = uav.rc->pitch.value;
	msgCommands.roll = uav.rc->roll.value;
	msgCommands.yaw = uav.rc->yaw.value;
	msgCommands.armed = uav.takeoff;
	msgCommands.flightMode = uav.flightmode;

	sendMessage(uavlink_message_command_encode(&msgCommands));
}

void loop()
{

	uav.LED.updateBlink();

	mainTimerOld = mainTimer;
	mainTimer = micros();
	mainTimerElapsed = mainTimer - mainTimerEnd;
	cpu_load = float(mainTimerEnd-mainTimerOld)/(mainTimer-mainTimerOld);

	if (mainTimerElapsed >= 1000) { // 1kHz

		loopCounter++;

		Task1000Hz();

		if ((loopCounter % 10) == 0)  {
			Task100Hz();
		}

		if ((loopCounter % 100) == 0) {
			Task10Hz();
		}


		mainTimerEnd = mainTimer;
	}

	if (loopCounter >= 1000) {
		loopCounter = 0;
	}
}
