/*
 * MPU6050.cpp
 *
 *  Created on: 19 janv. 2013
 *      Author: marc
 */

#include "MPU6050.h"

MPU6050::MPU6050()
{
	devAddr = MPU6050_DEFAULT_ADDRESS;
}

MPU6050::MPU6050(uint8_t address)
{
	devAddr = address;
}

void MPU6050::initialize()
{
	I2C::writeByte(devAddr, MPU6050_RA_PWR_MGMT_1, 0x80 ); // RESET MPU
	delay(100);
	I2C::writeByte(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_CLOCK_PLL_ZGYRO); // Set clock to Z GYRO
	I2C::writeByte(devAddr, MPU6050_RA_PWR_MGMT_2, 0);

	// Sample Rate
	I2C::writeByte(devAddr, MPU6050_RA_SMPLRT_DIV, 0x00); // Set to 1Khz

	I2C::writeByte(devAddr, MPU6050_RA_CONFIG, MPU6050_DLPF_BW_42); // LP ACC => 260Hz, GYRO 256 Hz
	I2C::writeByte(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GYRO_FS_1000 << 3); // Gyro FS_SEL = 2: Full scale set to 1000 deg/sec
	I2C::writeByte(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACCEL_FS_4 << 3); // ACC FS_SEL = 2: Full scale set to 2 g/sec
	I2C::writeByte(devAddr, MPU6050_RA_INT_PIN_CFG, 0x02); // I2C_BYPASS_EN=1
}

bool MPU6050::testConnection()
{
	return getDeviceID() == 0x34;
}

uint8_t MPU6050::getDeviceID() {
    I2C::readBits(devAddr, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, buffer);
    return buffer[0];
}

void MPU6050::setClockSource(uint8_t source) {
    I2C::writeBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

void MPU6050::enableI2CAux()
{
  //at this stage, the MAG is configured via the original MAG init function in I2C bypass mode
  //now we configure MPU as a I2C Master device to handle the MAG via the I2C AUX port (done here for HMC5883)
  I2C::writeByte(devAddr, 0x6A, 0b00100000);       //USER_CTRL     -- DMP_EN=0 ; FIFO_EN=0 ; I2C_MST_EN=1 (I2C master mode) ; I2C_IF_DIS=0 ; FIFO_RESET=0 ; I2C_MST_RESET=0 ; SIG_COND_RESET=0
  I2C::writeByte(devAddr, 0x37, 0x00);             //INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=0 ; CLKOUT_EN=0
  I2C::writeByte(devAddr, 0x24, 0x0D);             //I2C_MST_CTRL  -- MULT_MST_EN=0 ; WAIT_FOR_ES=0 ; SLV_3_FIFO_EN=0 ; I2C_MST_P_NSR=0 ; I2C_MST_CLK=13 (I2C slave speed bus = 400kHz)
  I2C::writeByte(devAddr, 0x25, 0x80|HMC5883_ADDRESS);//I2C_SLV0_ADDR -- I2C_SLV4_RW=1 (read operation) ; I2C_SLV4_ADDR=MAG_ADDRESS
  I2C::writeByte(devAddr, 0x26, HMC5883_DATA_REGISTER);//I2C_SLV0_REG  -- 6 data bytes of MAG are stored in 6 registers. First register address is MAG_DATA_REGISTER
  I2C::writeByte(devAddr, 0x27, 0x86);             //I2C_SLV0_CTRL -- I2C_SLV0_EN=1 ; I2C_SLV0_BYTE_SW=0 ; I2C_SLV0_REG_DIS=0 ; I2C_SLV0_GRP=0 ; I2C_SLV0_LEN=3 (3x2 bytes)
}

void MPU6050::getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
    I2C::readBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 14, buffer);
    *ax = (((int16_t)buffer[0]) << 8) | buffer[1];
    *ay = (((int16_t)buffer[2]) << 8) | buffer[3];
    *az = (((int16_t)buffer[4]) << 8) | buffer[5];
    *gx = (((int16_t)buffer[8]) << 8) | buffer[9];
    *gy = (((int16_t)buffer[10]) << 8) | buffer[11];
    *gz = (((int16_t)buffer[12]) << 8) | buffer[13];
}

void MPU6050::getMotion9(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* mx, int16_t* my, int16_t* mz) {
    I2C::readBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 20, buffer);
    *ax = (((int16_t)buffer[0]) << 8) | buffer[1];
    *ay = (((int16_t)buffer[2]) << 8) | buffer[3];
    *az = (((int16_t)buffer[4]) << 8) | buffer[5];
    *gx = (((int16_t)buffer[8]) << 8) | buffer[9];
    *gy = (((int16_t)buffer[10]) << 8) | buffer[11];
    *gz = (((int16_t)buffer[12]) << 8) | buffer[13];
    *mx = (((int16_t)buffer[14]) << 8) | buffer[15];
    *my = (((int16_t)buffer[16]) << 8) | buffer[17];
    *mz = (((int16_t)buffer[18]) << 8) | buffer[19];
}


void MPU6050::getAcceleration(int16_t* x, int16_t* y, int16_t* z) {
    I2C::readBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 6, buffer);
    *x = (((int16_t)buffer[0]) << 8) | buffer[1];
    *y = (((int16_t)buffer[2]) << 8) | buffer[3];
    *z = (((int16_t)buffer[4]) << 8) | buffer[5];
}

void MPU6050::getRotation(int16_t* x, int16_t* y, int16_t* z) {
    I2C::readBytes(devAddr, MPU6050_RA_GYRO_XOUT_H, 6, buffer);
    *x = (((int16_t)buffer[0]) << 8) | buffer[1];
    *y = (((int16_t)buffer[2]) << 8) | buffer[3];
    *z = (((int16_t)buffer[4]) << 8) | buffer[5];
}

void MPU6050::getMag(int16_t* x, int16_t* y, int16_t* z) {
    I2C::readBytes(devAddr, MPU6050_RA_EXT_SENS_DATA_00, 6, buffer);
    *x = (((int16_t)buffer[0]) << 8) | buffer[1];
    *y = (((int16_t)buffer[2]) << 8) | buffer[3];
    *z = (((int16_t)buffer[4]) << 8) | buffer[5];
}

int16_t MPU6050::getTemperature() {
    I2C::readBytes(devAddr, MPU6050_RA_TEMP_OUT_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

