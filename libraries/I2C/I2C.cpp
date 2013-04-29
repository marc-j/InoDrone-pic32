/*
 * I2C.cpp
 *
 *  Created on: 19 janv. 2013
 *      Author: marc
 */

#include "I2C.h"

I2C::I2C() {}


int8_t I2C::readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data) {
    uint8_t b;
    uint8_t count = readByte(devAddr, regAddr, &b);
    *data = b & (1 << bitNum);
    return count;
}

int8_t I2C::readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data) {
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t count, b;
    if ((count = readByte(devAddr, regAddr, &b)) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *data = b;
    }
    return count;
}

int8_t I2C::readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data)
{
	return readBytes(devAddr, regAddr, 1, data);
}

int8_t I2C::readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data)
{
	int8_t count = 0;

	Wire.beginTransmission(devAddr);
	Wire.send(regAddr);
	Wire.endTransmission();

	Wire.beginTransmission(devAddr);
	Wire.requestFrom(devAddr, length);
	for (; Wire.available(); count++) {
		data[count] = Wire.receive();
	}
	Wire.endTransmission();


	return count;
}

bool I2C::writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    readByte(devAddr, regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return writeByte(devAddr, regAddr, b);
}

bool I2C::writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b;
    if (readByte(devAddr, regAddr, &b) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        b &= ~(mask); // zero all important bits in existing byte
        b |= data; // combine data with existing byte
        return writeByte(devAddr, regAddr, b);
    } else {
        return false;
    }
}

bool I2C::writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data) {
    return writeBytes(devAddr, regAddr, 1, &data);
}

bool I2C::writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data)
{

	Wire.beginTransmission(devAddr);
	Wire.send(regAddr);

	for (uint8_t i=0; i < length; i++) {
		Wire.send( data[i]);
	}

	Wire.endTransmission();

	return 0;
}

