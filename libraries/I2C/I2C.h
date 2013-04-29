/*
 * I2C.h
 *
 *  Created on: 19 janv. 2013
 *      Author: marc
 */

#ifndef I2C_H_
#define I2C_H_

#define TWI_FREQ 400000 // Set I2C Bus to 400khz
#include "Wire.h"

class I2C {
public:
	I2C();

	static int8_t readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data);
	static int8_t readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
	static int8_t readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data);
	static int8_t readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);

	static bool writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
	static bool writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
	static bool writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);
	static bool writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
};

#endif /* I2C_H_ */
