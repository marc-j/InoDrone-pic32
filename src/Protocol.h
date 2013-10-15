/*
 * Protocol.h
 *
 *  Created on: 19 janv. 2013
 *      Author: marc
 */

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include <inttypes.h>

#include "UAVLink.h"
#include "UAV.h"


#ifndef PROTOCOL_BAUDS
#define PROTOCOL_BAUDS 115200
#endif

#define PROTOCOL_BUFFER_SIZE 256

#define PROTOCOL_BRG ((( __PIC32_pbClk / PROTOCOL_BAUDS) / 16 ) -1 )

class Protocol {
public:
	Protocol();

	void start(UAV *uav);
	uint8_t isConnected();
	void receiveByte(const uint8_t data);
	void write(uint8_t byte);
	void print(const char *str);
	void modemReceive(const char* command);

    enum PROTOCOL_STEP {
        STX1,
        STX2,
        CTX,
        LTX,
        DTX,
        CRC
    };

    enum PROTOCOL_STATUS {
    	WAITING,
    	PAIRING
    };
private:
    uint8_t step;
    uint8_t len;
    uint8_t buffer[PROTOCOL_BUFFER_SIZE];
    uint8_t buffer_pos;
    uint8_t crc;
    UAV *uav;
    uint8_t bluetoothModemAck;
    uint8_t connected;
    uint8_t status;

	char bufferBee[128];
	uint8_t bufferBee_pos;
	int8_t stepBee;

    void datasReceive(const uint8_t* datas);
};

extern Protocol protocol;

#endif /* PROTOCOL_H_ */
