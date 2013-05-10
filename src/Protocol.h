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
	void receiveByte(const uint8_t data);
	void write(uint8_t byte);

    enum PROTOCOL_STEP {
        STX1,
        STX2,
        CTX,
        LTX,
        DTX,
        CRC
    };
private:
    uint8_t step;
    uint8_t len;
    uint8_t buffer[PROTOCOL_BUFFER_SIZE];
    uint8_t buffer_pos;
    uint8_t crc;
    UAV *uav;

    void datasReceive(const uint8_t* datas);
};

extern Protocol protocol;

#endif /* PROTOCOL_H_ */
