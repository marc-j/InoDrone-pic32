/*
 * Protocol.cpp
 *
 *  Created on: 19 janv. 2013
 *      Author: marc
 */

#include <inttypes.h>

#include <plib.h>

#include "wiring.h"
#include "wiring_private.h"

#include "Protocol.h"

Protocol::Protocol() {
	step = STX1;
	buffer_pos = 0;
	len = 0;
	crc = 0;
}

void Protocol::start(UAV *uav)
{
	this->uav = uav;
	U5STA = 0;
	U5MODEbits.PDSEL = 0; // 0 = 8bit data, no parity
	U5MODEbits.STSEL = 0; // 0 = 1 Stop bit
	U5MODEbits.BRGH = 1;  // High Speed serial
	U5STAbits.URXEN = 1; // Enabled RX
	U5STAbits.UTXEN = 1; // Enabled TX
	U5STAbits.URXISEL = 0; //
	U5MODEbits.ON = 1; // Enabled UART
	U5BRG = PROTOCOL_BRG;

	INTEnable((INT_SOURCE)INT_SOURCE_UART_RX(UART5), INT_ENABLED);
	// Set Interrupt priorities
	INTSetVectorPriority((INT_VECTOR)INT_VECTOR_UART(UART5), INT_PRIORITY_LEVEL_2);
	INTSetVectorSubPriority((INT_VECTOR)INT_VECTOR_UART(UART5), INT_SUB_PRIORITY_LEVEL_0);
}

void Protocol::receiveByte()
{
	uint8_t data = U5RXREG;
	write(data);

    switch(step) {
        case STX1:
            if (data == 0xFF)
                step = STX2;
            break;
        case STX2:
            if (data == 0xFF){
                step = CTX;
                len = 0;
                buffer_pos = 0;
                crc = 0;
            } else {
                step = STX1;
            }
            break;
        case CTX:
            buffer[buffer_pos++] = data;
            crc ^= data;
            step = LTX;
            break;
        case LTX:
        	buffer[buffer_pos++] = data;
            crc ^= data;
            len = data;
            if  ( len <= 0 ) {
                step = STX1;
            } else {
                step = DTX;
            }
            break;
        case DTX:
        	buffer[buffer_pos++] = data;
            crc ^= data;
            len--;
            if (len == 0) {
                step = CRC;
            }
            break;
        case CRC:
            if( crc == data ){
                datasReceive(buffer);
            }
            step = STX1;
            break;
    }

}

void Protocol::datasReceive(const uint8_t* datas)
{
	uavlink_message_t msg;
	msg.cmd = datas[0];
	msg.len = datas[1];
	for (int i=2, j=0; j< msg.len; i++, j++) {
		msg.datas[j] = datas[i];
	}

	switch(msg.cmd) {
		case UAVLINK_MSG_COMMAND:

			break;
		case UAVLINK_MSG_SYSTEM:
			uavlink_message_system_t system;
			uavlink_message_system_decode(&msg, &system);

			uav->flightmode = system.flightMode;
			break;
	}
}

void Protocol::write(uint8_t byte)
{
    while(U5STAbits.UTXBF != 0);
    U5TXREG = byte;
    while(U5STAbits.TRMT != 1);
}


extern "C" {

	#define __UART_5_ISR    __ISR(_UART_5_VECTOR, ipl4)
	void __UART_5_ISR  UART_5_InterruptRoutine(void) {

		if(INTGetFlag((INT_SOURCE) INT_SOURCE_UART_RX(UART5))) {
			INTClearFlag((INT_SOURCE) INT_SOURCE_UART_RX(UART5));
			protocol.receiveByte();
		}

	}
}; // extern C

Protocol protocol = Protocol();



