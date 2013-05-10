/*
 * Protocol.cpp
 *
 *  Created on: 19 janv. 2013
 *      Author: marc
 */

#include "Protocol.h"
#include <plib.h>

#include <wiring.h>
#include <wiring_private.h>

static uint8_t tx_buffer[255];
static volatile uint8_t *TXHeadPtr = tx_buffer, *TXTailPtr = tx_buffer;

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
	U5MODE = 0;
	U5MODEbits.PDSEL = 0; // 0 = 8bit data, no parity
	U5MODEbits.STSEL = 0; // 0 = 1 Stop bit
	U5MODEbits.BRGH = 0;  // Normal Speed serial
	U5MODEbits.RXINV = 0;
	U5STAbits.URXEN = 1; // Enabled RX
	U5STAbits.UTXEN = 1; // Enabled TX
	U5STAbits.UTXISEL = 0; //
	U5STAbits.URXISEL = 0; //
	U5MODEbits.ON = 1; // Enabled UART
	U5BRG = PROTOCOL_BRG;

	INTEnable((INT_SOURCE)INT_SOURCE_UART_RX(UART5), INT_ENABLED);
	//INTEnable((INT_SOURCE)INT_SOURCE_UART_TX(UART5), INT_ENABLED);
	// Set Interrupt priorities
	INTSetVectorPriority((INT_VECTOR)INT_VECTOR_UART(UART5), INT_PRIORITY_LEVEL_2);
	INTSetVectorSubPriority((INT_VECTOR)INT_VECTOR_UART(UART5), INT_SUB_PRIORITY_LEVEL_0);
}

void Protocol::receiveByte(const uint8_t data)
{
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
            	//uint8_t nBuffer[PROTOCOL_BUFFER_SIZE];
            	//memcpy(nBuffer, buffer, len);
                datasReceive(buffer);
            }
            step = STX1;
            break;
    }

}

void Protocol::datasReceive(const uint8_t* datas)
{
	uav->safe_timer = millis();

	uavlink_message_t msg;
	msg.cmd = datas[0];
	msg.len = datas[1];
	for (int i=2, j=0; j< msg.len; i++, j++) {
		msg.datas[j] = datas[i];
	}

	switch(msg.cmd) {
		case UAVLINK_MSG_COMMAND:
			uavlink_message_command_t command;
			uavlink_message_command_decode(&msg, &command);

			uav->CMD.throttle 	= command.throttle;
			uav->CMD.pitch 		= command.pitch;
			uav->CMD.roll 		= command.roll;
			uav->CMD.yaw 		= command.yaw;
			uav->flightmode 	= command.flightMode;
			uav->takeoff 		= command.armed;
			break;
	}
}

void Protocol::write(uint8_t byte)
{

	/*INTEnable((INT_SOURCE)INT_SOURCE_UART_TX(UART5), INT_DISABLED);

	(*TXHeadPtr++) = byte;
    if(TXHeadPtr >= tx_buffer + sizeof(tx_buffer))
        TXHeadPtr = tx_buffer;

    if(TXHeadPtr != TXTailPtr)
    	INTEnable((INT_SOURCE)INT_SOURCE_UART_TX(UART5), INT_ENABLED);*/

    while(U5STAbits.UTXBF != 0);
    UARTSendDataByte(UART5, byte);
    while(U5STAbits.TRMT != 1);

    delayMicroseconds(10);
}


extern "C" {

	#define __UART_5_ISR    __ISR(_UART_5_VECTOR, ipl4)
	void __UART_5_ISR  UART_5_InterruptRoutine(void) {

		if(INTGetFlag((INT_SOURCE) INT_SOURCE_UART_RX(UART5))) {
			uint8_t data = U5RXREG & 0xFF;
			protocol.receiveByte(data);
			INTClearFlag((INT_SOURCE) INT_SOURCE_UART_RX(UART5));
		}

		/*if(INTGetFlag((INT_SOURCE) INT_SOURCE_UART_TX(UART5))) {
            // Transmit a byte, if pending, if possible
             if(TXHeadPtr != TXTailPtr)
             {
                 // Clear the TX interrupt flag before transmitting again
            	 INTClearFlag((INT_SOURCE) INT_SOURCE_UART_TX(UART5));

                 if (UARTTransmitterIsReady(UART5) ) {
					 U5TXREG = *TXTailPtr++;
					 if(TXTailPtr >= tx_buffer + sizeof(tx_buffer))
						 TXTailPtr = tx_buffer;
                 }
             }
             else    // Disable the TX interrupt if we are done so that we don't keep entering this ISR
             {
            	 INTEnable((INT_SOURCE)INT_SOURCE_UART_TX(UART5), INT_DISABLED);
             }

		}*/

	}
}; // extern C

Protocol protocol = Protocol();



