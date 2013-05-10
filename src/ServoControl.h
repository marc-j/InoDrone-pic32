/*
 * ServoControl.h
 *
 *  Created on: 10 mai 2013
 *      Author: alienx
 */

#ifndef SERVOCONTROL_H_
#define SERVOCONTROL_H_

/*#define VAL_PPM_MIN 3000
#define VAL_PPM_MAX 25000*/

#include <inttypes.h>

#define T2_PERIOD (( __PIC32_pbClk / 64 / 100) - 1) //((PR2VAL  + 1) * 8 / __PIC32_pbClk) 10ms

#define PB_CLOCK_1MS       __PIC32_pbClk/1000
#define PB_CLOCK_1_5MS     (15*__PIC32_pbClk)/10000
#define PB_CLOCK_2MS       (2*__PIC32_pbClk)/1000

#define VAL_PPM_MAX        3000 //PB_CLOCK_2MS/64
#define VAL_PPM_CENTER     PB_CLOCK_1_5MS/64
#define VAL_PPM_MIN        600 //PB_CLOCK_1MS/64

class ServoControl {
public:
	ServoControl();
	void init();

	void setMotor(uint8_t oc, uint16_t value);
};

#endif /* SERVOCONTROL_H_ */
