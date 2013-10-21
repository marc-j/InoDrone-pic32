/*
 * FourtOrderFilter.h
 *
 *  Created on: 21 oct. 2013
 *      Author: alienx
 */

#ifndef FOURTORDERFILTER_H_
#define FOURTORDERFILTER_H_

class FourtOrderFilter {
public:
	FourtOrderFilter(float v);
	float compute(float);

private:
	struct fourthOrderData
	{
	  float  inputTm1,  inputTm2,  inputTm3,  inputTm4;
	  float outputTm1, outputTm2, outputTm3, outputTm4;
	} filterParameters;
};

#endif /* FOURTORDERFILTER_H_ */
