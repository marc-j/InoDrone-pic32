/*
 * FourtOrderFilter.cpp
 *
 *  Created on: 21 oct. 2013
 *      Author: alienx
 */

#include "FourtOrderFilter.h"

FourtOrderFilter::FourtOrderFilter(float v) {
	filterParameters.inputTm1 = v;
	filterParameters.inputTm2 = v;
	filterParameters.inputTm3 = v;
	filterParameters.inputTm4 = v;

	filterParameters.outputTm1 = v;
	filterParameters.outputTm2 = v;
	filterParameters.outputTm3 = v;
	filterParameters.outputTm4 = v;
}

float FourtOrderFilter::compute(float currentInput)
{
	  // cheby2(4,60,12.5/50)
	  #define _b0  0.001893594048567
	  #define _b1 -0.002220262954039
	  #define _b2  0.003389066536478
	  #define _b3 -0.002220262954039
	  #define _b4  0.001893594048567

	  #define _a1 -3.362256889209355
	  #define _a2  4.282608240117919
	  #define _a3 -2.444765517272841
	  #define _a4  0.527149895089809

	  float output;

	  output = _b0 * currentInput                +
	           _b1 * filterParameters.inputTm1  +
	           _b2 * filterParameters.inputTm2  +
	           _b3 * filterParameters.inputTm3  +
	           _b4 * filterParameters.inputTm4  -
	           _a1 * filterParameters.outputTm1 -
	           _a2 * filterParameters.outputTm2 -
	           _a3 * filterParameters.outputTm3 -
	           _a4 * filterParameters.outputTm4;

	  filterParameters.inputTm4 = filterParameters.inputTm3;
	  filterParameters.inputTm3 = filterParameters.inputTm2;
	  filterParameters.inputTm2 = filterParameters.inputTm1;
	  filterParameters.inputTm1 = currentInput;

	  filterParameters.outputTm4 = filterParameters.outputTm3;
	  filterParameters.outputTm3 = filterParameters.outputTm2;
	  filterParameters.outputTm2 = filterParameters.outputTm1;
	  filterParameters.outputTm1 = output;

	  return output;
}
