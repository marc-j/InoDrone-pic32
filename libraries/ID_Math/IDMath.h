/*
 * IDMath.h
 *
 *  Created on: 27 janv. 2013
 *
 *  Code from AeroQuad
 */

#ifndef IDMATH_H_
#define IDMATH_H_

float vectorDotProduct(int length, float vector1[], float vector2[]);
void vectorCrossProduct(float vectorC[3], float vectorA[3], float vectorB[3]);
void vectorScale(int length, float scaledVector[], float inputVector[], float scalar);
void vectorAdd(int length, float vectorC[], float vectorA[], float vectorB[]);
void vectorSubtract(int length, float vectorC[], float vectorA[], float vectorB[]);

void matrixMultiply(int aRows, int aCols_bRows, int bCols, float matrixC[], float matrixA[], float matrixB[]);
void matrixAdd(int rows, int cols, float matrixC[], float matrixA[], float matrixB[]);
short isSwitched(float previousError, float currentError);

#endif /* IDMATH_H_ */
