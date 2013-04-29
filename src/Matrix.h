/*
 * Matrix.h
 *
 *  Created on: 27 avr. 2013
 *      Author: alienx
 */

#ifndef MATRIX_H_
#define MATRIX_H_

#include <stdlib.h>
#include <stdio.h>
#include "inttypes.h"
#include "math.h"


  typedef struct __vector3 {
    int16_t x;
    int16_t y;
    int16_t z;
  } vector3;

  typedef struct __vector9 {
    vector3 ACC;
    vector3 GYRO;
    vector3 MAG;
    vector3 EULER;
  } vector9;

  typedef struct __vector3f {
    float x;
    float y;
    float z;
  } vector3f;


  extern void vector_cross(const vector3f *a, const vector3f *b, vector3f *out);
  extern float vector_dot(const vector3f *a, const vector3f *b);
  extern void vector_normalize(vector3f *a);

class Matrix {
public:
	int row;
	int col;
	float **mat;

	void init();
	Matrix();
	Matrix(const Matrix& m);
	Matrix(int rownum, int colnum);

	~Matrix();

	Matrix& operator =(const Matrix& Other);
	friend Matrix operator+(Matrix const& a, Matrix const& b);
	friend Matrix operator-(Matrix const& a, Matrix const& b);
	friend Matrix operator*(Matrix const& a, Matrix const& b);
	// Call for Matrix values (ex: A(0,1))
	float& operator() (unsigned row, unsigned col);

	Matrix invert_3x3();
};

#endif /* MATRIX_H_ */
