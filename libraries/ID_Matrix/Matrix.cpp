/*
 * Matrix.cpp
 *
 *  Created on: 27 avr. 2013
 *      Author: alienx
 */

#include "Matrix.h"

// Vector functions
void vector_cross(const vector3f *a, const vector3f *b, vector3f *out)
{
	out->x = a->y * b->z - a->z * b->y;
	out->y = a->z * b->x - a->x * b->z;
	out->z = a->x * b->y - a->y * b->x;
}

float vector_dot(const vector3f *a, const vector3f *b)
{
  return a->x * b->x + a->y * b->y + a->z * b->z;
}

void vector_normalize(vector3f *a)
{
	float mag = sqrt(vector_dot(a, a));
	a->x /= mag;
	a->y /= mag;
	a->z /= mag;
}

void Matrix::init()
{
	mat = (float**)malloc(sizeof(float*)*row);

	for (int i=0; i<row;i++) {
		mat[i] = (float*)malloc(sizeof(float)*col);
		for (int j=0; j<col; j++) {
			mat[i][j] = 0.0f;
		}
	}
}

Matrix::Matrix() {
	row = 3;
	col = 3;
	init();
}

Matrix::Matrix(const Matrix& m)
{
	row = m.row;
	col = m.col;

	mat = (float**)malloc(sizeof(float*)*row);

	for (int i=0; i<row; i++)
	{
		mat[i] = (float*)malloc(sizeof(float)*col);
		for(int j=0; j<col; j++)
		{
			(*this)(i, j) = m.mat[i][j];
		}
	}

}

// User constructor
Matrix::Matrix(int rownum , int colnum)
{

	row=rownum;
	col=colnum;
	init();
}

Matrix::~Matrix() {
	for(int i=0; i<row; i++)
	{
			free(this->mat[i]);
	}
	free(this->mat);
}

Matrix& Matrix::operator =(const Matrix& Other)
{
	row=Other.row;
	col=Other.col;

    // Destruction de la ressource (ici un pointeur brut)
	for (int i=0; i<row; i++)
	{
		free (mat[i]);
	}
    free(this->mat);

    // Allocation d'une nouvelle
    init();

    for (int i=0; i<this->row; i++)
	{
		// Handles columns of right Matrix
		for (int j=0; j<this->col; j++)
		{
			// Handles the multiplication
				this->mat[i][j] =Other.mat[i][j];
		}
	}
    return *this;
}

// Addition
Matrix operator+(Matrix const& a, Matrix const& b)
{
	Matrix copie;

	// Copie de la ressource
    for (int i=0; i<a.row; i++)
	{
		// Handles columns of right Matrix
		for (int j=0; j<a.col; j++)
		{
			// Handles the multiplication
				copie.mat[i][j] = a.mat[i][j] + b.mat[i][j];
		}
	}


    return copie;
}

// Substraction
Matrix operator-(Matrix const& a, Matrix const& b)
{
	Matrix copie;

	// Copie de la ressource
    for (int i=0; i<a.row; i++)
	{
		// Handles columns of right Matrix
		for (int j=0; j<a.col; j++)
		{
			// Handles the multiplication
				copie.mat[i][j] = a.mat[i][j] - b.mat[i][j];
		}
	}


    return copie;
}

// Multiplication
Matrix operator *(Matrix const& a, Matrix const& b)
{
	Matrix copie;
	int k;

	// Handles rows of left Matrix
	for (int i=0; i<a.row; i++)
	{
		// Handles columns of right Matrix
		for (int j=0; j<b.col; j++)
		{
			// Handles the multiplication
			for (k=0, copie.mat[i][j]=0.0; k<a.col; k++)
			{
				copie.mat[i][j] += a.mat[i][k] * b.mat[k][j];
			}
		}
	}

	return copie;
}



float& Matrix:: operator() (unsigned row, unsigned col)
{
	return this->mat[row][col];
}

// Returns the inverse of a 3x3 matrix
Matrix Matrix::invert_3x3 ()
{

	Matrix C (3,3);
	float det=	this->mat[0][0]*this->mat[1][1]*this->mat[2][2]
				+ this->mat[0][1]*this->mat[1][2]*this->mat[2][0]
				+ this->mat[0][2]*this->mat[1][0]*this->mat[2][1]
				- this->mat[0][2]*this->mat[1][1]*this->mat[2][0]
				- this->mat[1][2]*this->mat[2][1]*this->mat[0][0]
				- this->mat[2][2]*this->mat[0][1]*this->mat[1][0];



	C.mat[0][0]=(this->mat[1][1]*this->mat[2][2] - this->mat[1][2]*this->mat[2][1])/det;
	C.mat[0][1]=(this->mat[0][2]*this->mat[2][1] - this->mat[0][1]*this->mat[2][2])/det;
	C.mat[0][2]=(this->mat[0][1]*this->mat[2][1] - this->mat[1][1]*this->mat[2][0])/det;

	C.mat[1][0]=(this->mat[1][2]*this->mat[2][0] - this->mat[1][0]*this->mat[2][2])/det;
	C.mat[1][1]=(this->mat[0][0]*this->mat[2][2] - this->mat[0][2]*this->mat[2][0])/det;
	C.mat[1][2]=(this->mat[0][2]*this->mat[1][0] - this->mat[0][0]*this->mat[1][2])/det;

	C.mat[2][0]=(this->mat[1][0]*this->mat[2][1] - this->mat[1][1]*this->mat[2][0])/det;
	C.mat[2][1]=(this->mat[0][1]*this->mat[2][0] - this->mat[0][0]*this->mat[2][1])/det;
	C.mat[2][2]=(this->mat[0][0]*this->mat[1][1] - this->mat[0][1]*this->mat[1][0])/det;


	return C;
}

