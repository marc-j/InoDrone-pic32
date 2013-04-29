/*
 * QCMath.cpp
 *
 *  Created on: 27 janv. 2013
 *      Author: marc
 */

#include "QCMath.h"

float vectorDotProduct(int length, float vector1[], float vector2[])
{
  float dotProduct = 0;
  //int   i;

  for (int i = 0; i < length; i++)
  {
  dotProduct += vector1[i] * vector2[i];
  }

  return (dotProduct);
}

void vectorCrossProduct(float vectorC[3], float vectorA[3], float vectorB[3])
{
  vectorC[0] = (vectorA[1] * vectorB[2]) - (vectorA[2] * vectorB[1]);
  vectorC[1] = (vectorA[2] * vectorB[0]) - (vectorA[0] * vectorB[2]);
  vectorC[2] = (vectorA[0] * vectorB[1]) - (vectorA[1] * vectorB[0]);
}

/**
*  Subtract vector a from vector b, both of length m
*/
void vectorSubtract(int length, float vectorC[], float vectorA[], float vectorB[])
{
  for(int i = 0; i < length; i++)
  {
     vectorC[i] = vectorA[i] - vectorB[i];
  }
}

void vectorScale(int length, float scaledVector[], float inputVector[], float scalar)
{
  for (int i = 0; i < length; i++)
  {
   scaledVector[i] = inputVector[i] * scalar;
  }
}

/**
 *  Compute sum of 2 vectors
 *  Add vector a to vector b, both of length m
 *  Place result in vector c
 *
 *  Call as: vectorAdd(m, c, b, a)
 */
void vectorAdd(int length, float vectorC[], float vectorA[], float vectorB[])
{
  for(int i = 0; i < length; i++)
  {
     vectorC[i] = vectorA[i] + vectorB[i];
  }
}

/**
 * Matrix Multiply
 *  Multiply matrix A times matrix B, matrix A dimension m x n, matrix B dimension n x p
 *  Result placed in matrix C, dimension m x p
 *
 * Call as: matrixMultiply(m, n, p, C, A, B)
 */
void matrixMultiply(int aRows, int aCols_bRows, int bCols, float matrixC[], float matrixA[], float matrixB[])
{
  for (int i = 0; i < aRows * bCols; i++)
  {
    matrixC[i] = 0.0;
  }

  for (int i = 0; i < aRows; i++)
  {
    for(int j = 0; j < aCols_bRows; j++)
    {
      for(int k = 0;  k < bCols; k++)
      {
       matrixC[i * bCols + k] += matrixA[i * aCols_bRows + j] * matrixB[j * bCols + k];
      }
    }
  }
}

/**
 *  Matrix Addition
 *  Add matrix A to matrix B, dimensions m x n
 *  Result placed in matrix C, dimension m x n
 *
 *  Call as: matrixAdd(m, n, C, A, B)
 */
void matrixAdd(int rows, int cols, float matrixC[], float matrixA[], float matrixB[])
{
  for (int i = 0; i < rows * cols; i++)
  {
    matrixC[i] = matrixA[i] + matrixB[i];
  }
}

