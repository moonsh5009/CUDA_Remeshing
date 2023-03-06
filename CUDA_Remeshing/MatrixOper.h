#ifndef __MATRIX_OPERATION_H__
#define __MATRIX_OPERATION_H__

#pragma once
#include "MatrixClass.h"

namespace M_Matrix {
	inline __host__ __device__
		void operator+=(floatMatrix& A, const floatMatrix& B)
	{
		uint size = A._rowSize * A._colSize;
		for (uint i = 0u; i < size; i++)
			A[i] += B[i];
	}
	inline __host__ __device__
		void operator+=(floatMatrix& A, const float b)
	{
		uint size = A._rowSize * A._colSize;
		for (uint i = 0u; i < size; i++)
			A[i] += b;
	}
	inline __host__ __device__
		void operator-=(floatMatrix& A, const floatMatrix& B)
	{
		uint size = A._rowSize * A._colSize;
		for (uint i = 0u; i < size; i++)
			A[i] -= B[i];
	}
	inline __host__ __device__
		void operator-=(floatMatrix& A, const float b)
	{
		uint size = A._rowSize * A._colSize;
		for (uint i = 0u; i < size; i++)
			A[i] -= b;
	}
	inline __host__ __device__
		void operator*=(floatMatrix& A, const floatMatrix& B)
	{
		uint size = A._rowSize * A._colSize;
		for (uint i = 0u; i < size; i++)
			A[i] *= B[i];
	}
	inline __host__ __device__
		void operator*=(floatMatrix& A, const float b)
	{
		uint size = A._rowSize * A._colSize;
		for (uint i = 0u; i < size; i++)
			A[i] *= b;
	}
	inline __host__ __device__
		void operator/=(floatMatrix& A, const floatMatrix& B)
	{
		uint size = A._rowSize * A._colSize;
		for (uint i = 0u; i < size; i++)
			A[i] /= B[i];
	}
	inline __host__ __device__
		void operator/=(floatMatrix& A, const float b)
	{
		uint size = A._rowSize * A._colSize;
		for (uint i = 0u; i < size; i++)
			A[i] /= b;
	}

	inline __host__ __device__
		void operator+=(doubleMatrix& A, const doubleMatrix& B)
	{
		uint size = A._rowSize * A._colSize;
		for (uint i = 0u; i < size; i++)
			A[i] += B[i];
	}
	inline __host__ __device__
		void operator+=(doubleMatrix& A, const double b)
	{
		uint size = A._rowSize * A._colSize;
		for (uint i = 0u; i < size; i++)
			A[i] += b;
	}
	inline __host__ __device__
		void operator-=(doubleMatrix& A, const doubleMatrix& B)
	{
		uint size = A._rowSize * A._colSize;
		for (uint i = 0u; i < size; i++)
			A[i] -= B[i];
	}
	inline __host__ __device__
		void operator-=(doubleMatrix& A, const double b)
	{
		uint size = A._rowSize * A._colSize;
		for (uint i = 0u; i < size; i++)
			A[i] -= b;
	}
	inline __host__ __device__
		void operator*=(doubleMatrix& A, const doubleMatrix& B)
	{
		uint size = A._rowSize * A._colSize;
		for (uint i = 0u; i < size; i++)
			A[i] *= B[i];
	}
	inline __host__ __device__
		void operator*=(doubleMatrix& A, const double b)
	{
		uint size = A._rowSize * A._colSize;
		for (uint i = 0u; i < size; i++)
			A[i] *= b;
	}
	inline __host__ __device__
		void operator/=(doubleMatrix& A, const doubleMatrix& B)
	{
		uint size = A._rowSize * A._colSize;
		for (uint i = 0u; i < size; i++)
			A[i] /= B[i];
	}
	inline __host__ __device__
		void operator/=(doubleMatrix& A, const double b)
	{
		uint size = A._rowSize * A._colSize;
		for (uint i = 0u; i < size; i++)
			A[i] /= b;
	}

	inline __host__ __device__
		void Add(const floatMatrix& A, const floatMatrix& B, floatMatrix& X)
	{
		uint size = X._rowSize * X._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] = A[i] + B[i];
	}
	inline __host__ __device__
		void Add(const floatMatrix& A, const float b, floatMatrix& X)
	{
		uint size = X._rowSize * X._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] = A[i] + b;
	}
	inline __host__ __device__
		void Add(floatMatrix& X, const floatMatrix& A)
	{
		uint size = X._rowSize * X._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] += A[i];
	}
	inline __host__ __device__
		void Add(floatMatrix& X, const float a)
	{
		uint size = X._rowSize * X._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] += a;
	}

	inline __host__ __device__
		void Add(const doubleMatrix& A, const doubleMatrix& B, doubleMatrix& X)
	{
		uint size = X._rowSize * X._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] = A[i] + B[i];
	}
	inline __host__ __device__
		void Add(const doubleMatrix& A, const double b, doubleMatrix& X)
	{
		uint size = X._rowSize * X._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] = A[i] + b;
	}
	inline __host__ __device__
		void Add(doubleMatrix& X, const doubleMatrix& A)
	{
		uint size = X._rowSize * X._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] += A[i];
	}
	inline __host__ __device__
		void Add(doubleMatrix& X, const double a)
	{
		uint size = X._rowSize * X._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] += a;
	}

	inline __host__ __device__
		void Sub(const floatMatrix& A, const floatMatrix& B, floatMatrix& X)
	{
		uint size = X._rowSize * X._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] = A[i] - B[i];
	}
	inline __host__ __device__
		void Sub(const floatMatrix& A, const float b, floatMatrix& X)
	{
		uint size = X._rowSize * X._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] = A[i] - b;
	}
	inline __host__ __device__
		void Sub(floatMatrix& X, const floatMatrix& A)
	{
		uint size = X._rowSize * X._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] -= A[i];
	}
	inline __host__ __device__
		void Sub(floatMatrix& X, const float a)
	{
		uint size = X._rowSize * X._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] -= a;
	}

	inline __host__ __device__
		void Sub(const doubleMatrix& A, const doubleMatrix& B, doubleMatrix& X)
	{
		uint size = X._rowSize * X._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] = A[i] - B[i];
	}
	inline __host__ __device__
		void Sub(const doubleMatrix& A, const double b, doubleMatrix& X)
	{
		uint size = X._rowSize * X._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] = A[i] - b;
	}
	inline __host__ __device__
		void Sub(doubleMatrix& X, const doubleMatrix& A)
	{
		uint size = X._rowSize * X._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] -= A[i];
	}
	inline __host__ __device__
		void Sub(doubleMatrix& X, const double a)
	{
		uint size = X._rowSize * X._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] -= a;
	}

	/*inline __host__ __device__
		void Mult(const floatMatrix& A, const floatMatrix& B, floatMatrix& X)
	{
		uint size = X._rowSize * X._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] = A[i] * B[i];
	}*/
	inline __host__ __device__
		void Mult(const floatMatrix& A, const float b, floatMatrix& X)
	{
		uint size = X._rowSize * X._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] = A[i] * b;
	}
	inline __host__ __device__
		void Mult(floatMatrix& X, const float a)
	{
		uint size = X._rowSize * X._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] *= a;
	}
	/*inline __host__ __device__
		void Mult(const doubleMatrix& A, const doubleMatrix& B, doubleMatrix& X)
	{
		uint size = X._rowSize * X._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] = A[i] * B[i];
	}*/
	inline __host__ __device__
		void Mult(const doubleMatrix& A, const double b, doubleMatrix& X)
	{
		uint size = X._rowSize * X._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] = A[i] * b;
	}
	inline __host__ __device__
		void Mult(doubleMatrix& X, const double a)
	{
		uint size = X._rowSize * X._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] *= a;
	}

	inline __host__ __device__
		void Div(const floatMatrix& A, const floatMatrix& B, floatMatrix& X)
	{
		uint size = X._rowSize * X._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] = A[i] / B[i];
	}
	inline __host__ __device__
		void Div(const floatMatrix& A, const float b, floatMatrix& X)
	{
		uint size = X._rowSize * X._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] = A[i] / b;
	}
	inline __host__ __device__
		void Div(floatMatrix& X, const floatMatrix& A)
	{
		uint size = X._rowSize * X._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] /= A[i];
	}
	inline __host__ __device__
		void Div(floatMatrix& X, const float a)
	{
		uint size = X._rowSize * X._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] /= a;
	}

	inline __host__ __device__
		void Div(const doubleMatrix& A, const doubleMatrix& B, doubleMatrix& X)
	{
		uint size = X._rowSize * X._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] = A[i] / B[i];
	}
	inline __host__ __device__
		void Div(const doubleMatrix& A, const double b, doubleMatrix& X)
	{
		uint size = X._rowSize * X._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] = A[i] / b;
	}
	inline __host__ __device__
		void Div(doubleMatrix& X, const doubleMatrix& A)
	{
		uint size = X._rowSize * X._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] /= A[i];
	}
	inline __host__ __device__
		void Div(doubleMatrix& X, const double a)
	{
		uint size = X._rowSize * X._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] /= a;
	}

	inline __host__ __device__
		void Mult(const floatMatrix& A, const floatMatrix& B, floatMatrix& X)
	{
		if (A._colSize == B._rowSize && X._rowSize == A._rowSize && X._colSize == B._colSize) {
			for (uint row = 0u; row < X._rowSize; row++) {
				for (uint col = 0u; col < X._colSize; col++) {
					float tmp = 0.f;
					for (uint s = 0u; s < A._colSize; s++)
						tmp += A(row, s) * B(s, col);
					X(row, col) = tmp;
				}
			}
		}
		else {
			printf("Error : M_Matrix : Multi\n");
		}
	}
	inline __host__ __device__
		void Mult(const doubleMatrix& A, const doubleMatrix& B, doubleMatrix& X)
	{
		if (A._colSize == B._rowSize && X._rowSize == A._rowSize && X._colSize == B._colSize) {
			for (uint row = 0u; row < X._rowSize; row++) {
				for (uint col = 0u; col < X._colSize; col++) {
					double tmp = 0.0;
					for (uint s = 0u; s < A._colSize; s++)
						tmp += A(row, s) * B(s, col);
					X(row, col) = tmp;
				}
			}
		}
		else {
			printf("Error : M_Matrix : Multi\n");
		}
	}

	inline __host__ __device__
		float Dot(const floatMatrix& A, const floatMatrix& B)
	{
		float result = 0.f;
		uint size = A._rowSize * A._colSize;
		for (uint i = 0u; i < size; i++)
			result += A[i] * B[i];
		return result;
	}
	inline __host__ __device__
		double Dot(const doubleMatrix& A, const doubleMatrix& B)
	{
		double result = 0.f;
		uint size = A._rowSize * A._colSize;
		for (uint i = 0u; i < size; i++)
			result += A[i] * B[i];
		return result;
	}
}

#endif