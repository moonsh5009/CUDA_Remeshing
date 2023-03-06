#ifndef __MATRIX_H__
#define __MATRIX_H__

#pragma once
#include "MatrixOper.h"

namespace M_Matrix {
	inline __host__ __device__
		void Copy(floatMatrix& X, const floatMatrix& Y)
	{
		if (X._rowSize == Y._rowSize && X._colSize == Y._colSize) {
			uint size = X._rowSize * X._colSize;
			for (uint i = 0u; i < size; i++)
				X[i] = Y[i];
		}
		else {
			printf("Error : M_Matrix : Copy\n");
		}
	}
	inline __host__ __device__
		void Copy(doubleMatrix& X, const doubleMatrix& Y)
	{
		if (X._rowSize == Y._rowSize && X._colSize == Y._colSize) {
			uint size = X._rowSize * X._colSize;
			for (uint i = 0u; i < size; i++)
				X[i] = Y[i];
		}
		else {
			printf("Error : M_Matrix : Copy\n");
		}
	}
	inline __host__ __device__
		void Copy(doubleMatrix& X, const floatMatrix& Y)
	{
		if (X._rowSize == Y._rowSize && X._colSize == Y._colSize) {
			uint size = X._rowSize * X._colSize;
			for (uint i = 0u; i < size; i++)
				X[i] = (float)Y[i];
		}
		else {
			printf("Error : M_Matrix : Copy\n");
		}
	}
	inline __host__ __device__
		void Copy(floatMatrix& X, const doubleMatrix& Y)
	{
		if (X._rowSize == Y._rowSize && X._colSize == Y._colSize) {
			uint size = X._rowSize * X._colSize;
			for (uint i = 0u; i < size; i++)
				X[i] = (double)Y[i];
		}
		else {
			printf("Error : M_Matrix : Copy\n");
		}
	}

	inline __host__ __device__
		void Copy(floatMatrix& X, const float* Y)
	{
		uint size = X._rowSize * X._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] = Y[i];
	}
	inline __host__ __device__
		void Copy(doubleMatrix& X, const double* Y)
	{
		uint size = X._rowSize * X._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] = Y[i];
	}
	inline __host__ __device__
		void Copy(doubleMatrix& X, const float* Y)
	{
		uint size = X._rowSize * X._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] = (float)Y[i];
	}
	inline __host__ __device__
		void Copy(floatMatrix& X, const double* Y)
	{
		uint size = X._rowSize * X._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] = (double)Y[i];
	}

	inline  __host__ __device__
		void Copy(float* X, const floatMatrix& Y)
	{
		uint size = Y._rowSize * Y._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] = Y[i];
	}
	inline __host__ __device__
		void Copy(double* X, const doubleMatrix& Y)
	{
		uint size = Y._rowSize * Y._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] = Y[i];
	}
	inline __host__ __device__
		void Copy(float* X, const doubleMatrix& Y)
	{
		uint size = Y._rowSize * Y._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] = (float)Y[i];
	}
	inline __host__ __device__
		void Copy(double* X, const floatMatrix& Y)
	{
		uint size = Y._rowSize * Y._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] = (double)Y[i];
	}

	inline __host__ __device__
		void getCol(const floatMatrix& X, uint col, floatMatrix& V)
	{
		for (uint i = 0u; i < X._rowSize; i++)
			V[i] = X(i, col);
	}
	inline __host__ __device__
		void getCol(const floatMatrix& X, uint col, float* V)
	{
		for (uint i = 0u; i < X._rowSize; i++)
			V[i] = X(i, col);
	}
	inline __host__ __device__
		void getRow(const floatMatrix& X, uint row, floatMatrix& V)
	{
		for (uint i = 0u; i < X._colSize; i++)
			V[i] = X(row, i);
	}
	inline __host__ __device__
		void getRow(const floatMatrix& X, uint row, float* V)
	{
		for (uint i = 0u; i < X._colSize; i++)
			V[i] = X(row, i);
	}
	inline __host__ __device__
		void setCol(floatMatrix& X, uint col, const floatMatrix& V)
	{
		for (uint i = 0u; i < X._rowSize; i++)
			X(i, col) = V[i];
	}
	inline __host__ __device__
		void setCol(floatMatrix& X, uint col, const float* V)
	{
		for (uint i = 0u; i < X._rowSize; i++)
			X(i, col) = V[i];
	}
	inline __host__ __device__
		void setRow(floatMatrix& X, uint row, const floatMatrix& V)
	{
		for (uint i = 0u; i < X._colSize; i++)
			X(row, i) = V[i];
	}
	inline __host__ __device__
		void setRow(floatMatrix& X, uint row, const float* V)
	{
		for (uint i = 0u; i < X._colSize; i++)
			X(row, i) = V[i];
	}

	inline __host__ __device__
		void getCol(const doubleMatrix& X, uint col, doubleMatrix& V)
	{
		for (uint i = 0u; i < X._rowSize; i++)
			V[i] = X(i, col);
	}
	inline __host__ __device__
		void getCol(const doubleMatrix& X, uint col, double* V)
	{
		for (uint i = 0u; i < X._rowSize; i++)
			V[i] = X(i, col);
	}
	inline __host__ __device__
		void getRow(const doubleMatrix& X, uint row, doubleMatrix& V)
	{
		for (uint i = 0u; i < X._colSize; i++)
			V[i] = X(row, i);
	}
	inline __host__ __device__
		void getRow(const doubleMatrix& X, uint row, double* V)
	{
		for (uint i = 0u; i < X._colSize; i++)
			V[i] = X(row, i);
	}
	inline __host__ __device__
		void setCol(doubleMatrix& X, uint col, const doubleMatrix& V)
	{
		for (uint i = 0u; i < X._rowSize; i++)
			X(i, col) = V[i];
	}
	inline __host__ __device__
		void setCol(doubleMatrix& X, uint col, const double* V)
	{
		for (uint i = 0u; i < X._rowSize; i++)
			X(i, col) = V[i];
	}
	inline __host__ __device__
		void setRow(doubleMatrix& X, uint row, const doubleMatrix& V)
	{
		for (uint i = 0u; i < X._colSize; i++)
			X(row, i) = V[i];
	}
	inline __host__ __device__
		void setRow(doubleMatrix& X, uint row, const double* V)
	{
		for (uint i = 0u; i < X._colSize; i++)
			X(row, i) = V[i];
	}

	inline __host__ __device__
		void SubMatrix(const floatMatrix& X, uint startRow, uint startCol, floatMatrix* V)
	{
		for (uint row = 0u; row < V->_rowSize; row++)
			for (uint col = 0u; col < V->_colSize; col++)
				V->operator()(row, col) = X(row + startRow, col + startCol);
	}
	inline __host__ __device__
		void SubMatrix(const doubleMatrix& X, uint startRow, uint startCol, doubleMatrix* V)
	{
		for (uint row = 0u; row < V->_rowSize; row++)
			for (uint col = 0u; col < V->_colSize; col++)
				V->operator()(row, col) = X(row + startRow, col + startCol);
	}

	inline __host__ __device__
		void Cofactor(const floatMatrix& X, uint i, uint j, floatMatrix* V)
	{
		for (uint row = 0u; row < V->_rowSize; row++) {
			for (uint col = 0u; col < V->_colSize; col++) {
				if (row != i && col != j)
					V->operator()(row - (uint)(row > i), col - (uint)(col > j)) = X(row, col);
			}
		}
	}
	inline __host__ __device__
		void Cofactor(const doubleMatrix& X, uint i, uint j, doubleMatrix* V)
	{
		for (uint row = 0u; row < V->_rowSize; row++) {
			for (uint col = 0u; col < V->_colSize; col++) {
				if (row != i && col != j)
					V->operator()(row - (uint)(row > i), col - (uint)(col > j)) = X(row, col);
			}
		}
	}

	inline __host__ __device__
		void Clear(floatMatrix& X)
	{
		uint size = X._rowSize * X._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] = 0.f;
	}
	inline __host__ __device__
		void Clear(doubleMatrix& X)
	{
		uint size = X._rowSize * X._colSize;
		for (uint i = 0u; i < size; i++)
			X[i] = 0.0;
	}

	inline __host__ __device__
		void Identity(floatMatrix& X) {
		for (uint row = 0u; row < X._rowSize; row++) {
			for (uint col = 0u; col < X._colSize; col++) {
				if (row == col)
					X(row, col) = 1.f;
				else
					X(row, col) = 0.f;
			}
		}
	}
	inline __host__ __device__
		void Identity(doubleMatrix& X) {
		for (uint row = 0u; row < X._rowSize; row++) {
			for (uint col = 0u; col < X._colSize; col++) {
				if (row == col)
					X(row, col) = 1.0;
				else
					X(row, col) = 0.0;
			}
		}
	}

	inline __host__ __device__
		float Trace(const floatMatrix& X)
	{
		float result = 0.f;
		uint size = min(X._rowSize, X._colSize);
		for (uint i = 0u; i < size; i++)
			result += X(i, i);
		return result;
	}
	inline __host__ __device__
		double Trace(const doubleMatrix& X)
	{
		double result = 0.0;
		uint size = min(X._rowSize, X._colSize);
		for (uint i = 0u; i < size; i++)
			result += X(i, i);
		return result;
	}

	inline __host__ __device__
		float SquaredNorm(const floatMatrix& X)
	{
		float result = 0.f;
		uint size = X._rowSize * X._colSize;
		for (uint i = 0u; i < size; i++)
			result += X[i] * X[i];
		return result;
	}
	inline __host__ __device__
		double SquaredNorm(const doubleMatrix& X)
	{
		double result = 0.0;
		uint size = X._rowSize * X._colSize;
		for (uint i = 0u; i < size; i++)
			result += X[i] * X[i];
		return result;
	}

	inline __host__ __device__
		float Det(const float2x2& X)
	{
		return X[0] * X[3] - X[1] * X[2];
	}
	inline __host__ __device__
		float Det(const float3x3& X)
	{
		return 
			X[0] * X[4] * X[8] + 
			X[3] * X[7] * X[2] +
			X[6] * X[1] * X[5] -
			X[0] * X[7] * X[5] -
			X[6] * X[4] * X[2] -
			X[3] * X[1] * X[8];
	}
	inline __host__ __device__
		float Det(const float4x4& X)
	{
		float result = 0.f;
		float3x3 cof;

		Cofactor(X, 0, 0, &cof);
		result += X[0] * Det(cof);
		Cofactor(X, 0, 1, &cof);
		result -= X[1] * Det(cof);
		Cofactor(X, 0, 2, &cof);
		result += X[2] * Det(cof);
		Cofactor(X, 0, 3, &cof);
		result -= X[3] * Det(cof);

		return result;
	}
	inline __host__ __device__
		double Det(const double2x2& X)
	{
		return X[0] * X[3] - X[1] * X[2];
	}
	inline __host__ __device__
		double Det(const double3x3& X)
	{
		return
			X[0] * X[4] * X[8] +
			X[3] * X[7] * X[2] +
			X[6] * X[1] * X[5] -
			X[0] * X[7] * X[5] -
			X[6] * X[4] * X[2] -
			X[3] * X[1] * X[8];
	}
	inline __host__ __device__
		double Det(const double4x4& X)
	{
		double result = 0.f;
		double3x3 cof;

		Cofactor(X, 0, 0, &cof);
		result += X[0] * Det(cof);
		Cofactor(X, 0, 1, &cof);
		result -= X[1] * Det(cof);
		Cofactor(X, 0, 2, &cof);
		result += X[2] * Det(cof);
		Cofactor(X, 0, 3, &cof);
		result -= X[3] * Det(cof);

		return result;
	}

	inline __host__ __device__
		void Transpose(const floatMatrix& X, floatMatrix& T) 
	{
		if (X._rowSize == T._colSize && X._colSize == T._rowSize) {
			for (uint row = 0u; row < T._rowSize; row++)
				for (uint col = 0u; col < T._colSize; col++)
					T(row, col) = X(col, row);
		}
		else {
			printf("Error : M_Matrix : transpose\n");
		}
	}
	inline __host__ __device__
		void Transpose(const doubleMatrix& X, doubleMatrix& T)
	{
		if (X._rowSize == T._colSize && X._colSize == T._rowSize) {
			for (uint row = 0u; row < T._rowSize; row++)
				for (uint col = 0u; col < T._colSize; col++)
					T(row, col) = X(col, row);
		}
		else {
			printf("Error : M_Matrix : transpose\n");
		}
	}
	inline __host__ __device__
		void Transpose(float2x2& X)
	{
		float tmp;
		tmp = X[1]; X[1] = X[2]; X[2] = tmp;
	}
	inline __host__ __device__
		void Transpose(float3x3& X)
	{
		float tmp;
		tmp = X[1]; X[1] = X[3]; X[3] = tmp;
		tmp = X[2]; X[2] = X[6]; X[6] = tmp;
		tmp = X[5]; X[5] = X[7]; X[7] = tmp;
	}
	inline __host__ __device__
		void Transpose(double2x2& X)
	{
		double tmp;
		tmp = X[1]; X[1] = X[2]; X[2] = tmp;
	}
	inline __host__ __device__
		void Transpose(double3x3& X)
	{
		double tmp;
		tmp = X[1]; X[1] = X[3]; X[3] = tmp;
		tmp = X[2]; X[2] = X[6]; X[6] = tmp;
		tmp = X[5]; X[5] = X[7]; X[7] = tmp;
	}

	inline __host__ __device__
		void Inverse(const float2x2& X, float2x2& invX) {
		float inv = 1.f / Det(X);
		invX[0] = X[3] * inv;
		invX[1] = -X[1] * inv;
		invX[2] = -X[2] * inv;
		invX[3] = X[0] * inv;
	}
	inline __host__ __device__
		void Inverse(const float3x3& X, float3x3& invX) {
		float inv = 1.f / Det(X);
		invX[0] = inv * (X[4] * X[8] - X[5] * X[7]);
		invX[1] = inv * (X[2] * X[7] - X[1] * X[8]);
		invX[2] = inv * (X[1] * X[5] - X[2] * X[4]);

		invX[3] = inv * (X[5] * X[6] - X[3] * X[8]);
		invX[4] = inv * (X[0] * X[8] - X[2] * X[6]);
		invX[5] = inv * (X[2] * X[3] - X[0] * X[5]);

		invX[6] = inv * (X[3] * X[7] - X[4] * X[6]);
		invX[7] = inv * (X[1] * X[6] - X[0] * X[7]);
		invX[8] = inv * (X[0] * X[4] - X[1] * X[3]);
	}
	inline __host__ __device__
		void Inverse(const double2x2& X, double2x2& invX) {
		double inv = 1.0 / Det(X);
		invX[0] = X[3] * inv;
		invX[1] = -X[1] * inv;
		invX[2] = -X[2] * inv;
		invX[3] = X[0] * inv;
	}
	inline __host__ __device__
		void Inverse(const double3x3& X, double3x3& invX) {
		double inv = 1.0 / Det(X);
		invX[0] = inv * (X[4] * X[8] - X[5] * X[7]);
		invX[1] = inv * (X[2] * X[7] - X[1] * X[8]);
		invX[2] = inv * (X[1] * X[5] - X[2] * X[4]);

		invX[3] = inv * (X[5] * X[6] - X[3] * X[8]);
		invX[4] = inv * (X[0] * X[8] - X[2] * X[6]);
		invX[5] = inv * (X[2] * X[3] - X[0] * X[5]);

		invX[6] = inv * (X[3] * X[7] - X[4] * X[6]);
		invX[7] = inv * (X[1] * X[6] - X[0] * X[7]);
		invX[8] = inv * (X[0] * X[4] - X[1] * X[3]);
	}

	inline __host__ __device__
		void Print(const floatMatrix& X) 
	{
		printf("\n");
		for (uint row = 0u; row < X._rowSize; row++) {
			for (uint col = 0u; col < X._colSize; col++)
				printf(" %10.5f", X(row, col));
			printf("\n");
		}
		printf("\n");
	}
	inline __host__ __device__
		void Print(const doubleMatrix& X)
	{
		printf("\n");
		for (uint row = 0u; row < X._rowSize; row++) {
			for (uint col = 0u; col < X._colSize; col++)
				printf(" %10.5f", X(row, col));
			printf("\n");
		}
		printf("\n");
	}
}

#endif