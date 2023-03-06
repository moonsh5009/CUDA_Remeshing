#ifndef __MATRIX_CLASS_H__
#define __MATRIX_CLASS_H__

#pragma once
#include "../include/CUDA_Custom/DeviceManager.h"

namespace M_Matrix {
	/*class floatMatrix {
	public:
		float* _ptr;
		uint _rowSize;
		uint _colSize;
	public:
		__host__ __device__
			floatMatrix() {
			_ptr = nullptr;
		}
		__host__ __device__
			~floatMatrix() {}
	public:
		__host__ __device__
			float operator[](uint i) const
		{
			return _ptr[i];
		}
		__host__ __device__
			float operator()(uint row, uint col) const
		{
			return _ptr[(row * _rowSize) + col];
		}
		__host__ __device__
			float& operator[](uint i)
		{
			return _ptr[i];
		}
		__host__ __device__
			float& operator()(uint row, uint col)
		{
			return _ptr[(row * _rowSize) + col];
		}
	};
	class doubleMatrix {
	public:
		double* _ptr;
		uint _rowSize;
		uint _colSize;
	public:
		__host__ __device__
			doubleMatrix() {
			_ptr = nullptr;
		}
		__host__ __device__
			~doubleMatrix() {}
	public:
		__host__ __device__
			double operator[](uint i) const
		{
			return _ptr[i];
		}
		__host__ __device__
			double operator()(uint row, uint col) const
		{
			return _ptr[(row * _colSize) + col];
		}
		__host__ __device__
			double& operator[](uint i)
		{
			return _ptr[i];
		}
		__host__ __device__
			double& operator()(uint row, uint col)
		{
			return _ptr[(row * _colSize) + col];
		}
	};

	template<uint rowSize, uint colSize>
	class floatXxX : public floatMatrix {
	private:
		float _buffer[2];
	public:
		__host__ __device__
			floatXxX() {
			_ptr = _buffer;
			_rowSize = rowSize;
			_colSize = colSize;
		}
		__host__ __device__
			~floatXxX() {
		}
	};

	template<uint rowSize, uint colSize>
	class doubleXxX : public doubleMatrix {
	private:
		double _buffer[2];
	public:
		__host__ __device__
			doubleXxX() {
			_ptr = _buffer;
			_rowSize = rowSize;
			_colSize = colSize;
			printf("%d, %d\n", rowSize, colSize);
		}
		__host__ __device__
			~doubleXxX() {
		}
	};

	typedef floatXxX<1, 2> float1x2; typedef floatXxX<1, 3> float1x3; typedef floatXxX<1, 4> float1x4;
	typedef floatXxX<2, 1> float2x1; typedef floatXxX<2, 2> float2x2; typedef floatXxX<2, 3> float2x3; typedef floatXxX<2, 4> float2x4;
	typedef floatXxX<3, 1> float3x1; typedef floatXxX<3, 2> float3x2; typedef floatXxX<3, 3> float3x3; typedef floatXxX<3, 4> float3x4;
	typedef floatXxX<4, 1> float4x1; typedef floatXxX<4, 2> float4x2; typedef floatXxX<4, 3> float4x3; typedef floatXxX<4, 4> float4x4;
	typedef doubleXxX<1, 2> double1x2; typedef doubleXxX<1, 3> double1x3; typedef doubleXxX<1, 4> double1x4;
	typedef doubleXxX<2, 1> double2x1; typedef doubleXxX<2, 2> double2x2; typedef doubleXxX<2, 3> double2x3; typedef doubleXxX<2, 4> double2x4;
	typedef doubleXxX<3, 1> double3x1; typedef doubleXxX<3, 2> double3x2; typedef doubleXxX<3, 3> double3x3; typedef doubleXxX<3, 4> double3x4;
	typedef doubleXxX<4, 1> double4x1; typedef doubleXxX<4, 2> double4x2; typedef doubleXxX<4, 3> double4x3; typedef doubleXxX<4, 4> double4x4;
#if 1
	typedef doubleMatrix	REALMatrix;
	typedef double1x2 REAL1x2; typedef double1x3 REAL1x3; typedef double1x4 REAL1x4;
	typedef double2x1 REAL2x1; typedef double2x2 REAL2x2; typedef double2x3 REAL2x3; typedef double2x4 REAL2x4;
	typedef double3x1 REAL3x1; typedef double3x2 REAL3x2; typedef double3x3 REAL3x3; typedef double3x4 REAL3x4;
	typedef double4x1 REAL4x1; typedef double4x2 REAL4x2; typedef double4x3 REAL4x3; typedef double4x4 REAL4x4;
#else
	typedef floatMatrix	REALMatrix;
	typedef float1x2 REAL1x2; typedef float1x3 REAL1x3; typedef float1x4 REAL1x4;
	typedef float2x1 REAL2x1; typedef float2x2 REAL2x2; typedef float2x3 REAL2x3; typedef float2x4 REAL2x4;
	typedef float3x1 REAL3x1; typedef float3x2 REAL3x2; typedef float3x3 REAL3x3; typedef float3x4 REAL3x4;
	typedef float4x1 REAL4x1; typedef float4x2 REAL4x2; typedef float4x3 REAL4x3; typedef float4x4 REAL4x4;
#endif*/
	class floatMatrix;
	class float1x2; class float1x3; class float1x4;
	class float2x1; class float2x2; class float2x3; class float2x4;
	class float3x1; class float3x2; class float3x3; class float3x4;
	class float4x1; class float4x2; class float4x3; class float4x4;

	class doubleMatrix;
	class double1x2; class double1x3; class double1x4;
	class double2x1; class double2x2; class double2x3; class double2x4;
	class double3x1; class double3x2; class double3x3; class double3x4;
	class double4x1; class double4x2; class double4x3; class double4x4;

#if 1
	typedef doubleMatrix	REALMatrix;
	typedef double1x2 REAL1x2; typedef double1x3 REAL1x3; typedef double1x4 REAL1x4;
	typedef double2x1 REAL2x1; typedef double2x2 REAL2x2; typedef double2x3 REAL2x3; typedef double2x4 REAL2x4;
	typedef double3x1 REAL3x1; typedef double3x2 REAL3x2; typedef double3x3 REAL3x3; typedef double3x4 REAL3x4;
	typedef double4x1 REAL4x1; typedef double4x2 REAL4x2; typedef double4x3 REAL4x3; typedef double4x4 REAL4x4;
#else
	typedef floatMatrix	REALMatrix;
	typedef float1x2 REAL1x2; typedef float1x3 REAL1x3; typedef float1x4 REAL1x4;
	typedef float2x1 REAL2x1; typedef float2x2 REAL2x2; typedef float2x3 REAL2x3; typedef float2x4 REAL2x4;
	typedef float3x1 REAL3x1; typedef float3x2 REAL3x2; typedef float3x3 REAL3x3; typedef float3x4 REAL3x4;
	typedef float4x1 REAL4x1; typedef float4x2 REAL4x2; typedef float4x3 REAL4x3; typedef float4x4 REAL4x4;
#endif

	class floatMatrix {
	public:
		float* _ptr;
		uint _rowSize;
		uint _colSize;
	public:
		__host__ __device__
			floatMatrix() {
			_ptr = nullptr;
		}
		__host__ __device__
			~floatMatrix() {}
	public:
		__host__ __device__
			float operator[](uint i) const
		{
			return _ptr[i];
		}
		__host__ __device__
			float operator()(uint row, uint col) const
		{
			return _ptr[(row * _rowSize) + col];
		}
		__host__ __device__
			float& operator[](uint i)
		{
			return _ptr[i];
		}
		__host__ __device__
			float& operator()(uint row, uint col)
		{
			return _ptr[(row * _rowSize) + col];
		}
		__host__ __device__
			void operator=(const floatMatrix& X)
		{
			if (_rowSize == X._rowSize && _colSize == X._colSize) {
				uint size = X._rowSize * X._colSize;
				for (uint i = 0u; i < size; i++)
					_ptr[i] = X[i];
			}
			else
				printf("Error : M_Matrix : floatMatrix::operator=\n");
		}
	};
	class float1x2 : public floatMatrix {
	private:
		float _buffer[2];
	public:
		__host__ __device__
			float1x2() {
			_ptr = _buffer;
			_rowSize = 1u;
			_colSize = 2u;
		}
		__host__ __device__
			~float1x2() {
		}
	};
	class float1x3 : public floatMatrix {
	private:
		float _buffer[3];
	public:
		__host__ __device__
			float1x3() {
			_ptr = _buffer;
			_rowSize = 1u;
			_colSize = 3u;
		}
		__host__ __device__
			~float1x3() {
		}
	};
	class float1x4 : public floatMatrix {
	private:
		float _buffer[4];
	public:
		__host__ __device__
			float1x4() {
			_ptr = _buffer;
			_rowSize = 1u;
			_colSize = 4u;
		}
		__host__ __device__
			~float1x4() {
		}
	};
	class float2x1 : public floatMatrix {
	private:
		float _buffer[2];
	public:
		__host__ __device__
			float2x1()
		{
			_ptr = _buffer;
			_rowSize = 2u;
			_colSize = 1u;
		}
		__host__ __device__
			~float2x1() {
		}
	};
	class float2x2 : public floatMatrix {
	private:
		float _buffer[4];
	public:
		__host__ __device__
			float2x2()
		{
			_ptr = _buffer;
			_rowSize = 2u;
			_colSize = 2u;
		}
		__host__ __device__
			~float2x2() {
		}
	};
	class float2x3 : public floatMatrix {
	private:
		float _buffer[6];
	public:
		__host__ __device__
			float2x3()
		{
			_ptr = _buffer;
			_rowSize = 2u;
			_colSize = 3u;
		}
		__host__ __device__
			~float2x3() {
		}
	};
	class float2x4 : public floatMatrix {
	private:
		float _buffer[8];
	public:
		__host__ __device__
			float2x4()
		{
			_ptr = _buffer;
			_rowSize = 2u;
			_colSize = 4u;
		}
		__host__ __device__
			~float2x4() {
		}
	};
	class float3x1 : public floatMatrix {
	private:
		float _buffer[3];
	public:
		__host__ __device__
			float3x1()
		{
			_ptr = _buffer;
			_rowSize = 3u;
			_colSize = 1u;
		}
		__host__ __device__
			~float3x1() {
		}
	};
	class float3x2 : public floatMatrix {
	private:
		float _buffer[6];
	public:
		__host__ __device__
			float3x2()
		{
			_ptr = _buffer;
			_rowSize = 3u;
			_colSize = 2u;
		}
		__host__ __device__
			~float3x2() {
		}
	};
	class float3x3 : public floatMatrix {
	private:
		float _buffer[9];
	public:
		__host__ __device__
			float3x3()
		{
			_ptr = _buffer;
			_rowSize = 3u;
			_colSize = 3u;
		}
		__host__ __device__
			~float3x3() {
		}
	};
	class float3x4 : public floatMatrix {
	private:
		float _buffer[12];
	public:
		__host__ __device__
			float3x4()
		{
			_ptr = _buffer;
			_rowSize = 3u;
			_colSize = 4u;
		}
		__host__ __device__
			~float3x4() {
		}
	};
	class float4x1 : public floatMatrix {
	private:
		float _buffer[4];
	public:
		__host__ __device__
			float4x1()
		{
			_ptr = _buffer;
			_rowSize = 4u;
			_colSize = 1u;
		}
		__host__ __device__
			~float4x1() {
		}
	};
	class float4x2 : public floatMatrix {
	private:
		float _buffer[8];
	public:
		__host__ __device__
			float4x2()
		{
			_ptr = _buffer;
			_rowSize = 4u;
			_colSize = 2u;
		}
		__host__ __device__
			~float4x2() {
		}
	};
	class float4x3 : public floatMatrix {
	private:
		float _buffer[12];
	public:
		__host__ __device__
			float4x3()
		{
			_ptr = _buffer;
			_rowSize = 4u;
			_colSize = 3u;
		}
		__host__ __device__
			~float4x3() {
		}
	};
	class float4x4 : public floatMatrix {
	private:
		float _buffer[16];
	public:
		__host__ __device__
			float4x4()
		{
			_ptr = _buffer;
			_rowSize = 4u;
			_colSize = 4u;
		}
		__host__ __device__
			~float4x4() {
		}
	};

	class doubleMatrix {
	public:
		double* _ptr;
		uint _rowSize;
		uint _colSize;
	public:
		__host__ __device__
			doubleMatrix() {
			_ptr = nullptr;
		}
		__host__ __device__
			~doubleMatrix() {}
	public:
		__host__ __device__
			double operator[](uint i) const
		{
			return _ptr[i];
		}
		__host__ __device__
			double operator()(uint row, uint col) const
		{
			return _ptr[(row * _colSize) + col];
		}
		__host__ __device__
			double& operator[](uint i)
		{
			return _ptr[i];
		}
		__host__ __device__
			double& operator()(uint row, uint col)
		{
			return _ptr[(row * _colSize) + col];
		}
		__host__ __device__
			void operator=(const doubleMatrix& X)
		{
			if (_rowSize == X._rowSize && _colSize == X._colSize) {
				uint size = X._rowSize * X._colSize;
				for (uint i = 0u; i < size; i++)
					_ptr[i] = X[i];
			}
			else
				printf("Error : M_Matrix : doubleMatrix::operator=\n");
		}
	public:
	};
	class double1x2 : public doubleMatrix {
	private:
		double _buffer[2];
	public:
		__host__ __device__
			double1x2() {
			_ptr = _buffer;
			_rowSize = 1u;
			_colSize = 2u;
		}
		__host__ __device__
			~double1x2() {
		}
	};
	class double1x3 : public doubleMatrix {
	private:
		double _buffer[3];
	public:
		__host__ __device__
			double1x3() {
			_ptr = _buffer;
			_rowSize = 1u;
			_colSize = 3u;
		}
		__host__ __device__
			~double1x3() {
		}
	};
	class double1x4 : public doubleMatrix {
	private:
		double _buffer[4];
	public:
		__host__ __device__
			double1x4() {
			_ptr = _buffer;
			_rowSize = 1u;
			_colSize = 4u;
		}
		__host__ __device__
			~double1x4() {
		}
	};
	class double2x1 : public doubleMatrix {
	private:
		double _buffer[2];
	public:
		__host__ __device__
			double2x1()
		{
			_ptr = _buffer;
			_rowSize = 2u;
			_colSize = 1u;
		}
		__host__ __device__
			~double2x1() {
		}
	};
	class double2x2 : public doubleMatrix {
	private:
		double _buffer[4];
	public:
		__host__ __device__
			double2x2()
		{
			_ptr = _buffer;
			_rowSize = 2u;
			_colSize = 2u;
		}
		__host__ __device__
			~double2x2() {
		}
	};
	class double2x3 : public doubleMatrix {
	private:
		double _buffer[6];
	public:
		__host__ __device__
			double2x3()
		{
			_ptr = _buffer;
			_rowSize = 2u;
			_colSize = 3u;
		}
		__host__ __device__
			~double2x3() {
		}
	};
	class double2x4 : public doubleMatrix {
	private:
		double _buffer[8];
	public:
		__host__ __device__
			double2x4()
		{
			_ptr = _buffer;
			_rowSize = 2u;
			_colSize = 4u;
		}
		__host__ __device__
			~double2x4() {
		}
	};
	class double3x1 : public doubleMatrix {
	private:
		double _buffer[3];
	public:
		__host__ __device__
			double3x1()
		{
			_ptr = _buffer;
			_rowSize = 3u;
			_colSize = 1u;
		}
		__host__ __device__
			~double3x1() {
		}
	};
	class double3x2 : public doubleMatrix {
	private:
		double _buffer[6];
	public:
		__host__ __device__
			double3x2()
		{
			_ptr = _buffer;
			_rowSize = 3u;
			_colSize = 2u;
		}
		__host__ __device__
			~double3x2() {
		}
	};
	class double3x3 : public doubleMatrix {
	private:
		double _buffer[9];
	public:
		__host__ __device__
			double3x3()
		{
			_ptr = _buffer;
			_rowSize = 3u;
			_colSize = 3u;
		}
		__host__ __device__
			~double3x3() {
		}
	};
	class double3x4 : public doubleMatrix {
	private:
		double _buffer[12];
	public:
		__host__ __device__
			double3x4()
		{
			_ptr = _buffer;
			_rowSize = 3u;
			_colSize = 4u;
		}
		__host__ __device__
			~double3x4() {
		}
	};
	class double4x1 : public doubleMatrix {
	private:
		double _buffer[4];
	public:
		__host__ __device__
			double4x1()
		{
			_ptr = _buffer;
			_rowSize = 4u;
			_colSize = 1u;
		}
		__host__ __device__
			~double4x1() {
		}
	};
	class double4x2 : public doubleMatrix {
	private:
		double _buffer[8];
	public:
		__host__ __device__
			double4x2()
		{
			_ptr = _buffer;
			_rowSize = 4u;
			_colSize = 2u;
		}
		__host__ __device__
			~double4x2() {
		}
	};
	class double4x3 : public doubleMatrix {
	private:
		double _buffer[12];
	public:
		__host__ __device__
			double4x3()
		{
			_ptr = _buffer;
			_rowSize = 4u;
			_colSize = 3u;
		}
		__host__ __device__
			~double4x3() {
		}
	};
	class double4x4 : public doubleMatrix {
	private:
		double _buffer[16];
	public:
		__host__ __device__
			double4x4()
		{
			_ptr = _buffer;
			_rowSize = 4u;
			_colSize = 4u;
		}
		__host__ __device__
			~double4x4() {
		}
	};
}

#endif