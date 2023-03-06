#ifndef __PROJECTIVE_DYNAMICS_CUH__
#define __PROJECTIVE_DYNAMICS_CUH__

#pragma once
#include "ProjectiveDynamics.h"
#include "../include/CUDA_Custom/DeviceManager.cuh"

__global__ void initProject_kernel(
	ObjParam cloths, PDParam pdParam)
{
	uint id = threadIdx.x + blockDim.x * blockIdx.x;
	if (id >= cloths._numNodes)
		return;

	uint ino = id * 3u;
	REAL m = cloths._ms[id];
	REAL3 X, v;
	getVector(cloths._ns, id, X);
	getVector(cloths._vs, id, v);

	X += pdParam._dt * v;
	setVector(pdParam._Xs, id, X);
	X *= pdParam._invdt2 * m;
	setVector(pdParam._Zs, id, X);
}

__device__ REAL calcStVenantKirchhoffEnergyDensity(const REAL2x2& E, REAL firstLame, REAL secondLame)
{
	REAL trace = Trace(E);
	return secondLame * SquaredNorm(E) + 0.5 * firstLame * trace * trace;
}
__device__ void calcStVenantKirchhoffPiolaStress(const REAL3x2& F, const REAL2x2& E, REAL firstLame, REAL secondLame, REAL3x2& P)
{
	REAL trace = Trace(E);
	REAL3x2 tmp;
	Copy(tmp, F);
	Mult(F, E, P);
	Mult(P, 2.0 * secondLame);
	Mult(F, trace * firstLame, tmp);
	Add(P, tmp);
}
__device__ void convertVecToCrossOp(const REAL3& v, REAL3x3& X)
{
	X[0] = +0.0; X[1] = -v.z; X[2] = +v.y;
	X[3] = +v.z; X[4] = +0.0; X[5] = -v.x;
	X[6] = -v.y; X[7] = +v.x; X[8] = +0.0;
};
__device__ void multVVT(const REAL3& a, const REAL3& b, REAL3x3& X)
{
	X[0] = a.x * b.x; X[1] = a.x * b.y; X[2] = a.x * b.z;
	X[3] = a.y * b.x; X[4] = a.y * b.y; X[5] = a.y * b.z;
	X[6] = a.z * b.x; X[7] = a.z * b.y; X[8] = a.z * b.z;
};
__device__ void calc_grad_of_normalized_cross_prod(const REAL3& p, const REAL3& n, REAL3x3& X)
{
	REAL3x3 T;
	convertVecToCrossOp(p * -1.0, X);
	multVVT(n, Cross(n, p), T);
	Add(X, T);
};
__global__ void compEdgeErrorProject_kernel(
	ObjParam cloths, PDParam pdParam, EdgeConstraint constraint)
{
	uint id = threadIdx.x + blockDim.x * blockIdx.x;
	if (id >= constraint._numConstraints)
		return;

	uint ino = id << 1u;
	uint ino0 = constraint._inos[ino + 0u];
	uint ino1 = constraint._inos[ino + 1u];
	if (ino1 == 0xffffffff)
		return;

	uint phase = cloths._nodePhases[ino0];
	REAL w = constraint._ws[phase];

	REAL3 x0, x1;
	getVector(pdParam._Xs, ino0, x0);
	getVector(pdParam._Xs, ino1, x1);

	REAL3 d = x0 - x1;
	REAL restLength = constraint._restLengths[id];
	REAL length = Length(d);
	if (length > 1.0e-40) {
		REAL newL = restLength / length;
		d *= newL;
		x0 -= d;
		x1 += d;
	}
	REAL3 error0 = x1 * w;
	REAL3 error1 = x0 * w;

	/*REAL3 x01 = x1 - x0;
	REAL restLength = constraint._restLengths[id];
	REAL length = Length(x01);
	if (length > 1.0e-40) {
		REAL newL = (restLength - length) / length;
		x01 *= newL;
		x0 -= x01;
		x1 += x01;
	}
	REAL3 error0 = x0 * w;
	REAL3 error1 = x1 * w;*/

	atomicAdd_REAL(pdParam._Bs + ino0, w);
	atomicAdd_REAL(pdParam._Bs + ino1, w);

	ino0 *= 3u; ino1 *= 3u;
	atomicAdd_REAL(pdParam._newXs + ino0 + 0u, error0.x);
	atomicAdd_REAL(pdParam._newXs + ino0 + 1u, error0.y);
	atomicAdd_REAL(pdParam._newXs + ino0 + 2u, error0.z);
	atomicAdd_REAL(pdParam._newXs + ino1 + 0u, error1.x);
	atomicAdd_REAL(pdParam._newXs + ino1 + 1u, error1.y);
	atomicAdd_REAL(pdParam._newXs + ino1 + 2u, error1.z);
}
__global__ void compTriangleErrorProject_kernel(
	ObjParam cloths, PDParam pdParam, TriangleConstraint constraint)
{
	uint id = threadIdx.x + blockDim.x * blockIdx.x;
	if (id >= constraint._numConstraints)
		return;

	uint ino = id * 3u;
	uint ino0 = constraint._inos[ino + 0u];
	uint ino1 = constraint._inos[ino + 1u];
	uint ino2 = constraint._inos[ino + 2u];

	uint phase = cloths._nodePhases[ino0];
	REAL w = constraint._ws[phase];

	REAL w0 = cloths._isFixeds[ino0] ? 0.0 : cloths._invMs[ino0];
	REAL w1 = cloths._isFixeds[ino1] ? 0.0 : cloths._invMs[ino1];
	REAL w2 = cloths._isFixeds[ino2] ? 0.0 : cloths._invMs[ino2];

	REAL firstLame = constraint._firstLames[phase];
	REAL secondLame = constraint._secondLames[phase];
	REAL restArea = constraint._restAreas[id];

	REAL3 x0, x1, x2;
	getVector(pdParam._Xs, ino0, x0);
	getVector(pdParam._Xs, ino1, x1);
	getVector(pdParam._Xs, ino2, x2);
	
	REAL3 x01 = x1 - x0;
	REAL3 x02 = x2 - x0;

	REAL3x2 D, F, P;
	REAL2x3 FT;
	REAL2x2 invD, E;
	setCol(D, 0, &x01.x);
	setCol(D, 1, &x02.x);
	Copy(invD, constraint._invDs + (id << 2u));

	Mult(D, invD, F);
	Transpose(F, FT);

	Mult(FT, F, E);
	E[0] = 0.5 * (E[0] - 1.0);
	E[1] = 0.5 * E[1];
	E[2] = 0.5 * E[2];
	E[3] = 0.5 * (E[3] - 1.0);

	REAL density = restArea * calcStVenantKirchhoffEnergyDensity(E, firstLame, secondLame);
	calcStVenantKirchhoffPiolaStress(F, E, firstLame, secondLame, P);
	Transpose(invD);
	Mult(P, invD, D);
	Mult(D, restArea);

	REAL3 grad0, grad1, grad2;
	getCol(D, 0, &grad1.x);
	getCol(D, 1, &grad2.x);
	grad0.x = -grad1.x - grad2.x;
	grad0.y = -grad1.y - grad2.y;
	grad0.z = -grad1.z - grad2.z;
	
	REAL s = -2.0 * density / (LengthSquared(grad0) * w0 + LengthSquared(grad1) * w1 + LengthSquared(grad2) * w2 + FLT_EPSILON);
	grad0 *= s * w0;
	grad1 *= s * w1;
	grad2 *= s * w2;

	REAL3 error0 = w * (x0 + grad0);
	REAL3 error1 = w * (x1 + grad1);
	REAL3 error2 = w * (x2 + grad2);

	atomicAdd_REAL(pdParam._Bs + ino0, w);
	atomicAdd_REAL(pdParam._Bs + ino1, w);
	atomicAdd_REAL(pdParam._Bs + ino2, w);

	ino0 *= 3u; ino1 *= 3u; ino2 *= 3u;
	atomicAdd_REAL(pdParam._newXs + ino0 + 0u, error0.x);
	atomicAdd_REAL(pdParam._newXs + ino0 + 1u, error0.y);
	atomicAdd_REAL(pdParam._newXs + ino0 + 2u, error0.z);
	atomicAdd_REAL(pdParam._newXs + ino1 + 0u, error1.x);
	atomicAdd_REAL(pdParam._newXs + ino1 + 1u, error1.y);
	atomicAdd_REAL(pdParam._newXs + ino1 + 2u, error1.z);
	atomicAdd_REAL(pdParam._newXs + ino2 + 0u, error2.x);
	atomicAdd_REAL(pdParam._newXs + ino2 + 1u, error2.y);
	atomicAdd_REAL(pdParam._newXs + ino2 + 2u, error2.z);
}
__global__ void compDihedralErrorProject_kernel(
	ObjParam cloths, PDParam pdParam, DihedralConstraint constraint)
{
	uint id = threadIdx.x + blockDim.x * blockIdx.x;
	if (id >= constraint._numConstraints)
		return;

	uint ino = id << 1u;
	uint ino0 = cloths._es[ino + 0u];
	uint ino1 = cloths._es[ino + 1u];
	uint ino2 = constraint._inos[ino + 0u];
	uint ino3 = constraint._inos[ino + 1u];
	if (ino3 == 0xffffffff)
		return;

	uint phase = cloths._nodePhases[ino0];
	REAL w = constraint._ws[phase];

	REAL w0 = cloths._isFixeds[ino0] ? 0.0 : cloths._invMs[ino0];
	REAL w1 = cloths._isFixeds[ino1] ? 0.0 : cloths._invMs[ino1];
	REAL w2 = cloths._isFixeds[ino2] ? 0.0 : cloths._invMs[ino2];
	REAL w3 = cloths._isFixeds[ino3] ? 0.0 : cloths._invMs[ino3];

	REAL3 x0, x1, x2, x3;
	getVector(pdParam._Xs, ino0, x0);
	getVector(pdParam._Xs, ino1, x1);
	getVector(pdParam._Xs, ino2, x2);
	getVector(pdParam._Xs, ino3, x3);

	REAL3 x01 = x1 - x0;
	REAL3 x02 = x2 - x0;
	REAL3 x03 = x3 - x0;
	REAL3 norm12 = Cross(x01, x02);
	REAL3 norm13 = Cross(x01, x03);
	REAL lnorm12 = Length(norm12);
	REAL lnorm13 = Length(norm13);
	if (lnorm12 < 1.0e-40 || lnorm13 < 1.0e-40)
		return;

	lnorm12 = 1.0 / lnorm12;
	lnorm13 = 1.0 / lnorm13;
	norm12 *= lnorm12;
	norm13 *= lnorm13;

	REAL dot23 = min(max(Dot(norm12, norm13), -1.0), 1.0);
	REAL common_coeff = 1.0 - dot23 * dot23;
	if (common_coeff < 1.0e-20)
		return;

	REAL angle = (-0.6981317 * dot23 * dot23 - 0.8726646) * dot23 + 1.570796; //acos(x0dotx1);
	REAL restAngle = constraint._restAngles[id];

	REAL3 grad0, grad1, grad2, grad3;
	REAL3x3 T0, T1;
	calc_grad_of_normalized_cross_prod(x02, norm12, T0);
	calc_grad_of_normalized_cross_prod(x03, norm13, T1);
	Mult(T0, lnorm12);
	Mult(T1, lnorm13);
	grad1.x = T0[0] * norm13.x + T0[3] * norm13.y + T0[6] * norm13.z;
	grad1.y = T0[1] * norm13.x + T0[4] * norm13.y + T0[7] * norm13.z;
	grad1.z = T0[2] * norm13.x + T0[5] * norm13.y + T0[8] * norm13.z;
	grad1.x += T1[0] * norm12.x + T1[3] * norm12.y + T1[6] * norm12.z;
	grad1.y += T1[1] * norm12.x + T1[4] * norm12.y + T1[7] * norm12.z;
	grad1.z += T1[2] * norm12.x + T1[5] * norm12.y + T1[8] * norm12.z;

	calc_grad_of_normalized_cross_prod(x01, norm12, T0);
	calc_grad_of_normalized_cross_prod(x01, norm13, T1);
	Mult(T0, -lnorm12);
	Mult(T1, -lnorm13);
	grad2.x = T0[0] * norm13.x + T0[3] * norm13.y + T0[6] * norm13.z;
	grad2.y = T0[1] * norm13.x + T0[4] * norm13.y + T0[7] * norm13.z;
	grad2.z = T0[2] * norm13.x + T0[5] * norm13.y + T0[8] * norm13.z;
	grad3.x = T1[0] * norm12.x + T1[3] * norm12.y + T1[6] * norm12.z;
	grad3.y = T1[1] * norm12.x + T1[4] * norm12.y + T1[7] * norm12.z;
	grad3.z = T1[2] * norm12.x + T1[5] * norm12.y + T1[8] * norm12.z;

	common_coeff = -1.0 / sqrt(common_coeff);
	grad1 *= common_coeff;
	grad2 *= common_coeff;
	grad3 *= common_coeff;
	grad0.x = -grad1.x - grad2.x - grad3.x;
	grad0.y = -grad1.y - grad2.y - grad3.y;
	grad0.z = -grad1.z - grad2.z - grad3.z;

	REAL s = -2.0 * (angle - restAngle) /
		(LengthSquared(grad0) * w0 + LengthSquared(grad1) * w1 + LengthSquared(grad2) * w2 + LengthSquared(grad3) * w3 + FLT_EPSILON);
	grad0 *= s * w0;
	grad1 *= s * w1;
	grad2 *= s * w2;
	grad3 *= s * w3;

	REAL3 error0 = w * (x0 + grad0);
	REAL3 error1 = w * (x1 + grad1);
	REAL3 error2 = w * (x2 + grad2);
	REAL3 error3 = w * (x3 + grad3);

	atomicAdd_REAL(pdParam._Bs + ino0, w);
	atomicAdd_REAL(pdParam._Bs + ino1, w);
	atomicAdd_REAL(pdParam._Bs + ino2, w);
	atomicAdd_REAL(pdParam._Bs + ino3, w);

	ino0 *= 3u; ino1 *= 3u; ino2 *= 3u; ino3 *= 3u;
	atomicAdd_REAL(pdParam._newXs + ino0 + 0u, error0.x);
	atomicAdd_REAL(pdParam._newXs + ino0 + 1u, error0.y);
	atomicAdd_REAL(pdParam._newXs + ino0 + 2u, error0.z);
	atomicAdd_REAL(pdParam._newXs + ino1 + 0u, error1.x);
	atomicAdd_REAL(pdParam._newXs + ino1 + 1u, error1.y);
	atomicAdd_REAL(pdParam._newXs + ino1 + 2u, error1.z);
	atomicAdd_REAL(pdParam._newXs + ino2 + 0u, error2.x);
	atomicAdd_REAL(pdParam._newXs + ino2 + 1u, error2.y);
	atomicAdd_REAL(pdParam._newXs + ino2 + 2u, error2.z);
	atomicAdd_REAL(pdParam._newXs + ino3 + 0u, error3.x);
	atomicAdd_REAL(pdParam._newXs + ino3 + 1u, error3.y);
	atomicAdd_REAL(pdParam._newXs + ino3 + 2u, error3.z);
}

__global__ void updateXsProject_kernel(
	ObjParam cloths, PDParam pdParam, REAL* maxError)
{
	extern __shared__ REAL s_maxError[];
	uint id = threadIdx.x + blockDim.x * blockIdx.x;
	uint ino;

	s_maxError[threadIdx.x] = 0.0;
	if (id < cloths._numNodes) {
		uchar isFixed = cloths._isFixeds[id];
		if (!isFixed > 0.0) {
			REAL m = cloths._ms[id];
			REAL b = pdParam._Bs[id];
			REAL3 X, prevX, newX;
			getVector(pdParam._Xs, id, X);

			if (b > 0.0) {
				getVector(pdParam._prevXs, id, prevX);
				getVector(pdParam._newXs, id, newX);

				newX *= 1.0 / (b + m * pdParam._invdt2);
				newX = pdParam._omg * (pdParam._underRelax * (newX - X) + X - prevX) + prevX;

				setVector(pdParam._Xs, id, newX);

				s_maxError[threadIdx.x] = Length(newX - X);
			}
			setVector(pdParam._prevXs, id, X);
		}
	}
	for (ino = blockDim.x >> 1u; ino > 32u; ino >>= 1u) {
		__syncthreads();
		if (threadIdx.x < ino)
			if (s_maxError[threadIdx.x] < s_maxError[threadIdx.x + ino])
				s_maxError[threadIdx.x] = s_maxError[threadIdx.x + ino];
	}
	__syncthreads();
	if (threadIdx.x < 32u) {
		warpMax(s_maxError, threadIdx.x);
		if (threadIdx.x == 0u)
			atomicMax_REAL(maxError, s_maxError[0]);
	}
}

#endif