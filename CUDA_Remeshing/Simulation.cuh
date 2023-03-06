#ifndef __SIMULATION_CUH__
#define __SIMULATION_CUH__

#pragma once
#include "Simulation.h"
#include "../include/CUDA_Custom/DeviceManager.cuh"

__global__ void initMasses_kernel(ObjParam obj, REAL* masses, uchar* isFixeds) {
	uint id = blockIdx.x * blockDim.x + threadIdx.x;
	if (id >= obj._numNodes)
		return;

	uchar isFixed = isFixeds[id];
	REAL m = masses[id];
	REAL invM = 1.0 / m;
	/*if (isFixed) {
		m = invM = 0.0;
	}*/
	obj._ms[id] = m;
	obj._invMs[id] = invM;
}
__global__ void initClothMasses_kernel(ClothParam cloth, REAL* masses, uchar* isFixeds) {
	uint id = blockIdx.x * blockDim.x + threadIdx.x;
	if (id >= cloth._numNodes)
		return;

	uchar isFixed = isFixeds[id];
	uint phase = cloth._nodePhases[id];
	REAL mf = cloth._mfs[id];
	REAL m = masses[id];

	m += mf;
	REAL invM = 1.0 / m;
	/*if (isFixed) {
		m = invM = 0.0;
	}*/
	cloth._ms[id] = m;
	cloth._invMs[id] = invM;
}
__global__ void compGravityForce_kernel(REAL* forces, REAL* ms, REAL3 gravity, uint numNodes) {
	uint id = blockIdx.x * blockDim.x + threadIdx.x;
	if (id >= numNodes)
		return;

	uint ino = id * 3u;

	REAL m = ms[id];

	REAL3 f;
	f.x = forces[ino + 0u];
	f.y = forces[ino + 1u];
	f.z = forces[ino + 2u];

	f += m * gravity;

	forces[ino + 0u] = f.x;
	forces[ino + 1u] = f.y;
	forces[ino + 2u] = f.z;
}
__global__ void compRotationForce_kernel(
	REAL* ns, REAL* vs, REAL* forces, REAL* ms, uint* nodePhases, 
	REAL3* pivots, REAL3* degrees, REAL invdt, uint numNodes) 
{
	uint id = threadIdx.x + blockDim.x * blockIdx.x;
	if (id >= numNodes)
		return;

	uint phase = nodePhases[id];
	REAL3 pivot = pivots[phase];
	REAL3 degree = degrees[phase];

	REAL m = ms[id];
	id *= 3u;

	REAL3 force;
	force.x = forces[id + 0u];
	force.y = forces[id + 1u];
	force.z = forces[id + 2u];

	degree.x *= M_PI * 0.00555555555555555555555555555556;
	degree.y *= M_PI * 0.00555555555555555555555555555556;
	degree.z *= M_PI * 0.00555555555555555555555555555556;

	REAL cx = cos(degree.x);
	REAL sx = sin(degree.x);
	REAL cy = cos(degree.y);
	REAL sy = -sin(degree.y);
	REAL cz = cos(degree.z);
	REAL sz = sin(degree.z);

	REAL3 x, px;
	x.x = ns[id + 0u];
	x.y = ns[id + 1u];
	x.z = ns[id + 2u];
	x -= pivot;

	px.x = x.x * cz * cy + x.y * (cz * sy * sx - sz * cx) + x.z * (cz * sy * cx + sz * sx);
	px.y = x.x * sz * cy + x.y * (sz * sy * sx + cz * cx) + x.z * (sz * sy * cx - cz * sx);
	px.z = x.x * -sy + x.y * cy * sx + x.z * cy * cx;

	px = invdt * (px - x);
	x.x = vs[id + 0u];
	x.y = vs[id + 1u];
	x.z = vs[id + 2u];
	force += m * invdt * (px - x);
	forces[id + 0u] = force.x;
	forces[id + 1u] = force.y;
	forces[id + 2u] = force.z;
}

__global__ void applyForce_kernel(REAL* vs, REAL* forces, REAL* invMs, uchar* isFixeds, REAL dt, uint numNodes) {
	uint id = blockIdx.x * blockDim.x + threadIdx.x;
	if (id >= numNodes)
		return;

	uint ino = id * 3u;

	REAL invM = invMs[id];
	uchar isFixed = isFixeds[id];
	if (isFixed) invM = 0.0;

	REAL3 v, f;

	v.x = vs[ino + 0u];
	v.y = vs[ino + 1u];
	v.z = vs[ino + 2u];
	f.x = forces[ino + 0u];
	f.y = forces[ino + 1u];
	f.z = forces[ino + 2u];

	v += dt * invM * f;

	vs[ino + 0u] = v.x;
	vs[ino + 1u] = v.y;
	vs[ino + 2u] = v.z;
}
__global__ void updateVelocity_kernel(
	REAL* n0s, REAL* n1s, REAL* vs, REAL invdt, uint numNodes)
{
	uint id = blockIdx.x * blockDim.x + threadIdx.x;
	if (id >= numNodes)
		return;

	uint ino = id * 3u;
	REAL3 n0, n1, v;

	n0.x = n0s[ino + 0u];
	n0.y = n0s[ino + 1u];
	n0.z = n0s[ino + 2u];
	n1.x = n1s[ino + 0u];
	n1.y = n1s[ino + 1u];
	n1.z = n1s[ino + 2u];

	v = invdt * (n1 - n0);

	vs[ino + 0u] = v.x;
	vs[ino + 1u] = v.y;
	vs[ino + 2u] = v.z;
}
__global__ void updatePosition_kernel(REAL* ns, REAL* vs, REAL dt, uint numNodes) {
	uint id = blockIdx.x * blockDim.x + threadIdx.x;
	if (id >= numNodes)
		return;

	uint ino = id * 3u;
	REAL3 n, v;

	n.x = ns[ino + 0u];
	n.y = ns[ino + 1u];
	n.z = ns[ino + 2u];
	v.x = vs[ino + 0u];
	v.y = vs[ino + 1u];
	v.z = vs[ino + 2u];

	n += dt * v;

	ns[ino + 0u] = n.x;
	ns[ino + 1u] = n.y;
	ns[ino + 2u] = n.z;
}

__global__ void Damping_kernel(
	REAL* vs, REAL w, uint numNodes)
{
	uint id = blockIdx.x * blockDim.x + threadIdx.x;
	if (id >= numNodes)
		return;

	uint ino = id * 3u;
	REAL3 v;

	v.x = vs[ino + 0u];
	v.y = vs[ino + 1u];
	v.z = vs[ino + 2u];

	v *= w;

	vs[ino + 0u] = v.x;
	vs[ino + 1u] = v.y;
	vs[ino + 2u] = v.z;
}
__global__ void Damping_kernel(
	REAL* vs, uchar* isFixeds, REAL w, uint numNodes)
{
	uint id = blockIdx.x * blockDim.x + threadIdx.x;
	if (id >= numNodes)
		return;
	if (!isFixeds[id]) {
		uint ino = id * 3u;
		REAL3 v;

		v.x = vs[ino + 0u];
		v.y = vs[ino + 1u];
		v.z = vs[ino + 2u];

		v *= w;

		vs[ino + 0u] = v.x;
		vs[ino + 1u] = v.y;
		vs[ino + 2u] = v.z;
	}
}

__device__ bool RayBoxTest_device(const M_Ray& ray, const AABB& aabb) {
	const float* dir;
	float minDiff[3], maxDiff[3];

	//float tmin[3], tmax[3];
	float tminMax = -FLT_MAX;
	float tmaxMin = FLT_MAX;
	float inv, tmp0, tmp1;

	dir = &ray._dir.x;
	minDiff[0] = aabb._min.x - ray._pos.x;
	minDiff[1] = aabb._min.y - ray._pos.y;
	minDiff[2] = aabb._min.z - ray._pos.z;
	maxDiff[0] = aabb._max.x - ray._pos.x;
	maxDiff[1] = aabb._max.y - ray._pos.y;
	maxDiff[2] = aabb._max.z - ray._pos.z;
	for (int i = 0; i < 3; i++) {
		inv = 1.0 / dir[i];
		minDiff[i] *= inv;
		maxDiff[i] *= inv;
		if (minDiff[i] < maxDiff[i]) {
			tmp0 = minDiff[i];
			tmp1 = maxDiff[i];
		}
		else {
			tmp1 = minDiff[i];
			tmp0 = maxDiff[i];
		}
		if (tminMax < tmp0)
			tminMax = tmp0;
		if (tmaxMin > tmp1)
			tmaxMin = tmp1;
	}
	return (tminMax <= tmaxMin);
}
__device__ bool RaySphereTest_device(
	const M_Ray& ray, const REAL3& center, const float radius,
	float& t)
{
	float3 diff = make_float3(center.x - ray._pos.x, center.y - ray._pos.y, center.z - ray._pos.z);
	float a = Dot(diff, diff);
	float b = Dot(diff, ray._dir);

	float d2 = a - b * b;
	float r2 = radius * radius;

	t = b - sqrtf(r2 - d2);
	return (a <= r2);
}
__global__ void rayCasing_kernel(ObjParam obj, M_Ray ray, float radius, float zFar, uint2* infos) {
	uint id = blockIdx.x * blockDim.x + threadIdx.x;
	if (id >= obj._numNodes)
		return;

	REAL3 n0;
	getVector(obj._ns, id, n0);
	
	float t = zFar;
	RaySphereTest_device(ray, n0, radius, t);

	uint2 info;
	info.x = 0xffffffff;
	info.y = id;
	if (t < zFar) {
		info.x = (uint)(t * 100.0f);
		//printf("CUDA %d, %d\n", info.y, info.x);
	}
	infos[id] = info;
}

#endif