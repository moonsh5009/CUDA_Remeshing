#ifndef __BVH_CUH__
#define __BVH_CUH__

#include "../include/CUDA_Custom/DeviceManager.cuh"
#include "BVH.h"

//-------------------------------------------------------------------
inline __forceinline__ __device__ uint splitBy3(uint x) {
	/*x = x & 0x1fffff;
	x = (x | x << 32ull) & 0x1f00000000ffff;
	x = (x | x << 16ull) & 0x1f0000ff0000ff;
	x = (x | x << 8ull) & 0x100f00f00f00f00f;
	x = (x | x << 4ull) & 0x10c30c30c30c30c3;
	x = (x | x << 2ull) & 0x1249249249249249;*/
	if (x == 1024u) x--;
	x = (x | x << 16u) & 0b00000011000000000000000011111111;
	x = (x | x << 8u) & 0b00000011000000001111000000001111;
	x = (x | x << 4u) & 0b00000011000011000011000011000011;
	x = (x | x << 2u) & 0b00001001001001001001001001001001;
	return x;
}
inline __forceinline__ __device__ uint getGridIndex(int3 p, uint3 size) {
	return __umul24(__umul24(((uint)p.z) & (size.z - 1), size.y) + (((uint)p.y) & (size.y - 1)), size.x) + (((uint)p.x) & (size.x - 1));
}
inline __forceinline__ __device__ uint getZindex(int3 p, uint3 size) {
	uint x = ((uint)(p.x + (size.x >> 1u))) & (size.x - 1u);
	uint y = ((uint)(p.y + (size.y >> 1u))) & (size.y - 1u);
	uint z = ((uint)(p.z + (size.z >> 1u))) & (size.z - 1u);

	return splitBy3(x) | splitBy3(y) << 1u | splitBy3(z) << 2u;

	/*uint id = 0u, i = 0u, bin0, bin1;
	while ((1u << i) <= x || (1u << i) <= y || (1u << i) <= z) {
		bin0 = 1u << i;
		bin1 = i << 1u;
		id |= (x & bin0) << bin1;
		id |= (y & bin0) << bin1 + 1u;
		id |= (z & bin0) << bin1 + 2u;
		i++;
	}

	return id;*/
}
inline __forceinline__ __device__ int3 getGridPos(const REAL3& x, REAL radius) {
	radius = 1.0 / radius;
	int3 p = make_int3(
		int(x.x * radius),
		int(x.y * radius),
		int(x.z * radius));
	return p;
}
//-------------------------------------------------------------------
inline __global__ void compDiameter_kernel(
	uint* fs, REAL* ns, REAL* diameter, uint numFaces)
{
	__shared__ REAL s_diameter[BLOCKSIZE];
	uint id = blockDim.x * blockIdx.x + threadIdx.x;
	if (id >= numFaces) {
		s_diameter[threadIdx.x] = 0.0;
		return;
	}

	uint ino = id * 3u;
	uint ino0 = fs[ino + 0u];
	uint ino1 = fs[ino + 1u];
	uint ino2 = fs[ino + 2u];
	REAL3 n0, n1, n2;
	getVector(ns, ino0, n0);
	getVector(ns, ino1, n1);
	getVector(ns, ino2, n2);

	float xs[3], ys[3], zs[3];
	xs[0] = fabsf((float)n0.x - (float)n1.x);
	xs[1] = fabsf((float)n0.x - (float)n2.x);
	xs[2] = fabsf((float)n1.x - (float)n2.x);
	ys[0] = fabsf((float)n0.y - (float)n1.y);
	ys[1] = fabsf((float)n0.y - (float)n2.y);
	ys[2] = fabsf((float)n1.y - (float)n2.y);
	zs[0] = fabsf((float)n0.z - (float)n1.z);
	zs[1] = fabsf((float)n0.z - (float)n2.z);
	zs[2] = fabsf((float)n1.z - (float)n2.z);
	for (int i = 1; i < 3; i++) {
		if (xs[0] < xs[i])
			xs[0] = xs[i];
		if (ys[0] < ys[i])
			ys[0] = ys[i];
		if (zs[0] < zs[i])
			zs[0] = zs[i];
	}
	if (xs[0] > ys[0] && xs[0] > zs[0])
		s_diameter[threadIdx.x] = xs[0];
	else if (ys[0] > zs[0])
		s_diameter[threadIdx.x] = ys[0];
	else
		s_diameter[threadIdx.x] = zs[0];
	/*REAL3 n01 = n1 - n0;
	REAL3 n02 = n2 - n0;
	REAL area = Length(Cross(n01, n02));
	REAL l01 = Length(n01);
	REAL l02 = Length(n02);
	REAL l12 = Length(n01 - n02);
	if (l01 > l02 && l01 > l12) 
		s_diameter[threadIdx.x] = area / l01;
	else if (l02 > l12) 
		s_diameter[threadIdx.x] = area / l02;
	else 
		s_diameter[threadIdx.x] = area / l12;*/

	for (uint s = blockDim.x >> 1u; s > 32u; s >>= 1u) {
		__syncthreads();
		if (threadIdx.x < s) {
			if (s_diameter[threadIdx.x] < s_diameter[threadIdx.x + s])
				s_diameter[threadIdx.x] = s_diameter[threadIdx.x + s];
		}
	}
	__syncthreads();

	if (threadIdx.x < 32u) {
		warpMax(s_diameter, threadIdx.x);
		if (threadIdx.x == 0)
			atomicMax_REAL(diameter, s_diameter[0]);
	}
}
inline __global__ void initTriInfo_kernel(
	uint* fs, REAL* ns, TriInfo* infos, REAL* diameter, uint numFaces)
{
	uint id = blockDim.x * blockIdx.x + threadIdx.x;
	if (id >= numFaces)
		return;

	uint ino = id * 3u;
	uint ino0 = fs[ino + 0u];
	uint ino1 = fs[ino + 1u];
	uint ino2 = fs[ino + 2u];
	REAL3 n0, n1, n2;
	getVector(ns, ino0, n0);
	getVector(ns, ino1, n1);
	getVector(ns, ino2, n2);
	
	REAL3 cen = (n0 + n1 + n2) / 3.0;
	int3 gridPos = getGridPos(cen, 0.005);
	TriInfo info;
	info._id = id;
	info._pos = getZindex(gridPos, make_uint3(512u, 512u, 512u));
	infos[id] = info;
}
inline __global__ void buildBVH_kernel(
	TriInfo* infos, BVHParam bvh)
{
	uint id = blockDim.x * blockIdx.x + threadIdx.x;
	if (id >= bvh._size)
		return;

	bvh._levels[id] = Log2(id + 1u);

	uint ileaf = bvh._size - bvh._numFaces;
	if (id >= ileaf) {
		uint fid;
		id -= ileaf;
		fid = id + bvh._pivot;
		if (fid >= bvh._numFaces)
			fid -= bvh._numFaces;

		TriInfo info = infos[fid];
		bvh._faces[id] = info._id;
	}
}
//-------------------------------------------------------------------
inline __device__ void RefitBVHLeaf(
	uint fid, AABB& aabb,
	const uint* fs, const REAL* ns,
	const uint* nodePhases, const REAL* thicknesses)
{
	fid *= 3u;
	uint ino0 = fs[fid + 0u];
	uint ino1 = fs[fid + 1u];
	uint ino2 = fs[fid + 2u];
	uint phase = nodePhases[ino0];
	REAL delta = thicknesses[phase];
	delta *= COL_CLEARANCE_RATIO;
	REAL3 p0, p1, p2;
	getVector(ns, ino0, p0);
	getVector(ns, ino1, p1);
	getVector(ns, ino2, p2);

	setAABB(aabb, p0, delta + 1.0e-10);
	addAABB(aabb, p1, delta + 1.0e-10);
	addAABB(aabb, p2, delta + 1.0e-10);
}
inline __device__ void RefitBVHLeaf(
	uint fid, AABB& aabb,
	const uint* fs, const REAL* ns, const REAL* vs,
	const uint* nodePhases, const REAL* thicknesses,
	const REAL dt)
{
	fid *= 3u;
	uint ino0 = fs[fid + 0u];
	uint ino1 = fs[fid + 1u];
	uint ino2 = fs[fid + 2u];
	uint phase = nodePhases[ino0];
	REAL delta = thicknesses[phase];

	REAL3 p0, p1, p2;
	getVector(ns, ino0, p0);
	getVector(ns, ino1, p1);
	getVector(ns, ino2, p2);

	setAABB(aabb, p0, delta * COL_CCD_THICKNESS * 1000.0 + 1.0e-10);
	addAABB(aabb, p1, delta * COL_CCD_THICKNESS * 1000.0 + 1.0e-10);
	addAABB(aabb, p2, delta * COL_CCD_THICKNESS * 1000.0 + 1.0e-10);

	REAL3 v0, v1, v2;
	getVector(vs, ino0, v0);
	getVector(vs, ino1, v1);
	getVector(vs, ino2, v2);
	p0 += v0 * dt; p1 += v1 * dt; p2 += v2 * dt;

	addAABB(aabb, p0, delta + 1.0e-10);
	addAABB(aabb, p1, delta + 1.0e-10);
	addAABB(aabb, p2, delta + 1.0e-10);
}
inline __global__ void RefitBVHKernel(
	BVHParam bvh, uint num)
{
	uint id = blockDim.x * blockIdx.x + threadIdx.x;
	if (id >= num)
		return;

	uint ind = num - 1u + id;
	uint ichild = (ind << 1) + 1u;
	AABB parent, lchild, rchild;
	getBVHAABB(lchild, bvh, ichild);
	getBVHAABB(rchild, bvh, ichild + 1u);

	setAABB(parent, lchild);
	addAABB(parent, rchild);
	updateBVHAABB(bvh, parent, ind);
}
inline __global__ void RefitNodeBVHKernel(
	BVHParam bvh, uint level)
{
	uint id = blockDim.x * blockIdx.x + threadIdx.x;
	uint currLev = level;
	uint ind0, ind, ichild;
	AABB parent, lchild, rchild;
	while (currLev > 5u) {
		ind0 = (1u << currLev--);
		if (id < ind0--) {
			ind = ind0 + id;
			ichild = (ind << 1u) + 1u;
			getBVHAABB(lchild, bvh, ichild);
			getBVHAABB(rchild, bvh, ichild + 1);

			setAABB(parent, lchild);
			addAABB(parent, rchild);
			updateBVHAABB(bvh, parent, ind);
		}
		__syncthreads();
	}
	while (currLev != 0xffffffff) {
		ind0 = (1u << currLev--);
		if (id < ind0--) {
			ind = ind0 + id;
			ichild = (ind << 1u) + 1u;
			getBVHAABB(lchild, bvh, ichild);
			getBVHAABB(rchild, bvh, ichild + 1u);

			setAABB(parent, lchild);
			addAABB(parent, rchild);
			updateBVHAABB(bvh, parent, ind);
		}
	}
}

inline __global__ void RefitLeafBVHKernel(
	uint* fs, REAL* ns, uint* nodePhases, REAL* thicknesses, BVHParam bvh, uint num)
{
	uint id = blockDim.x * blockIdx.x + threadIdx.x;
	if (id >= num)
		return;

	uint ind = num - 1u + id;
	uint ileaf = bvh._size - bvh._numFaces;
	AABB node;
	if (ind < ileaf) {
		uint ichild = (ind << 1u) + 1u;
		AABB lchild, rchild;
		uint lfid = bvh._faces[ichild - ileaf];
		uint rfid = bvh._faces[ichild + 1u - ileaf];
		RefitBVHLeaf(lfid, lchild, fs, ns, nodePhases, thicknesses);
		RefitBVHLeaf(rfid, rchild, fs, ns, nodePhases, thicknesses);
		updateBVHAABB(bvh, lchild, ichild);
		updateBVHAABB(bvh, rchild, ichild + 1u);

		setAABB(node, lchild);
		addAABB(node, rchild);
	}
	else {
		uint fid = bvh._faces[ind - ileaf];
		RefitBVHLeaf(fid, node, fs, ns, nodePhases, thicknesses);
	}
	updateBVHAABB(bvh, node, ind);
}
inline __global__ void RefitLeafBVHKernel(
	uint* fs, REAL* ns, REAL* vs, uint* nodePhases, REAL* thicknesses, BVHParam bvh, uint num,
	const REAL dt)
{
	uint id = blockDim.x * blockIdx.x + threadIdx.x;
	if (id >= num)
		return;

	uint ind = num - 1u + id;
	uint ileaf = bvh._size - bvh._numFaces;
	AABB node;
	if (ind < ileaf) {
		uint ichild = (ind << 1u) + 1u;
		AABB lchild, rchild;
		uint lfid = bvh._faces[ichild - ileaf];
		uint rfid = bvh._faces[ichild + 1u - ileaf];
		RefitBVHLeaf(lfid, lchild, fs, ns, vs, nodePhases, thicknesses, dt);
		RefitBVHLeaf(rfid, rchild, fs, ns, vs, nodePhases, thicknesses, dt);
		updateBVHAABB(bvh, lchild, ichild);
		updateBVHAABB(bvh, rchild, ichild + 1u);

		setAABB(node, lchild);
		addAABB(node, rchild);
	}
	else {
		uint fid = bvh._faces[ind - ileaf];
		RefitBVHLeaf(fid, node, fs, ns, vs, nodePhases, thicknesses, dt);
	}
	updateBVHAABB(bvh, node, ind);
}

#endif