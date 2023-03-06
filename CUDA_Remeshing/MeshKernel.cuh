#ifndef __MESH_KERNEL_CUH__
#define __MESH_KERNEL_CUH__

#include "MeshKernel.h"
#include "../include/CUDA_Custom/DeviceManager.cuh"

__global__ void initNbs_kernel(
	uint* nbs, uint* buffers, uint numNbs)
{
	uint id = blockIdx.x * blockDim.x + threadIdx.x;
	if (id >= numNbs)
		return;

	uint nb = buffers[(id << 1u) + 1u];
	nbs[id] = nb;
}
__global__ void getNbFsBuffer_kernel(
	uint* fs, uint2* buffers, uint numFaces)
{
	uint id = blockIdx.x * blockDim.x + threadIdx.x;
	if (id >= numFaces)
		return;

	uint ino = id * 3u;
	uint ino0 = fs[ino + 0u];
	uint ino1 = fs[ino + 1u];
	uint ino2 = fs[ino + 2u];
	buffers[ino + 0u] = make_uint2(ino0, id);
	buffers[ino + 1u] = make_uint2(ino1, id);
	buffers[ino + 2u] = make_uint2(ino2, id);
}
__global__ void getNbENsBuffer_kernel(
	uint* es, uint2* ebuffers, uint2* nbuffers, uint numEdges)
{
	uint id = blockIdx.x * blockDim.x + threadIdx.x;
	if (id >= numEdges)
		return;

	uint ino = id << 1u;
	uint ino0 = es[ino + 0u];
	uint ino1 = es[ino + 1u];
	ebuffers[ino + 0u] = make_uint2(ino0, id);
	ebuffers[ino + 1u] = make_uint2(ino1, id);
	nbuffers[ino + 0u] = make_uint2(ino0, ino1);
	nbuffers[ino + 1u] = make_uint2(ino1, ino0);
}
__global__ void buildEnbXs_kernel(
	uint* fs, uint* es, uint* inbFs, uint* nbFs,
	uint* EnbFs, uint* EnbNs, uint numEdges)
{
	uint id = blockIdx.x * blockDim.x + threadIdx.x;
	if (id >= numEdges)
		return;

	uint ino = id << 1u;
	uint ino0 = es[ino + 0u];
	uint ino1 = es[ino + 1u];

	uint EnbF[2], EnbN[2];
	EnbF[0] = EnbF[1] = EnbN[0] = EnbN[1] = 0xffffffff;

	uint num = 0u;
	for (uint inbF0 = inbFs[ino0]; inbF0 < inbFs[ino0 + 1u]; inbF0++) {
		uint nbF0 = nbFs[inbF0];
		for (uint inbF1 = inbFs[ino1]; inbF1 < inbFs[ino1 + 1u]; inbF1++) {
			uint nbF1 = nbFs[inbF1];

			if (nbF0 == nbF1) {
				EnbF[num] = nbF0;
				for (uint i = 0u; i < 3u; i++) {
					uint node = fs[nbF0 * 3u + i];
					if (node != ino0 && node != ino1) {
						EnbN[num] = node;
						break;
					}
				}
				num++;
				break;
			}
		}
		if (num == 2u)
			break;
	}
	EnbFs[ino + 0u] = EnbF[0];
	EnbFs[ino + 1u] = EnbF[1];
	EnbNs[ino + 0u] = EnbN[0];
	EnbNs[ino + 1u] = EnbN[1];
}
__global__ void buildFnbEs_kernel(
	uint* fs, uint* inbEs, uint* nbEs, uint* FnbEs, uint numFaces)
{
	uint id = blockIdx.x * blockDim.x + threadIdx.x;
	if (id >= numFaces)
		return;

	uint ino = id * 3u;
	uint inos[3];
	inos[0] = fs[ino + 0u];
	inos[1] = fs[ino + 1u];
	inos[2] = fs[ino + 2u];

	uint inbEEnd, jnbEEnd;
	uint inbE, jnbE;
	uint nbEi, nbEj;
	for (uint i = 0u; i < 3u; i++) {
		uint j = (i + 1u) % 3u;

		inbEEnd = inbEs[inos[i] + 1u];
		for (inbE = inbEs[inos[i]]; inbE < inbEEnd; inbE++) {
			nbEi = nbEs[inbE];

			jnbEEnd = inbEs[inos[j] + 1u];
			for (jnbE = inbEs[inos[j]]; jnbE < jnbEEnd; jnbE++) {
				nbEj = nbEs[jnbE];
				if (nbEi == nbEj) {
					FnbEs[ino + (3u - i - j)] = nbEi;
					break;
				}
			}
			if (jnbE < jnbEEnd)
				break;
		}
	}
}

__global__ void compNormals_kernel(uint* fs, REAL* ns, REAL* fNorms, REAL* nNorms, uint numFaces) {
	uint id = threadIdx.x + blockDim.x * blockIdx.x;
	if (id >= numFaces)
		return;

	id *= 3u;
	uint iv0 = fs[id + 0u]; iv0 *= 3u;
	uint iv1 = fs[id + 1u]; iv1 *= 3u;
	uint iv2 = fs[id + 2u]; iv2 *= 3u;

	REAL3 v0, v1, v2;
	v0.x = ns[iv0 + 0u]; v0.y = ns[iv0 + 1u]; v0.z = ns[iv0 + 2u];
	v1.x = ns[iv1 + 0u]; v1.y = ns[iv1 + 1u]; v1.z = ns[iv1 + 2u];
	v2.x = ns[iv2 + 0u]; v2.y = ns[iv2 + 1u]; v2.z = ns[iv2 + 2u];

	REAL3 norm = Cross(v1 - v0, v2 - v0);
	Normalize(norm);

	fNorms[id + 0u] = norm.x;
	fNorms[id + 1u] = norm.y;
	fNorms[id + 2u] = norm.z;

	REAL radian = AngleBetweenVectors(v1 - v0, v2 - v0);
	//radian = 1.0;
	atomicAdd_REAL(nNorms + iv0 + 0u, norm.x * radian);
	atomicAdd_REAL(nNorms + iv0 + 1u, norm.y * radian);
	atomicAdd_REAL(nNorms + iv0 + 2u, norm.z * radian);

	radian = AngleBetweenVectors(v2 - v1, v0 - v1);
	//radian = 1.0;
	atomicAdd_REAL(nNorms + iv1 + 0u, norm.x * radian);
	atomicAdd_REAL(nNorms + iv1 + 1u, norm.y * radian);
	atomicAdd_REAL(nNorms + iv1 + 2u, norm.z * radian);

	radian = AngleBetweenVectors(v0 - v2, v1 - v2);
	//radian = 1.0;
	atomicAdd_REAL(nNorms + iv2 + 0u, norm.x * radian);
	atomicAdd_REAL(nNorms + iv2 + 1u, norm.y * radian);
	atomicAdd_REAL(nNorms + iv2 + 2u, norm.z * radian);
}
__global__ void nodeNormNormalize_kernel(REAL* nNorms, uint numNodes) {
	uint id = threadIdx.x + blockDim.x * blockIdx.x;
	if (id >= numNodes)
		return;

	id *= 3u;
	REAL3 norm;
	norm.x = nNorms[id + 0u];
	norm.y = nNorms[id + 1u];
	norm.z = nNorms[id + 2u];

	Normalize(norm);

	nNorms[id + 0u] = norm.x;
	nNorms[id + 1u] = norm.y;
	nNorms[id + 2u] = norm.z;
}

__global__ void initEdgeConstraints_kernel(
	uint* es, REAL* ns, REAL* restLengths, uint numEdges)
{
	uint id = blockIdx.x * blockDim.x + threadIdx.x;
	if (id >= numEdges)
		return;

	uint ino = id << 1u;
	uint ino0 = es[ino + 0u];
	uint ino1 = es[ino + 1u];
	if (ino1 == 0xffffffff)
		return;

	REAL3 n0, n1;
	getVector(ns, ino0, n0);
	getVector(ns, ino1, n1);
	REAL length = Length(n0 - n1);

	restLengths[id] = length;
}
__global__ void initTriangleConstraints_kernel(
	uint* fs, REAL* ns, REAL* restAreas, REAL* invDs, uint numFaces)
{
	uint id = blockIdx.x * blockDim.x + threadIdx.x;
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

	REAL3 n01 = n1 - n0;
	REAL3 n02 = n2 - n0;
	REAL area = Length(Cross(n01, n02)) * 0.5;

	REAL3 u = n01; Normalize(u);
	REAL3 v = n02 - Dot(u, n02) * u;
	Normalize(v);
	
	REAL2x2 D, invD;
	D[0] = Dot(n01, u); D[2] = Dot(n01, v);
	D[1] = Dot(n02, u); D[3] = Dot(n02, v);
	Inverse(D, invD);

	restAreas[id] = area;
	Copy(invDs + (id << 2u), invD);
}
__global__ void initDihedralConstraints_kernel(
	uint* fs, uint* es, REAL* ns, uint* EnbNs, 
	REAL* restAngles, uint numEdges)
{
	uint id = blockIdx.x * blockDim.x + threadIdx.x;
	if (id >= numEdges)
		return;

	uint ino = id << 1u;
	uint ino0 = es[ino + 0u];
	uint ino1 = es[ino + 1u];
	uint ino2 = EnbNs[ino + 0u];
	uint ino3 = EnbNs[ino + 1u];
	if (ino3 == 0xffffffff)
		return;

	REAL3 n0, n1, n2, n3;
	getVector(ns, ino0, n0);
	getVector(ns, ino1, n1);
	getVector(ns, ino2, n2);
	getVector(ns, ino3, n3);

	REAL3 n01 = n1 - n0;
	REAL3 n02 = n2 - n0;
	REAL3 n03 = n3 - n0;
	REAL3 cross12 = Cross(n01, n02);
	REAL3 cross13 = Cross(n01, n03);
	Normalize(cross12);
	Normalize(cross13);
	
	REAL dot23 = min(max(Dot(cross12, cross13), -1.0), 1.0);
	REAL angle = (-0.6981317 * dot23 * dot23 - 0.8726646) * dot23 + 1.570796; //acos(n0dotn1);

	restAngles[id] = angle;
}

#endif