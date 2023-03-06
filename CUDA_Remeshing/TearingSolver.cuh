#ifndef __TEARING_SOLVER_CUH__
#define __TEARING_SOLVER_CUH__

#pragma once
#include "TearingSolver.h"
#include "../include/CUDA_Custom/DeviceManager.cuh"

#define TEARING_TOLERANCE		1.0e-5

#define FACECUT_E0N				1
#define FACECUT_E1N				2
#define FACECUT_E2N				4
#define FACECUT_E0E1			8
#define FACECUT_E0E2			16
#define FACECUT_E1E2			32

#define CUTP_N0					0
#define CUTP_N1					1
#define CUTP_N2					2
#define CUTP_E0					4
#define CUTP_E1					5
#define CUTP_E2					6
#define CUTP_F					8

__global__ void compExtendEdge_kernel(
	ClothParam cloths, REAL* edgeExtends)
{
	uint id = blockDim.x * blockIdx.x + threadIdx.x;
	if (id >= cloths._numEdges)
		return;

	uint ino0 = cloths._es[(id << 1u) + 0u];
	uint ino1 = cloths._es[(id << 1u) + 1u];
	REAL3 n0, n1;
	getVector(cloths._predNs, ino0, n0);
	getVector(cloths._predNs, ino1, n1);

	REAL length = Length(n0 - n1);
	REAL restLength = cloths._restLength[id];
	REAL edgeLimit = cloths._edgeLimits[id];
	edgeExtends[id] = length / restLength - edgeLimit;
}
__global__ void compNodeTearingGradient_kernel(
	ClothParam cloths, uint* inbEs, uint* nbEs,
	REAL* edgeExtends, REAL* nodeExtends, REAL* nodeExtGrads)
{
	uint id = blockDim.x * blockIdx.x + threadIdx.x;
	if (id >= cloths._numNodes)
		return;

	REAL maxExt = 0.0;
	REAL ext;

	uint maxExtId = 0xffffffff;
	uint ino, i;

	uint istart = inbEs[id];
	uint iend = inbEs[id + 1];
	for (i = istart; i < iend; i++) {
		ino = nbEs[i];
		ext = edgeExtends[ino];
		if (maxExt < ext) {
			maxExt = ext;
			maxExtId = ino;
		}
	}

	if (maxExtId != 0xffffffff) {
		REAL3 rgrad = make_REAL3(0.0);
		REAL rext = maxExt;
		uint num = 1u;

		uint ino0 = cloths._es[(maxExtId << 1u) + 0u];
		uint ino1 = cloths._es[(maxExtId << 1u) + 1u];
		REAL3 n0, n1;
		getVector(cloths._ns, ino0, n0);
		getVector(cloths._ns, ino1, n1);
		n1 -= n0; Normalize(n1);

		REAL3 dir = n1;
		rgrad += maxExt * n1;

		uint istart = inbEs[id];
		uint iend = inbEs[id + 1];
		for (i = istart; i < iend; i++) {
			ino = nbEs[i];
			ext = edgeExtends[ino];
			if (ino != maxExtId && ext > 0.0) {
				ino0 = cloths._es[(ino << 1u) + 0u];
				ino1 = cloths._es[(ino << 1u) + 1u];
				getVector(cloths._ns, ino0, n0);
				getVector(cloths._ns, ino1, n1);
				n1 -= n0;
				Normalize(n1);
				if (Dot(dir, n1) < 0.0) {
					n1.x = -n1.x;
					n1.y = -n1.y;
					n1.z = -n1.z;
				}
				rgrad += ext * n1;
				rext += ext;
				num++;
			}
		}
		if (num == 1)
			rgrad = dir;
		else {
			Normalize(rgrad);
			rext /= (REAL)num;
		}

		nodeExtends[id] = rext;
		setVector(nodeExtGrads, id, rgrad);
	}
}

__device__ void getNbEs(uint iface, uint* FnbEs, uint id, uint* nbEs) {
	uint num = 0;
	for (uint i = 0u; i < 3u; i++) {
		uint FnbE = FnbEs[iface * 3u + i];
		if (id != FnbE) {
			nbEs[num++] = FnbE;
		}
	}
}
__device__ void getTearingPoint(uint iface, uint* FnbEs, uint id, uint* nbEs) {
	uint num = 0;
	for (uint i = 0u; i < 3u; i++) {
		uint FnbE = FnbEs[iface * 3u + i];
		if (id != FnbE) {
			nbEs[num++] = FnbE;
		}
	}
}

__device__ void getEdgeXEdgeWeight(
	const REAL3& pa, const REAL3& pb, const REAL3& pc, const REAL3& pd, 
	REAL& w)
{
	REAL3 vcd = pd - pc;
	REAL3 vab = pb - pa;
	REAL3 vac = pc - pa;

	REAL3 norm;
	norm = Cross(vab, vcd);
	REAL nl2 = LengthSquared(norm);

	REAL3 tmp;
	tmp = Cross(vac, vab);
	REAL w_acab = Dot(tmp, norm);
	tmp = Cross(vac, vcd);
	REAL w_accd = Dot(tmp, norm);

	if (nl2 > 1.0e-40 && w_acab >= 0.0 && w_acab <= nl2 && w_accd >= 0.0 && w_accd <= nl2)
	{
		w = w_accd / nl2;
		if (w > 1.0)		w = 1.0;
		else if (w < 0.0)	w = 0.0;
	}
}
__device__ void getEdgeXPlaneWeight(
	const CutPlane& plane, const REAL3& n0, const REAL3& n1,
	REAL& w)
{
	REAL p0 = Dot(plane._norm, n0) - plane._pos;
	REAL p1 = Dot(plane._norm, n1) - plane._pos;
	if (p0 * p1 <= 0.0) {
		if (p0 < 0.0) p0 = -p0;
		if (p1 < 0.0) p1 = -p1;

		if (p0 > 0.0 || p1 > 0.0) {
			REAL l = p0 / (p0 + p1);
			if (l > 1.0)		l = 1.0;
			else if (l < 0.0)	l = 0.0;

			REAL u = Dot(plane._u, n0) * (1.0 - l) + Dot(plane._u, n1) * l;
			REAL v = Dot(plane._v, n0) * (1.0 - l) + Dot(plane._v, n1) * l;
			if (u < plane._uMax && u > plane._uMin && v < plane._vMax && v > plane._vMin)
				w = l;
		}
		else {
			//w = 2.0;
			//printf("asdfasdf\n");
		}
	}
}

__global__ void getElementsCutInfo_kernel(
	ClothParam cloths, uint* FnbEs, CutPlane* cutPlanes, uint numCutPlanes,
	uchar* faceCutInfos, uchar* fedgeCutInfos, REAL* edgeCutWs, uint* edgeCutNums,
	bool* isApplied)
{
	uint id = blockDim.x * blockIdx.x + threadIdx.x;
	if (id >= cloths._numFaces)
		return;

	uint i, j;
	uint inos[3], enos[3];
	REAL3 ns[3];
	for (i = 0u; i < 3u; i++) {
		inos[i] = cloths._fs[id * 3u + i];
		enos[i] = FnbEs[id * 3u + i];
		getVector(cloths._ns, inos[i], ns[i]);
	}

	REAL ls[3];
	uchar2 ies[3];
	for (i = 0u; i < 3u; i++) {
		uint i0 = cloths._es[(enos[i] << 1u) + 0u];
		uint i1 = cloths._es[(enos[i] << 1u) + 1u];

		if (i0 == inos[0])		ies[i].x = 0u;
		else if (i0 == inos[1]) ies[i].x = 1u;
		else					ies[i].x = 2u;
		if (i1 == inos[0])		ies[i].y = 0u;
		else if (i1 == inos[1]) ies[i].y = 1u;
		else					ies[i].y = 2u;
		ls[i] = cloths._restLength[enos[i]];
	}

	uchar faceCutInfo = 0u;
	uchar fedgeCutInfo = 0u;
	REAL tolerance = cloths._thicknesses[cloths._nodePhases[inos[0]]] * 0.5;
	REAL ws[3];
	uchar cutEs[3];
	uchar cutNs[3];
	uchar numCutEs, numCutNs, tmp;
	for (uint icutPlane = 0u; icutPlane < numCutPlanes; icutPlane++) {
		CutPlane plane = cutPlanes[icutPlane];

		cutNs[0] = cutNs[1] = cutNs[2] = 3u;
		cutEs[0] = cutEs[1] = cutEs[2] = 3u;
		ws[0] = ws[1] = ws[2] = -1.0;
		numCutNs = numCutEs = 0u;
		for (i = 0u; i < 3u; i++) {
			getEdgeXPlaneWeight(plane, ns[ies[i].x], ns[ies[i].y], ws[i]);
			if (ws[i] == 2.0) {
				/*if (numCutNs == 0u) {
					cutNs[numCutNs++] = ies[i].x;
					cutNs[numCutNs++] = ies[i].y;
				}
				else {
					for (j = 0u; j < numCutNs; j++) {
						if (ies[i].x == cutNs[j])
							break;
					}
					if (j == numCutNs)
						cutNs[numCutNs++] = ies[i].x;

					for (j = 0u; j < numCutNs; j++) {
						if (ies[i].y == cutNs[j])
							break;
					}
					if (j == numCutNs)
						cutNs[numCutNs++] = ies[i].y;
				}*/
			}
			else if (ws[i] > -1.0) {
				if (ls[i] * min(ws[i], 1.0 - ws[i]) < tolerance) {
					if (ws[i] < 0.5) {
						ws[i] = 0.0;
						tmp = ies[i].x;
					}
					else {
						ws[i] = 1.0;
						tmp = ies[i].y;
					}
					for (j = 0u; j < numCutNs; j++) {
						if (tmp == cutNs[j])
							break;
					}
					if (j == numCutNs)
						cutNs[numCutNs++] = tmp;
				}
				else cutEs[numCutEs++] = i;
			}
		}
		if (numCutEs == 3u)
			printf("getElementsCutInfo_kernel Error %f %f %f\n", ws[0], ws[1], ws[2]);
		if (numCutNs == 0u) {
			if (numCutEs == 2u) {
				faceCutInfo |= 1u << (cutEs[0] + cutEs[1] + 2u);
				atomicAdd_REAL(edgeCutWs + enos[cutEs[0]], ws[cutEs[0]]);
				atomicAdd_REAL(edgeCutWs + enos[cutEs[1]], ws[cutEs[1]]);
				atomicAdd(edgeCutNums + enos[cutEs[0]], 1u);
				atomicAdd(edgeCutNums + enos[cutEs[1]], 1u);
			}
		}
		else if (numCutNs == 1u) {
			if (numCutEs == 1u) {
				if (ies[cutEs[0]].x != cutNs[0] && ies[cutEs[0]].y != cutNs[0])
					faceCutInfo |= 1u << cutEs[0];
				else {
					if (ies[cutEs[0]].x == cutNs[0])
						tmp = 1u;
					else tmp = 2u;

					fedgeCutInfo |= tmp << (cutEs[0] << 1u);
				}
				atomicAdd_REAL(edgeCutWs + enos[cutEs[0]], ws[cutEs[0]]);
				atomicAdd(edgeCutNums + enos[cutEs[0]], 1u);
			}
			else if (numCutEs == 2u) {
				printf("getElementsCutInfo_kernel Error %f %f %f\n", ws[0], ws[1], ws[2]);
			}
		}
		else if (numCutNs == 2u) {
			for (i = 0u; i < 3u; i++) {
				if ((ies[i].x == cutNs[0] && ies[i].y == cutNs[1]) ||
					(ies[i].x == cutNs[1] && ies[i].y == cutNs[0])) {
					fedgeCutInfo |= 3u << (i << 1u);
					break;
				}
			}
			/*if (numCutEs == 0u) {
				for (i = 0u; i < 3u; i++) {
					if ((ies[i].x == cutNs[0] && ies[i].y == cutNs[1]) ||
						(ies[i].x == cutNs[1] && ies[i].y == cutNs[0])) {
						fedgeCutInfo |= 3u << (i << 1u);
						break;
					}
				}
			}
			else fedgeCutInfo |= 3u << (cutEs[0] << 1u);*/
		}
	}
	if (faceCutInfo)
		*isApplied = true;
	faceCutInfos[id] = faceCutInfo;
	fedgeCutInfos[id] = fedgeCutInfo;
}
__global__ void compCutPointWeight_E_kernel(
	ClothParam cloths, uint* FnbEs, uint* EnbFs, 
	uchar* fedgeCutInfos, uchar* edgeCutInfos, REAL* edgeCutWs, uint* edgeCutNums,
	bool* isApplied)
{
	uint id = blockDim.x * blockIdx.x + threadIdx.x;
	if (id >= cloths._numEdges)
		return;

	uchar edgeCutInfo = 0u;
	uint iface;
	uchar fCutInfo;
	for (uint i = 0u; i < 2u; i++) {
		iface = EnbFs[(id << 1u) + i];
		if (iface == 0xffffffff)
			break;

		fCutInfo = fedgeCutInfos[iface];
		if (fCutInfo) {
			for (uint j = 0u; j < 3u; j++) {
				if (FnbEs[iface * 3u + j] == id) {
					edgeCutInfo |= (fCutInfo >> (j << 1u)) & 3u;
					break;
				}
			}
		}
	}

	uint edgeCutNum = edgeCutNums[id];
	REAL w = -1.0;
	if (edgeCutNum) {
		w = edgeCutWs[id];
		w /= (REAL)edgeCutNum;
	}
	else if (iface == 0xffffffff || edgeCutInfo != 3u)
		edgeCutInfo = 0u;

	if (edgeCutInfo)
		*isApplied = true;

	edgeCutInfos[id] = edgeCutInfo;
	edgeCutWs[id] = w;
	edgeCutNums[id] = 0u;
}
__global__ void compCutPointWeight_F_kernel(
	ClothParam cloths, uchar* faceCutInfos, REAL* faceCutWs)
{
	uint id = blockDim.x * blockIdx.x + threadIdx.x;
	if (id >= cloths._numFaces)
		return;

	uchar faceCutInfo = faceCutInfos[id];
	REAL w0, w1;
	w0 = w1 = -1.0;
	if (faceCutInfo) {
		// 0 1 2: 7 == 3 || 5 || 6
		// 0 5: 33 == 33
		// 1 4: 18 == 18
		// 2 3: 12 == 12
		// 1 2 4 8 16 32
#if 1
		/*uchar n0 = (faceCutInfo >> 5u) & 1u;
		uchar n1 = (faceCutInfo >> 4u) & 1u;
		uchar n2 = (faceCutInfo >> 3u) & 1u;
		uchar n3 = (faceCutInfo >> 2u) & 1u;
		uchar n4 = (faceCutInfo >> 1u) & 1u;
		uchar n5 = faceCutInfo & 1u;
		if (n0 + n1 + n2 + n3 + n4 + n5 > 1u)
			printf("%d%d%d%d%d%d\n", n0, n1, n2, n3, n4, n5);*/

		if ((faceCutInfo & 3u) == 3u || (faceCutInfo & 5u) == 5u || (faceCutInfo & 6u) == 6u ||
			(faceCutInfo & 12u) == 12u || (faceCutInfo & 18u) == 18u || (faceCutInfo & 33u) == 33u)
		{
			w0 = w1 = 1.0 / 3.0;
		}
#else

#endif
	}
	faceCutWs[(id << 1u) + 0u] = w0;
	faceCutWs[(id << 1u) + 1u] = w1;
}

__global__ void compNumNbCutEdges_F_kernel(
	ClothParam cloths, uint* FnbEs, uchar* faceCutInfos, REAL* faceCutWs,
	uint* numEnbCEs, uint* numNnbCEs)
{
	uint id = blockDim.x * blockIdx.x + threadIdx.x;
	if (id >= cloths._numFaces)
		return;

	uchar faceCutInfo = faceCutInfos[id];
	uint ino0 = cloths._fs[id * 3u + 0u];
	uint ino1 = cloths._fs[id * 3u + 1u];
	uint ino2 = cloths._fs[id * 3u + 2u];
	uint eno0 = FnbEs[id * 3u + 0u];
	uint eno1 = FnbEs[id * 3u + 1u];
	uint eno2 = FnbEs[id * 3u + 2u];

	uchar e0n = faceCutInfo & 1u;
	uchar e1n = (faceCutInfo >> 1u) & 1u;
	uchar e2n = (faceCutInfo >> 2u) & 1u;
	uchar e0e1 = (faceCutInfo >> 3u) & 1u;
	uchar e0e2 = (faceCutInfo >> 4u) & 1u;
	uchar e1e2 = (faceCutInfo >> 5u) & 1u;

	atomicAdd(numNnbCEs + ino0, e0n);
	atomicAdd(numNnbCEs + ino1, e1n);
	atomicAdd(numNnbCEs + ino2, e2n);
	if (faceCutWs[id << 1u] == -1.0) {
		atomicAdd(numEnbCEs + eno0, e0n + e0e1 + e0e2);
		atomicAdd(numEnbCEs + eno1, e1n + e0e1 + e1e2);
		atomicAdd(numEnbCEs + eno2, e2n + e0e2 + e1e2);
	}
	else {
		atomicAdd(numEnbCEs + eno0, e0n | e0e1 | e0e2);
		atomicAdd(numEnbCEs + eno1, e1n | e0e1 | e1e2);
		atomicAdd(numEnbCEs + eno2, e2n | e0e2 | e1e2);
	}
}
__global__ void compNumNbCutEdges_E_kernel(
	ClothParam cloths, uint* EnbFs, uchar* edgeCutInfos, REAL* edgeCutWs,
	uint* numEnbCEs, uint* numNnbCEs)
{
	uint id = blockDim.x * blockIdx.x + threadIdx.x;
	if (id >= cloths._numEdges)
		return;

	uchar edgeCutInfo = edgeCutInfos[id];
	bool isCut = edgeCutWs[id] > -1.0;
	if (EnbFs[(id << 1u) + 1u] == 0xffffffff) {
		if (isCut)
			atomicAdd(numEnbCEs + id, 1u);
	}
	else {
		if (edgeCutInfo & 1u) {
			uint ino0 = cloths._es[(id << 1u) + 0u];
			atomicAdd(numNnbCEs + ino0, 1u);
			if (isCut)
				atomicAdd(numEnbCEs + id, 1u);
		}
		if (edgeCutInfo & 2u) {
			uint ino1 = cloths._es[(id << 1u) + 1u];
			atomicAdd(numNnbCEs + ino1, 1u);
			if (isCut)
				atomicAdd(numEnbCEs + id, 1u);
		}
	}
}
__global__ void compNumNbCutEdges_N_kernel(
	ClothParam cloths, uint* inbEs, uint* nbEs, uint* EnbFs, 
	uint* numNnbCEs)
{
	uint id = blockDim.x * blockIdx.x + threadIdx.x;
	if (id >= cloths._numNodes)
		return;

	uint iend = inbEs[id + 1];
	for (uint i = inbEs[id]; i < iend; i++) {
		uint ino = nbEs[i];
		if (EnbFs[(ino << 1u) + 1u] == 0xffffffff) {
			atomicAdd(numNnbCEs + id, 1u);
			break;
		}
	}
}

__global__ void compExtraNode_F_kernel(
	ClothParam cloths, uint* FnbEs, uchar* faceCutInfos, REAL* faceCutWs,
	uint* numEnbCEs, uint* numNnbCEs)
{
	uint id = blockDim.x * blockIdx.x + threadIdx.x;
	if (id >= cloths._numFaces)
		return;

	/*REAL w0 = faceCutWs[(id << 1u) + 0u];
	REAL w1 = faceCutWs[(id << 1u) + 1u];
	if (w0 == -1.0) {
		uchar faceCutInfo = faceCutInfos[id];
		uint i0, i1;
		i0 = i1 = 0xffffffff;

		if (faceCutInfo == FACECUT_E0N)
			i0 = 0u;
		else if (faceCutInfo == FACECUT_E1N)
			i0 = 1u;
		else if (faceCutInfo == FACECUT_E2N)
			i0 = 2u;
		else if (faceCutInfo == FACECUT_E0E1) {
			i0 = 0u;
			i1 = 1u;
		}
		else if (faceCutInfo == FACECUT_E0E2) {
			i0 = 0u;
			i1 = 2u;
		}
		else if (faceCutInfo == FACECUT_E1E2) {
			i0 = 1u;
			i1 = 2u;
		}
		if (i0 != 0xffffffff) {
			uint ino = FnbEs[id * 3u + i0], jno;
			if (i1 == 0xffffffff) {
				jno = cloths._fs[id * 3u + i0];
				if (numEnbCEs[ino] == 1u && numNnbCEs[jno] == 1u) {
					faceCutWs[(id << 1u) + 0u] = 1.0 / 3.0;
					faceCutWs[(id << 1u) + 1u] = 1.0 / 3.0;
				}
			}
			else {
				jno = FnbEs[id * 3u + i1];
				if (numEnbCEs[ino] == 1u && numEnbCEs[jno] == 1u) {
					faceCutWs[(id << 1u) + 0u] = 1.0 / 3.0;
					faceCutWs[(id << 1u) + 1u] = 1.0 / 3.0;
				}
			}
		}
	}*/
}
__global__ void compExtraNode_E_kernel(
	ClothParam cloths, uint* EnbFs, uchar* edgeCutInfos, REAL* edgeCutWs,
	uint* numEnbCEs, uint* numNnbCEs)
{
	uint id = blockDim.x * blockIdx.x + threadIdx.x;
	if (id >= cloths._numEdges)
		return;

	/*uchar edgeCutInfo = edgeCutInfos[id];
	bool isCut = edgeCutWs[id] != -1.0;
	uint ino0 = cloths._es[(id << 1u) + 0u];
	uint ino1 = cloths._es[(id << 1u) + 1u];

	if (edgeCutInfo) {
		if (!isCut) {
			if (numNnbCEs[ino0] == 1u && numNnbCEs[ino1] == 1u) {
				edgeCutWs[id] = 0.5;
				numEnbCEs[id] = 2u;
			}
		}
		else if (isCut && numEnbCEs[id] == 1u && EnbFs[(id << 1u) + 1u] != 0xffffffff) {
			if (edgeCutInfo == 1u) {
				if (numNnbCEs[ino0] == 1u) {
					atomicAdd(numNnbCEs + ino1, 1u);
					atomicAdd(numEnbCEs + id, 1u);
					edgeCutInfos[id] = 3u;
				}
			}
			else if (edgeCutInfo == 2u) {
				if (numNnbCEs[ino1] == 1u) {
					atomicAdd(numNnbCEs + ino0, 1u);
					atomicAdd(numEnbCEs + id, 1u);
					edgeCutInfos[id] = 3u;
				}
			}
		}
	}*/
}

__global__ void compNumNewNodes_F_kernel(
	ClothParam cloths, uchar* faceCutInfos, REAL* faceCutWs, uint* numFnewNs)
{
	uint id = blockDim.x * blockIdx.x + threadIdx.x;
	if (id >= cloths._numFaces)
		return;

	uint numNewNodes = 0u;
	if (faceCutWs[id << 1u] > -1.0) {
		uint faceCutInfo = faceCutInfos[id];
		if (faceCutInfo & (FACECUT_E0N | FACECUT_E0E1 | FACECUT_E0E2)) {
			numNewNodes++;
			if (faceCutInfo & FACECUT_E0N)
				numNewNodes++;
		}
		if (faceCutInfo & (FACECUT_E1N | FACECUT_E0E1 | FACECUT_E1E2)) {
			numNewNodes++;
			if (faceCutInfo & FACECUT_E1N)
				numNewNodes++;
		}
		if (faceCutInfo & (FACECUT_E2N | FACECUT_E0E2 | FACECUT_E1E2)) {
			numNewNodes++;
			if (faceCutInfo & FACECUT_E2N)
				numNewNodes++;
		}
	}
	if (id == 0u)
		numFnewNs[0] = 0;
	numFnewNs[id + 1u] = numNewNodes;
}
__global__ void compNumNewNodes_E_kernel(
	ClothParam cloths, REAL* edgeCutWs, uint* numEnbCEs, uint* numEnewNs)
{
	uint id = blockDim.x * blockIdx.x + threadIdx.x;
	if (id >= cloths._numEdges)
		return;

	uint numNewNodes = 0u;
	if (edgeCutWs[id] > -1.0)
		numNewNodes = numEnbCEs[id];

	if (id == 0u)
		numEnewNs[0] = 0;
	numEnewNs[id + 1] = numNewNodes;
}
__global__ void compNumNewNodes_N_kernel(
	ClothParam cloths, uint* numNnbCEs, uint* numNnewNs)
{
	uint id = blockDim.x * blockIdx.x + threadIdx.x;
	if (id >= cloths._numNodes)
		return;

	uint numNewNodes = numNnbCEs[id];
	if (!numNewNodes) numNewNodes = 1u;
	if (id == 0u)
		numNnewNs[0] = 0;
	numNnewNs[id + 1] = numNewNodes;
}

__global__ void compPaddingNodeIds_kernel(
	uint* inewNs, uint offset, uint size)
{
	uint id = blockDim.x * blockIdx.x + threadIdx.x;
	if (id > size)
		return;
	
	inewNs[id] += offset;
}

__global__ void compGenNewNodes_N_kernel(
	ClothParam cloths, uint* iNnewNs,
	REAL* newNs, REAL* newN0s)
{
	uint id = blockDim.x * blockIdx.x + threadIdx.x;
	if (id >= cloths._numNodes)
		return;

	//REAL3 x = make_REAL3(FLT_MAX);
	REAL3 x, x0;
	getVector(cloths._ns, id, x);
	getVector(cloths._restNs, id, x0);

	uint iend = iNnewNs[id + 1];
	for (uint i = iNnewNs[id]; i < iend; i++) {
		setVector(newNs, i, x);
		setVector(newN0s, i, x0);
	}
}
__global__ void compGenNewNodes_E_kernel(
	ClothParam cloths, REAL* edgeCutWs, uint* iEnewNs, 
	REAL* newNs, REAL* newN0s)
{
	uint id = blockDim.x * blockIdx.x + threadIdx.x;
	if (id >= cloths._numEdges)
		return;

	uint inodeStart = iEnewNs[id];
	uint inodeEnd = iEnewNs[id + 1];
	if (inodeEnd - inodeStart > 0u) {
		REAL w = edgeCutWs[id];
		uint ino0 = cloths._es[(id << 1u) + 0u];
		uint ino1 = cloths._es[(id << 1u) + 1u];
		REAL3 a, b;

		getVector(cloths._ns, ino0, a);
		getVector(cloths._ns, ino1, b);
		REAL3 x = a + (b - a) * w;

		getVector(cloths._restNs, ino0, a);
		getVector(cloths._restNs, ino1, b);
		REAL3 x0 = a + (b - a) * w;

		for (uint i = inodeStart; i < inodeEnd; i++) {
			setVector(newNs, i, x);
			setVector(newN0s, i, x0);
		}
	}
}
__global__ void compGenNewNodes_F_kernel(
	ClothParam cloths, REAL* faceCutWs, uint* iFnewNs,
	REAL* newNs, REAL* newN0s)
{
	uint id = blockDim.x * blockIdx.x + threadIdx.x;
	if (id >= cloths._numFaces)
		return;

	uint inodeStart = iFnewNs[id];
	uint inodeEnd = iFnewNs[id + 1];
	if (inodeEnd - inodeStart > 0u) {
		REAL w0 = faceCutWs[(id << 1u) + 0u];
		REAL w1 = faceCutWs[(id << 1u) + 1u];
		uint ino0 = cloths._fs[id * 3u + 0u];
		uint ino1 = cloths._fs[id * 3u + 1u];
		uint ino2 = cloths._fs[id * 3u + 2u];
		REAL3 a, b, c;

		getVector(cloths._ns, ino0, a);
		getVector(cloths._ns, ino1, b);
		getVector(cloths._ns, ino2, c);
		REAL3 x = a * w0 + b * w1 + c * (1.0 - w0 - w1);

		getVector(cloths._restNs, ino0, a);
		getVector(cloths._restNs, ino1, b);
		getVector(cloths._restNs, ino2, c);
		REAL3 x0 = a * w0 + b * w1 + c * (1.0 - w0 - w1);

		for (uint i = inodeStart; i < inodeEnd; i++) {
			setVector(newNs, i, x);
			setVector(newN0s, i, x0);
		}
	}
}

__global__ void compNumNewFaceInfos_kernel(
	ClothParam cloths, uint* FnbEs, uchar* faceCutInfos, REAL* faceCutWs, REAL* edgeCutWs,
	uint* inewFs)
{
	uint id = blockDim.x * blockIdx.x + threadIdx.x;
	if (id >= cloths._numFaces)
		return;

	uchar numFaceInfos = 0u;

	uint eno0 = FnbEs[id * 3u + 0u];
	uint eno1 = FnbEs[id * 3u + 1u];
	uint eno2 = FnbEs[id * 3u + 2u];
	bool isCut0 = edgeCutWs[eno0] > -1.0;
	bool isCut1 = edgeCutWs[eno1] > -1.0;
	bool isCut2 = edgeCutWs[eno2] > -1.0;
	uchar faceCutInfo = faceCutInfos[id];
	if (faceCutInfo) {
		if (faceCutWs[id << 1u] > -1.0) {
			if ((faceCutInfo & (FACECUT_E0N | FACECUT_E0E1 | FACECUT_E0E2)) || isCut0)
				numFaceInfos += 2u;
			else numFaceInfos++;
			if ((faceCutInfo & (FACECUT_E1N | FACECUT_E0E1 | FACECUT_E1E2)) || isCut1)
				numFaceInfos += 2u;
			else numFaceInfos++;
			if ((faceCutInfo & (FACECUT_E2N | FACECUT_E0E2 | FACECUT_E1E2)) || isCut2)
				numFaceInfos += 2u;
			else numFaceInfos++;
		}
		else {
			// Node: 0, 1, 2, Edge: 4, 5, 6, Face: 8
			if (faceCutInfo & FACECUT_E0N) {
				if ((faceCutInfo & FACECUT_E0E1) || isCut1)
					numFaceInfos += 2u;
				else numFaceInfos++;
				if ((faceCutInfo & FACECUT_E0E2) || isCut2)
					numFaceInfos += 2u;
				else numFaceInfos++;
			}
			else if (faceCutInfo & FACECUT_E1N) {
				if ((faceCutInfo & FACECUT_E1E2) || isCut2) 
					numFaceInfos += 2u;
				else numFaceInfos++;
				if ((faceCutInfo & FACECUT_E0E1) || isCut0)
					numFaceInfos += 2u;
				else numFaceInfos++;
			}
			else if (faceCutInfo & FACECUT_E2N) {
				if ((faceCutInfo & FACECUT_E0E2) || isCut0)
					numFaceInfos += 2u;
				else numFaceInfos++;
				if ((faceCutInfo & FACECUT_E1E2) || isCut1)
					numFaceInfos += 2u;
				else numFaceInfos++;
			}
			else if (faceCutInfo == (FACECUT_E0E1 | FACECUT_E0E2 | FACECUT_E1E2) || 
				faceCutInfo == (FACECUT_E0E1 | FACECUT_E0E2) ||
				faceCutInfo == (FACECUT_E0E1 | FACECUT_E1E2) ||
				faceCutInfo == (FACECUT_E0E2 | FACECUT_E1E2) ||
				(faceCutInfo == FACECUT_E0E1 && isCut2) ||
				(faceCutInfo == FACECUT_E0E2 && isCut1) ||
				(faceCutInfo == FACECUT_E1E2 && isCut0))
				numFaceInfos = 4u;
			else
				numFaceInfos = 3u;
		}
	}
	else {
		numFaceInfos = (uchar)isCut0 + (uchar)isCut1 + (uchar)isCut2 + 1u;
	}
	if (id == 0u)
		inewFs[0] = 0u;
	inewFs[id + 1u] = numFaceInfos;
}
__global__ void compNewFaceInfo_kernel(
	ClothParam cloths, uint* FnbEs, uchar* faceCutInfos, REAL* faceCutWs, REAL* edgeCutWs,
	uint* inewFs, uint* iFnewNs, ushort* newFaceInfos, uint* newFs)
{
	uint id = blockDim.x * blockIdx.x + threadIdx.x;
	if (id >= cloths._numFaces)
		return;

	uint ifaceInfo = inewFs[id];
	uint inodeStart = iFnewNs[id];
	uint inodeEnd = iFnewNs[id + 1u];
	uint inode = inodeStart;
	uint test0 = 0u;
	uint test1 = 0u;
	uint eno0 = FnbEs[id * 3u + 0u];
	uint eno1 = FnbEs[id * 3u + 1u];
	uint eno2 = FnbEs[id * 3u + 2u];
	bool isCut0 = edgeCutWs[eno0] > -1.0;
	bool isCut1 = edgeCutWs[eno1] > -1.0;
	bool isCut2 = edgeCutWs[eno2] > -1.0;
	uchar faceCutInfo = faceCutInfos[id];

	if (faceCutInfo) {
		if (faceCutWs[id << 1u] > -1.0) {
			newFs[ifaceInfo * 3u + 2u] = inode;
			if ((faceCutInfo & (FACECUT_E0N | FACECUT_E0E1 | FACECUT_E0E2)) || isCut0) {
				newFaceInfos[ifaceInfo++] = CUTP_N1 | (CUTP_E0 << 4u) | (CUTP_F << 8u);
				if (faceCutInfo & (FACECUT_E0N | FACECUT_E0E1 | FACECUT_E0E2)) {
					if (++inode >= inodeEnd) {
						inode = inodeStart;
						test0++;
					}
					test1++;
				}
				newFs[ifaceInfo * 3u + 2u] = inode;
				newFaceInfos[ifaceInfo++] = CUTP_E0 | (CUTP_N2 << 4u) | (CUTP_F << 8u);
			}
			else newFaceInfos[ifaceInfo++] = CUTP_N1 | (CUTP_N2 << 4u) | (CUTP_F << 8u);

			if (faceCutInfo & FACECUT_E2N) {
				if (++inode >= inodeEnd) {
					inode = inodeStart;
					test0++;
				}
				test1++;
			}
			newFs[ifaceInfo * 3u + 2u] = inode;
			if ((faceCutInfo & (FACECUT_E1N | FACECUT_E0E1 | FACECUT_E1E2)) || isCut1) {
				newFaceInfos[ifaceInfo++] = CUTP_N2 | (CUTP_E1 << 4u) | (CUTP_F << 8u);
				if (faceCutInfo & (FACECUT_E1N | FACECUT_E0E1 | FACECUT_E1E2)) {
					if (++inode >= inodeEnd) {
						inode = inodeStart;
						test0++;
					}
					test1++;
				}
				newFs[ifaceInfo * 3u + 2u] = inode;
				newFaceInfos[ifaceInfo++] = CUTP_E1 | (CUTP_N0 << 4u) | (CUTP_F << 8u);
			}
			else newFaceInfos[ifaceInfo++] = CUTP_N2 | (CUTP_N0 << 4u) | (CUTP_F << 8u);

			if (faceCutInfo & FACECUT_E0N) {
				if (++inode >= inodeEnd) {
					inode = inodeStart;
					test0++;
				}
				test1++;
			}
			newFs[ifaceInfo * 3u + 2u] = inode;
			if ((faceCutInfo & (FACECUT_E2N | FACECUT_E0E2 | FACECUT_E1E2)) || isCut2) {
				newFaceInfos[ifaceInfo++] = CUTP_N0 | (CUTP_E2 << 4u) | (CUTP_F << 8u);
				if (faceCutInfo & (FACECUT_E2N | FACECUT_E0E2 | FACECUT_E1E2)) {
					if (++inode >= inodeEnd) {
						inode = inodeStart;
						test0++;
					}
					test1++;
				}
				newFs[ifaceInfo * 3u + 2u] = inode;
				newFaceInfos[ifaceInfo++] = CUTP_E2 | (CUTP_N1 << 4u) | (CUTP_F << 8u);
			}
			else newFaceInfos[ifaceInfo++] = CUTP_N0 | (CUTP_N1 << 4u) | (CUTP_F << 8u);
			//printf("asdfasdf %d %d %d\n", test0, test1, inodeEnd - inodeStart);
			if (test0 > 1)
				printf("faceasdf %d, %d %d %d\n", id, test0, test1, inodeEnd - inodeStart);
		}
		else {
			// Node: 0, 1, 2, Edge: 4, 5, 6, Face: 8
			if (faceCutInfo & FACECUT_E0N) {
				if ((faceCutInfo & FACECUT_E0E1) || isCut1) {
					newFaceInfos[ifaceInfo++] = CUTP_N2 | (CUTP_E1 << 4u) | (CUTP_E0 << 8u);
					newFaceInfos[ifaceInfo++] = CUTP_E1 | (CUTP_N0 << 4u) | (CUTP_E0 << 8u);
				}
				else newFaceInfos[ifaceInfo++] = CUTP_N2 | (CUTP_N0 << 4u) | (CUTP_E0 << 8u);
				if ((faceCutInfo & FACECUT_E0E2) || isCut2) {
					newFaceInfos[ifaceInfo++] = CUTP_E0 | (CUTP_N0 << 4u) | (CUTP_E2 << 8u);
					newFaceInfos[ifaceInfo++] = CUTP_E0 | (CUTP_E2 << 4u) | (CUTP_N1 << 8u);
				}
				else newFaceInfos[ifaceInfo++] = CUTP_E0 | (CUTP_N0 << 4u) | (CUTP_N1 << 8u);
			}
			else if (faceCutInfo & FACECUT_E1N) {
				if ((faceCutInfo & FACECUT_E1E2) || isCut2) {
					newFaceInfos[ifaceInfo++] = CUTP_N0 | (CUTP_E2 << 4u) | (CUTP_E1 << 8u);
					newFaceInfos[ifaceInfo++] = CUTP_E2 | (CUTP_N1 << 4u) | (CUTP_E1 << 8u);
				}
				else newFaceInfos[ifaceInfo++] = CUTP_N0 | (CUTP_N1 << 4u) | (CUTP_E1 << 8u);
				if ((faceCutInfo & FACECUT_E0E1) || isCut0) {
					newFaceInfos[ifaceInfo++] = CUTP_E1 | (CUTP_N1 << 4u) | (CUTP_E0 << 8u);
					newFaceInfos[ifaceInfo++] = CUTP_E1 | (CUTP_E0 << 4u) | (CUTP_N2 << 8u);
				}
				else newFaceInfos[ifaceInfo++] = CUTP_E1 | (CUTP_N1 << 4u) | (CUTP_N2 << 8u);
			}
			else if (faceCutInfo & FACECUT_E2N) {
				if ((faceCutInfo & FACECUT_E0E2) || isCut0) {
					newFaceInfos[ifaceInfo++] = CUTP_N1 | (CUTP_E0 << 4u) | (CUTP_E2 << 8u);
					newFaceInfos[ifaceInfo++] = CUTP_E0 | (CUTP_N2 << 4u) | (CUTP_E2 << 8u);
				}
				else newFaceInfos[ifaceInfo++] = CUTP_N1 | (CUTP_N2 << 4u) | (CUTP_E2 << 8u);
				if ((faceCutInfo & FACECUT_E1E2) || isCut1) {
					newFaceInfos[ifaceInfo++] = CUTP_E2 | (CUTP_N2 << 4u) | (CUTP_E1 << 8u);
					newFaceInfos[ifaceInfo++] = CUTP_E2 | (CUTP_E1 << 4u) | (CUTP_N0 << 8u);
				}
				else newFaceInfos[ifaceInfo++] = CUTP_E2 | (CUTP_N2 << 4u) | (CUTP_N0 << 8u);
			}
			else if (faceCutInfo == (FACECUT_E0E1 | FACECUT_E0E2 | FACECUT_E1E2) ||
				faceCutInfo == (FACECUT_E0E1 | FACECUT_E0E2) ||
				faceCutInfo == (FACECUT_E0E1 | FACECUT_E1E2) ||
				faceCutInfo == (FACECUT_E0E2 | FACECUT_E1E2) ||
				(faceCutInfo == FACECUT_E0E1 && isCut2) ||
				(faceCutInfo == FACECUT_E0E2 && isCut1) ||
				(faceCutInfo == FACECUT_E1E2 && isCut0))
			{
				newFaceInfos[ifaceInfo++] = CUTP_E0 | (CUTP_E1 << 4u) | (CUTP_E2 << 8u);
				newFaceInfos[ifaceInfo++] = CUTP_N0 | (CUTP_E2 << 4u) | (CUTP_E1 << 8u);
				newFaceInfos[ifaceInfo++] = CUTP_E2 | (CUTP_N1 << 4u) | (CUTP_E0 << 8u);
				newFaceInfos[ifaceInfo++] = CUTP_E1 | (CUTP_E0 << 4u) | (CUTP_N2 << 8u);
			}
			else {
				if (faceCutInfo & FACECUT_E0E1) {
					newFaceInfos[ifaceInfo++] = CUTP_N2 | (CUTP_E1 << 4u) | (CUTP_E0 << 8u);
					newFaceInfos[ifaceInfo++] = CUTP_E1 | (CUTP_N0 << 4u) | (CUTP_E0 << 8u);
					newFaceInfos[ifaceInfo++] = CUTP_E0 | (CUTP_N0 << 4u) | (CUTP_N1 << 8u);
				}
				else if (faceCutInfo & FACECUT_E1E2) {
					newFaceInfos[ifaceInfo++] = CUTP_N0 | (CUTP_E2 << 4u) | (CUTP_E1 << 8u);
					newFaceInfos[ifaceInfo++] = CUTP_E2 | (CUTP_N1 << 4u) | (CUTP_E1 << 8u);
					newFaceInfos[ifaceInfo++] = CUTP_E1 | (CUTP_N1 << 4u) | (CUTP_N2 << 8u);
				}
				else if (faceCutInfo & FACECUT_E0E2) {
					newFaceInfos[ifaceInfo++] = CUTP_N1 | (CUTP_E0 << 4u) | (CUTP_E2 << 8u);
					newFaceInfos[ifaceInfo++] = CUTP_E0 | (CUTP_N2 << 4u) | (CUTP_E2 << 8u);
					newFaceInfos[ifaceInfo++] = CUTP_E2 | (CUTP_N2 << 4u) | (CUTP_N0 << 8u);
				}
			}
		}
	}
	else {
		if (isCut0 && isCut1 && isCut2) {
			newFaceInfos[ifaceInfo++] = CUTP_E0 | (CUTP_E1 << 4u) | (CUTP_E2 << 8u);
			newFaceInfos[ifaceInfo++] = CUTP_N0 | (CUTP_E2 << 4u) | (CUTP_E1 << 8u);
			newFaceInfos[ifaceInfo++] = CUTP_E2 | (CUTP_N1 << 4u) | (CUTP_E0 << 8u);
			newFaceInfos[ifaceInfo++] = CUTP_E1 | (CUTP_E0 << 4u) | (CUTP_N2 << 8u);
		}
		else if (isCut0 && isCut1) {
			newFaceInfos[ifaceInfo++] = CUTP_E1 | (CUTP_E0 << 4u) | (CUTP_N2 << 8u);
			newFaceInfos[ifaceInfo++] = CUTP_N0 | (CUTP_E0 << 4u) | (CUTP_E1 << 8u);
			newFaceInfos[ifaceInfo++] = CUTP_N0 | (CUTP_N1 << 4u) | (CUTP_E0 << 8u);
		}
		else if (isCut1 && isCut2) {
			newFaceInfos[ifaceInfo++] = CUTP_E2 | (CUTP_E1 << 4u) | (CUTP_N0 << 8u);
			newFaceInfos[ifaceInfo++] = CUTP_N1 | (CUTP_E1 << 4u) | (CUTP_E2 << 8u);
			newFaceInfos[ifaceInfo++] = CUTP_N1 | (CUTP_N2 << 4u) | (CUTP_E1 << 8u);
		}
		else if (isCut2 && isCut0) {
			newFaceInfos[ifaceInfo++] = CUTP_E0 | (CUTP_E2 << 4u) | (CUTP_N1 << 8u);
			newFaceInfos[ifaceInfo++] = CUTP_N2 | (CUTP_E2 << 4u) | (CUTP_E0 << 8u);
			newFaceInfos[ifaceInfo++] = CUTP_N2 | (CUTP_N0 << 4u) | (CUTP_E2 << 8u);
		}
		else if (isCut0) {
			newFaceInfos[ifaceInfo++] = CUTP_N0 | (CUTP_N1 << 4u) | (CUTP_E0 << 8u);
			newFaceInfos[ifaceInfo++] = CUTP_N0 | (CUTP_E0 << 4u) | (CUTP_N2 << 8u);
		}
		else if (isCut1) {
			newFaceInfos[ifaceInfo++] = CUTP_N1 | (CUTP_N2 << 4u) | (CUTP_E1 << 8u);
			newFaceInfos[ifaceInfo++] = CUTP_N1 | (CUTP_E1 << 4u) | (CUTP_N0 << 8u);
		}
		else if (isCut2) {
			newFaceInfos[ifaceInfo++] = CUTP_N2 | (CUTP_N0 << 4u) | (CUTP_E2 << 8u);
			newFaceInfos[ifaceInfo++] = CUTP_N2 | (CUTP_E2 << 4u) | (CUTP_N1 << 8u);
		}
		else newFaceInfos[ifaceInfo++] = CUTP_N0 | (CUTP_N1 << 4u) | (CUTP_N2 << 8u);
	}
}

__global__ void compNewFaceMapping_E_kernel(
	ClothParam cloths, uint* FnbEs, uint* EnbFs,
	uchar* faceCutInfos, uchar* edgeCutInfos, ushort* newFaceInfos,
	uint* iEnewNs, uint* inewFs, uint* newFs)
{
	uint id = blockDim.x * blockIdx.x + threadIdx.x;
	if (id >= cloths._numEdges)
		return;

	uint inodeStart = iEnewNs[id];
	uint inodeEnd = iEnewNs[id + 1u];
	if (inodeEnd > inodeStart) {
		uint inode = inodeStart;
		uint test0 = 0u;
		uint test1 = 0u;
		uint i, j;
		uint currFace;

		ushort faceInfo, prevFaceInfo;
		uchar cutInfo;
		uchar nodeId, edgeId, prevId;
		uint ifaceInfoStart, ifaceInfoEnd;

		uchar inos[3];
		uchar i0, i1;
		uchar curri0, curri1;
		bool nodeChange;

		for (uint n = 0u; n < 2u; n++) {
			currFace = EnbFs[(id << 1u) + n];
			if (currFace != 0xffffffff) {
				for (i = 0u; i < 3u; i++) {
					if (cloths._fs[currFace * 3u + i] == cloths._es[(id << 1u) + n])
						nodeId = i;
					if (FnbEs[currFace * 3u + i] == id)
						edgeId = i;
				}
				prevId = nodeId;

				cutInfo = faceCutInfos[currFace];

				ifaceInfoStart = inewFs[currFace];
				ifaceInfoEnd = inewFs[currFace + 1u];
				prevFaceInfo = 0u;
				i = ifaceInfoStart;
				while (i < ifaceInfoEnd) {
					faceInfo = newFaceInfos[i];
					if (faceInfo != prevFaceInfo) {
						inos[0] = faceInfo & 15u;
						inos[1] = (faceInfo >> 4u) & 15u;
						inos[2] = (faceInfo >> 8u) & 15u;

						curri0 = curri1 = 3u;
						if ((edgeId | 4u) == inos[0])		curri0 = 0u;
						else if ((edgeId | 4u) == inos[1])	curri0 = 1u;
						else if ((edgeId | 4u) == inos[2])	curri0 = 2u;
						if (curri0 < 3u) {
							if (prevId == inos[0])		curri1 = 0u;
							else if (prevId == inos[1])	curri1 = 1u;
							else if (prevId == inos[2])	curri1 = 2u;
							if (curri1 < 3u) {
								newFs[i * 3u + curri0] = inode;

								prevId = inos[3u - curri0 - curri1];
								prevFaceInfo = faceInfo;
								if (prevId == (3u - nodeId - edgeId))
									break;

								if (prevId == CUTP_F) {
									i0 = (edgeId << 1u) + 1u;
									i1 = i0 + 2u;
									i0 %= 3u; i1 %= 3u;
									i0 += 3u; i1 += 3u;
									nodeChange = (cutInfo & ((1u << edgeId) | (1u << i0) | (1u << i1)));
								}
								else {
									if (edgeId == 0u) {
										nodeChange = (
											prevId == 0u && (cutInfo & FACECUT_E0N) ||
											prevId == 5u && (cutInfo & FACECUT_E0E1) ||
											prevId == 6u && (cutInfo & FACECUT_E0E2));
									}
									else if (edgeId == 1u) {
										nodeChange = (
											prevId == 1u && (cutInfo & FACECUT_E1N) ||
											prevId == 4u && (cutInfo & FACECUT_E0E1) ||
											prevId == 6u && (cutInfo & FACECUT_E1E2));
									}
									else {
										nodeChange = (
											prevId == 2u && (cutInfo & FACECUT_E2N) ||
											prevId == 4u && (cutInfo & FACECUT_E0E2) ||
											prevId == 5u && (cutInfo & FACECUT_E1E2));
									}
								}

								i = ifaceInfoStart;
								if (nodeChange) {
									if (++inode >= inodeEnd) {
										inode = inodeStart;
										test0++;
									}
									test1++;
								}
								continue;
							}
						}
					}
					i++;
				}
				if (n == 0u && (edgeCutInfos[id] & 2u)) {
					if (++inode >= inodeEnd) {
						inode = inodeStart;
						test0++;
					}
					test1++;
				}
			}
		}
		if (test0 > 1)
			printf("edgeasdf %d, %d %d %d\n", id, test0, test1, inodeEnd - inodeStart);
	}
}
__global__ void compNewFaceMapping_N_kernel(
	ClothParam cloths, uint* inbEs, uint* nbEs, uint* FnbEs, uint* EnbFs,
	uchar* faceCutInfos, uchar* edgeCutInfos, REAL* edgeCutWs, ushort* newFaceInfos,
	uint* iNnewNs, uint* inewFs, uint* newFs)
{
	uint id = blockDim.x * blockIdx.x + threadIdx.x;
	if (id >= cloths._numNodes)
		return;

	uint inodeStart = iNnewNs[id];
	uint inodeEnd = iNnewNs[id + 1u];
	if (inodeEnd > inodeStart/* && inbEs[id] != inbEs[id + 1u]*/) {
		uint inode = inodeStart;
		uint test0 = 0u;
		uint test1 = 0u;

		uint startEdge, prevEdge, currFace;
		uint i, j, ino, jno;

		i = inbEs[id]; startEdge = nbEs[i];
		for (; i < inbEs[id + 1u]; i++) {
			ino = nbEs[i];
			if (EnbFs[(ino << 1u) + 1u] == 0xffffffff) {
				startEdge = ino;
				break;
			}
		}
		currFace = EnbFs[(startEdge << 1u) + 0u];
		prevEdge = startEdge;

		ushort faceInfo, prevFaceInfo;
		uchar cutInfo;
		uchar nodeId, edgeId, prevId;
		uint ifaceInfoStart, ifaceInfoEnd;

		uchar inos[3];
		uchar curri0, curri1;

		bool nodeChange;
		do {
			nodeId = edgeId = 3u;
			for (i = 0u; i < 3u; i++) {
				if (cloths._fs[currFace * 3u + i] == id)
					nodeId = i;
				else if (FnbEs[currFace * 3u + i] == prevEdge)
					edgeId = i;
			}
			if (edgeCutWs[prevEdge] > -1.0)
				prevId = edgeId | 4u;
			else
				prevId = 3u - nodeId - edgeId;

			cutInfo = faceCutInfos[currFace];
			nodeChange = (cutInfo & (1u << nodeId));

			ifaceInfoStart = inewFs[currFace];
			ifaceInfoEnd = inewFs[currFace + 1u];
			prevFaceInfo = 0u;
			i = ifaceInfoStart;
			while (i < ifaceInfoEnd) {
				faceInfo = newFaceInfos[i];
				if (faceInfo != prevFaceInfo) {
					inos[0] = faceInfo & 15u;
					inos[1] = (faceInfo >> 4u) & 15u;
					inos[2] = (faceInfo >> 8u) & 15u;

					curri0 = curri1 = 3u;
					if (nodeId == inos[0])		curri0 = 0u;
					else if (nodeId == inos[1])	curri0 = 1u;
					else if (nodeId == inos[2])	curri0 = 2u;
					if (curri0 < 3u) {
						if (prevId == inos[0])		curri1 = 0u;
						else if (prevId == inos[1])	curri1 = 1u;
						else if (prevId == inos[2])	curri1 = 2u;
						if (curri1 < 3u) {
							newFs[i * 3u + curri0] = inode;

							prevId = inos[3u - curri0 - curri1];
							prevFaceInfo = faceInfo;
							if (prevId == edgeId || prevId == ((3u - nodeId - edgeId) | 4u))
								break;

							i = ifaceInfoStart;
							if (nodeChange) {
								if (++inode >= inodeEnd) {
									inode = inodeStart;
									test0++;
								}
								test1++;
							}
							continue;
						}
					}
				}
				i++;
			}

			prevEdge = FnbEs[currFace * 3u + (3u - nodeId - edgeId)];

			ino = EnbFs[(prevEdge << 1u) + 0u];
			if (currFace == ino)
				ino = EnbFs[(prevEdge << 1u) + 1u];
			currFace = ino;
			if (currFace == 0xffffffff)
				break;

			nodeId = (uchar)(cloths._es[(prevEdge << 1u) + 1u] == id);
			cutInfo = edgeCutInfos[prevEdge];
			if (cutInfo & (1u << nodeId)) {
				if (++inode >= inodeEnd) {
					inode = inodeStart;
					test0++;
				}
				test1++;
			}

		} while (prevEdge != startEdge && currFace != 0xffffffff);
		if (test0 > 1)
			printf("nodeasdf %d, %d %d %d\n", id, test0, test1, inodeEnd - inodeStart);
	}
}

__global__ void getNewEdgeId_kernel(
	uint* newFs, uint* inbFs, uint* nbFs, uint numNewFaces, uint* numNewEdges)
{
	uint id = blockDim.x * blockIdx.x + threadIdx.x;
	if (id >= numNewFaces)
		return;
	
	uint num = 0u;
	uint inos[3];
	inos[0] = newFs[id * 3u + 0u];
	inos[1] = newFs[id * 3u + 1u];
	inos[2] = newFs[id * 3u + 2u];

	uint istart, iend, jstart, jend, ino, jno, iface, jface;
	for (uint i = 0u; i < 3u; i++) {
		uint j = (i + 1u) % 3u;
		uint of = 0xffffffff;

		istart = inbFs[inos[i]];
		iend = inbFs[inos[i] + 1u];
		jstart = inbFs[inos[j]];
		jend = inbFs[inos[j] + 1u];

		for (ino = istart; ino < iend; ino++) {
			iface = nbFs[ino];
			for (jno = jstart; jno < jend; jno++) {
				jface = nbFs[jno];
				if (iface == jface)
					break;
			}
			if (jno < jend && id != iface) {
				of = iface;
				break;
			}
		}
		if (of == 0xffffffff || id < of)
			num++;
	}
	if (id == 0u)
		numNewEdges[0] = 0u;
	numNewEdges[id + 1u] = num;
}
__global__ void getNewEdge_kernel(
	uint* newFs, uint* inbFs, uint* nbFs, uint numNewFaces, uint* iNewEs, uint* newEs)
{
	uint id = blockDim.x * blockIdx.x + threadIdx.x;
	if (id >= numNewFaces)
		return;

	uint iedge = iNewEs[id] << 1u;
	uint inos[3];
	inos[0] = newFs[id * 3u + 0u];
	inos[1] = newFs[id * 3u + 1u];
	inos[2] = newFs[id * 3u + 2u];

	uint istart, iend, jstart, jend, ino, jno, iface, jface;
	for (uint i = 0u; i < 3u; i++) {
		uint j = (i + 1u) % 3u;
		uint of = 0xffffffff;

		istart = inbFs[inos[i]];
		iend = inbFs[inos[i] + 1u];
		jstart = inbFs[inos[j]];
		jend = inbFs[inos[j] + 1u];

		for (ino = istart; ino < iend; ino++) {
			iface = nbFs[ino];
			for (jno = jstart; jno < jend; jno++) {
				jface = nbFs[jno];
				if (iface == jface)
					break;
			}
			if (jno < jend && id != iface) {
				of = iface;
				break;
			}
		}
		if (of == 0xffffffff || id < of) {
			if (inos[i] < inos[j]) {
				newEs[iedge + 0u] = inos[i];
				newEs[iedge + 1u] = inos[j];
			}
			else {
				newEs[iedge + 0u] = inos[j];
				newEs[iedge + 1u] = inos[i];
			}
			iedge += 2u;
		}
	}
}

__global__ void initNewNodeMasses_N_kernel(
	ClothParam cloths, uint* iNnewNs, REAL* newMs)
{
	uint id = blockDim.x * blockIdx.x + threadIdx.x;
	if (id >= cloths._numNodes)
		return;

	uint ino = iNnewNs[id];
	newMs[ino] = cloths._ms[id];
}
__global__ void compNewNodeParams_F_kernel(
	ClothParam cloths, uint* inbFs, uint* inbEs,
	REAL* faceCutWs, uint* iFnewNs, uint* iNnewNs,
	REAL* newMs, REAL* newVs, uint* newNodePhases, uchar* newIsFixeds)
{
	uint id = blockDim.x * blockIdx.x + threadIdx.x;
	if (id >= cloths._numFaces)
		return;

	uint inodeStart = iFnewNs[id];
	uint inodeEnd = iFnewNs[id + 1u];
	uint fnumNodes = inodeEnd - inodeStart;
	if (fnumNodes > 0u) {
		uint ino0 = cloths._fs[id * 3u + 0u];
		uint ino1 = cloths._fs[id * 3u + 1u];
		uint ino2 = cloths._fs[id * 3u + 2u];
		REAL w0 = faceCutWs[(id << 1u) + 0u];
		REAL w1 = faceCutWs[(id << 1u) + 1u];
		REAL w2 = 1.0 - w0 - w1;

		uint i0num = 1u + inbFs[ino0 + 1u] - inbFs[ino0] + inbEs[ino0 + 1] - inbEs[ino0];
		uint i1num = 1u + inbFs[ino1 + 1u] - inbFs[ino1] + inbEs[ino1 + 1] - inbEs[ino1];
		uint i2num = 1u + inbFs[ino2 + 1u] - inbFs[ino2] + inbEs[ino2 + 1] - inbEs[ino2];
		REAL m0 = cloths._ms[ino0] / (REAL)i0num * w0;
		REAL m1 = cloths._ms[ino1] / (REAL)i1num * w1;
		REAL m2 = cloths._ms[ino2] / (REAL)i2num * w2;
		atomicAdd_REAL(newMs + iNnewNs[ino0], -m0);
		atomicAdd_REAL(newMs + iNnewNs[ino1], -m1);
		atomicAdd_REAL(newMs + iNnewNs[ino2], -m2);
		REAL m = (m0 + m1 + m2) / (REAL)fnumNodes;

		REAL3 v0, v1, v2;
		getVector(cloths._vs, ino0, v0);
		getVector(cloths._vs, ino1, v1);
		getVector(cloths._vs, ino2, v2);
		REAL3 v = v0 * w0 + v1 * w1 + v2 * w2;

		uint nodePhase = cloths._nodePhases[ino0];
		for (uint ino = inodeStart; ino < inodeEnd; ino++) {
			newMs[ino] = m;
			setVector(newVs, ino, v);
			newNodePhases[ino] = nodePhase;
			newIsFixeds[ino] = 0u;
		}
	}
}
__global__ void compNewNodeParams_E_kernel(
	ClothParam cloths, uint* inbFs, uint* inbEs,
	REAL* edgeCutWs, uint* iEnewNs, uint* iNnewNs,
	REAL* newMs, REAL* newVs, uint* newNodePhases, uchar* newIsFixeds)
{
	uint id = blockDim.x * blockIdx.x + threadIdx.x;
	if (id >= cloths._numEdges)
		return;

	uint inodeStart = iEnewNs[id];
	uint inodeEnd = iEnewNs[id + 1u];
	uint enumNodes = inodeEnd - inodeStart;
	if (enumNodes > 0u) {
		uint ino0 = cloths._es[(id << 1u) + 0u];
		uint ino1 = cloths._es[(id << 1u) + 1u];
		REAL w = edgeCutWs[id];

		uint i0num = 1u + inbFs[ino0 + 1u] - inbFs[ino0] + inbEs[ino0 + 1] - inbEs[ino0];
		uint i1num = 1u + inbFs[ino1 + 1u] - inbFs[ino1] + inbEs[ino1 + 1] - inbEs[ino1];
		REAL m0 = cloths._ms[ino0] / (REAL)i0num * (1.0 - w);
		REAL m1 = cloths._ms[ino1] / (REAL)i1num * w;
		atomicAdd_REAL(newMs + iNnewNs[ino0], -m0);
		atomicAdd_REAL(newMs + iNnewNs[ino1], -m1);
		REAL m = (m0 + m1) / (REAL)enumNodes;

		REAL3 v0, v1;
		getVector(cloths._vs, ino0, v0);
		getVector(cloths._vs, ino1, v1);
		REAL3 v = v0 + (v1 - v0) * w;

		uint nodePhase = cloths._nodePhases[ino0];
		for (uint ino = inodeStart; ino < inodeEnd; ino++) {
			newMs[ino] = m;
			setVector(newVs, ino, v);
			newNodePhases[ino] = nodePhase;
			newIsFixeds[ino] = 0u;
		}
	}
}
__global__ void compNewNodeParams_N_kernel(
	ClothParam cloths, uint* iNnewNs,
	REAL* newMs, REAL* newVs, uint* newNodePhases, uchar* newIsFixeds)
{
	uint id = blockDim.x * blockIdx.x + threadIdx.x;
	if (id >= cloths._numNodes)
		return;

	uint inodeStart = iNnewNs[id];
	uint inodeEnd = iNnewNs[id + 1u];

	REAL m = newMs[inodeStart];
	m /= (REAL)(inodeEnd - inodeStart);

	REAL3 v;
	getVector(cloths._vs, id, v);

	uint nodePhase = cloths._nodePhases[id];
	uint isFixed = cloths._isFixeds[id];

	for (uint ino = inodeStart; ino < inodeEnd; ino++) {
		newMs[ino] = m;
		setVector(newVs, ino, v);
		newNodePhases[ino] = nodePhase;
		newIsFixeds[ino] = isFixed;
	}
}

__global__ void compFixedPosition_kernel(
	ClothParam cloths, uint* inbFs, uint* nbFs)
{
	uint id = blockDim.x * blockIdx.x + threadIdx.x;
	if (id >= cloths._numNodes)
		return;

	uint iface = nbFs[inbFs[id]];
	uint ino0 = cloths._fs[iface * 3u + 0u];
	uint ino1 = cloths._fs[iface * 3u + 1u];
	uint ino2 = cloths._fs[iface * 3u + 2u];

	REAL3 n0, n1, n2, n;
	getVector(cloths._ns, ino0, n0);
	getVector(cloths._ns, ino1, n1);
	getVector(cloths._ns, ino2, n2);
	getVector(cloths._ns, id, n);

	REAL3 dir = (n0 + n1 + n2) * 0.3333333333333333333 - n;
	uint phase = cloths._nodePhases[id];
	REAL tolerance = cloths._thicknesses[phase] * 0.5;
	REAL l = Length(dir);
	if (l > tolerance)
		dir *= tolerance / l;
	n += dir;
	setVector(cloths._ns, id, n);
}

#endif