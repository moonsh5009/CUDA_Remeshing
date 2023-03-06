#ifndef __COLLISION_RESPONSE_CUH__
#define __COLLISION_RESPONSE_CUH__

#pragma once
#include "CollisionManager.cuh"
#include "CollisionDetection.cuh"

//-------------------------------------------------------------------------
__device__ bool resolveCollisionProximity_device(
	bool isFV,
	const REAL3& p0, const REAL3& p1, const REAL3& p2, const REAL3& p3,
	const REAL3& v0, const REAL3& v1, const REAL3& v2, const REAL3& v3,
	REAL invM0, REAL invM1, REAL invM2, REAL invM3,
	REAL w0, REAL w1, const REAL3& norm,
	REAL** impulses, REAL** colWs,
	bool isProximity, REAL friction, REAL thickness, REAL dt)
{
#if 1
	REAL stiffness = 10.0;
	bool result = false;

	REAL3 q0, q1, q2, q3;
	REAL dist, w2, w3;

	if (isFV) {
		w2 = 1.0 - w0 - w1;
		w3 = 1.0;
	}
	else {
		w3 = w1; w1 = w0;
		w0 = 1.0 - w1;
		w2 = 1.0 - w3;
	}

	REAL iP0 = invM0 * w0 * w0;
	REAL iP1 = invM1 * w1 * w1;
	REAL iP2 = invM2 * w2 * w2;
	REAL iP3 = invM3 * w3 * w3;
	REAL iPt = (iP0 + iP1 + iP2 + iP3);
	if (iPt > 0.0) {
		REAL3 relV;
		REAL relVN;
		if (isFV) {
			relV = v3 - v0 * w0 - v1 * w1 - v2 * w2;
			dist = Dot(p3 - p0 * w0 - p1 * w1 - p2 * w2, norm);
		}
		else {
			relV = v2 + (v3 - v2) * w3 - v0 - (v1 - v0) * w1;
			dist = Dot(p2 + (p3 - p2) * w3 - p0 - (p1 - p0) * w1, norm);
		}
		relVN = Dot(relV, norm);

		REAL imp;
		imp = (thickness - dist) / dt - relVN;
		//imp = (max(min(thickness, dist), thickness * 0.5) - dist) / dt - relVN;
		/*if (isProximity) {
			REAL d = thickness - dist;
			REAL imp_r = min(dt * stiffness * d, 0.1 * d / dt - relVN);
			REAL imp_c = (thickness - dist) / dt - relVN;
			imp = max(imp_c, imp_r);
		}
		else {
			imp = (thickness - dist) / dt - relVN;
		}*/

		if (imp > 0.0) {
			REAL3 impulse = imp * norm;
			if (isProximity) {
				REAL3 relVT = relV - relVN * norm;
				REAL lrelVT = Length(relVT);
				if (lrelVT)
					impulse -= min(friction * imp / lrelVT, 1.0) * relVT;
			}
			impulse *= 1.0 / iPt;

			/*REAL imp_v;
			imp_v = w0 * invM0;
			if (imp_v > 0.0) {
				q0 = -imp_v * impulse;
				atomicAdd_REAL(impulses[0] + 0u, q0.x);
				atomicAdd_REAL(impulses[0] + 1u, q0.y);
				atomicAdd_REAL(impulses[0] + 2u, q0.z);
				atomicAdd_REAL(colWs[0], 1.0);
			}
			imp_v = w1 * invM1;
			if (imp_v > 0.0) {
				q1 = -imp_v * impulse;
				atomicAdd_REAL(impulses[1] + 0u, q1.x);
				atomicAdd_REAL(impulses[1] + 1u, q1.y);
				atomicAdd_REAL(impulses[1] + 2u, q1.z);
				atomicAdd_REAL(colWs[1], 1.0);
			}
			imp_v = w2 * invM2;
			if (imp_v > 0.0) {
				if (isFV) imp_v = -imp_v;
				q2 = imp_v * impulse;
				atomicAdd_REAL(impulses[2] + 0u, q2.x);
				atomicAdd_REAL(impulses[2] + 1u, q2.y);
				atomicAdd_REAL(impulses[2] + 2u, q2.z);
				atomicAdd_REAL(colWs[2], 1.0);
			}
			imp_v = w3 * invM3;
			if (imp_v > 0.0) {
				q3 = imp_v * impulse;
				atomicAdd_REAL(impulses[3] + 0u, q3.x);
				atomicAdd_REAL(impulses[3] + 1u, q3.y);
				atomicAdd_REAL(impulses[3] + 2u, q3.z);
				atomicAdd_REAL(colWs[3], 1.0);
			}*/
			REAL imp_v;
			imp_v = w0 * w0 * invM0;
			if (imp_v > 0.0) {
				q0 = -imp_v * impulse;
				atomicAdd_REAL(impulses[0] + 0u, q0.x);
				atomicAdd_REAL(impulses[0] + 1u, q0.y);
				atomicAdd_REAL(impulses[0] + 2u, q0.z);
				atomicAdd_REAL(colWs[0], w0);
			}
			imp_v = w1 * w1 * invM1;
			if (imp_v > 0.0) {
				q1 = -imp_v * impulse;
				atomicAdd_REAL(impulses[1] + 0u, q1.x);
				atomicAdd_REAL(impulses[1] + 1u, q1.y);
				atomicAdd_REAL(impulses[1] + 2u, q1.z);
				atomicAdd_REAL(colWs[1], w1);
			}
			imp_v = w2 * w2 * invM2;
			if (imp_v > 0.0) {
				if (isFV) imp_v = -imp_v;
				q2 = imp_v * impulse;
				atomicAdd_REAL(impulses[2] + 0u, q2.x);
				atomicAdd_REAL(impulses[2] + 1u, q2.y);
				atomicAdd_REAL(impulses[2] + 2u, q2.z);
				atomicAdd_REAL(colWs[2], w2);
			}
			imp_v = w3 * w3 * invM3;
			if (imp_v > 0.0) {
				q3 = imp_v * impulse;
				atomicAdd_REAL(impulses[3] + 0u, q3.x);
				atomicAdd_REAL(impulses[3] + 1u, q3.y);
				atomicAdd_REAL(impulses[3] + 2u, q3.z);
				atomicAdd_REAL(colWs[3], w3);
			}

			result = true;
		}
	}
	return result;
#else
	bool result = false;

	REAL3 q0, q1, q2, q3;
	REAL dist, w2, w3;

	if (isFV) {
		w2 = 1.0 - w0 - w1;
		w3 = 1.0;
	}
	else {
		w3 = w1; w1 = w0;
		w0 = 1.0 - w1;
		w2 = 1.0 - w3;
	}

	REAL iP0 = w0 * w0;
	REAL iP1 = w1 * w1;
	REAL iP2 = w2 * w2;
	REAL iP3 = w3 * w3;
	REAL iPt = iP0 + iP1 + iP2 + iP3;
	if (iPt > 0.0) {
		REAL3 relV;
		REAL relVN;
		if (isFV)
			relV = v3 - v0 * w0 - v1 * w1 - v2 * w2;
		else
			relV = v2 + (v3 - v2) * w3 - v0 - (v1 - v0) * w1;

		dist = Dot(p3 - p0, norm);
		if (dist < 0.0)
			printf("asdfasdf %f %d\n", dist, isFV);
		/*if (isFV)
			dist = Dot(p3 - p0 * w0 - p1 * w1 - p2 * w2, norm);
		else
			dist = Dot(p2 + (p3 - p2) * w3 - p0 - (p1 - p0) * w1, norm);*/
		relVN = Dot(relV, norm);

		REAL imp = (thickness - dist) / dt - relVN;
		if (imp > 0.0) {
			REAL3 impulse = imp * norm;
			if (isProximity) {
				REAL3 relVT = relV - relVN * norm;
				REAL lrelVT = Length(relVT);
				if (lrelVT)
					impulse -= min(friction * imp / lrelVT, 1.0) * relVT;
			}
			impulse *= 1.0 / iPt;

			REAL imp_v;
			imp_v = w0 * invM0;
			if (imp_v > 0.0) {
				q0 = -imp_v * impulse;
				atomicAdd_REAL(impulses[0] + 0u, q0.x);
				atomicAdd_REAL(impulses[0] + 1u, q0.y);
				atomicAdd_REAL(impulses[0] + 2u, q0.z);
				atomicAdd_REAL(colWs[0], 1.0);
			}
			imp_v = w1 * invM1;
			if (imp_v > 0.0) {
				q1 = -imp_v * impulse;
				atomicAdd_REAL(impulses[1] + 0u, q1.x);
				atomicAdd_REAL(impulses[1] + 1u, q1.y);
				atomicAdd_REAL(impulses[1] + 2u, q1.z);
				atomicAdd_REAL(colWs[1], 1.0);
			}
			imp_v = w2 * invM2;
			if (imp_v > 0.0) {
				if (isFV) imp_v = -imp_v;
				q2 = imp_v * impulse;
				atomicAdd_REAL(impulses[2] + 0u, q2.x);
				atomicAdd_REAL(impulses[2] + 1u, q2.y);
				atomicAdd_REAL(impulses[2] + 2u, q2.z);
				atomicAdd_REAL(colWs[2], 1.0);
			}
			imp_v = w3 * invM3;
			if (imp_v > 0.0) {
				q3 = imp_v * impulse;
				atomicAdd_REAL(impulses[3] + 0u, q3.x);
				atomicAdd_REAL(impulses[3] + 1u, q3.y);
				atomicAdd_REAL(impulses[3] + 2u, q3.z);
				atomicAdd_REAL(colWs[3], 1.0);
			}

			result = true;
		}
	}
	return result;
#endif
}
__device__ bool ClothDetectedRigidImpactZone_device(
	bool isFV,
	const REAL3& p0, const REAL3& p1, const REAL3& p2, const REAL3& p3,
	const REAL3& v0, const REAL3& v1, const REAL3& v2, const REAL3& v3,
	REAL w0, REAL w1, const REAL3& norm,
	REAL thickness, REAL dt)
{
	bool result = false;
#if 1
	REAL dist, w2, w3;

	if (isFV) {
		w2 = 1.0 - w0 - w1;
		w3 = 1.0;
	}
	else {
		w3 = w1; w1 = w0;
		w0 = 1.0 - w1;
		w2 = 1.0 - w3;
	}

	REAL3 relV;
	REAL relVN;
	if (isFV)
		relV = v3 - v0 * w0 - v1 * w1 - v2 * w2;
	else
		relV = v2 + (v3 - v2) * w3 - v0 - (v1 - v0) * w1;

	dist = Dot(p3 - p0, norm);
	relVN = Dot(relV, norm);

	REAL imp = (min(thickness, dist) * 0.1 - dist) / dt - relVN;

	if (imp > 0.0)
		result = true;
#else
	REAL dist, w2, w3;

	if (isFV) {
		w2 = 1.0 - w0 - w1;
		w3 = 1.0;
	}
	else {
		w3 = w1; w1 = w0;
		w0 = 1.0 - w1;
		w2 = 1.0 - w3;
	}

	if (isFV)
		dist = Dot(p3 - p0 * w0 - p1 * w1 - p2 * w2, norm);
	else
		dist = Dot(p2 + (p3 - p2) * w3 - p0 - (p1 - p0) * w1, norm);
	
	//if (dist < thickness) {
		REAL t = 10.0;
		isDetected_CCD(isFV, p0, p1, p2, p3, p0 + v0 * dt, p1 + v1 * dt, p2 + v2 * dt, p3 + v3 * dt, thickness, &t);
		//getCCDTime_device(isFV, p0, p1, p2, p3, p0 + v0 * dt, p1 + v1 * dt, p2 + v2 * dt, p3 + v3 * dt, thickness, &t);
		if (t <= 1.0) {
			//printf("%f\n", t);
			result = true;
		}
	//}
#endif

	return result;
}
//-------------------------------------------------------------------------
__global__ void compDetectedRigidImpactZone_CE_kernel(
	ContactElemParam ceParam,
	ObjParam clothParam, ObjParam obsParam,
	const REAL dt, bool* isApplied)
{
	uint id = threadIdx.x + blockDim.x * blockIdx.x;
	if (id >= ceParam._size)
		return;

	ContactElem ce = ceParam._elems[id];

	ObjParam* param;
	bool isCCD = true;

	REAL3 ps[4], vs[4];
	REAL thicknessi, thicknessj;
	for (uint i = 0u; i < 4u; i++) {
		if (ce._type[i] == TYPE_MESH_CLOTH)
			param = &clothParam;
		else
			param = &obsParam;
		getVector(param->_ns, ce._i[i], ps[i]);
		getVector(param->_vs, ce._i[i], vs[i]);

		uint phase = param->_nodePhases[ce._i[i]];
		if (i == 0) {
			thicknessi = param->_thicknesses[phase];
		}
		else if (i == 3) {
			thicknessj = param->_thicknesses[phase];
		}
	}

	if (ClothDetectedRigidImpactZone_device(
		ce._isFV,
		ps[0], ps[1], ps[2], ps[3],
		vs[0], vs[1], vs[2], vs[3],
		ce._w[0], ce._w[1], ce._norm, (thicknessi + thicknessj) * 0.5, dt))
	{
		*isApplied = true;
	}
	else isCCD = false;

	ce._isCCD = isCCD;
	ceParam._elems[id] = ce;
	/*uint id = threadIdx.x + blockDim.x * blockIdx.x;
	if (id >= ceParam._size)
		return;

	ContactElem ce = ceParam._elems[id];

	ObjParam* param;
	bool isCCD = false;

	REAL3 ps[4], vs[4];
	REAL thicknessi, thicknessj;
	REAL* vPtrs[4];
	REAL lv = 0.0;
	for (uint i = 0u; i < 4u; i++) {
		if (ce._type[i] == TYPE_SPH_PARTICLE) {
			getVector(sphParam._xs, ce._i[i], ps[i]);
			getVector(sphParam._vs, ce._i[i], vs[i]);
			vPtrs[i] = sphParam._vs + ce._i[i] * 3u;

			uint phase = sphParam._phases[ce._i[i]];
			REAL s = sphParam._ss[ce._i[i]];
			thicknessj = sphParam._radii[phase];
			thicknessj *= S3TO1(s);
		}
		else {
			if (ce._type[i] == TYPE_MESH_CLOTH) {
				param = &clothParam;
				if (param->_isFixeds[ce._i[i]])
					isCCD = true;
			}
			else {
				param = &obsParam;
				isCCD = true;
			}
			getVector(param->_ns, ce._i[i], ps[i]);
			getVector(param->_vs, ce._i[i], vs[i]);
			vPtrs[i] = param->_vs + ce._i[i] * 3u;

			uint phase = param->_nodePhases[ce._i[i]];
			if (i == 0) {
				thicknessi = param->_thicknesses[phase];
			}
			else if (i == 3) {
				thicknessj = param->_thicknesses[phase];
			}
		}
	}

	if (!isCCD) {
		for (uint i = 0u; i < 4u; i++) {
			if (Length(vs[i]) * dt > (thicknessi + thicknessj) * 0.5) {
				isCCD = true;
				break;
			}
		}
	}
	if (!isCCD) {
		if (ClothDetectedRigidImpactZone_device(
			ce._isFV,
			ps[0], ps[1], ps[2], ps[3],
			vs[0], vs[1], vs[2], vs[3],
			ce._w[0], ce._w[1], ce._norm, (thicknessi + thicknessj) * 0.5, dt))
		{
			printf("%d\n", id);
			for (uint i = 0u; i < 4u; i++)
				setVector(vPtrs[i], 0, make_REAL3(0.0));
		}
	}*/
}
__global__ void compCollisionImpulse_CE_kernel(
	ContactElemParam ceParam,
	ObjParam clothParam, ObjParam obsParam,
	bool isProximity, const REAL dt)
{
	uint id = threadIdx.x + blockDim.x * blockIdx.x;
	if (id >= ceParam._size)
		return;

	ContactElem ce = ceParam._elems[id];

	ObjParam* param;

	uint ino;
	REAL3 ps[4], vs[4];
	REAL invMs[4];
	REAL* impulses[4];
	REAL* colWs[4];
	REAL thicknessi, thicknessj;

	bool isFixed;
	REAL friction = 0.0;

	for (uint i = 0u; i < 4u; i++) {
		if (ce._type[i] == TYPE_MESH_CLOTH) {
			param = &clothParam;
			isFixed = param->_isFixeds[ce._i[i]];
			if (!isFixed)
				invMs[i] = param->_invMs[ce._i[i]];
			else
				invMs[i] = 0.0;
		}
		else {
			param = &obsParam;
			invMs[i] = 0.0;
		}

		getVector(param->_ns, ce._i[i], ps[i]);
		getVector(param->_vs, ce._i[i], vs[i]);

		uint phase = param->_nodePhases[ce._i[i]];
		if (i == 0) {
			thicknessi = param->_thicknesses[phase];
			friction += param->_frictions[phase];
		}
		else if (i == 3) {
			thicknessj = param->_thicknesses[phase];
			friction += param->_frictions[phase];
		}

		impulses[i] = param->_impulses + ce._i[i] * 3u;
		colWs[i] = param->_colWs + ce._i[i];
	}
	friction *= 0.5;

	resolveCollisionProximity_device(
		ce._isFV,
		ps[0], ps[1], ps[2], ps[3],
		vs[0], vs[1], vs[2], vs[3],
		invMs[0], invMs[1], invMs[2], invMs[3],
		ce._w[0], ce._w[1], ce._norm,
		impulses, colWs, isProximity, friction, (thicknessi + thicknessj) * 0.5, dt);
}
//-------------------------------------------------------------------------
__global__ void applyClothCollision_kernel(
	ObjParam clothParam, REAL dt, bool* isApplied)
{
	uint id = threadIdx.x + blockDim.x * blockIdx.x;
	if (id >= clothParam._numNodes)
		return;

	uint ino = id * 3u;
	REAL colW = clothParam._colWs[id];
	REAL3 impulse = make_REAL3(0.0);
	if (colW > 0.0) {
		impulse.x = clothParam._impulses[ino + 0u];
		impulse.y = clothParam._impulses[ino + 1u];
		impulse.z = clothParam._impulses[ino + 2u];
		impulse *= 1.0 / colW;

		*isApplied = true;
	}

	clothParam._impulses[ino + 0u] = impulse.x;
	clothParam._impulses[ino + 1u] = impulse.y;
	clothParam._impulses[ino + 2u] = impulse.z;
}
__global__ void SmoothingImpulse_kernel(
	ObjParam param, uint* inbNs, uint* nbNs, REAL* prevImpulses, REAL* impulses)
{
	uint id = threadIdx.x + blockDim.x * blockIdx.x;
	if (id >= param._numNodes)
		return;

	REAL3 nbImps = make_REAL3(0.0);
	REAL3 impulse, nbImp;

	uint istart = inbNs[id];
	uint iend = inbNs[id + 1u];
	uint ino, i;

	REAL w, ws;
	REAL limp, lnbImp;

	getVector(prevImpulses, id, impulse);
	if (!param._isFixeds[id]) {
		limp = Length(impulse);

		ws = 0.0;
		for (i = istart; i < iend; i++) {
			ino = nbNs[i];
			getVector(prevImpulses, ino, nbImp);
			lnbImp = Length(nbImp);
			lnbImp *= limp;

			w = 0.0;
			if (lnbImp > 1.0e-40)
				w = Dot(impulse, nbImp) / lnbImp;
			w = w + 1.0;

			nbImps += (nbImp - impulse) * w;
			ws += w;
		}
		if (ws > 1.0e-40)	ws = 1.0 / ws;
		else				ws = 0.0;
		nbImps *= ws;

		impulse += nbImps * LAPLACIAN_LAMBDA;
	}
	setVector(impulses, id, impulse);
}
__global__ void applyClothImpulse_kernel(
	ObjParam clothParam)
{
	uint id = threadIdx.x + blockDim.x * blockIdx.x;
	if (id >= clothParam._numNodes)
		return;

	if (!clothParam._isFixeds[id]) {
		uint ino = id * 3u;
		REAL3 impulse;
		impulse.x = clothParam._impulses[ino + 0u];
		impulse.y = clothParam._impulses[ino + 1u];
		impulse.z = clothParam._impulses[ino + 2u];

		REAL3 v;
		v.x = clothParam._vs[ino + 0u];
		v.y = clothParam._vs[ino + 1u];
		v.z = clothParam._vs[ino + 2u];
		v += impulse;

		clothParam._vs[ino + 0u] = v.x;
		clothParam._vs[ino + 1u] = v.y;
		clothParam._vs[ino + 2u] = v.z;
	}
}
//-------------------------------------------------------------------------
__global__ void ApplyRigidImpactZone_kernel(
	ObjParam clothParam, ObjParam obsParam,
	RIZoneParam riz, const REAL dt)
{
	uint id = blockDim.x * blockIdx.x + threadIdx.x;
	if (id >= riz._size)
		return;

	uint istart = riz._zones[id];
	uint iend = riz._zones[id + 1];
	uint i;
	uint2 ino;

	bool isFixed;
	REAL3 gc = make_REAL3(0.0);
	REAL3 av = make_REAL3(0.0);
	REAL3 p, v, q;
	REAL tmp, mass;
	tmp = 0.0;
	for (i = istart; i < iend; i++) {
		ino = riz._ids[i];
		if (ino.y == TYPE_MESH_CLOTH) {
			isFixed = clothParam._isFixeds[ino.x];
			if (!isFixed)
				mass = clothParam._ms[ino.x];
			else
				mass = 1.0e+10;
			getVector(clothParam._ns, ino.x, p);
			getVector(clothParam._vs, ino.x, v);
		}
		else if (ino.y == TYPE_MESH_OBSTACLE) {
			mass = 1.0e+10;
			getVector(obsParam._ns, ino.x, p);
			getVector(obsParam._vs, ino.x, v);
		}

		gc += p * mass;
		av += v * mass;
		tmp += mass;
	}
	tmp = 1.0 / tmp;
	gc *= tmp;
	av *= tmp;

	REAL3 L = make_REAL3(0.0);
	REAL I[9] = { 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0 };
	for (i = istart; i < iend; i++) {
		ino = riz._ids[i];
		if (ino.y == TYPE_MESH_CLOTH) {
			isFixed = clothParam._isFixeds[ino.x];
			if (!isFixed)
				mass = clothParam._ms[ino.x];
			else
				mass = 1.0e+10;
			getVector(clothParam._ns, ino.x, p);
			getVector(clothParam._vs, ino.x, v);
		}
		else if (ino.y == TYPE_MESH_OBSTACLE) {
			mass = 1.0e+10;
			getVector(obsParam._ns, ino.x, p);
			getVector(obsParam._vs, ino.x, v);
		}

		q = p - gc;
		L += mass * Cross(q, v - av);

		tmp = Dot(q, q);
		/*I[0] += tmp - q.x * q.x;	I[1] += -q.x * q.y;			I[2] += -q.x * q.z;
		I[3] += -q.y * q.x;			I[4] += tmp - q.y * q.y;	I[5] += -q.y * q.z;
		I[6] += -q.z * q.x;			I[7] += -q.z * q.y;			I[8] += tmp - q.z * q.z;*/
		I[0] += mass * (tmp - q.x * q.x);	I[1] += mass * (-q.x * q.y);		I[2] += mass * (-q.x * q.z);
		I[3] += mass * (-q.y * q.x);		I[4] += mass * (tmp - q.y * q.y);	I[5] += mass * (-q.y * q.z);
		I[6] += mass * (-q.z * q.x);		I[7] += mass * (-q.z * q.y);		I[8] += mass * (tmp - q.z * q.z);
	}
	REAL Iinv[9];
	CalcInvMat3(Iinv, I);
	REAL3 omg;
	omg.x = Iinv[0] * L.x + Iinv[1] * L.y + Iinv[2] * L.z;
	omg.y = Iinv[3] * L.x + Iinv[4] * L.y + Iinv[5] * L.z;
	omg.z = Iinv[6] * L.x + Iinv[7] * L.y + Iinv[8] * L.z;

	REAL lomg = Length(omg);
	if (lomg) {
		omg *= 1.0 / lomg;
		REAL3 xf, xr, px;
		REAL comg = cos(dt * lomg);
		REAL somg = sin(dt * lomg);
		for (i = istart; i < iend; i++) {
			ino = riz._ids[i];
			if (ino.y == TYPE_MESH_CLOTH) {
				isFixed = clothParam._isFixeds[ino.x];
				if (!isFixed) {
					getVector(clothParam._ns, ino.x, p);

					q = p - gc;
					xf = Dot(q, omg) * omg;
					xr = q - xf;
					px = gc + dt * av + xf + comg * xr + Cross(somg * omg, xr);
					v = (px - p) / dt;

					setVector(clothParam._vs, ino.x, v);
				}
			}
			else if (ino.y == TYPE_MESH_OBSTACLE) {
			}
		}
	}
}

#endif