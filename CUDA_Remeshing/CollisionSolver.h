#ifndef __COLLISION_SOLVER_H__
#define __COLLISION_SOLVER_H__

#pragma once
#include "Cloth.h"
#include "Obstacle.h"

//----------------------------------------------
//#define USED_SDF
//----------------------------------------------
//#define COLLISION_TESTTIMER
//----------------------------------------------

struct ContactElem {
	bool _isFV;
	bool _isCCD;
	uchar _type[4];
	uint _i[4];
	REAL3 _norm;
	REAL _w[2];
};
struct ContactElemParam {
	ContactElem	*_elems;
	uint		*d_tmp;
	uint		_size;
};
class ContactElems {
public:
	Dvector<uint>		_lastBvttIds;
	uint				_lastBvhSize;
public:
	Dvector<ContactElem>	_elems;
	Dvector<uint>			d_tmp;
	uint					_size;
public:
	ContactElems() { d_tmp.resize(1); _size = 0u; }
	~ContactElems() {}
public:
	inline void resize(void) {
		_elems.resize(_size);
	}
	inline void resize(uint size) {
		_elems.resize(size);
		_size = size;
	}
	inline void extend(void) {
		_elems.extend(_size);
	}
	inline void clear(void) {
		_elems.clear();
		d_tmp.clear();
		_size = 0u;
	}
	inline ContactElemParam param(void) {
		ContactElemParam p;
		p._elems = _elems._list;
		p.d_tmp = d_tmp._list;
		p._size = _size;
		return p;
	}
};
struct ContactElem_CMP
{
	__host__ __device__
		bool operator()(const ContactElem& a, const ContactElem& b) {
		if (a._isFV != b._isFV)
			return a._isFV < b._isFV;

		if (a._type[0] != b._type[0])
			return a._type[0] < b._type[0];
		if (a._type[1] != b._type[1])
			return a._type[1] < b._type[1];
		if (a._type[2] != b._type[2])
			return a._type[2] < b._type[2];
		if (a._type[3] != b._type[3])
			return a._type[3] < b._type[3];

		if (a._i[0] != b._i[0])
			return a._i[0] < b._i[0];
		if (a._i[1] != b._i[1])
			return a._i[1] < b._i[1];
		if (a._i[2] != b._i[2])
			return a._i[2] < b._i[2];
		return a._i[3] < b._i[3];
	}
};

struct ContactElemSDF {
	REAL _dist;
	REAL3 _norm;
	REAL _w[2];
};
struct ContactElemSDFParam {
	ContactElemSDF* _felems;
	ContactElemSDF* _nelems;
	uint* d_tmp;
	uint			_size;
};
class ContactElemsSDF {
public:
	Dvector<ContactElemSDF> _felems;
	Dvector<ContactElemSDF> _nelems;
public:
	ContactElemsSDF() {}
	~ContactElemsSDF() {}
public:
	inline void resize(uint numFaces, uint numNodes) {
		_felems.resize(numFaces);
		_nelems.resize(numNodes);
	}
	inline ContactElemSDFParam param(void) {
		ContactElemSDFParam p;
		p._felems = _felems._list;
		p._nelems = _nelems._list;
		return p;
	}
};


typedef vector<set<uint2, uint2_CMP>> RIZone;
struct RIZoneParam {
	uint2* _ids;
	uint* _zones;
	uint	_size;
};
class DRIZone {
public:
	Dvector<uint2>	_ids;
	Dvector<uint>	_zones;
public:
	DRIZone() {}
	virtual ~DRIZone() {}
public:
	inline void clear(void) {
		_ids.clear();
		_zones.clear();
	}
	inline RIZoneParam param(void) {
		RIZoneParam p;
		p._ids = _ids._list;
		p._zones = _zones._list;
		p._size = _zones.size() - 1u;
		return p;
	}
};

namespace CollisionSolver {
	//--------------------------------------------------------------------------------------
	void getSelfLastBvtts(
		BVHParam& clothBvh,
		Dvector<uint2>& lastBvtts, Dvector<uint>& LastBvttIds,
		uint lastBvhSize, uint& lastBvttSize);
	void getObstacleLastBvtts(
		BVHParam& clothBvh, BVHParam& obsBvh,
		Dvector<uint2>& lastBvtts, Dvector<uint>& LastBvttIds,
		uint lastBvhSize, uint& lastBvttSize);
	//--------------------------------------------------------------------------------------
	void getContactElements(
		ContactElems& ceParam, Cloth* cloths, Obstacle* obstacles);
	void getClothCCDtime(
		Cloth* cloths, Obstacle* obstacles,
		const REAL dt, REAL* minTime);
	void compEpsilonCollision(
		Cloth* cloths, Obstacle* obstacles,
		const REAL dt);
	//--------------------------------------------------------------------------------------
	void MakeRigidImpactZone(
		const ContactElems& d_ceParam,
		RIZone& h_riz, DRIZone& d_riz,
		const PrefixArray<uint>& clothNbNs, const PrefixArray<uint>& obsNbNs);
	bool ResolveRigidImpactZone(
		ContactElems& ceParam,
		RIZone& h_riz, DRIZone& d_riz,
		const ObjParam& clothParam, const ObjParam& obsParam,
		const PrefixArray<uint>& clothNbNs, const PrefixArray<uint>& obsNbNs,
		const REAL dt);
	void compRigidImpactZone(
		ContactElems& ceParam,
		RIZone& h_riz, DRIZone& d_riz,
		Cloth* cloths, Obstacle* obstacles,
		const REAL dt);
	//--------------------------------------------------------------------------------------
	void compCollisionImpulse(
		ContactElems& ceParam,
		Cloth* cloths, Obstacle* obstacles,
		bool isProximity, const REAL dt);
	//--------------------------------------------------------------------------------------
	bool applyImpulse(
		Cloth* cloths, Obstacle* obstacles, uint smoothing, REAL dt);
	//--------------------------------------------------------------------------------------
};

#endif