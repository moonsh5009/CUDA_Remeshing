#ifndef __BVH_TREE__
#define __BVH_TREE__

#pragma once
#include "RTriangle.h"

#define REFIT_BLOCKSIZE		512u
//#define CHECK_DETECTION		

using namespace std;

struct TriInfo {
	uint _id;
	uint _pos;
};

struct BVHNode
{
	uint _level;
	uint _face;
	uint _path;
	AABB _aabb;
};
struct BVHParam
{
public:
	uint* _levels;
	uint* _faces;
	REAL* _mins[3];
	REAL* _maxs[3];
#ifdef CHECK_DETECTION
	uint* _isDetecteds;
#endif
public:
	uint _maxLevel;
	uint _size;
	uint _pivot;
	uint _numFaces;
};

class BVH
{
public:
	Dvector<uint> _levels;
	Dvector<uint> _faces;
	Dvector<REAL> _mins[3];
	Dvector<REAL> _maxs[3];
#ifdef CHECK_DETECTION
	Dvector<uint> _isDetecteds;
#endif
	uint				_test;
	vector<TriInfo>		h_triInfos;
public:
	uint		_maxLevel;
	uint		_size;
	uint		_pivot;
	uint		_numFaces;
public:
	BVHParam	_param;
public:
	BVH() {
		_size = 0u;
	};
	virtual ~BVH() {}
public:
	void initBVHTreeDevice(uint numFaces) {
		_maxLevel = Log2(numFaces - 1u << 1u);
		_pivot = (1u << _maxLevel) - numFaces;
		_size = (1u << _maxLevel + 1u) - 1u - (_pivot << 1u);
		_pivot = ((1u << (_maxLevel - 1u)) - _pivot) << 1u;
		_numFaces = numFaces;

		_faces.resize(_numFaces);
		_levels.resize(_size);
		_mins[0].resize(_size);
		_mins[1].resize(_size);
		_mins[2].resize(_size);
		_maxs[0].resize(_size);
		_maxs[1].resize(_size);
		_maxs[2].resize(_size);
#ifdef CHECK_DETECTION
		_isDetecteds.resize(_numFaces);
#endif
	}
	inline void free(void) {
		_faces.clear();
		_levels.clear();
		_mins[0].clear();
		_mins[1].clear();
		_mins[2].clear();
		_maxs[0].clear();
		_maxs[1].clear();
		_maxs[2].clear();
	}
	inline void setParam(void) {
		_param._levels = _levels._list;
		_param._faces = _faces._list;
		_param._mins[0] = _mins[0]._list;
		_param._mins[1] = _mins[1]._list;
		_param._mins[2] = _mins[2]._list;
		_param._maxs[0] = _maxs[0]._list;
		_param._maxs[1] = _maxs[1]._list;
		_param._maxs[2] = _maxs[2]._list;
#ifdef CHECK_DETECTION
		_param._isDetecteds = _isDetecteds._list;
#endif
		_param._maxLevel = _maxLevel;
		_param._size = _size;
		_param._pivot = _pivot;
		_param._numFaces = _numFaces;
	}
public:
	void build(Dvector<uint>& fs, Dvector<REAL>& ns);
	void refitProximity(uint* fs,REAL* ns, uint* nodePhases, REAL* thicknesses);
	void refitCCD(uint* fs, REAL* ns, REAL* vs, uint* nodePhases, REAL* thicknesses, REAL dt);
	void draw(const AABB& aabb);
	void draw(void);
};

inline __device__ void getBVHAABB(AABB& aabb, BVHParam& bvh, uint ind) {
	aabb._min.x = bvh._mins[0][ind];
	aabb._min.y = bvh._mins[1][ind];
	aabb._min.z = bvh._mins[2][ind];
	aabb._max.x = bvh._maxs[0][ind];
	aabb._max.y = bvh._maxs[1][ind];
	aabb._max.z = bvh._maxs[2][ind];
}
inline __device__ void updateBVHAABB(BVHParam& bvh, const AABB& aabb, uint ind) {
	bvh._mins[0][ind] = aabb._min.x;
	bvh._mins[1][ind] = aabb._min.y;
	bvh._mins[2][ind] = aabb._min.z;
	bvh._maxs[0][ind] = aabb._max.x;
	bvh._maxs[1][ind] = aabb._max.y;
	bvh._maxs[2][ind] = aabb._max.z;
}
inline __host__ __device__ __forceinline__ int getBVHIndex(uint path, uint level) {
	return (1u << level) - 1u + path;
}
inline __host__ __device__ __forceinline__ int getBVHChild(uint path, uint level) {
	return (1u << level + 1u) - 1u + (path << 1u);
}
inline __device__ void setBVHNode(const BVHParam& bvh, const BVHNode& node, uint ind) {
	uint ileaf = bvh._size - bvh._numFaces;
	if (ind >= ileaf) 
		bvh._faces[ind - ileaf] = node._face;
	bvh._levels[ind] = node._level;
	bvh._mins[0][ind] = node._aabb._min.x;
	bvh._mins[1][ind] = node._aabb._min.y;
	bvh._mins[2][ind] = node._aabb._min.z;
	bvh._maxs[0][ind] = node._aabb._max.x;
	bvh._maxs[1][ind] = node._aabb._max.y;
	bvh._maxs[2][ind] = node._aabb._max.z;
}
inline __device__ void getBVHNode(BVHNode& node, const BVHParam& bvh, uint ind) {
	uint ileaf = bvh._size - bvh._numFaces;
	if (ind >= ileaf) 
		node._face = bvh._faces[ind - ileaf];
	node._level = bvh._levels[ind];
	node._aabb._min.x = bvh._mins[0][ind];
	node._aabb._min.y = bvh._mins[1][ind];
	node._aabb._min.z = bvh._mins[2][ind];
	node._aabb._max.x = bvh._maxs[0][ind];
	node._aabb._max.y = bvh._maxs[1][ind];
	node._aabb._max.z = bvh._maxs[2][ind];
}

#endif