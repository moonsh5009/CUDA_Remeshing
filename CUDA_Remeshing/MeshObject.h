#ifndef __MESH_OBJECT_H__
#define __MESH_OBJECT_H__

#pragma once
#include "MeshKernel.h"

class MeshObject
{
public:
	ObjParam				*_param;
	BVH						*_bvh;
	RTriangle				*_RTri;

public:
	Dvector<REAL>			d_impulses;
	Dvector<REAL>			d_colWs;
	Dvector<REAL>			d_thicknesses;
	Dvector<REAL>			d_frictions;

public:
	uint					_numFaces;
	uint					_numEdges;
	uint					_numNodes;
	uint					_numPhases;
	uchar					_type;
public:
	Dvector<uint>			d_fs;
	Dvector<uint>			d_es;
	Dvector<REAL>			d_ns;
	Dvector<REAL>			d_n0s;
	Dvector<REAL>			d_vs;
	Dvector<REAL>			d_forces;
	Dvector<uint>			d_nodePhases;
public:
	vector<uint>			h_fs;
	vector<uint>			h_es;
	vector<REAL>			h_ns;
	vector<REAL>			h_vs;
	vector<uint>			h_nodePhases;
	vector<uint>			h_fs0;
	vector<uint>			h_es0;
	vector<REAL>			h_ns0;
	vector<REAL>			h_vs0;
	vector<uint>			h_nodePhases0;

public:
	Dvector<REAL>			d_ms;
	Dvector<REAL>			d_invMs;
	Dvector<uchar>			d_isFixeds;
public:
	vector<REAL>			h_ms;
	vector<uchar>			h_isFixeds;
	vector<REAL>			h_ms0;
	vector<uchar>			h_isFixeds0;

public:
	DPrefixArray<uint>		d_nbFs;
	DPrefixArray<uint>		d_nbEs;
	DPrefixArray<uint>		d_nbNs;
	Dvector<uint>			d_EnbFs;
	Dvector<uint>			d_EnbNs;
	Dvector<uint>			d_FnbEs;
public:
	PrefixArray<uint>		h_nbFs;
	PrefixArray<uint>		h_nbEs;
	PrefixArray<uint>		h_nbNs;
	vector<uint>			h_EnbFs;
	vector<uint>			h_EnbNs;

public:
	Dvector<REAL>			d_fNorms;
	Dvector<REAL>			d_nNorms;
public:
	vector<REAL>			h_fNorms;
	vector<REAL>			h_nNorms;

public:
	vector<float4>			h_frontColors;
	vector<float4>			h_backColors;
	vector<REAL>			h_thicknesses;
	vector<REAL>			h_frictions;
	vector<float4>			h_frontColors0;
	vector<float4>			h_backColors0;
	vector<REAL>			h_thicknesses0;
	vector<REAL>			h_frictions0;

public:
	MeshObject() { }
	virtual ~MeshObject() {}
public:
	inline virtual void setParam(void) {
		_param->_impulses = d_impulses._list;
		_param->_colWs = d_colWs._list;
		_param->_thicknesses = d_thicknesses._list;
		_param->_frictions = d_frictions._list;

		_param->_fs = d_fs._list;
		_param->_es = d_es._list;
		_param->_ns = d_ns._list;
		_param->_vs = d_vs._list;
		_param->_ms = d_ms._list;
		_param->_invMs = d_invMs._list;
		_param->_isFixeds = d_isFixeds._list;
		_param->_nodePhases = d_nodePhases._list;
		_param->_forces = d_forces._list;
		_param->_numFaces = _numFaces;
		_param->_numEdges = _numEdges;
		_param->_numNodes = _numNodes;
		_param->_type = _type;
	}
public:
	virtual void	init(void);
	virtual void	reset(void);
	virtual void	save(void);
	virtual void	updateElements(void);
public:
	void	addMesh(
		Mesh* mesh, REAL mass, REAL thickness, REAL friction, float4 frontColor, float4 backColor);
	void	initBVH(void);
public:
	void	initNbXs(void);
	void	initNormal(void);
	void	computeNormal(void);
public:
	void	draw(void);
	void	drawWire(void);
	virtual void	drawSurface(void);
public:
	void	copyToDevice(void);
	virtual void	copyToHost(void);
	void	copyNbToDevice(void);
	void	copyNbToHost(void);
	void	copyNormToDevice(void);
	void	copyNormToHost(void);
};
#endif