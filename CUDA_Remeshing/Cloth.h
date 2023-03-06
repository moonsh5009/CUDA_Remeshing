#ifndef __CLOTH_H__
#define __CLOTH_H__

#pragma once
#include "MeshObject.h"

#define CLOTH_STRAIN			1
#define CLOTH_BENDING			1

class Cloth : public MeshObject
{
public:
	PDParam				*_pdParam;

#if CLOTH_STRAIN==0
	EdgeConstraint		*_edgeConstraints;
#else
	TriangleConstraint	*_triangleConstraints;
#endif
	
#if CLOTH_BENDING==0
	EdgeConstraint		*_edgeBendingConstraints;
#else
	DihedralConstraint	*_dihedralBendingConstraints;
#endif

public:
	Dvector<REAL>	d_Bs;
	Dvector<REAL>	d_Zs;
	Dvector<REAL>	d_Xs;
	Dvector<REAL>	d_newXs;
	Dvector<REAL>	d_prevXs;
public:
	Dvector<REAL>	d_restSolidFractions;
	Dvector<REAL>	d_maxFluidMasses;
	Dvector<REAL>	d_ss;
	Dvector<REAL>	d_mfs;
	vector<REAL>	h_restSolidFractions;
	vector<REAL>	h_maxFluidMasses;
	vector<REAL>	h_ss;
	vector<REAL>	h_mfs;
	vector<REAL>	h_mfs0;
public:
	Dvector<REAL>	d_restNs;
	vector<REAL>	h_restNs;
	vector<REAL>	h_restNs0;
public:
	Dvector<REAL>	d_strainWs;
	Dvector<REAL>	d_bendingWs;
	vector<REAL>	h_strainWs;
	vector<REAL>	h_bendingWs;
	vector<REAL>	h_strainWs0;
	vector<REAL>	h_bendingWs0;
#if CLOTH_STRAIN==1
	Dvector<REAL>	d_firstLames;
	Dvector<REAL>	d_secondLames;
	vector<REAL>	h_firstLames;
	vector<REAL>	h_secondLames;
	vector<REAL>	h_firstLames0;
	vector<REAL>	h_secondLames0;
#endif
public:
	Dvector<REAL>	d_restLengths;

#if CLOTH_STRAIN==1
	Dvector<REAL>	d_restAreas;
	Dvector<REAL>	d_invDs;
#endif

#if CLOTH_BENDING==0
	Dvector<REAL>	d_restBendingLengths;
#else
	Dvector<REAL>	d_restBendingAngles;
#endif

public:
	Dvector<REAL>	d_edgeLimits;
	vector<REAL>	h_edgeLimits;
	vector<REAL>	h_edgeLimits0;

	Dvector<REAL>	d_isExtends;
	Dvector<REAL>	d_deformGrads;
	Dvector<REAL>	d_tearingPoints;
	Dvector<uint>	d_tearingFaces;
public:
	vector<M_ClickNode>	h_clickNodes;
public:
	Cloth() {
		_type = TYPE_MESH_OBSTACLE;
		init();
	}
	virtual ~Cloth() {}
public:
	inline void setPDParam(void) {
		_pdParam->_Bs = d_Bs._list;
		_pdParam->_Zs = d_Zs._list;
		_pdParam->_Xs = d_Xs._list;
		_pdParam->_newXs = d_newXs._list;
		_pdParam->_prevXs = d_prevXs._list;
	}
	inline virtual void setParam(void) {
		MeshObject::setParam();
		((ClothParam*)_param)->_mfs = d_mfs._list;
		((ClothParam*)_param)->_ss = d_ss._list;
		((ClothParam*)_param)->_restSolidFractions = d_restSolidFractions._list;
		((ClothParam*)_param)->_maxFluidMass = d_maxFluidMasses._list;

		((ClothParam*)_param)->_predNs = d_Xs._list;
		((ClothParam*)_param)->_restNs = d_restNs._list;
		((ClothParam*)_param)->_restLength = d_restLengths._list;
		((ClothParam*)_param)->_edgeLimits = d_edgeLimits._list;
		setPDParam();
	}
public:
	virtual void	init(void);
	virtual void	reset(void);
	virtual void	save(void);
	virtual void	updateElements(void);
public:
	void	addCloth(
		Mesh* mesh, REAL mass, REAL 
		, REAL friction,
		REAL restSolidFraction, REAL maxFluidMass,
		float4 frontColor, float4 backColor, bool isSaved = true);
	void	initConstraints(void);
public:
	void	fix(void);
	void	moveFixed(REAL3 vel);
	void	moveFixed(REAL3 lvel, REAL3 rvel);
	void	rotateFixed(REAL3 degreeL, REAL3 degreeR, REAL moveL, REAL moveR, REAL invdt);
public:
	virtual void	drawSurface(void);
	virtual void	copyToHost(void);
};
#endif