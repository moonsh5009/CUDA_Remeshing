#ifndef __PARAMATERS_H__
#define __PARAMATERS_H__

#pragma once
#include <fstream>
#include <string>
#include "../GL/freeglut.h"
#include "../include/CUDA_Custom/PrefixArray.h"
#include "Matrix.h"

#define WINDOW_WIDTH				800
#define WINDOW_HEIGHT				600

#define SCENE						0
#define QUALITY						0	// 0: Low, 1: Medium, 2: High

#define SMOOTHED_RENDERING			0

#define TYPE_MESH_OBSTACLE			1
#define TYPE_MESH_CLOTH				2

//----------------------------------------------
#define LAPLACIAN_LAMBDA		0.25
#if QUALITY==0
#define COLLISION_SMOOTHING		5
#elif QUALITY==1
#define COLLISION_SMOOTHING		10
#else
#define COLLISION_SMOOTHING		15
#endif
//----------------------------------------------

#define COL_CCD_THICKNESS			1.0e-6
#define COL_CLEARANCE_RATIO			2.0

#define MIN_VOLUME					0.05

using namespace M_Matrix;

struct M_Ray {
	float3 _pos;
	float3 _dir;
};
struct M_ClickNode {
	float3	_offset;
	float	_dist0;
	uint	_id;
};

struct PDParam {
	REAL	*_Bs;
	REAL	*_Zs;
	REAL	*_Xs;
	REAL	*_newXs;
	REAL	*_prevXs;
	REAL	_dt;
	REAL	_invdt2;
	REAL	_omg;
	REAL	_underRelax;
};
struct BaseConstraint {
	uint	*_inos;
	REAL	*_ws;
	uint	_numConstraints;
};
struct EdgeConstraint : public BaseConstraint {
	REAL	*_restLengths;
};
struct TriangleConstraint : public BaseConstraint {
	REAL	*_firstLames;
	REAL	*_secondLames;

	REAL	*_restAreas;
	REAL	*_invDs;
};
struct DihedralConstraint : public BaseConstraint {
	REAL	*_restAngles;
};

struct ObjParam {
	REAL	*_impulses;
	REAL	*_colWs;
	REAL	*_thicknesses;
	REAL	*_frictions;

	uint	*_fs;
	uint	*_es;
	REAL	*_ns;
	REAL	*_vs;
	REAL	*_invMs;
	REAL	*_ms;
	uint	*_nodePhases;
	uchar	*_isFixeds;
	REAL	*_forces;

	uint	_numFaces;
	uint	_numEdges;
	uint	_numNodes;
	uchar	_type;
};
struct ClothParam : public ObjParam {
	REAL	*_restSolidFractions;
	REAL	*_maxFluidMass;
	REAL	*_mfs;
	REAL	*_ss;

	REAL	*_predNs;
	REAL	*_restNs;
	REAL	*_restLength;
	REAL	*_edgeLimits;
};

#endif