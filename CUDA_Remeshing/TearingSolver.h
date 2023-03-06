#ifndef __TEARING_SOLVER_H__
#define __TEARING_SOLVER_H__

#pragma once
#include "Cloth.h"

struct CutPlane {
	REAL3 _norm;
	REAL3 _u;
	REAL3 _v;
	REAL _pos;
	REAL _uMax;
	REAL _uMin;
	REAL _vMax;
	REAL _vMin;
};
namespace TearingSolver {
	void findExtendTriangle(Cloth* cloths);
	bool getElementsCutInfo(
		Cloth* cloths, Dvector<CutPlane>& d_cutPlanes,
		Dvector<uchar>& faceCutInfos, Dvector<REAL>& faceCutWs,
		Dvector<uchar>& edgeCutInfos, Dvector<REAL>& edgeCutWs);
	void genNewNodes(
		Cloth* cloths,
		Dvector<uchar>& faceCutInfos, Dvector<REAL>& faceCutWs,
		Dvector<uchar>& edgeCutInfos, Dvector<REAL>& edgeCutWs,
		Dvector<uint>& iFnewNs, Dvector<uint>& iEnewNs, Dvector<uint>& iNnewNs,
		Dvector<REAL>& newNodes, Dvector<REAL>& newN0s);
	void genNewFaces(
		Cloth* cloths, 
		Dvector<uchar>& faceCutInfos, Dvector<REAL>& faceCutWs,
		Dvector<uchar>& edgeCutInfos, Dvector<REAL>& edgeCutWs,
		Dvector<uint>& iFnewNs, Dvector<uint>& iEnewNs, Dvector<uint>& iNnewNs,
		Dvector<uint>& newFaces);
	void compNewNodeParams(
		Cloth* cloths, Dvector<REAL>& faceCutWs, Dvector<REAL>& edgeCutWs,
		Dvector<uint>& iFnewNs, Dvector<uint>& iEnewNs, Dvector<uint>& iNnewNs,
		Dvector<REAL>& newNodes);
	void buildNewAdjacency(
		Cloth* cloths, Dvector<uint>& newFaces, uint numNewNodes, Dvector<uint>& newEdges);
	void compCutting(Cloth* cloths, Dvector<CutPlane>& cutPlanes);
}

#endif