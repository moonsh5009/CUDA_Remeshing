#ifndef __MESH_KERNEL_H__
#define __MESH_KERNEL_H__

#pragma once
#include "BVH.h"

namespace MeshKernel {
	void buildNbFs(
		const Dvector<uint>& fs, DPrefixArray<uint>& nbFs, uint numFaces, uint numNodes);
	void buildNbENs(
		const Dvector<uint>& es, DPrefixArray<uint>& nbEs, DPrefixArray<uint>& nbNs, uint numEdges, uint numNodes);
	void buildEnbXs(
		const Dvector<uint>& fs, const Dvector<uint>& es, const DPrefixArray<uint>& nbFs, 
		Dvector<uint>& EnbFs, Dvector<uint>& EnbNs, uint numEdges);
	void buildFnbEs(
		const Dvector<uint>& fs, const DPrefixArray<uint>& nbEs, Dvector<uint>& FnbEs, uint numFaces);
	void buildXnbXs(
		const Dvector<uint>& fs, const Dvector<uint>& es, 
		DPrefixArray<uint>& nbFs, DPrefixArray<uint>& nbEs, DPrefixArray<uint>& nbNs,
		Dvector<uint>& EnbFs, Dvector<uint>& EnbNs, Dvector<uint>& FnbEs,
		uint numFaces, uint numEdges, uint numNodes);
	void computeNormal(
		const Dvector<uint>& fs, const Dvector<REAL>& ns, Dvector<REAL>& fNorms, Dvector<REAL>& nNorms,
		uint numFaces, uint numNodes);

	void initEdgeConstraints(
		const Dvector<uint>& es, const Dvector<REAL>& ns, Dvector<REAL>& restLengths, uint numEdges);
	void initTriangleConstraints(
		const Dvector<uint>& fs, const Dvector<REAL>& ns, Dvector<REAL>& restAreas, Dvector<REAL>& invDs, uint numFaces);
	void initDihedralConstraints(
		const Dvector<uint>& fs, const Dvector<uint>& es, const Dvector<REAL>& ns, const Dvector<uint>& EnbNs, 
		Dvector<REAL>& restAngles, uint numEdges);
}

#endif