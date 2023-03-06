#include "TearingSolver.cuh"

void TearingSolver::findExtendTriangle(Cloth* cloths) {
	/*compExtendEdge_kernel << <divup(cloths->_numEdges, BLOCKSIZE), BLOCKSIZE >> > (
		*((ClothParam*)cloths->_param), cloths->d_isExtends());
	CUDA_CHECK(cudaPeekAtLastError());

	Dvector<REAL> nodeExtends(cloths->_numNodes);
	cloths->d_deformGrads.memset(0);
	compNodeTearingGradient_kernel << <divup(cloths->_numNodes, BLOCKSIZE), BLOCKSIZE >> > (
		*((ClothParam*)cloths->_param), cloths->d_nbEs._index(), cloths->d_nbEs._array(),
		cloths->d_isExtends(), nodeExtends(), cloths->d_deformGrads());
	CUDA_CHECK(cudaPeekAtLastError());*/
}
bool TearingSolver::getElementsCutInfo(
	Cloth* cloths, Dvector<CutPlane>& cutPlanes, 
	Dvector<uchar>& faceCutInfos, Dvector<REAL>& faceCutWs,
	Dvector<uchar>& edgeCutInfos, Dvector<REAL>& edgeCutWs)
{
	Dvector<uchar> fedgeCutInfos(cloths->_numFaces << 1u);
	Dvector<uint> edgeCutNums(cloths->_numEdges);
	edgeCutWs.memset(0);
	edgeCutNums.memset(0);

	bool result;
	bool* d_isApplied;
	CUDA_CHECK(cudaMalloc((void**)&d_isApplied, sizeof(bool)));
	CUDA_CHECK(cudaMemset(d_isApplied, 0, sizeof(bool)));
	getElementsCutInfo_kernel << <divup(cloths->_numFaces, BLOCKSIZE), BLOCKSIZE >> > (
		*((ClothParam*)cloths->_param), cloths->d_FnbEs(), cutPlanes(), cutPlanes.size(),
		faceCutInfos(), fedgeCutInfos(), edgeCutWs(), edgeCutNums(), d_isApplied);
	CUDA_CHECK(cudaPeekAtLastError());
	compCutPointWeight_E_kernel << <divup(cloths->_numEdges, BLOCKSIZE), BLOCKSIZE >> > (
		*((ClothParam*)cloths->_param), cloths->d_FnbEs(), cloths->d_EnbFs(),
		fedgeCutInfos(), edgeCutInfos(), edgeCutWs(), edgeCutNums(), d_isApplied);
	CUDA_CHECK(cudaPeekAtLastError());
	CUDA_CHECK(cudaMemcpy(&result, d_isApplied, sizeof(bool), cudaMemcpyDeviceToHost));
	CUDA_CHECK(cudaFree(d_isApplied));

	if (result) {
		compCutPointWeight_F_kernel << <divup(cloths->_numFaces, BLOCKSIZE), BLOCKSIZE >> > (
			*((ClothParam*)cloths->_param), faceCutInfos(), faceCutWs());
		CUDA_CHECK(cudaPeekAtLastError());
	}
	return result;
}
void TearingSolver::genNewNodes(
	Cloth* cloths, 
	Dvector<uchar>& faceCutInfos, Dvector<REAL>& faceCutWs,
	Dvector<uchar>& edgeCutInfos, Dvector<REAL>& edgeCutWs,
	Dvector<uint>& iFnewNs, Dvector<uint>& iEnewNs, Dvector<uint>& iNnewNs,
	Dvector<REAL>& newNodes, Dvector<REAL>& newN0s)
{
	Dvector<uint> numNnbCEs(cloths->_numNodes);
	Dvector<uint> numEnbCEs(cloths->_numEdges);
	uint numNnewNs = 0u;
	uint numEnewNs = 0u;
	uint numFnewNs = 0u;
	numNnbCEs.memset(0);
	numEnbCEs.memset(0);

	compNumNbCutEdges_F_kernel << <divup(cloths->_numFaces, BLOCKSIZE), BLOCKSIZE >> > (
		*((ClothParam*)cloths->_param), cloths->d_FnbEs(), faceCutInfos(), faceCutWs(),
		numEnbCEs(), numNnbCEs());
	CUDA_CHECK(cudaPeekAtLastError());
	compNumNbCutEdges_E_kernel << <divup(cloths->_numEdges, BLOCKSIZE), BLOCKSIZE >> > (
		*((ClothParam*)cloths->_param), cloths->d_EnbFs(), edgeCutInfos(), edgeCutWs(),
		numEnbCEs(), numNnbCEs());
	CUDA_CHECK(cudaPeekAtLastError());
	compNumNbCutEdges_N_kernel << <divup(cloths->_numNodes, BLOCKSIZE), BLOCKSIZE >> > (
		*((ClothParam*)cloths->_param), cloths->d_nbEs._index(), cloths->d_nbEs._array(), cloths->d_EnbFs(),
		numNnbCEs());
	CUDA_CHECK(cudaPeekAtLastError());

	compExtraNode_E_kernel << <divup(cloths->_numEdges, BLOCKSIZE), BLOCKSIZE >> > (
		*((ClothParam*)cloths->_param), cloths->d_EnbFs(), edgeCutInfos(), edgeCutWs(),
		numEnbCEs(), numNnbCEs());
	CUDA_CHECK(cudaPeekAtLastError());
	compExtraNode_F_kernel << <divup(cloths->_numFaces, BLOCKSIZE), BLOCKSIZE >> > (
		*((ClothParam*)cloths->_param), cloths->d_FnbEs(), faceCutInfos(), faceCutWs(),
		numEnbCEs(), numNnbCEs());
	CUDA_CHECK(cudaPeekAtLastError());

	compNumNewNodes_F_kernel << <divup(cloths->_numEdges, BLOCKSIZE), BLOCKSIZE >> > (
		*((ClothParam*)cloths->_param), faceCutInfos(), faceCutWs(), iFnewNs());
	CUDA_CHECK(cudaPeekAtLastError());
	compNumNewNodes_E_kernel << <divup(cloths->_numEdges, BLOCKSIZE), BLOCKSIZE >> > (
		*((ClothParam*)cloths->_param), edgeCutWs(), numEnbCEs(), iEnewNs());
	CUDA_CHECK(cudaPeekAtLastError());
	compNumNewNodes_N_kernel << <divup(cloths->_numNodes, BLOCKSIZE), BLOCKSIZE >> > (
		*((ClothParam*)cloths->_param), numNnbCEs(), iNnewNs());
	CUDA_CHECK(cudaPeekAtLastError());

	thrust::inclusive_scan(thrust::device_ptr<uint>(iFnewNs.begin()),
		thrust::device_ptr<uint>(iFnewNs.end()), iFnewNs.begin());
	thrust::inclusive_scan(thrust::device_ptr<uint>(iEnewNs.begin()),
		thrust::device_ptr<uint>(iEnewNs.end()), iEnewNs.begin());
	thrust::inclusive_scan(thrust::device_ptr<uint>(iNnewNs.begin()),
		thrust::device_ptr<uint>(iNnewNs.end()), iNnewNs.begin());
	CUDA_CHECK(cudaMemcpy(&numFnewNs, iFnewNs() + cloths->_numFaces, sizeof(uint), cudaMemcpyDeviceToHost));
	CUDA_CHECK(cudaMemcpy(&numEnewNs, iEnewNs() + cloths->_numEdges, sizeof(uint), cudaMemcpyDeviceToHost));
	CUDA_CHECK(cudaMemcpy(&numNnewNs, iNnewNs() + cloths->_numNodes, sizeof(uint), cudaMemcpyDeviceToHost));

	compPaddingNodeIds_kernel << <divup(cloths->_numEdges, BLOCKSIZE), BLOCKSIZE >> > (
		iEnewNs(), numNnewNs, cloths->_numEdges);
	CUDA_CHECK(cudaPeekAtLastError());
	compPaddingNodeIds_kernel << <divup(cloths->_numFaces, BLOCKSIZE), BLOCKSIZE >> > (
		iFnewNs(), numNnewNs + numEnewNs, cloths->_numFaces);
	CUDA_CHECK(cudaPeekAtLastError());

	printf("%d, %d, %d\n", numNnewNs, numEnewNs, numFnewNs);
	uint newNodeSize = numNnewNs + numEnewNs + numFnewNs;
	newNodes.resize(newNodeSize * 3u);
	newN0s.resize(newNodeSize * 3u);

	compGenNewNodes_N_kernel << <divup(cloths->_numNodes, BLOCKSIZE), BLOCKSIZE >> > (
		*((ClothParam*)cloths->_param), iNnewNs(), newNodes(), newN0s());
	CUDA_CHECK(cudaPeekAtLastError());
	compGenNewNodes_E_kernel << <divup(cloths->_numEdges, BLOCKSIZE), BLOCKSIZE >> > (
		*((ClothParam*)cloths->_param), edgeCutWs(), iEnewNs(), newNodes(), newN0s());
	CUDA_CHECK(cudaPeekAtLastError());
	compGenNewNodes_F_kernel << <divup(cloths->_numFaces, BLOCKSIZE), BLOCKSIZE >> > (
		*((ClothParam*)cloths->_param), faceCutWs(), iFnewNs(), newNodes(), newN0s());
	CUDA_CHECK(cudaPeekAtLastError());
}
void TearingSolver::genNewFaces(
	Cloth* cloths, 
	Dvector<uchar>& faceCutInfos, Dvector<REAL>& faceCutWs,
	Dvector<uchar>& edgeCutInfos, Dvector<REAL>& edgeCutWs,
	Dvector<uint>& iFnewNs, Dvector<uint>& iEnewNs, Dvector<uint>& iNnewNs,
	Dvector<uint>& newFaces)
{
	Dvector<uint> inewFs(cloths->_numFaces + 1u);
	uint newFaceSize = 0u;
	compNumNewFaceInfos_kernel << <divup(cloths->_numFaces, BLOCKSIZE), BLOCKSIZE >> > (
		*((ClothParam*)cloths->_param), cloths->d_FnbEs(), faceCutInfos(), faceCutWs(), edgeCutWs(), inewFs());
	CUDA_CHECK(cudaPeekAtLastError());
	thrust::inclusive_scan(thrust::device_ptr<uint>(inewFs.begin()),
		thrust::device_ptr<uint>(inewFs.end()), inewFs.begin());
	CUDA_CHECK(cudaMemcpy(&newFaceSize, inewFs() + cloths->_numFaces, sizeof(uint), cudaMemcpyDeviceToHost));

	Dvector<ushort> newFaceInfos(newFaceSize);
	newFaces.resize(newFaceSize * 3u);

	compNewFaceInfo_kernel << <divup(cloths->_numFaces, BLOCKSIZE), BLOCKSIZE >> > (
		*((ClothParam*)cloths->_param), cloths->d_FnbEs(), faceCutInfos(), faceCutWs(), edgeCutWs(),
		inewFs(), iFnewNs(), newFaceInfos(), newFaces());
	CUDA_CHECK(cudaPeekAtLastError());

	compNewFaceMapping_E_kernel << <divup(cloths->_numEdges, BLOCKSIZE), BLOCKSIZE >> > (
		*((ClothParam*)cloths->_param), cloths->d_FnbEs(), cloths->d_EnbFs(), 
		faceCutInfos(), edgeCutInfos(), newFaceInfos(),
		iEnewNs(), inewFs(), newFaces());
	CUDA_CHECK(cudaPeekAtLastError());
	compNewFaceMapping_N_kernel << <divup(cloths->_numNodes, BLOCKSIZE), BLOCKSIZE >> > (
		*((ClothParam*)cloths->_param), cloths->d_nbEs._index(), cloths->d_nbEs._array(),
		cloths->d_FnbEs(), cloths->d_EnbFs(), faceCutInfos(), edgeCutInfos(), edgeCutWs(), newFaceInfos(),
		iNnewNs(), inewFs(), newFaces());
	CUDA_CHECK(cudaPeekAtLastError());
}
void TearingSolver::compNewNodeParams(
	Cloth* cloths, Dvector<REAL>& faceCutWs, Dvector<REAL>& edgeCutWs,
	Dvector<uint>& iFnewNs, Dvector<uint>& iEnewNs, Dvector<uint>& iNnewNs,
	Dvector<REAL>& newNodes)
{
	uint numNewNodes = newNodes.size() / 3u;
	Dvector<REAL> newMs(numNewNodes);
	Dvector<REAL> newVs(numNewNodes * 3u);
	Dvector<uint> newNodePhases(numNewNodes);
	Dvector<uchar> newIsFixeds(numNewNodes);

	cloths->d_ms = cloths->h_ms;
	initNewNodeMasses_N_kernel << <divup(cloths->_numNodes, BLOCKSIZE), BLOCKSIZE >> > (
		*((ClothParam*)cloths->_param), iNnewNs(), newMs());
	CUDA_CHECK(cudaPeekAtLastError());

	compNewNodeParams_F_kernel << <divup(cloths->_numFaces, BLOCKSIZE), BLOCKSIZE >> > (
		*((ClothParam*)cloths->_param), cloths->d_nbFs._index(), cloths->d_nbEs._index(),
		faceCutWs(), iFnewNs(), iNnewNs(), newMs(), newVs(), newNodePhases(), newIsFixeds());
	CUDA_CHECK(cudaPeekAtLastError());
	compNewNodeParams_E_kernel << <divup(cloths->_numEdges, BLOCKSIZE), BLOCKSIZE >> > (
		*((ClothParam*)cloths->_param), cloths->d_nbFs._index(), cloths->d_nbEs._index(),
		edgeCutWs(), iEnewNs(), iNnewNs(), newMs(), newVs(), newNodePhases(), newIsFixeds());
	CUDA_CHECK(cudaPeekAtLastError());
	compNewNodeParams_N_kernel << <divup(cloths->_numNodes, BLOCKSIZE), BLOCKSIZE >> > (
		*((ClothParam*)cloths->_param), iNnewNs(), newMs(), newVs(), newNodePhases(), newIsFixeds());
	CUDA_CHECK(cudaPeekAtLastError());

	newMs.copyToHost(cloths->h_ms);
	newVs.copyToHost(cloths->h_vs);
	newVs.copyToDevice(cloths->d_vs);
	newNodePhases.copyToHost(cloths->h_nodePhases);
	newNodePhases.copyToDevice(cloths->d_nodePhases);
	newIsFixeds.copyToHost(cloths->h_isFixeds);
	newIsFixeds.copyToDevice(cloths->d_isFixeds);

	cloths->d_mfs.resize(numNewNodes);
	cloths->d_mfs.memset(0);
	cloths->d_mfs.copyToHost(cloths->h_mfs);
}
void TearingSolver::buildNewAdjacency(
	Cloth* cloths, Dvector<uint>& newFaces, uint numNewNodes, Dvector<uint>& newEdges)
{
	uint numNewFaces = newFaces.size() / 3u;
	uint numNewEdges = 0u;
	Dvector<uint> inewEs(numNewFaces + 1u);

	DPrefixArray<uint> nbFs;
	MeshKernel::buildNbFs(newFaces, nbFs, numNewFaces, numNewNodes);

	getNewEdgeId_kernel << <divup(numNewFaces, BLOCKSIZE), BLOCKSIZE >> > (
		newFaces(), nbFs._index(), nbFs._array(), numNewFaces, inewEs());
	CUDA_CHECK(cudaPeekAtLastError());

	thrust::inclusive_scan(thrust::device_ptr<uint>(inewEs.begin()),
		thrust::device_ptr<uint>(inewEs.end()), inewEs.begin());
	CUDA_CHECK(cudaMemcpy(&numNewEdges, inewEs() + numNewFaces, sizeof(uint), cudaMemcpyDeviceToHost));

	newEdges.resize(numNewEdges << 1u);
	getNewEdge_kernel << <divup(numNewFaces, BLOCKSIZE), BLOCKSIZE >> > (
		newFaces(), nbFs._index(), nbFs._array(), numNewFaces, inewEs(), newEdges());
	CUDA_CHECK(cudaPeekAtLastError());
	thrust::sort(thrust::device_ptr<uint2>((uint2*)newEdges()),
		thrust::device_ptr<uint2>(((uint2*)newEdges()) + numNewEdges), uint2_CMP());
}

#define CUTTEST 1
void TearingSolver::compCutting(Cloth* cloths, Dvector<CutPlane>& cutPlanes)
{
	CUDA_CHECK(cudaDeviceSynchronize());
	ctimer timer = CNOW;

	Dvector<uchar> faceCutInfos(cloths->_numFaces);
	Dvector<uchar> edgeCutInfos(cloths->_numEdges);
	Dvector<REAL> faceCutWs(cloths->_numFaces << 1u);
	Dvector<REAL> edgeCutWs(cloths->_numEdges);
	if (getElementsCutInfo(cloths, cutPlanes, faceCutInfos, faceCutWs, edgeCutInfos, edgeCutWs)) {
		Dvector<uint> iFnewNs(cloths->_numFaces + 1u);
		Dvector<uint> iEnewNs(cloths->_numEdges + 1u);
		Dvector<uint> iNnewNs(cloths->_numNodes + 1u);
		Dvector<REAL> newNs;
		Dvector<REAL> newN0s;
		Dvector<uint> newFs;
		Dvector<uint> newEs;
		genNewNodes(cloths, 
			faceCutInfos, faceCutWs, 
			edgeCutInfos, edgeCutWs, 
			iFnewNs, iEnewNs, iNnewNs, newNs, newN0s);

		if (newNs.size() != cloths->_numNodes * 3u) {
			genNewFaces(cloths,
				faceCutInfos, faceCutWs,
				edgeCutInfos, edgeCutWs,
				iFnewNs, iEnewNs, iNnewNs, newFs);
			faceCutInfos.clear();
			edgeCutInfos.clear();

			buildNewAdjacency(cloths, newFs, newN0s.size() / 3u, newEs);
			compNewNodeParams(cloths, faceCutWs, edgeCutWs, iFnewNs, iEnewNs, iNnewNs, newNs);

			cloths->_numFaces = newFs.size() / 3u;
			cloths->_numEdges = newEs.size() >> 1u;
			cloths->_numNodes = newNs.size() / 3u;
			printf("%d, %d, %d\n", cloths->_numFaces, cloths->_numEdges, cloths->_numNodes);

			newFs.copyToHost(cloths->h_fs);
			newEs.copyToHost(cloths->h_es);
			newNs.copyToHost(cloths->h_ns);
			newN0s.copyToHost(cloths->h_restNs);
			newFs.copyToDevice(cloths->d_fs);
			newEs.copyToDevice(cloths->d_es);
			newNs.copyToDevice(cloths->d_ns);
			newN0s.copyToDevice(cloths->d_restNs);

			cloths->updateElements();
			cloths->setParam();

			compFixedPosition_kernel << <divup(cloths->_numNodes, BLOCKSIZE), BLOCKSIZE >> > (
				*((ClothParam*)cloths->_param), cloths->d_nbFs._index(), cloths->d_nbFs._array());
			CUDA_CHECK(cudaPeekAtLastError());
			cloths->d_ns.copyToHost(cloths->h_ns);
		}
	}

	CUDA_CHECK(cudaDeviceSynchronize());
	printf("Tearing %f msec\n", (CNOW - timer) / 10000.0);
}