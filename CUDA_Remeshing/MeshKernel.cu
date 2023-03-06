#include "MeshKernel.cuh"

void MeshKernel::buildNbFs(
	const Dvector<uint>& fs, DPrefixArray<uint>& nbFs, uint numFaces, uint numNodes)
{
	if (numFaces) {
		Dvector<uint2> buffers(numFaces * 3u);
		nbFs._array.resize(numFaces * 3u);
		nbFs._index.resize(numNodes + 1u);

		getNbFsBuffer_kernel << <divup(numFaces, MAX_BLOCKSIZE), MAX_BLOCKSIZE >> > (
			fs(), buffers(), numFaces);
		CUDA_CHECK(cudaPeekAtLastError());

		thrust::sort(thrust::device_ptr<uint2>((uint2*)buffers.begin()),
			thrust::device_ptr<uint2>((uint2*)buffers.end()), uint2_CMP());

		reorderIdsUint2_kernel << < divup(buffers.size(), MAX_BLOCKSIZE), MAX_BLOCKSIZE, (MAX_BLOCKSIZE + 1u) * sizeof(uint) >> > (
			buffers(), nbFs._index(), buffers.size(), nbFs._index.size());
		CUDA_CHECK(cudaPeekAtLastError());
		initNbs_kernel << < divup(buffers.size(), MAX_BLOCKSIZE), MAX_BLOCKSIZE >> > (
			nbFs._array(), (uint*)buffers(), buffers.size());
		CUDA_CHECK(cudaPeekAtLastError());
	}
}
void MeshKernel::buildNbENs(
	const Dvector<uint>& es, DPrefixArray<uint>& nbEs, DPrefixArray<uint>& nbNs, uint numEdges, uint numNodes)
{
	if (numEdges) {
		Dvector<uint2> ebuffers(numEdges << 1u);
		Dvector<uint2> nbuffers(numEdges << 1u);
		nbEs._array.resize(numEdges << 1u);
		nbNs._array.resize(numEdges << 1u);
		nbEs._index.resize(numNodes + 1u);
		nbNs._index.resize(numNodes + 1u);

		getNbENsBuffer_kernel << <divup(numEdges, MAX_BLOCKSIZE), MAX_BLOCKSIZE >> > (
			es(), ebuffers(), nbuffers(), numEdges);
		CUDA_CHECK(cudaPeekAtLastError());

		thrust::sort(thrust::device_ptr<uint2>((uint2*)ebuffers.begin()),
			thrust::device_ptr<uint2>((uint2*)ebuffers.end()), uint2_CMP());
		thrust::sort(thrust::device_ptr<uint2>((uint2*)nbuffers.begin()),
			thrust::device_ptr<uint2>((uint2*)nbuffers.end()), uint2_CMP());

		reorderIdsUint2_kernel << < divup(ebuffers.size(), MAX_BLOCKSIZE), MAX_BLOCKSIZE, (MAX_BLOCKSIZE + 1u) * sizeof(uint) >> > (
			ebuffers(), nbEs._index(), ebuffers.size(), nbEs._index.size());
		CUDA_CHECK(cudaPeekAtLastError());
		reorderIdsUint2_kernel << < divup(nbuffers.size(), MAX_BLOCKSIZE), MAX_BLOCKSIZE, (MAX_BLOCKSIZE + 1u) * sizeof(uint) >> > (
			nbuffers(), nbNs._index(), nbuffers.size(), nbNs._index.size());
		CUDA_CHECK(cudaPeekAtLastError());
		initNbs_kernel << < divup(ebuffers.size(), MAX_BLOCKSIZE), MAX_BLOCKSIZE >> > (
			nbEs._array(), (uint*)ebuffers(), ebuffers.size());
		CUDA_CHECK(cudaPeekAtLastError());
		initNbs_kernel << < divup(nbuffers.size(), MAX_BLOCKSIZE), MAX_BLOCKSIZE >> > (
			nbNs._array(), (uint*)nbuffers(), nbuffers.size());
		CUDA_CHECK(cudaPeekAtLastError());
	}
}
void MeshKernel::buildEnbXs(
	const Dvector<uint>& fs, const Dvector<uint>& es, const DPrefixArray<uint>& nbFs,
	Dvector<uint>& EnbFs, Dvector<uint>& EnbNs, uint numEdges)
{
	EnbFs.resize(numEdges << 1u);
	EnbNs.resize(numEdges << 1u);
	buildEnbXs_kernel << <divup(numEdges, MAX_BLOCKSIZE), MAX_BLOCKSIZE >> > (
		fs(), es(), nbFs._index(), nbFs._array(), EnbFs(), EnbNs(), numEdges);
	CUDA_CHECK(cudaPeekAtLastError());
}
void MeshKernel::buildFnbEs(
	const Dvector<uint>& fs, const DPrefixArray<uint>& nbEs, Dvector<uint>& FnbEs, uint numFaces)
{
	FnbEs.resize(numFaces * 3u);
	buildFnbEs_kernel << <divup(numFaces, MAX_BLOCKSIZE), MAX_BLOCKSIZE >> > (
		fs(), nbEs._index(), nbEs._array(), FnbEs(), numFaces);
	CUDA_CHECK(cudaPeekAtLastError());
}
void MeshKernel::buildXnbXs(
	const Dvector<uint>& fs, const Dvector<uint>& es,
	DPrefixArray<uint>& nbFs, DPrefixArray<uint>& nbEs, DPrefixArray<uint>& nbNs,
	Dvector<uint>& EnbFs, Dvector<uint>& EnbNs, Dvector<uint>& FnbEs,
	uint numFaces, uint numEdges, uint numNodes)
{
	buildNbFs(fs, nbFs, numFaces, numNodes);
	buildNbENs(es, nbEs, nbNs, numEdges, numNodes);
	buildEnbXs(fs, es, nbFs, EnbFs, EnbNs, numEdges);
	buildFnbEs(fs, nbEs, FnbEs, numFaces);
}

void MeshKernel::computeNormal(
	const Dvector<uint>& fs, const Dvector<REAL>& ns, Dvector<REAL>& fNorms, Dvector<REAL>& nNorms,
	uint numFaces, uint numNodes)
{
	if (!numFaces)
		return;

	nNorms.memset(0);
	compNormals_kernel << <divup(numFaces, BLOCKSIZE), BLOCKSIZE >> > (
		fs(), ns(), fNorms(), nNorms(), numFaces);
	CUDA_CHECK(cudaPeekAtLastError());

	nodeNormNormalize_kernel << <divup(numNodes, MAX_BLOCKSIZE), MAX_BLOCKSIZE >> > (
		nNorms(), numNodes);
	CUDA_CHECK(cudaPeekAtLastError());
}


void MeshKernel::initEdgeConstraints(
	const Dvector<uint>& es, const Dvector<REAL>& ns, Dvector<REAL>& restLengths, uint numEdges)
{
	if (numEdges) {
		restLengths.resize(numEdges);
		initEdgeConstraints_kernel << <divup(numEdges, BLOCKSIZE), BLOCKSIZE >> > (
			es(), ns(), restLengths(), numEdges);
		CUDA_CHECK(cudaPeekAtLastError());
	}
}
void MeshKernel::initTriangleConstraints(
	const Dvector<uint>& fs, const Dvector<REAL>& ns, Dvector<REAL>& restAreas, Dvector<REAL>& invDs, uint numFaces) 
{
	if (numFaces) {
		restAreas.resize(numFaces);
		invDs.resize(numFaces << 2u);
		initTriangleConstraints_kernel << <divup(numFaces, BLOCKSIZE), BLOCKSIZE >> > (
			fs(), ns(), restAreas(), invDs(), numFaces);
		CUDA_CHECK(cudaPeekAtLastError());
	}
}
void MeshKernel::initDihedralConstraints(
	const Dvector<uint>& fs, const Dvector<uint>& es, const Dvector<REAL>& ns, const Dvector<uint>& EnbNs,
	Dvector<REAL>& restAngles, uint numEdges)
{
	if (numEdges) {
		restAngles.resize(numEdges);
		initDihedralConstraints_kernel << <divup(numEdges, BLOCKSIZE), BLOCKSIZE >> > (
			fs(), es(), ns(), EnbNs(), restAngles(), numEdges);
		CUDA_CHECK(cudaPeekAtLastError());
	}
}