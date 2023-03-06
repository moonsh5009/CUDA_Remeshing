#include "ProjectiveDynamics.cuh"

void ProjectiveDynamics::initProject(Cloth* cloths) {
	initProject_kernel << <divup(cloths->_numNodes, MAX_BLOCKSIZE), MAX_BLOCKSIZE >> > (
		*cloths->_param, *cloths->_pdParam);
	CUDA_CHECK(cudaPeekAtLastError());
}
void ProjectiveDynamics::compErrorProject(Cloth* cloths) {
#if CLOTH_STRAIN==0
	compEdgeErrorProject_kernel << <divup(cloths->_edgeConstraints->_numConstraints, BLOCKSIZE), BLOCKSIZE >> > (
		*cloths->_param, *cloths->_pdParam, *cloths->_edgeConstraints);
	CUDA_CHECK(cudaPeekAtLastError());
#else
	compTriangleErrorProject_kernel << <divup(cloths->_triangleConstraints->_numConstraints, BLOCKSIZE), BLOCKSIZE >> > (
		*cloths->_param, *cloths->_pdParam, *cloths->_triangleConstraints);
	CUDA_CHECK(cudaPeekAtLastError());
#endif
#if CLOTH_BENDING==0
	compEdgeErrorProject_kernel << <divup(cloths->_edgeBendingConstraints->_numConstraints, BLOCKSIZE), BLOCKSIZE >> > (
		*cloths->_param, *cloths->_pdParam, *cloths->_edgeBendingConstraints);
	CUDA_CHECK(cudaPeekAtLastError());
#else
	compDihedralErrorProject_kernel << <divup(cloths->_dihedralBendingConstraints->_numConstraints, BLOCKSIZE), BLOCKSIZE >> > (
		*cloths->_param, *cloths->_pdParam, *cloths->_dihedralBendingConstraints);
	CUDA_CHECK(cudaPeekAtLastError());
#endif

}
void ProjectiveDynamics::updateXsProject(
	Cloth* cloths, REAL* maxError)
{
	REAL* d_maxError;
	CUDA_CHECK(cudaMalloc((void**)&d_maxError, sizeof(REAL)));
	CUDA_CHECK(cudaMemset(d_maxError, 0, sizeof(REAL)));

	updateXsProject_kernel << <divup(cloths->_numNodes, BLOCKSIZE), BLOCKSIZE, BLOCKSIZE * sizeof(REAL) >> > (
		*cloths->_param, *cloths->_pdParam, d_maxError);
	CUDA_CHECK(cudaPeekAtLastError());

	CUDA_CHECK(cudaMemcpy(maxError, d_maxError, sizeof(REAL), cudaMemcpyDeviceToHost));
	CUDA_CHECK(cudaFree(d_maxError));
}