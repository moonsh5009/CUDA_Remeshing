#include "Simulation.cuh"

void Simulation::initMasses(Cloth* cloth, Obstacle* obstacle) {
	Dvector<REAL> ms;
	ms = cloth->h_ms;

	initClothMasses_kernel << <divup(cloth->_numNodes, MAX_BLOCKSIZE), MAX_BLOCKSIZE >> > (
		*((ClothParam*)cloth->_param), ms(), cloth->d_isFixeds());
	CUDA_CHECK(cudaPeekAtLastError());

	ms = obstacle->h_ms;
	initMasses_kernel << <divup(obstacle->_numNodes, MAX_BLOCKSIZE), MAX_BLOCKSIZE >> > (
		*obstacle->_param, ms(), obstacle->d_isFixeds());
	CUDA_CHECK(cudaPeekAtLastError());
}
void Simulation::compGravityForce(MeshObject* obj, const REAL3& gravity) {
	compGravityForce_kernel << <divup(obj->_numNodes, MAX_BLOCKSIZE), MAX_BLOCKSIZE >> > (
		obj->d_forces(), obj->d_ms(), gravity, obj->_numNodes);
	CUDA_CHECK(cudaPeekAtLastError());
}
void Simulation::compRotationForce(Obstacle* obj, const REAL dt) {
	compRotationForce_kernel << <divup(obj->_numNodes, MAX_BLOCKSIZE), MAX_BLOCKSIZE >> > (
		obj->d_ns(), obj->d_vs(), obj->d_forces(), obj->d_ms(), obj->d_nodePhases(),
		obj->d_pivots(), obj->d_degrees(), 1.0 / dt, obj->_numNodes);
	CUDA_CHECK(cudaPeekAtLastError());
}

void Simulation::applyForce(MeshObject* obj, const REAL dt) {
	applyForce_kernel << <divup(obj->_numNodes, MAX_BLOCKSIZE), MAX_BLOCKSIZE >> > (
		obj->d_vs(), obj->d_forces(), obj->d_invMs(), obj->d_isFixeds(), dt, obj->_numNodes);
	CUDA_CHECK(cudaPeekAtLastError());
}
void Simulation::updateVelocity(Dvector<REAL>& n0s, Dvector<REAL>& n1s, Dvector<REAL>& vs, const REAL invdt) {
	uint numNodes = n0s.size() / 3u;
	updateVelocity_kernel << <divup(numNodes, MAX_BLOCKSIZE), MAX_BLOCKSIZE >> > (
		n0s(), n1s(), vs(), invdt, numNodes);
	CUDA_CHECK(cudaPeekAtLastError());
}
void Simulation::updatePosition(Dvector<REAL>& ns, Dvector<REAL>& vs, const REAL dt) {
	uint numNodes = ns.size() / 3u;
	updatePosition_kernel << <divup(numNodes, MAX_BLOCKSIZE), MAX_BLOCKSIZE >> > (
		ns(), vs(), dt, numNodes);
	CUDA_CHECK(cudaPeekAtLastError());
}

void  Simulation::Damping(Dvector<REAL>& vs, REAL w) {
	uint numNodes = vs.size() / 3u;
	Damping_kernel << <divup(numNodes, MAX_BLOCKSIZE), MAX_BLOCKSIZE >> > (
		vs(), w, numNodes);
	CUDA_CHECK(cudaPeekAtLastError());
}
void  Simulation::Damping(Dvector<REAL>& vs, Dvector<uchar>& isFixeds, REAL w) {
	uint numNodes = vs.size() / 3u;
	Damping_kernel << <divup(numNodes, MAX_BLOCKSIZE), MAX_BLOCKSIZE >> > (
		vs(), isFixeds(), w, numNodes);
	CUDA_CHECK(cudaPeekAtLastError());
}
void Simulation::rayCasting(Cloth* cloths, const M_Ray& ray, float radius, float zFar) {
	uint clickNum = 1u;
	cloths->h_clickNodes.resize(clickNum);

	Dvector<uint2> d_infos(cloths->_numNodes);
	rayCasing_kernel << <divup(cloths->_numNodes, MAX_BLOCKSIZE), MAX_BLOCKSIZE >> > (
		*cloths->_param, ray, radius, zFar, d_infos());
	CUDA_CHECK(cudaPeekAtLastError());

	thrust::sort(thrust::device_ptr<uint2>(d_infos.begin()),
		thrust::device_ptr<uint2>(d_infos.end()), uint2_CMP());

	vector<uint2> h_info;
	float distance = 0.f;
	uint num = 0u;
	d_infos.copyToHost(h_info);
	for (int i = 0; i < clickNum; i++) {
		if (h_info[i].x != 0xffffffff) {
			uint id = h_info[i].y;
			float3 p = make_float3(
				cloths->h_ns[id * 3u + 0u], cloths->h_ns[id * 3u + 1u], cloths->h_ns[id * 3u + 2u]);
			float3 diff = p - ray._pos;
			distance += Dot(diff, ray._dir);
			num++;
		}
	}
	if (num > 0) {
		distance /= (float)num;
		for (int i = 0; i < clickNum; i++) {
			if (h_info[i].x != 0xffffffff) {
				uint id = h_info[i].y;
				float3 p = make_float3(
					cloths->h_ns[id * 3u + 0u], cloths->h_ns[id * 3u + 1u], cloths->h_ns[id * 3u + 2u]);
				float3 pos = ray._pos + distance * ray._dir;

				//printf("CPU %d %d\n", h_info[i].y, h_info[i].x);
				cloths->h_clickNodes[i]._dist0 = distance;
				cloths->h_clickNodes[i]._offset = p - pos;
				cloths->h_clickNodes[i]._id = id;
			}
			else {
				cloths->h_clickNodes[i]._id = 0xffffffff;
			}
		}
	}
}