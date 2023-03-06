#include "CollisionDetection.cuh"
#include "CollisionResponse.cuh"
#include "CollisionSolver.h"

//-------------------------------------------------------------------------
void CollisionSolver::getSelfLastBvtts(
	BVHParam& clothBvh,
	Dvector<uint2>& lastBvtts, Dvector<uint>& LastBvttIds,
	uint lastBvhSize, uint& lastBvttSize)
{
	getNumLastBvtts_kernel << <divup(lastBvhSize, BLOCKSIZE), BLOCKSIZE >> > (
		clothBvh, LastBvttIds(), lastBvhSize);
	CUDA_CHECK(cudaPeekAtLastError());

	thrust::inclusive_scan(thrust::device_ptr<uint>(LastBvttIds.begin()), 
		thrust::device_ptr<uint>(LastBvttIds.begin() + lastBvhSize + 1u),
		thrust::device_ptr<uint>(LastBvttIds.begin()));
	CUDA_CHECK(cudaMemcpy(&lastBvttSize, LastBvttIds() + lastBvhSize, sizeof(uint), cudaMemcpyDeviceToHost));

	if (lastBvtts.size() < lastBvttSize)
		lastBvtts.resize(lastBvttSize);
	getLastBvtts_kernel << <divup(lastBvhSize, BLOCKSIZE), BLOCKSIZE >> > (
		clothBvh, lastBvtts(), LastBvttIds(), lastBvhSize);
	CUDA_CHECK(cudaPeekAtLastError());
}
void CollisionSolver::getObstacleLastBvtts(
	BVHParam& clothBvh, BVHParam& obsBvh,
	Dvector<uint2>& lastBvtts, Dvector<uint>& LastBvttIds,
	uint lastBvhSize, uint& lastBvttSize)
{
	getNumLastBvtts_kernel << <divup(lastBvhSize, BLOCKSIZE), BLOCKSIZE >> > (
		clothBvh, obsBvh, LastBvttIds(), lastBvhSize);
	CUDA_CHECK(cudaPeekAtLastError());

	thrust::inclusive_scan(thrust::device_ptr<uint>(LastBvttIds.begin()),
		thrust::device_ptr<uint>(LastBvttIds.begin() + lastBvhSize + 1u),
		thrust::device_ptr<uint>(LastBvttIds.begin()));
	CUDA_CHECK(cudaMemcpy(&lastBvttSize, LastBvttIds() + lastBvhSize, sizeof(uint), cudaMemcpyDeviceToHost));

	if (lastBvtts.size() < lastBvttSize)
		lastBvtts.resize(lastBvttSize);
	getLastBvtts_kernel << <divup(lastBvhSize, BLOCKSIZE), BLOCKSIZE >> > (
		clothBvh, obsBvh, lastBvtts(), LastBvttIds(), lastBvhSize);
	CUDA_CHECK(cudaPeekAtLastError());
}
//-------------------------------------------------------------------------
void CollisionSolver::getContactElements(
	ContactElems& ceParam, Cloth* cloths, Obstacle* obstacles)
{
	cloths->_bvh->refitProximity(cloths->d_fs(), cloths->d_ns(), cloths->d_nodePhases(), cloths->d_thicknesses());
	obstacles->_bvh->refitProximity(obstacles->d_fs(), obstacles->d_ns(), obstacles->d_nodePhases(), obstacles->d_thicknesses());

	ceParam._lastBvhSize = 1u << cloths->_bvh->_maxLevel - 1u;
	ceParam._lastBvttIds.resize(ceParam._lastBvhSize + 1u);

	ceParam._size = 0u;
	//ceParam.resize(0u);
	ceParam.d_tmp.memset(0);

	Dvector<uint2> selfLastBvtts;
	Dvector<uint2> obsLastBvtts;
	uint selfLastBvttSize, obsLastBvttSize;
	getSelfLastBvtts(
		cloths->_bvh->_param, selfLastBvtts, ceParam._lastBvttIds, ceParam._lastBvhSize, selfLastBvttSize);
	getObstacleLastBvtts(
		cloths->_bvh->_param, obstacles->_bvh->_param, obsLastBvtts, ceParam._lastBvttIds, ceParam._lastBvhSize, obsLastBvttSize);

	getNumSelfContactElements_LastBvtt_kernel << <divup(selfLastBvttSize, BLOCKSIZE), BLOCKSIZE, BLOCKSIZE * sizeof(uint) >> > (
		*cloths->_param, cloths->_bvh->_param, cloths->_RTri->_param,
		selfLastBvtts(), selfLastBvttSize, ceParam.d_tmp());
	CUDA_CHECK(cudaPeekAtLastError());
	getNumObstacleContactElements_LastBvtt_kernel << <divup(obsLastBvttSize, BLOCKSIZE), BLOCKSIZE, BLOCKSIZE * sizeof(uint) >> > (
		*cloths->_param, cloths->_bvh->_param, cloths->_RTri->_param,
		*obstacles->_param, obstacles->_bvh->_param, obstacles->_RTri->_param,
		obsLastBvtts(), obsLastBvttSize, ceParam.d_tmp());
	CUDA_CHECK(cudaPeekAtLastError());

	CUDA_CHECK(cudaMemcpy(&ceParam._size, ceParam.d_tmp(), sizeof(uint), cudaMemcpyDeviceToHost));
	if (ceParam._size > 0u) {
		if (ceParam._elems.size() < ceParam._size)
			ceParam.resize();
		ceParam.d_tmp.memset(0);
		getSelfContactElements_LastBvtt_kernel << <divup(selfLastBvttSize, BLOCKSIZE), BLOCKSIZE >> > (
			*cloths->_param, cloths->_bvh->_param, cloths->_RTri->_param,
			selfLastBvtts(), selfLastBvttSize, ceParam.param());
		CUDA_CHECK(cudaPeekAtLastError());
		getObstacleContactElements_LastBvtt_kernel << <divup(obsLastBvttSize, BLOCKSIZE), BLOCKSIZE >> > (
			*cloths->_param, cloths->_bvh->_param, cloths->_RTri->_param,
			*obstacles->_param, obstacles->_bvh->_param, obstacles->_RTri->_param,
			obsLastBvtts(), obsLastBvttSize, ceParam.param());
		CUDA_CHECK(cudaPeekAtLastError());
		CUDA_CHECK(cudaMemcpy(&ceParam._size, ceParam.d_tmp(), sizeof(uint), cudaMemcpyDeviceToHost));

		thrust::sort(thrust::device_ptr<ContactElem>((ContactElem*)ceParam._elems()),
			thrust::device_ptr<ContactElem>(((ContactElem*)ceParam._elems()) + ceParam._size),
			ContactElem_CMP());
	}
	CUDA_CHECK(cudaDeviceSynchronize());
	printf("Collision %d\n", ceParam._size);
}
void CollisionSolver::getClothCCDtime(
	Cloth* cloths, Obstacle* obstacles,
	const REAL dt, REAL* minTime)
{
#ifdef COLLISION_TESTTIMER
	CUDA_CHECK(cudaDeviceSynchronize());
	ctimer timer = CNOW;
#endif
	* minTime = 10.0;

	REAL* d_minTime;
	CUDA_CHECK(cudaMalloc((void**)&d_minTime, sizeof(REAL)));
	CUDA_CHECK(cudaMemcpy(d_minTime, minTime, sizeof(REAL), cudaMemcpyHostToDevice));

	cloths->_bvh->refitCCD(cloths->d_fs(), cloths->d_ns(), cloths->d_vs(), cloths->d_nodePhases(), cloths->d_thicknesses(), dt);
	obstacles->_bvh->refitCCD(obstacles->d_fs(), obstacles->d_ns(), obstacles->d_vs(), obstacles->d_nodePhases(), obstacles->d_thicknesses(), dt);

	getSelfCCDtime_kernel << <divup(cloths->_numFaces, BLOCKSIZE), BLOCKSIZE >> > (
		*cloths->_param, cloths->_bvh->_param, cloths->_RTri->_param,
		dt, d_minTime);
	CUDA_CHECK(cudaPeekAtLastError());
	getObstacleCCDtime_kernel << <divup(cloths->_numFaces, BLOCKSIZE), BLOCKSIZE >> > (
		*cloths->_param, cloths->_bvh->_param, cloths->_RTri->_param,
		*obstacles->_param, obstacles->_bvh->_param, obstacles->_RTri->_param,
		dt, d_minTime);
	CUDA_CHECK(cudaPeekAtLastError());

	CUDA_CHECK(cudaMemcpy(minTime, d_minTime, sizeof(REAL), cudaMemcpyDeviceToHost));
	CUDA_CHECK(cudaFree(d_minTime));

#ifdef COLLISION_TESTTIMER
	CUDA_CHECK(cudaDeviceSynchronize());
	printf("CollisionSolver::getClothCCDtime: %lf msec\n", (CNOW - timer) / 10000.0);
#endif
}
void CollisionSolver::compEpsilonCollision(
	Cloth* cloths, Obstacle* obstacles,
	const REAL dt)
{
#ifdef COLLISION_TESTTIMER
	CUDA_CHECK(cudaDeviceSynchronize());
	ctimer timer = CNOW;
#endif

	Dvector<bool> clothIsDetecteds(cloths->_numNodes);
	//Dvector<bool> obsIsDetecteds(obstacles->_numNodes);
	clothIsDetecteds.memset(0);
	//obsIsDetecteds.memset(0);

	cloths->_bvh->refitCCD(cloths->d_fs(), cloths->d_ns(), cloths->d_vs(), cloths->d_nodePhases(), cloths->d_thicknesses(), dt);
	//obstacles->_bvh->refitCCD(obstacles->d_fs(), obstacles->d_ns(), obstacles->d_vs(), obstacles->d_nodePhases(), obstacles->d_thicknesses(), dt);

	compSelfCollisionEpsilon_kernel << <divup(cloths->_numFaces, BLOCKSIZE), BLOCKSIZE >> > (
		*cloths->_param, cloths->_bvh->_param, cloths->_RTri->_param,
		clothIsDetecteds(), dt);
	CUDA_CHECK(cudaPeekAtLastError());
	/*compObstacleCollisionEpsilon_kernel << <divup(cloths->_numFaces, BLOCKSIZE), BLOCKSIZE >> > (
		*cloths->_param, cloths->_bvh->_param, cloths->_RTri->_param,
		*obstacles->_param, obstacles->_bvh->_param, obstacles->_RTri->_param,
		obsIsDetecteds(), dt);
	CUDA_CHECK(cudaPeekAtLastError());*/
	applyCollisionEpsilon_kernel << <divup(cloths->_numNodes, BLOCKSIZE), BLOCKSIZE >> > (
		*cloths->_param, clothIsDetecteds());
	CUDA_CHECK(cudaPeekAtLastError());
	/*applyCollisionEpsilon_kernel << <divup(obstacles->_numNodes, BLOCKSIZE), BLOCKSIZE >> > (
		*obstacles->_param, obsIsDetecteds());
	CUDA_CHECK(cudaPeekAtLastError());*/

#ifdef COLLISION_TESTTIMER
	CUDA_CHECK(cudaDeviceSynchronize());
	printf("CollisionSolver::compEpsilonCollision: %lf msec\n", (CNOW - timer) / 10000.0);
#endif
}
//-------------------------------------------------------------------------
void CollisionSolver::MakeRigidImpactZone(
	const ContactElems& d_ceParam,
	RIZone& h_riz, DRIZone& d_riz,
	const PrefixArray<uint>& clothNbNs, const PrefixArray<uint>& obsNbNs)
{
	vector<ContactElem> h_ceParam(d_ceParam._size);
	CUDA_CHECK(cudaMemcpy(&h_ceParam[0], d_ceParam._elems(), d_ceParam._size * sizeof(ContactElem), cudaMemcpyDeviceToHost));

	uint2 nodes[4];
	for (uint ice = 0; ice < h_ceParam.size(); ice++) {
		if (!h_ceParam[ice]._isCCD)
			continue;

		nodes[0].x = h_ceParam[ice]._i[0];
		nodes[1].x = h_ceParam[ice]._i[1];
		nodes[2].x = h_ceParam[ice]._i[2];
		nodes[3].x = h_ceParam[ice]._i[3];
		nodes[0].y = h_ceParam[ice]._type[0];
		nodes[1].y = h_ceParam[ice]._type[1];
		nodes[2].y = h_ceParam[ice]._type[2];
		nodes[3].y = h_ceParam[ice]._type[3];

		set<uint> ind_inc;
		for (uint i = 0; i < 4; i++) {
			uint2 ino = nodes[i];
			for (uint iriz = 0; iriz < h_riz.size(); iriz++) {
				if (h_riz[iriz].find(ino) != h_riz[iriz].end())
					ind_inc.insert(iriz);
				else {
					if (ino.y == TYPE_MESH_CLOTH) {
						for (uint j = clothNbNs._index[ino.x]; j < clothNbNs._index[ino.x + 1u]; j++) {
							uint2 jno = make_uint2(clothNbNs._array[j], 0u);
							if (h_riz[iriz].find(jno) != h_riz[iriz].end()) {
								ind_inc.insert(iriz);
								break;
							}
						}
					}
					else if (ino.y == TYPE_MESH_OBSTACLE) {
						for (uint j = obsNbNs._index[ino.x]; j < obsNbNs._index[ino.x + 1u]; j++) {
							uint2 jno = make_uint2(obsNbNs._array[j], 1u);
							if (h_riz[iriz].find(jno) != h_riz[iriz].end()) {
								ind_inc.insert(iriz);
								break;
							}
						}
					}
				}
			}
		}
		uint ind0;
		if (ind_inc.size() == 0) {
			ind0 = (uint)h_riz.size();
			h_riz.resize(ind0 + 1u);
		}
		else if (ind_inc.size() == 1u)
			ind0 = *(ind_inc.begin());
		else {
			RIZone h_riz1;
			for (uint iriz = 0; iriz < h_riz.size(); iriz++) {
				if (ind_inc.find(iriz) != ind_inc.end()) continue;
				h_riz1.push_back(h_riz[iriz]);
			}
			ind0 = (uint)h_riz1.size();
			h_riz1.resize(ind0 + 1);
			for (auto itr = ind_inc.begin(); itr != ind_inc.end(); itr++) {
				uint ind1 = *itr;
				for (auto jtr = h_riz[ind1].begin(); jtr != h_riz[ind1].end(); jtr++)
					h_riz1[ind0].insert(*jtr);
			}
			h_riz = h_riz1;
		}

		for (uint i = 0; i < 4; i++)
			h_riz[ind0].insert(nodes[i]);
	}

	vector<uint2> h_ids;
	vector<uint> h_zones;
	h_zones.resize(h_riz.size() + 1u);
	h_zones[0] = 0;
	for (uint i = 0; i < h_riz.size(); i++)
		h_zones[i + 1] = h_zones[i] + h_riz[i].size();

	h_ids.resize(h_zones.back());
	uint n = 0;
	for (uint i = 0; i < h_riz.size(); i++) {
		for (auto jtr = h_riz[i].begin(); jtr != h_riz[i].end(); jtr++)
			h_ids[n++] = *jtr;
	}

	d_riz._ids = h_ids;
	d_riz._zones = h_zones;
}
bool CollisionSolver::ResolveRigidImpactZone(
	ContactElems& ceParam,
	RIZone& h_riz, DRIZone& d_riz,
	const ObjParam& clothParam, const ObjParam& obsParam,
	const PrefixArray<uint>& clothNbNs, const PrefixArray<uint>& obsNbNs,
	const REAL dt)
{
#ifdef COLLISION_TESTTIMER
	CUDA_CHECK(cudaDeviceSynchronize());
	ctimer timer = CNOW;
#endif
	CUDA_CHECK(cudaDeviceSynchronize());
	ctimer timer = CNOW;
	bool result = false;
	if (ceParam._size) {
		bool* d_applied;

		CUDA_CHECK(cudaMalloc((void**)&d_applied, sizeof(bool)));
		CUDA_CHECK(cudaMemset(d_applied, 0, sizeof(bool)));

		compDetectedRigidImpactZone_CE_kernel << <divup(ceParam._size, BLOCKSIZE), BLOCKSIZE >> > (
			ceParam.param(), clothParam, obsParam, dt, d_applied);
		CUDA_CHECK(cudaPeekAtLastError());

		CUDA_CHECK(cudaMemcpy(&result, d_applied, sizeof(bool), cudaMemcpyDeviceToHost));
		CUDA_CHECK(cudaFree(d_applied));

		if (result) {
			h_riz.clear();
			MakeRigidImpactZone(ceParam, h_riz, d_riz, clothNbNs, obsNbNs);

			/*printf("\n");
			for (int i = 0; i < h_riz.size(); i++) {
				for (auto k : h_riz[i]) {
					printf("(%d, %d)", k.x, k.y);
				}
				printf("\n");
			}
			printf("\n");*/

			ApplyRigidImpactZone_kernel << <divup(d_riz._zones.size() - 1u, BLOCKSIZE), BLOCKSIZE >> > (
				clothParam, obsParam, d_riz.param(), dt);
			CUDA_CHECK(cudaPeekAtLastError());
		}
	}
	CUDA_CHECK(cudaDeviceSynchronize());
	printf("Resolve Cloth Collision RIZ: %lf msec\n", (CNOW - timer) / 10000.0);

	return result;

#ifdef COLLISION_TESTTIMER
	CUDA_CHECK(cudaDeviceSynchronize());
	printf("Resolve Cloth Collision RIZ: %lf msec\n", (CNOW - timer) / 10000.0);
#endif
}
void CollisionSolver::compRigidImpactZone(
	ContactElems& ceParam,
	RIZone& h_riz, DRIZone& d_riz,
	Cloth* cloths, Obstacle* obstacles,
	const REAL dt) 
{
#ifdef COLLISION_TESTTIMER
	CUDA_CHECK(cudaDeviceSynchronize());
	ctimer timer = CNOW;
#endif

	uint itr = 0u;
	while (ResolveRigidImpactZone(
		ceParam, h_riz, d_riz, *cloths->_param, *obstacles->_param,
		cloths->h_nbNs, obstacles->h_nbNs, dt))
		itr++;
	//-------------------------------------------------------------------------------
	//CUDA_CHECK(cudaDeviceSynchronize());
	//printf("step 3: %lf msec\n", (CNOW - timer) / 10000.0);
	//timer = CNOW;
	//-------------------------------------------------------------------------------
	if (itr > 0u) printf("Rigid Impact Zone %d\n", itr);

#ifdef COLLISION_TESTTIMER
	CUDA_CHECK(cudaDeviceSynchronize());
	printf("CollisionSolver::compClothRigidImpactZone: %lf msec\n", (CNOW - timer) / 10000.0);
#endif
}
//-------------------------------------------------------------------------
void CollisionSolver::compCollisionImpulse(
	ContactElems& ceParam,
	Cloth* cloths, Obstacle* obstacles,
	bool isProximity, const REAL dt)
{
#ifdef COLLISION_TESTTIMER
	CUDA_CHECK(cudaDeviceSynchronize());
	ctimer timer = CNOW;
#endif

	compCollisionImpulse_CE_kernel << <divup(ceParam._size, BLOCKSIZE), BLOCKSIZE >> > (
		ceParam.param(), *cloths->_param, *obstacles->_param, isProximity, dt);
	CUDA_CHECK(cudaPeekAtLastError());

#ifdef COLLISION_TESTTIMER
	CUDA_CHECK(cudaDeviceSynchronize());
	printf("CollisionSolver::compClothCollisionImpulse: %lf msec\n", (CNOW - timer) / 10000.0);
#endif
}
//-------------------------------------------------------------------------
bool CollisionSolver::applyImpulse(
	Cloth* cloths, Obstacle* obstacles, uint smoothing, REAL dt)
{
#ifdef COLLISION_TESTTIMER
	CUDA_CHECK(cudaDeviceSynchronize());
	ctimer timer = CNOW;
#endif

	bool result = false;
	bool* d_applied;

	CUDA_CHECK(cudaMalloc((void**)&d_applied, sizeof(bool)));
	CUDA_CHECK(cudaMemset(d_applied, 0, sizeof(bool)));

	applyClothCollision_kernel << <divup(cloths->_numNodes, BLOCKSIZE), BLOCKSIZE >> > (
		*cloths->_param, dt, d_applied);
	CUDA_CHECK(cudaPeekAtLastError());

	CUDA_CHECK(cudaMemcpy(&result, d_applied, sizeof(bool), cudaMemcpyDeviceToHost));
	CUDA_CHECK(cudaFree(d_applied));
	if (result) {
		Dvector<REAL> buffer;
		buffer = cloths->d_impulses;
		for (int s = 0; s < smoothing; s++) {
			SmoothingImpulse_kernel << <divup(cloths->_numNodes, BLOCKSIZE), BLOCKSIZE >> > (
				*cloths->_param, cloths->d_nbNs._index(), cloths->d_nbNs._array(), cloths->d_impulses(), buffer());
			CUDA_CHECK(cudaPeekAtLastError());
			cloths->d_impulses = buffer;
		}
		applyClothImpulse_kernel << <divup(cloths->_numNodes, BLOCKSIZE), BLOCKSIZE >> > (
			*cloths->_param);
		CUDA_CHECK(cudaPeekAtLastError());
	}

#ifdef COLLISION_TESTTIMER
	CUDA_CHECK(cudaDeviceSynchronize());
	printf("CollisionSolver::applyImpulse: %lf msec\n", (CNOW - timer) / 10000.0);
#endif

	return result;
}