#include "System.h"

void System::init(REAL3& gravity, REAL dt) {
	_gravity = gravity;
	_dt = dt;
	_invdt = 1.0 / dt;

	/*_boundary._min = make_REAL3(-1.5, -1.5, -1.5);
	_boundary._max = make_REAL3(1.5, 1.5, 1.5);*/
	_boundary._min = make_REAL3(-5.5, -5.5, -5.5);
	_boundary._max = make_REAL3(5.5, 5.5, 5.5);

	_cloths = new Cloth();
	_obstacles = new Obstacle();

	_frame = 0u;
}

void System::addCloth(
	Mesh* mesh, REAL friction,
	REAL radius, REAL restDensity, REAL restFluidDensity, REAL restSolidFraction,
	REAL viscosity, REAL surfaceTension,
	float4 frontColor, float4 backColor, bool isSaved)
{
	REAL mass = radius * radius * radius * M_PI * 4.0 / 3.0 * restDensity;
	REAL fluidMass = radius * radius * radius * M_PI * 4.0 / 3.0 * restFluidDensity * (1.0 - restSolidFraction);
	_cloths->addCloth(mesh, mass, radius, friction, restSolidFraction, fluidMass, frontColor, backColor, isSaved);
}
void System::addObstacle(
	Mesh* mesh, REAL mass, REAL friction,
	REAL3& pivot, REAL3& rotation,
	REAL radius, REAL viscosity, REAL surfaceTension,
	float4 frontColor, float4 backColor, bool isSaved)
{
	_obstacles->addObject(mesh, mass, radius, friction, frontColor, backColor, pivot, rotation, isSaved);
}

void System::compProjectiveDynamics(REAL dt) {
	REAL invdt = 1.0 / dt;
	REAL invdt2 = invdt * invdt;

	REAL omg, maxError;

	uint itr = 0u;
	//REAL ww = 0.9962;
	REAL ww = 0.99;
	if (_cloths->_numNodes > 0u) {
		_cloths->_pdParam->_dt = dt;
		_cloths->_pdParam->_invdt2 = 1.0 / (dt * dt);
		_cloths->_pdParam->_underRelax = 0.9;
		ProjectiveDynamics::initProject(_cloths);
		_cloths->d_prevXs = _cloths->d_Zs;

		while (1) {
			if (itr < 11u)			_cloths->_pdParam->_omg = 1.0;
			else if (itr == 11u)	_cloths->_pdParam->_omg = 2.0 / (2.0 - ww * ww);
			else					_cloths->_pdParam->_omg = 4.0 / (4.0 - ww * ww * _cloths->_pdParam->_omg);
			itr++;

			_cloths->d_Bs.memset(0);
			_cloths->d_newXs = _cloths->d_Zs;
			ProjectiveDynamics::compErrorProject(_cloths);
			ProjectiveDynamics::updateXsProject(_cloths, &maxError);

			if (itr >= 100u)
				break;
		}
		Simulation::updateVelocity(_cloths->d_ns, _cloths->d_Xs, _cloths->d_vs, invdt);
	}
}
void System::compCollision(REAL dt) {
#if 1
	_cloths->d_n0s = _cloths->d_ns;
	_obstacles->d_n0s = _obstacles->d_ns;

	bool isDetected;
	REAL subDt = dt;
	REAL minTime;
	uint itr;

	CollisionSolver::getContactElements(_ceParam, _cloths, _obstacles);
	for (itr = 0u; itr < 100u; itr++) {
		_cloths->d_impulses.memset(0);
		_obstacles->d_impulses.memset(0);
		_cloths->d_colWs.memset(0);
		_obstacles->d_colWs.memset(0);
		CollisionSolver::compCollisionImpulse(_ceParam, _cloths, _obstacles, true, subDt);
		isDetected = CollisionSolver::applyImpulse(_cloths, _obstacles, COLLISION_SMOOTHING, subDt);
		if (!isDetected)
			break;
	}

	CollisionSolver::getClothCCDtime(_cloths, _obstacles, subDt, &minTime);

	while (minTime <= 1.0) {
		printf("CCD %.20f\n", minTime);
		minTime *= 0.8;
		Simulation::updatePosition(_cloths->d_ns, _cloths->d_vs, subDt * minTime);
		Simulation::updatePosition(_obstacles->d_ns, _obstacles->d_vs, subDt * minTime);
		subDt -= subDt * minTime;
		/*Simulation::updateVelocity(_cloths->d_n0s, _cloths->d_ns, _cloths->d_vs, 1.0 / (dt - subDt));
		Simulation::updateVelocity(_sphParticles->d_x0s, _sphParticles->d_xs, _sphParticles->d_vs, 1.0 / (dt - subDt));*/

		CollisionSolver::getContactElements(_ceParam, _cloths, _obstacles);
		if (subDt > dt * 0.4) {
			for (itr = 0u; itr < 100u; itr++) {
				_cloths->d_impulses.memset(0);
				_obstacles->d_impulses.memset(0);
				_cloths->d_colWs.memset(0);
				_obstacles->d_colWs.memset(0);
				CollisionSolver::compCollisionImpulse(_ceParam, _cloths, _obstacles, false, subDt);
				isDetected = CollisionSolver::applyImpulse(_cloths, _obstacles, 0, subDt);
				if (!isDetected)
					break;
			}
		}
		else {
			for (itr = 0u; itr < 100u; itr++) {
				_cloths->d_impulses.memset(0);
				_obstacles->d_impulses.memset(0);
				_cloths->d_colWs.memset(0);
				_obstacles->d_colWs.memset(0);
				CollisionSolver::compCollisionImpulse(_ceParam, _cloths, _obstacles, false, subDt);
				isDetected = CollisionSolver::applyImpulse(_cloths, _obstacles, 0, subDt);
				if (!isDetected)
					break;
			}
		}

		if ((/*minTime < 0.0001 || */subDt < 0.1 * dt) && isDetected) {
			RIZone h_riz;
			DRIZone d_riz;
			CollisionSolver::compRigidImpactZone(
				_ceParam, h_riz, d_riz, _cloths, _obstacles, subDt);
		}

		CollisionSolver::getClothCCDtime(_cloths, _obstacles, subDt, &minTime);
	}
	Simulation::updatePosition(_cloths->d_ns, _cloths->d_vs, subDt);
	Simulation::updateVelocity(_cloths->d_n0s, _cloths->d_ns, _cloths->d_vs, 1.0 / dt);
	_cloths->d_ns = _cloths->d_n0s;
	_obstacles->d_ns = _obstacles->d_n0s;
#else
	if (_cloths->_numNodes > 0.0) {
		_cloths->d_n0s = _cloths->d_ns;
		_obstacles->d_n0s = _obstacles->d_ns;

		bool isDetected;
		REAL subDt = dt;
		REAL minTime = 1.0;
		REAL dx = 0.0;
		uint itr;
		uint l = 0u;

		REAL invdt, invdt2, maxError, ww;
		while (subDt > 0.0) {
			invdt = 1.0 / subDt;
			invdt2 = invdt * invdt;
			ww = 0.99;

			CollisionSolver::getContactElements(_ceParam, _cloths, _obstacles);
			_cloths->_pdParam->_dt = subDt;
			_cloths->_pdParam->_invdt2 = invdt2;
			_cloths->_pdParam->_omg = 1.0;
			_cloths->_pdParam->_underRelax = 1.0;
			ProjectiveDynamics::initProject(_cloths);
			for (itr = 0u; itr < 100u; itr++) {
				if (itr < 50u && (subDt > dt * 0.3)) {
					_cloths->d_Xs = _cloths->d_ns;
					Simulation::updatePosition(_cloths->d_Xs, _cloths->d_vs, subDt);
					_cloths->d_Bs.memset(0.0);
					_cloths->d_newXs = _cloths->d_Zs;
					ProjectiveDynamics::compErrorProject(_cloths);
					ProjectiveDynamics::updateXsProject(_cloths, &maxError);
					Simulation::updateVelocity(_cloths->d_ns, _cloths->d_Xs, _cloths->d_vs, invdt);
				}
				_cloths->d_impulses.memset(0);
				_obstacles->d_impulses.memset(0);
				_cloths->d_colWs.memset(0);
				_obstacles->d_colWs.memset(0);
				CollisionSolver::compCollisionImpulse(_ceParam, _cloths, _obstacles, l==0, subDt);
				isDetected = CollisionSolver::applyImpulse(_cloths, _obstacles, 0, subDt);
				if (!isDetected)
					break;
			}
			/*for (itr = 0u; itr < 100u; itr++) {
				_cloths->d_impulses.memset(0);
				_obstacles->d_impulses.memset(0);
				_cloths->d_colWs.memset(0);
				_obstacles->d_colWs.memset(0);
				CollisionSolver::compCollisionImpulse(_ceParam, _cloths, _obstacles, false, subDt);
				isDetected = CollisionSolver::applyImpulse(_cloths, _obstacles, 0, subDt);
				if (!isDetected)
					break;
			}*/
			if ((/*minTime < 0.001 || */subDt < dt * 0.3) && isDetected) {
				RIZone h_riz;
				DRIZone d_riz;
				CollisionSolver::compRigidImpactZone(
					_ceParam, h_riz, d_riz, _cloths, _obstacles, subDt);
			}

			/*CollisionSolver::compEpsilonCollision(
				_cloths, _obstacles, subDt);*/


			CollisionSolver::getClothCCDtime(_cloths, _obstacles, subDt, &minTime);
			if (minTime > 1.0)
				break;

			printf("CCD %.20f\n", minTime);
			minTime *= 0.8;
			Simulation::updatePosition(_cloths->d_ns, _cloths->d_vs, subDt * minTime);
			Simulation::updatePosition(_obstacles->d_ns, _obstacles->d_vs, subDt * minTime);
			subDt -= subDt * minTime;
			l++;
		}
		Simulation::updatePosition(_cloths->d_ns, _cloths->d_vs, subDt);
		Simulation::updateVelocity(_cloths->d_n0s, _cloths->d_ns, _cloths->d_vs, 1.0 / dt);
		_cloths->d_ns = _cloths->d_n0s;
		_obstacles->d_ns = _obstacles->d_n0s;
	}
#endif
}

void System::update(void) {
	REAL subDt = _dt / (REAL)_subStep;

	Simulation::initMasses(_cloths, _obstacles);

#ifdef SYSTEM_TIME
	cudaDeviceSynchronize();
	ctimer timer = CNOW;
#endif

	// Compute Force
	{
		_cloths->d_forces.memset(0);
		_obstacles->d_forces.memset(0);

		Simulation::compGravityForce(_cloths, _gravity);
		Simulation::compRotationForce(_obstacles, subDt);

		Simulation::applyForce(_cloths, subDt);
		Simulation::applyForce(_obstacles, subDt);
	}

#ifdef SYSTEM_TIME
	cudaDeviceSynchronize();
	printf("Compute Force: %f msec\n", (CNOW - timer) / 10000.0);
	timer = CNOW;
#endif

	// Compute Projective Dynamics
	compProjectiveDynamics(subDt);

#ifdef SYSTEM_TIME
	cudaDeviceSynchronize();
	printf("Compute Projective Dynamics: %f msec\n", (CNOW - timer) / 10000.0);
	timer = CNOW;
#endif

	// Compute Collision
	compCollision(subDt);

#ifdef SYSTEM_TIME
	cudaDeviceSynchronize();
	printf("Compute Collision: %f msec\n", (CNOW - timer) / 10000.0);
	timer = CNOW;
#endif

	Simulation::updatePosition(_cloths->d_ns, _cloths->d_vs, subDt);
	Simulation::updatePosition(_obstacles->d_ns, _obstacles->d_vs, subDt);
}
void System::simulation(void) {
	ctimer timer;

	printf("\n===< Frame: %d >=======================\n", _frame);

	for (uint step = 1u; step <= _subStep; step++) {
		Simulation::Damping(_cloths->d_vs, _cloths->d_isFixeds, 0.99);
		printf("===< Step %d >=======================\n", step);

		CUDA_CHECK(cudaDeviceSynchronize());
		timer = CNOW;

		update();

		CUDA_CHECK(cudaDeviceSynchronize());
		printf("Update: %f\n", (CNOW - timer) / 10000.0);
		timer = CNOW;

		_cloths->computeNormal();
		_obstacles->computeNormal();

		CUDA_CHECK(cudaDeviceSynchronize());
		printf("Compute Normals: %f\n", (CNOW - timer) / 10000.0);
		timer = CNOW;

		_cloths->copyToHost();
		_obstacles->copyToHost();

		CUDA_CHECK(cudaDeviceSynchronize());
		printf("Copy to Host: %f\n", (CNOW - timer) / 10000.0);
	}
	_frame++;
}
void System::reset(void) {
	_frame = 0u;
	_cloths->reset();
	_obstacles->reset();
}
void System::clickNode(const M_Ray& ray, float radius, float zFar) {
	Simulation::rayCasting(_cloths, ray, radius, zFar);
}
void System::moveNode(const M_Ray& ray) {
#if 1
	vector<REAL> h_vs;
	_cloths->d_vs.copyToHost(h_vs);
	bool isApplied = false;
	for (int i = 0; i < _cloths->h_clickNodes.size(); i++) {
		uint id = _cloths->h_clickNodes[i]._id;
		if (id == 0xffffffff)
			continue;

		if (!_cloths->h_isFixeds[id]) {
			M_ClickNode node = _cloths->h_clickNodes[i];

			REAL3 p = make_REAL3(
					_cloths->h_ns[id * 3u + 0u], 
					_cloths->h_ns[id * 3u + 1u], 
					_cloths->h_ns[id * 3u + 2u]);
			REAL3 movePos = make_REAL3(
				ray._pos.x + node._dist0 * ray._dir.x + node._offset.x,
				ray._pos.y + node._dist0 * ray._dir.y + node._offset.y,
				ray._pos.z + node._dist0 * ray._dir.z + node._offset.z);

			REAL3 v = make_REAL3(
				h_vs[id * 3u + 0u],
				h_vs[id * 3u + 1u],
				h_vs[id * 3u + 2u]);

			//v = v + 0.8 * (_invdt * (movePos - p) - v);
			v = 0.8 * (movePos - p) * _invdt;

			h_vs[id * 3u + 0u] = v.x;
			h_vs[id * 3u + 1u] = v.y;
			h_vs[id * 3u + 2u] = v.z;
			isApplied = true;
		}
	}
	if (isApplied)
		_cloths->d_vs = h_vs;
#else
	bool isApplied = false;
	for (int i = 0; i < _cloths->h_clickNodes.size(); i++) {
		uint id = _cloths->h_clickNodes[i]._id;
		if (id == 0xffffffff)
			continue;

		M_ClickNode node = _cloths->h_clickNodes[i];

		REAL3 p = make_REAL3(
			_cloths->h_ns[id * 3u + 0u],
			_cloths->h_ns[id * 3u + 1u],
			_cloths->h_ns[id * 3u + 2u]);
		REAL3 movePos = make_REAL3(
			ray._pos.x + node._dist0 * ray._dir.x + node._offset.x,
			ray._pos.y + node._dist0 * ray._dir.y + node._offset.y,
			ray._pos.z + node._dist0 * ray._dir.z + node._offset.z);

		_cloths->h_ns[id * 3u + 0u] = movePos.x;
		_cloths->h_ns[id * 3u + 1u] = movePos.y;
		_cloths->h_ns[id * 3u + 2u] = movePos.z;
		_cloths->h_isFixeds[id] = 1u;
		isApplied = true;
	}
	if (isApplied) {
		_cloths->d_ns = _cloths->h_ns;
		_cloths->d_isFixeds = _cloths->h_isFixeds;
	}
#endif
}
void System::clickOff(void) {
	bool isApplied = false;
	for (int i = 0; i < _cloths->h_clickNodes.size(); i++) {
		if (_cloths->h_clickNodes[i]._id != 0xffffffff) {
			_cloths->h_isFixeds[_cloths->h_clickNodes[i]._id] = 0u;
			_cloths->h_clickNodes[i]._id = 0xffffffff;
			isApplied = true;
		}
	}
	if (isApplied) {
		_cloths->d_isFixeds = _cloths->h_isFixeds;
	}
}
void System::cutting(void) {
	Dvector<CutPlane> d_cutPlanes;
	d_cutPlanes = h_cutPlanes;
	TearingSolver::compCutting(_cloths, d_cutPlanes);
	h_cutPlanes.clear();
}

void System::draw(void) {
	_cloths->draw();
	_obstacles->draw();

	//_cloths->_bvh->draw();
	//_obstacles->_bvh->draw();
	//_obstacles->_priTree->draw();

	//drawBoundary();
}
void System::drawBoundary(void) {
	glPushMatrix();
	glDisable(GL_LIGHTING);
	glColor3d(1, 1, 1);
	glLineWidth(3.0f);

	glBegin(GL_LINES);
	glVertex3f(_boundary._min.x, _boundary._min.y, _boundary._min.z);
	glVertex3f(_boundary._max.x, _boundary._min.y, _boundary._min.z);
	glVertex3f(_boundary._min.x, _boundary._min.y, _boundary._min.z);
	glVertex3f(_boundary._min.x, _boundary._max.y, _boundary._min.z);
	glVertex3f(_boundary._min.x, _boundary._min.y, _boundary._min.z);
	glVertex3f(_boundary._min.x, _boundary._min.y, _boundary._max.z);
	glVertex3f(_boundary._max.x, _boundary._max.y, _boundary._max.z);
	glVertex3f(_boundary._min.x, _boundary._max.y, _boundary._max.z);
	glVertex3f(_boundary._max.x, _boundary._max.y, _boundary._max.z);
	glVertex3f(_boundary._max.x, _boundary._min.y, _boundary._max.z);
	glVertex3f(_boundary._max.x, _boundary._max.y, _boundary._max.z);
	glVertex3f(_boundary._max.x, _boundary._max.y, _boundary._min.z);
	glVertex3f(_boundary._max.x, _boundary._min.y, _boundary._min.z);
	glVertex3f(_boundary._max.x, _boundary._min.y, _boundary._max.z);
	glVertex3f(_boundary._max.x, _boundary._min.y, _boundary._min.z);
	glVertex3f(_boundary._max.x, _boundary._max.y, _boundary._min.z);
	glVertex3f(_boundary._min.x, _boundary._max.y, _boundary._min.z);
	glVertex3f(_boundary._min.x, _boundary._max.y, _boundary._max.z);
	glVertex3f(_boundary._min.x, _boundary._max.y, _boundary._min.z);
	glVertex3f(_boundary._max.x, _boundary._max.y, _boundary._min.z);
	glVertex3f(_boundary._min.x, _boundary._min.y, _boundary._max.z);
	glVertex3f(_boundary._min.x, _boundary._max.y, _boundary._max.z);
	glVertex3f(_boundary._min.x, _boundary._min.y, _boundary._max.z);
	glVertex3f(_boundary._max.x, _boundary._min.y, _boundary._max.z);
	glEnd();

	glLineWidth(1.0f);
	glEnable(GL_LIGHTING);
	glPopMatrix();
}