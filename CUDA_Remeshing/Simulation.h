#ifndef __SIMULATION_H__
#define __SIMULATION_H__

#pragma once
#include "Cloth.h"
#include "Obstacle.h"

namespace Simulation {
	void initMasses(Cloth* cloth, Obstacle* obstacle);
	void compGravityForce(MeshObject* obj, const REAL3& gravity);
	void compRotationForce(Obstacle* obj, const REAL dt);

	void applyForce(MeshObject* obj, const REAL dt);

	void updateVelocity(Dvector<REAL>& n0s, Dvector<REAL>& n1s, Dvector<REAL>& vs, const REAL invdt);
	void updatePosition(Dvector<REAL>& ns, Dvector<REAL>& vs, const REAL dt);

	void Damping(Dvector<REAL>& vs, REAL w);
	void Damping(Dvector<REAL>& vs, Dvector<uchar>& isFixeds, REAL w);

	void rayCasting(Cloth* cloths, const M_Ray& ray, float radius, float zFar);
}

#endif