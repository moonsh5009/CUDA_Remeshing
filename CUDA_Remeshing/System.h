#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#pragma once
#include "Simulation.h"
#include "ProjectiveDynamics.h"
#include "CollisionSolver.h"
#include "TearingSolver.h"

#define SYSTEM_TIME

class System {
public:
	Cloth				*_cloths;
	Obstacle			*_obstacles;
public:
	AABB				_boundary;
public:
	REAL3				_gravity;
	REAL				_dt;
	REAL				_invdt;
	uint				_frame;
	uint				_subStep;
public:
	ContactElems		_ceParam;
public:
	vector<CutPlane>	h_cutPlanes;
public:
	System() {}
	System(REAL3& gravity, REAL dt) {
		init(gravity, dt);
	}
	~System() {}
public:
	uint	numFaces(void) const {
		return _cloths->_numFaces + _obstacles->_numFaces;
	}
public:
	void	init(REAL3& gravity, REAL dt);
public:
	void	addCloth(
		Mesh* mesh, REAL friction,
		REAL radius, REAL restDensity, REAL restFluidDensity, REAL restSolidFraction,
		REAL viscosity, REAL surfaceTension,
		float4 frontColor, float4 backColor, bool isSaved = true);
	void	addObstacle(
		Mesh* mesh, REAL mass, REAL friction,
		REAL3& pivot, REAL3& rotation,
		REAL radius, REAL viscosity, REAL surfaceTension,
		float4 frontColor, float4 backColor, bool isSaved = true);
public:
	void	compProjectiveDynamics(REAL dt);
	void	compCollision(REAL dt);
public:
	void	update(void);
	void	simulation(void);
	void	reset(void);
public:
	void	clickNode(const M_Ray& ray, float radius, float zFar);
	void	moveNode(const M_Ray& ray);
	void	clickOff(void);
	void	cutting(void);
public:
	void	draw(void);
	void	drawBoundary(void);
};

#endif