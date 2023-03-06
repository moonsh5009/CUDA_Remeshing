#ifndef __PROJECTIVE_DYNAMICS_H__
#define __PROJECTIVE_DYNAMICS_H__

#pragma once
#include "Cloth.h"

namespace ProjectiveDynamics {
	void initProject(Cloth* cloths);
	void compErrorProject(Cloth* cloths);
	void updateXsProject(Cloth* cloths, REAL* maxError);
}

#endif