#include "Cloth.h"

void Cloth::init(void) {
	 MeshObject::init();

	 _param = new ClothParam();
	 _pdParam = new PDParam();

#if CLOTH_STRAIN==0
	 _edgeConstraints = new EdgeConstraint();
#else
	 _triangleConstraints = new TriangleConstraint();
#endif

#if CLOTH_BENDING==0
	 _edgeBendingConstraints = new EdgeConstraint();
#else
	 _dihedralBendingConstraints = new DihedralConstraint();
#endif

	 _type = TYPE_MESH_CLOTH;
}
void Cloth::reset(void) {
	MeshObject::reset();
	h_mfs = h_mfs0;
	d_mfs = h_mfs0;

	h_strainWs = h_strainWs0;
	h_bendingWs = h_bendingWs0;
	d_strainWs = h_strainWs0;
	d_bendingWs = h_bendingWs0;

#if CLOTH_STRAIN==1
	h_firstLames = h_firstLames0;
	h_secondLames = h_secondLames0;
	d_firstLames = h_firstLames;
	d_secondLames = h_secondLames;
#endif

	h_restNs = h_restNs0;
	d_restNs = h_restNs0;
	h_edgeLimits = h_edgeLimits0;
	d_edgeLimits = h_edgeLimits0;

	updateElements();
	setParam();
}
void Cloth::save(void) {
	MeshObject::save();
	h_mfs0 = h_mfs;

	h_strainWs0 = h_strainWs;
	h_bendingWs0 = h_bendingWs;
#if CLOTH_STRAIN==1
	h_firstLames0 = h_firstLames;
	h_secondLames0 = h_secondLames;
#endif

	h_restNs0 = h_restNs;
	h_edgeLimits0 = h_edgeLimits;
}
void Cloth::updateElements(void) {
	MeshObject::updateElements();
	initConstraints();

	d_Bs.resize(_numNodes);
	d_Xs.resize(_numNodes * 3u);
	d_Zs.resize(_numNodes * 3u);
	d_newXs.resize(_numNodes * 3u);
	d_prevXs.resize(_numNodes * 3u);

	h_restSolidFractions.resize(_numNodes);
	h_maxFluidMasses.resize(_numNodes, 0.0);
	h_ss.resize(_numNodes, 0.0);
	d_restSolidFractions.resize(_numNodes);
	d_maxFluidMasses.resize(_numNodes);
	d_ss.resize(_numNodes);

	d_isExtends.resize(_numEdges);
	//d_isExtends.resize(_numFaces);
	d_isExtends.memset(0);
	//d_deformGrads.resize(_numEdges * 3u);
	d_deformGrads.resize(_numNodes * 3u);
	//d_deformGrads.resize(_numFaces * 3u);
	d_deformGrads.memset(0);
}

void Cloth::addCloth(
	Mesh* mesh, REAL mass, REAL thickness, REAL friction, 
	REAL restSolidFraction, REAL maxFluidMass,
	float4 frontColor, float4 backColor, bool isSaved)
{
	h_restNs.insert(h_restNs.end(), mesh->_ns.begin(), mesh->_ns.end());
	d_restNs = h_restNs;

	h_strainWs.push_back(10000000.0 * mass);
	h_bendingWs.push_back(320000.0 * mass);
	d_strainWs = h_strainWs;
	d_bendingWs = h_bendingWs;
#if CLOTH_STRAIN==1
	REAL youngsModulus = 100.0;
	REAL poissonRatio = 0.1;
	h_firstLames.push_back(youngsModulus * poissonRatio / ((1.0 + poissonRatio) * (1.0 - 2.0 * poissonRatio)));
	h_secondLames.push_back(youngsModulus / (2.0 * (1.0 + poissonRatio)));
	d_firstLames = h_firstLames;
	d_secondLames = h_secondLames;
#endif

	for (uint i = 0; i < mesh->_numEdges; i++)
		h_edgeLimits.push_back(1.4 + 0.2 * (REAL)rand() / (REAL)RAND_MAX);
	d_edgeLimits = h_edgeLimits;

	addMesh(mesh, mass, thickness, friction, frontColor, backColor);

	h_mfs.resize(_numNodes, 0.0);
	d_mfs = h_mfs;

#if SCENE==0
	fix();
#elif SCENE==1
	fix();
#elif SCENE==2
#elif SCENE==3
	fix();
#elif SCENE==4
	fix();
#endif

	if (isSaved)
		save();

	updateElements();
	setParam();
}

void Cloth::initConstraints(void) {
	ctimer timer = CNOW;
	MeshKernel::initEdgeConstraints(d_es, d_restNs, d_restLengths, _numEdges);
#if CLOTH_STRAIN == 0
	_edgeConstraints->_inos = d_es._list;
	_edgeConstraints->_ws = d_strainWs._list;
	_edgeConstraints->_restLengths = d_restLengths._list;
	_edgeConstraints->_numConstraints = _numEdges;
#else
	MeshKernel::initTriangleConstraints(d_fs, d_restNs, d_restAreas, d_invDs, _numFaces);
	_triangleConstraints->_inos = d_fs._list;
	_triangleConstraints->_ws = d_strainWs._list;
	_triangleConstraints->_firstLames = d_firstLames._list;
	_triangleConstraints->_secondLames = d_secondLames._list;
	_triangleConstraints->_restAreas = d_restAreas._list;
	_triangleConstraints->_invDs = d_invDs._list;
	_triangleConstraints->_numConstraints = _numFaces;
#endif

#if CLOTH_BENDING == 0
	MeshKernel::initEdgeConstraints(d_EnbNs, d_restNs, d_restBendingLengths, _numEdges);
	_edgeBendingConstraints->_inos = d_EnbNs._list;
	_edgeBendingConstraints->_ws = d_bendingWs._list;
	_edgeBendingConstraints->_restLengths = d_restBendingLengths._list;
	_edgeBendingConstraints->_numConstraints = _numEdges;
#else
	MeshKernel::initDihedralConstraints(d_fs, d_es, d_restNs, d_EnbNs, d_restBendingAngles, _numEdges);
	_dihedralBendingConstraints->_inos = d_EnbNs._list;
	_dihedralBendingConstraints->_ws = d_bendingWs._list;
	_dihedralBendingConstraints->_restAngles = d_restBendingAngles._list;
	_dihedralBendingConstraints->_numConstraints = _numEdges;
#endif
	cudaDeviceSynchronize();
	printf("initConstraints %f msec\n", (CNOW - timer) / 10000.0);
}

void Cloth::fix(void) {
#if SCENE==0
	// xz
	/*uint mxmy = 0u;
	uint Mxmy = 0u;
	uint mxMy = 0u;
	uint MxMy = 0u;
	for (uint i = 1u; i < _numNodes; i++) {
		REAL3 n = make_REAL3(h_ns[i * 3u + 0u], h_ns[i * 3u + 1u], h_ns[i * 3u + 2u]);

		if (n.x - h_ns[mxmy * 3u + 0u] < 1.0e-5 && n.z - h_ns[mxmy * 3u + 2u] < 1.0e-5)
			mxmy = i;
		if (n.x - h_ns[Mxmy * 3u + 0u] > -1.0e-5 && n.z - h_ns[Mxmy * 3u + 2u] < 1.0e-5)
			Mxmy = i;
		if (n.x - h_ns[mxMy * 3u + 0u] < 1.0e-5 && n.z - h_ns[mxMy * 3u + 2u] > -1.0e-5)
			mxMy = i;
		if (n.x - h_ns[MxMy * 3u + 0u] > -1.0e-5 && n.z - h_ns[MxMy * 3u + 2u] > -1.0e-5)
			MxMy = i;
	}

	h_isFixeds[Mxmy] = 1u;
	h_isFixeds[MxMy] = 1u;
	h_isFixeds[mxmy] = 1u;
	h_isFixeds[mxMy] = 1u;*/
	// xy
	uint mxmy = 0u;
	uint Mxmy = 0u;
	uint mxMy = 0u;
	uint MxMy = 0u;
	for (uint i = 1u; i < _numNodes; i++) {
		REAL3 n = make_REAL3(h_ns[i * 3u + 0u], h_ns[i * 3u + 1u], h_ns[i * 3u + 2u]);

		if (n.x - h_ns[mxmy * 3u + 0u] < 1.0e-5 && n.y - h_ns[mxmy * 3u + 1u] < 1.0e-5)
			mxmy = i;
		if (n.x - h_ns[Mxmy * 3u + 0u] > -1.0e-5 && n.y - h_ns[Mxmy * 3u + 1u] < 1.0e-5)
			Mxmy = i;
		if (n.x - h_ns[mxMy * 3u + 0u] < 1.0e-5 && n.y - h_ns[mxMy * 3u + 1u] > -1.0e-5)
			mxMy = i;
		if (n.x - h_ns[MxMy * 3u + 0u] > -1.0e-5 && n.y - h_ns[MxMy * 3u + 1u] > -1.0e-5)
			MxMy = i;
	}

	h_isFixeds[Mxmy] = 1u;
	h_isFixeds[MxMy] = 1u;
	h_isFixeds[mxmy] = 1u;
	h_isFixeds[mxMy] = 1u;

	//uint mxMy = 0u;
	//uint MxMy = 0u;
	//for (uint i = 1u; i < _numNodes; i++) {
	//	REAL3 n = make_REAL3(h_ns[i * 3u + 0u], h_ns[i * 3u + 1u], h_ns[i * 3u + 2u]);

	//	if (n.x - h_ns[mxMy * 3u + 0u] < 1.0e-5 && n.y - h_ns[mxMy * 3u + 1u] > -1.0e-5)
	//		mxMy = i;
	//	if (n.x - h_ns[MxMy * 3u + 0u] > -1.0e-5 && n.y - h_ns[MxMy * 3u + 1u] > -1.0e-5)
	//		MxMy = i;
	//}

	////h_isFixeds[MxMy] = 1u;
	//h_isFixeds[mxMy] = 1u;

	/*for (uint i = 0u; i < _numNodes; i++) {
		REAL3 n = make_REAL3(h_ns[i * 3u + 0u], h_ns[i * 3u + 1u], h_ns[i * 3u + 2u]);

		if (n.x < -1.0 + 0.05 || n.x > 1.0 - 0.05)
			h_isFixeds[i] = 1u;
	}*/
#elif SCENE==1
	uint mxmy = 0u;
	uint Mxmy = 0u;
	uint mxMy = 0u;
	uint MxMy = 0u;
	for (uint i = 1u; i < _numNodes; i++) {
		REAL3 n = make_REAL3(h_ns[i * 3u + 0u], h_ns[i * 3u + 1u], h_ns[i * 3u + 2u]);

		if (n.x - h_ns[mxmy * 3u + 0u] < 1.0e-5 && n.z - h_ns[mxmy * 3u + 2u] < 1.0e-5)
			mxmy = i;
		if (n.x - h_ns[Mxmy * 3u + 0u] > -1.0e-5 && n.z - h_ns[Mxmy * 3u + 2u] < 1.0e-5)
			Mxmy = i;
		if (n.x - h_ns[mxMy * 3u + 0u] < 1.0e-5 && n.z - h_ns[mxMy * 3u + 2u] > -1.0e-5)
			mxMy = i;
		if (n.x - h_ns[MxMy * 3u + 0u] > -1.0e-5 && n.z - h_ns[MxMy * 3u + 2u] > -1.0e-5)
			MxMy = i;
	}

	h_isFixeds[Mxmy] = 1u;
	h_isFixeds[MxMy] = 1u;
	h_isFixeds[mxmy] = 1u;
	h_isFixeds[mxMy] = 1u;
#elif SCENE==2
#elif SCENE==3
	uint mxMy = 0u;
	uint MxMy = 0u;
	for (uint i = 1u; i < _numNodes; i++) {
		REAL3 n = make_REAL3(h_ns[i * 3u + 0u], h_ns[i * 3u + 1u], h_ns[i * 3u + 2u]);

		if (n.x - h_ns[mxMy * 3u + 0u] < 1.0e-5 && n.y - h_ns[mxMy * 3u + 1u] > -1.0e-5)
			mxMy = i;
		if (n.x - h_ns[MxMy * 3u + 0u] > -1.0e-5 && n.y - h_ns[MxMy * 3u + 1u] > -1.0e-5)
			MxMy = i;
	}

	h_isFixeds[MxMy] = 1u;
	h_isFixeds[mxMy] = 1u;
#elif SCENE==4
	/*uint mxmy = 0u;
	uint Mxmy = 0u;
	uint mxMy = 0u;
	uint MxMy = 0u;
	for (uint i = 1u; i < _numNodes; i++) {
		REAL3 n = make_REAL3(h_ns[i * 3u + 0u], h_ns[i * 3u + 1u], h_ns[i * 3u + 2u]);

		if (n.x - h_ns[mxmy * 3u + 0u] < 1.0e-5 && n.z - h_ns[mxmy * 3u + 2u] < 1.0e-5)
			mxmy = i;
		if (n.x - h_ns[Mxmy * 3u + 0u] > -1.0e-5 && n.z - h_ns[Mxmy * 3u + 2u] < 1.0e-5)
			Mxmy = i;
		if (n.x - h_ns[mxMy * 3u + 0u] < 1.0e-5 && n.z - h_ns[mxMy * 3u + 2u] > -1.0e-5)
			mxMy = i;
		if (n.x - h_ns[MxMy * 3u + 0u] > -1.0e-5 && n.z - h_ns[MxMy * 3u + 2u] > -1.0e-5)
			MxMy = i;
	}

	h_isFixeds[Mxmy] = 1u;
	h_isFixeds[MxMy] = 1u;
	h_isFixeds[mxmy] = 1u;
	h_isFixeds[mxMy] = 1u;*/
	for (uint i = 0u; i < _numNodes; i++) {
		REAL3 n = make_REAL3(h_ns[i * 3u + 0u], h_ns[i * 3u + 1u], h_ns[i * 3u + 2u]);

		if (n.x < -1.0 + 0.05 || n.x > 1.0 - 0.05)
			h_isFixeds[i] = 1u;
	}
#endif


	d_isFixeds = h_isFixeds;
}
void Cloth::moveFixed(REAL3 vel) {
	vector<REAL> h_vs;
	d_vs.copyToHost(h_vs);

	for (uint i = 0u; i < _numNodes; i++) {
		if (h_isFixeds[i]) {
			h_vs[i * 3u + 0u] = vel.x;
			h_vs[i * 3u + 1u] = vel.y;
			h_vs[i * 3u + 2u] = vel.z;
		}
	}
	d_vs = h_vs;
}
void Cloth::moveFixed(REAL3 lvel, REAL3 rvel) {
	vector<REAL> h_vs;
	d_vs.copyToHost(h_vs);

	for (uint i = 0u; i < _numNodes; i++) {
		if (h_isFixeds[i]) {
			REAL3 a = make_REAL3(h_ns[i * 3u + 0u], h_ns[i * 3u + 1u], h_ns[i * 3u + 2u]);
			if (a.x < 0.0) {
				h_vs[i * 3u + 0u] = lvel.x;
				h_vs[i * 3u + 1u] = lvel.y;
				h_vs[i * 3u + 2u] = lvel.z;
			}
			else {
				h_vs[i * 3u + 0u] = rvel.x;
				h_vs[i * 3u + 1u] = rvel.y;
				h_vs[i * 3u + 2u] = rvel.z;
			}
		}
	}
	d_vs = h_vs;
}
void Cloth::rotateFixed(REAL3 degreeL, REAL3 degreeR, REAL moveL, REAL moveR, REAL invdt) {
	vector<REAL> h_vs;
	d_vs.copyToHost(h_vs);

	REAL3 a, b, pa, pb, center;
	REAL3 pivot, va, vb;
	REAL cx, sx, cy, sy, cz, sz;

	REAL3 degree = degreeL * M_PI * 0.00555555555555555555555555555556;
	cx = cos(degree.x);
	sx = sin(degree.x);
	cy = cos(degree.y);
	sy = -sin(degree.y);
	cz = cos(degree.z);
	sz = sin(degree.z);

	for (uint i = 0u; i < _numNodes; i++) {
		if (h_isFixeds[i]) {
			a = make_REAL3(h_ns[i * 3u + 0u], h_ns[i * 3u + 1u], h_ns[i * 3u + 2u]);
			if (a.x < 0.0) {
				pa.x = a.x * cz * cy + a.y * (cz * sy * sx - sz * cx) + a.z * (cz * sy * cx + sz * sx);
				pa.y = a.x * sz * cy + a.y * (sz * sy * sx + cz * cx) + a.z * (sz * sy * cx - cz * sx);
				pa.z = a.x * -sy + a.y * cy * sx + a.z * cy * cx;

				pa.x += moveL;

				va = invdt * (pa - a);
				h_vs[i * 3u + 0u] = va.x;
				h_vs[i * 3u + 1u] = va.y;
				h_vs[i * 3u + 2u] = va.z;
			}
		}
	}

	degree = degreeR * M_PI * 0.00555555555555555555555555555556;
	cx = cos(degree.x);
	sx = sin(degree.x);
	cy = cos(degree.y);
	sy = -sin(degree.y);
	cz = cos(degree.z);
	sz = sin(degree.z);

	for (uint i = 0u; i < _numNodes; i++) {
		if (h_isFixeds[i]) {
			a = make_REAL3(h_ns[i * 3u + 0u], h_ns[i * 3u + 1u], h_ns[i * 3u + 2u]);
			if (a.x > 0.0) {
				pa.x = a.x * cz * cy + a.y * (cz * sy * sx - sz * cx) + a.z * (cz * sy * cx + sz * sx);
				pa.y = a.x * sz * cy + a.y * (sz * sy * sx + cz * cx) + a.z * (sz * sy * cx - cz * sx);
				pa.z = a.x * -sy + a.y * cy * sx + a.z * cy * cx;

				pa.x -= moveR;

				va = invdt * (pa - a);
				h_vs[i * 3u + 0u] = va.x;
				h_vs[i * 3u + 1u] = va.y;
				h_vs[i * 3u + 2u] = va.z;
			}
		}
	}
	d_vs = h_vs;
}

void Cloth::drawSurface(void)
{
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 1); // turn on two-sided lighting.
	float black[] = { 0.0f, 0.0f, 0.0f, 1.0f };
	float white[] = { 0.4f, 0.4f, 0.4f, 1.0f };
	glMaterialfv(GL_FRONT, GL_SPECULAR, white);
	glMaterialf(GL_FRONT, GL_SHININESS, 64);
	glMaterialfv(GL_BACK, GL_SPECULAR, black); // no specular highlights

	uint prevPhase = 0xffffffff;
	for (uint i = 0u; i < _numFaces; i++) {
		/*if (i == _bvh->h_triInfos[_bvh->_test]._id) {
			printf("%d, %d, %lu\n", i, _bvh->h_triInfos[_bvh->_test]._id, _bvh->h_triInfos[_bvh->_test]._pos);
			uint ino0 = h_fs[i * 3u + 0u];
			uint ino1 = h_fs[i * 3u + 1u];
			uint ino2 = h_fs[i * 3u + 2u];
			REAL3 a = make_REAL3(h_ns[ino0 * 3u + 0u], h_ns[ino0 * 3u + 1u], h_ns[ino0 * 3u + 2u]);
			REAL3 b = make_REAL3(h_ns[ino1 * 3u + 0u], h_ns[ino1 * 3u + 1u], h_ns[ino1 * 3u + 2u]);
			REAL3 c = make_REAL3(h_ns[ino2 * 3u + 0u], h_ns[ino2 * 3u + 1u], h_ns[ino2 * 3u + 2u]);

			float frontColor[4] = { 1.f, 0.f, 0.f, 1.f };
			float backColor[4] = { 1.f, 0.f, 0.f, 1.f };
			glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, frontColor);
			glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, backColor);

			glBegin(GL_TRIANGLES);

			glNormal3f(h_fNorms[i * 3u + 0u], h_fNorms[i * 3u + 1u], h_fNorms[i * 3u + 2u]);
			glVertex3f(a.x, a.y, a.z);
			glVertex3f(b.x, b.y, b.z);
			glVertex3f(c.x, c.y, c.z);

			glEnd();
			
			continue;
		}*/

		uint ino0 = h_fs[i * 3u + 0u];
		uint ino1 = h_fs[i * 3u + 1u];
		uint ino2 = h_fs[i * 3u + 2u];
		REAL3 a = make_REAL3(h_ns[ino0 * 3u + 0u], h_ns[ino0 * 3u + 1u], h_ns[ino0 * 3u + 2u]);
		REAL3 b = make_REAL3(h_ns[ino1 * 3u + 0u], h_ns[ino1 * 3u + 1u], h_ns[ino1 * 3u + 2u]);
		REAL3 c = make_REAL3(h_ns[ino2 * 3u + 0u], h_ns[ino2 * 3u + 1u], h_ns[ino2 * 3u + 2u]);
		uint phase = h_nodePhases[ino0];
		if (h_frontColors[phase].w == 0.0)
			continue;

		float frontColor[4];
		float backColor[4];

		/*float s0 = powf(1.f - min((1.0 - h_restSolidFractions[ino0]) * h_ss[ino0], 1.f), 1.0f);
		float s1 = powf(1.f - min((1.0 - h_restSolidFractions[ino1]) * h_ss[ino1], 1.f), 1.0f);
		float s2 = powf(1.f - min((1.0 - h_restSolidFractions[ino2]) * h_ss[ino2], 1.f), 1.0f);*/

		float s0, s1, s2;
		/*if (h_maxFluidMasses[ino0] > 0.0)
			s0 = pow(1.f - (1.0 - h_restSolidFractions[ino0]) * max(min(h_mfs[ino0] / (h_maxFluidMasses[ino0] + FLT_EPSILON), 1.0), 0.0), 1.0);
		else s0 = 1.f;
		if (h_maxFluidMasses[ino1] > 0.0)
			s1 = pow(1.f - (1.0 - h_restSolidFractions[ino1]) * max(min(h_mfs[ino1] / (h_maxFluidMasses[ino1] + FLT_EPSILON), 1.0), 0.0), 1.0);
		else s1 = 1.f;
		if (h_maxFluidMasses[ino2] > 0.0)
			s2 = pow(1.f - (1.0 - h_restSolidFractions[ino2]) * max(min(h_mfs[ino2] / (h_maxFluidMasses[ino2] + FLT_EPSILON), 1.0), 0.0), 1.0);
		else s2 = 1.f;*/
		s0 = s1 = s2 = 1.f;

		frontColor[3] = backColor[3] = 1.0;

		glBegin(GL_TRIANGLES);

#if SMOOTHED_RENDERING==1
		for (int j = 0; j < 3; j++) {
			frontColor[j] = (&h_frontColors[phase].x)[j] * s0;
			backColor[j] = (&h_backColors[phase].x)[j] * s0;
		}
		glMaterialfv(GL_FRONT, GL_DIFFUSE, frontColor);
		glMaterialfv(GL_BACK, GL_DIFFUSE, backColor);

		glNormal3f(h_nNorms[ino0 * 3u + 0u], h_nNorms[ino0 * 3u + 1u], h_nNorms[ino0 * 3u + 2u]);
		glVertex3f(a.x, a.y, a.z);

		for (int j = 0; j < 3; j++) {
			frontColor[j] = (&h_frontColors[phase].x)[j] * s1;
			backColor[j] = (&h_backColors[phase].x)[j] * s1;
		}
		glMaterialfv(GL_FRONT, GL_DIFFUSE, frontColor);
		glMaterialfv(GL_BACK, GL_DIFFUSE, backColor);

		glNormal3f(h_nNorms[ino1 * 3u + 0u], h_nNorms[ino1 * 3u + 1u], h_nNorms[ino1 * 3u + 2u]);
		glVertex3f(b.x, b.y, b.z);

		for (int j = 0; j < 3; j++) {
			frontColor[j] = (&h_frontColors[phase].x)[j] * s2;
			backColor[j] = (&h_backColors[phase].x)[j] * s2;
		}
		glMaterialfv(GL_FRONT, GL_DIFFUSE, frontColor);
		glMaterialfv(GL_BACK, GL_DIFFUSE, backColor);

		glNormal3f(h_nNorms[ino2 * 3u + 0u], h_nNorms[ino2 * 3u + 1u], h_nNorms[ino2 * 3u + 2u]);
		glVertex3f(c.x, c.y, c.z);
#else
		glNormal3f(h_fNorms[i * 3u + 0u], h_fNorms[i * 3u + 1u], h_fNorms[i * 3u + 2u]);

		for (int j = 0; j < 3; j++) {
			frontColor[j] = (&h_frontColors[phase].x)[j] * s0;
			backColor[j] = (&h_backColors[phase].x)[j] * s0;
		}
		glMaterialfv(GL_FRONT, GL_DIFFUSE, frontColor);
		glMaterialfv(GL_BACK, GL_DIFFUSE, backColor);

		glVertex3f(a.x, a.y, a.z);

		for (int j = 0; j < 3; j++) {
			frontColor[j] = (&h_frontColors[phase].x)[j] * s1;
			backColor[j] = (&h_backColors[phase].x)[j] * s1;
		}
		glMaterialfv(GL_FRONT, GL_DIFFUSE, frontColor);
		glMaterialfv(GL_BACK, GL_DIFFUSE, backColor);

		glVertex3f(b.x, b.y, b.z);

		for (int j = 0; j < 3; j++) {
			frontColor[j] = (&h_frontColors[phase].x)[j] * s2;
			backColor[j] = (&h_backColors[phase].x)[j] * s2;
		}
		glMaterialfv(GL_FRONT, GL_DIFFUSE, frontColor);
		glMaterialfv(GL_BACK, GL_DIFFUSE, backColor);

		glVertex3f(c.x, c.y, c.z);
#endif

		glEnd();
	}

	glDisable(GL_LIGHTING);
	glDisable(GL_LIGHT0);
	glPointSize(5.f);
	glColor3f(1.f, 0.f, 0.f);
	for (int i = 0; i < h_clickNodes.size(); i++) {
		uint id = h_clickNodes[i]._id;
		if (id == 0xffffffff)
			continue;

		float3 p = make_float3(h_ns[id * 3u + 0u], h_ns[id * 3u + 1u], h_ns[id * 3u + 2u]);
#if 1
		glColor3f(0.f, 0.f, 0.f);
		glPushMatrix();
		glTranslatef(p.x, p.y, p.z);
		glutSolidSphere(0.01f, 10, 10);
		glPopMatrix();
#else
		glBegin(GL_POINTS);
		glVertex3fv(&p.x);
		glEnd();
#endif
	}
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
}
void Cloth::copyToHost(void) {
	MeshObject::copyToHost();
	d_restSolidFractions.copyToHost(h_restSolidFractions);
	d_maxFluidMasses.copyToHost(h_maxFluidMasses);
	d_mfs.copyToHost(h_mfs);
	d_ss.copyToHost(h_ss);
}