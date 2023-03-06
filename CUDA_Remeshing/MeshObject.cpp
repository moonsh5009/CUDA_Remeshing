#include "MeshObject.h"

void MeshObject::init(void) {
	_bvh = new BVH();
	_RTri = new RTriangle();

	_numFaces = _numEdges = _numNodes = _numPhases = 0u;
}
void MeshObject::reset(void) {
	h_fs = h_fs0;
	h_ns = h_ns0;
	h_es = h_es0;
	h_vs = h_vs0;
	h_ms = h_ms0;
	h_isFixeds = h_isFixeds0;
	h_nodePhases = h_nodePhases0;
	d_fs = h_fs0;
	d_ns = h_ns0;
	d_es = h_es0;
	d_vs = h_vs0;
	d_ms = h_ms0;
	d_isFixeds = h_isFixeds0;
	d_nodePhases = h_nodePhases0;

	h_frontColors = h_frontColors0;
	h_backColors = h_backColors0;
	h_frictions = h_frictions0;
	h_thicknesses = h_thicknesses0;
	d_frictions = h_frictions0;
	d_thicknesses = h_thicknesses0;

	_numFaces = h_fs.size() / 3u;
	_numEdges = h_es.size() >> 1u;
	_numNodes = h_ns.size() / 3u;
	_numPhases = h_thicknesses.size();
}
void MeshObject::save(void) {
	h_fs0 = h_fs;
	h_es0 = h_es;
	h_ns0 = h_ns;
	h_vs0 = h_vs;
	h_ms0 = h_ms;
	h_isFixeds0 = h_isFixeds;
	h_nodePhases0 = h_nodePhases;

	h_frontColors0 = h_frontColors;
	h_backColors0 = h_backColors;
	h_frictions0 = h_frictions;
	h_thicknesses0 = h_thicknesses;
}
void MeshObject::updateElements(void) {
	initNbXs();
	initNormal();
	initBVH();

	d_forces.resize(_numNodes * 3u);
	d_ms.resize(_numNodes);
	d_invMs.resize(_numNodes);
	d_impulses.resize(_numNodes * 3u);
	d_colWs.resize(_numNodes);
}
void MeshObject::addMesh(
	Mesh* mesh, REAL mass, REAL thickness, REAL friction, float4 frontColor, float4 backColor)
{
	{
		h_fs.insert(h_fs.end(), mesh->_fs.begin(), mesh->_fs.end());
		h_es.insert(h_es.end(), mesh->_es.begin(), mesh->_es.end());
		h_ns.insert(h_ns.end(), mesh->_ns.begin(), mesh->_ns.end());
		for (uint i = _numFaces * 3u; i < h_fs.size(); i++)
			h_fs[i] += _numNodes;
		for (uint i = _numEdges << 1u; i < h_es.size(); i++)
			h_es[i] += _numNodes;

		_numFaces += mesh->_numFaces;
		_numEdges += mesh->_numEdges;
		_numNodes += mesh->_numNodes;
		h_nodePhases.resize(_numNodes, _numPhases++);

		h_vs.resize(_numNodes * 3u, 0.0);
		h_ms.resize(_numNodes, mass);
		h_isFixeds.resize(_numNodes, 0u);

		d_fs = h_fs;
		d_es = h_es;
		d_ns = h_ns;
		d_vs = h_vs;
		d_nodePhases = h_nodePhases;
		d_isFixeds = h_isFixeds;
	}
	{
		h_frontColors.push_back(frontColor);
		h_backColors.push_back(backColor);
		h_frictions.push_back(friction);
		h_thicknesses.push_back(thickness);
		d_frictions = h_frictions;
		d_thicknesses = h_thicknesses;
	}
}

void MeshObject::initNbXs(void) {
	MeshKernel::buildXnbXs(
		d_fs, d_es, d_nbFs, d_nbEs, d_nbNs, 
		d_EnbFs, d_EnbNs, d_FnbEs, 
		_numFaces, _numEdges, _numNodes);
	copyNbToHost();
}
void MeshObject::initNormal(void) {
	d_fNorms.resize(_numFaces * 3u);
	d_nNorms.resize(_numNodes * 3u);
	computeNormal();
}
void MeshObject::initBVH(void) {
	_bvh->build(d_fs, d_ns);
	_RTri->init(d_fs, d_nbFs);
}
void MeshObject::computeNormal(void) {
	MeshKernel::computeNormal(d_fs, d_ns, d_fNorms, d_nNorms, _numFaces, _numNodes);
	copyNormToHost();
}

void MeshObject::draw(void) {
	drawSurface();
#if SMOOTHED_RENDERING == 0
	drawWire();
#endif
}
void MeshObject::drawWire(void)
{
	glPushMatrix();
	glDisable(GL_LIGHTING);
	glColor3d(0, 0, 0);
	
	glBegin(GL_LINES);
	for (int i = 0; i < _numEdges; i++) {
		uint phase = h_nodePhases[h_es[i * 2 + 0]];
		if (h_frontColors[phase].w == 0.0)
			continue;

		for (int j = 0; j < 2; j++) {
			auto x = h_ns[h_es[i * 2 + j] * 3 + 0];
			auto y = h_ns[h_es[i * 2 + j] * 3 + 1];
			auto z = h_ns[h_es[i * 2 + j] * 3 + 2];
			glVertex3f(x, y, z);
		}
	}
	glEnd();

	glEnable(GL_LIGHTING);
	glPopMatrix();
}
void MeshObject::drawSurface(void)
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
		uint ino0 = h_fs[i * 3u + 0u];
		uint ino1 = h_fs[i * 3u + 1u];
		uint ino2 = h_fs[i * 3u + 2u];
		REAL3 a = make_REAL3(h_ns[ino0 * 3u + 0u], h_ns[ino0 * 3u + 1u], h_ns[ino0 * 3u + 2u]);
		REAL3 b = make_REAL3(h_ns[ino1 * 3u + 0u], h_ns[ino1 * 3u + 1u], h_ns[ino1 * 3u + 2u]);
		REAL3 c = make_REAL3(h_ns[ino2 * 3u + 0u], h_ns[ino2 * 3u + 1u], h_ns[ino2 * 3u + 2u]);

		uint phase = h_nodePhases[ino0];
		if (h_frontColors[phase].w == 0.0)
			continue;
		if (phase != prevPhase) {
			glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, &h_frontColors[phase].x);
			glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, &h_backColors[phase].x);
			prevPhase = phase;
		}

		glBegin(GL_TRIANGLES);

#if SMOOTHED_RENDERING
		glNormal3f(h_nNorms[ino0 * 3u + 0u], h_nNorms[ino0 * 3u + 1u], h_nNorms[ino0 * 3u + 2u]);
		glVertex3f(a.x, a.y, a.z);
		glNormal3f(h_nNorms[ino1 * 3u + 0u], h_nNorms[ino1 * 3u + 1u], h_nNorms[ino1 * 3u + 2u]);
		glVertex3f(b.x, b.y, b.z);
		glNormal3f(h_nNorms[ino2 * 3u + 0u], h_nNorms[ino2 * 3u + 1u], h_nNorms[ino2 * 3u + 2u]);
		glVertex3f(c.x, c.y, c.z);
#else
		glNormal3f(h_fNorms[i * 3u + 0u], h_fNorms[i * 3u + 1u], h_fNorms[i * 3u + 2u]);
		glVertex3f(a.x, a.y, a.z);
		glVertex3f(b.x, b.y, b.z);
		glVertex3f(c.x, c.y, c.z);
#endif

		glEnd();
	}
}

void MeshObject::copyToDevice(void) {
	d_ns.copyFromHost(h_ns);
}
void MeshObject::copyToHost(void) {
	d_ns.copyToHost(h_ns);
}
void MeshObject::copyNbToDevice(void) {
	d_nbFs.copyFromHost(h_nbFs);
	d_nbEs.copyFromHost(h_nbEs);
	d_nbNs.copyFromHost(h_nbNs);
	d_EnbFs.copyFromHost(h_EnbFs);
	d_EnbNs.copyFromHost(h_EnbNs);
}
void MeshObject::copyNbToHost(void) {
	d_nbFs.copyToHost(h_nbFs);
	d_nbEs.copyToHost(h_nbEs);
	d_nbNs.copyToHost(h_nbNs);
	d_EnbFs.copyToHost(h_EnbFs);
	d_EnbNs.copyToHost(h_EnbNs);
}
void MeshObject::copyNormToDevice(void) {
	d_fNorms.copyFromHost(h_fNorms);
	d_nNorms.copyFromHost(h_nNorms);
}
void MeshObject::copyNormToHost(void) {
	d_fNorms.copyToHost(h_fNorms);
	d_nNorms.copyToHost(h_nNorms);
}