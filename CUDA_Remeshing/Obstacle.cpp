#include "Obstacle.h"

void Obstacle::init(void) {
	MeshObject::init();

	_param = new ObjParam();
	_type = TYPE_MESH_OBSTACLE;

	_priTree = new PrimalTree();
}
void Obstacle::reset(void) {
	MeshObject::reset();
	h_pivots = h_pivots0;
	h_degrees = h_degrees0;
	d_pivots = h_pivots0;
	d_degrees = h_degrees0;

	updateElements();
	setParam();
}
void Obstacle::save(void) {
	MeshObject::save();
	h_pivots0 = h_pivots;
	h_degrees0 = h_degrees;
}
void Obstacle::updateElements(void) {
	MeshObject::updateElements();
}
void Obstacle::addObject(
	Mesh* mesh, REAL mass, REAL thickness, REAL friction,
	float4 frontColor, float4 backColor,
	REAL3& pivot, REAL3& rotation, bool isSaved) 
{
	h_pivots.push_back(pivot);
	h_degrees.push_back(rotation);
	d_pivots = h_pivots;
	d_degrees = h_degrees;
	addMesh(mesh, mass, thickness, friction, frontColor, backColor);

	if (isSaved)
		save();

	updateElements();
	/*ObjParam p;
	p._fs = &h_fs[0];
	p._ns = &h_ns[0];
	p._numFaces = _numFaces;
	p._numNodes = _numNodes;
	_priTree->buildTree(p, *_param, h_nbFs, h_fNorms, h_nNorms, 0.1, 7u);*/
	setParam();
}