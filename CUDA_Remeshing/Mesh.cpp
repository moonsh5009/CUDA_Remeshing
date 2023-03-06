#include "Mesh.h"

void Mesh::loadObj(const char* filename, REAL3 center, REAL3 scale) {
	CUDA_CHECK(cudaDeviceSynchronize());
	ctimer timer = CNOW;

	_fs.clear();
	_ns.clear();

	bool flag = true;
	ifstream fin;
	fin.open(filename);
	if (fin.is_open()) {
		while (!fin.eof()) {
			string head;
			fin >> head;
			if (head.length() > 1)
				continue;
			if (head[0] == 'v') {
				REAL3 x;
				fin >> x.x >> x.y >> x.z;
				_ns.push_back(x.x);
				_ns.push_back(x.y);
				_ns.push_back(x.z);
				if (flag) {
					_aabb._min = _aabb._max = x;
					flag = false;
				}
				else addAABB(_aabb, x);
			}
			else if (head[0] == 'f') {
				uint3 x;
				fin >> x.x >> x.y >> x.z;
				_fs.push_back(x.x - 1u);
				_fs.push_back(x.y - 1u);
				_fs.push_back(x.z - 1u);
			}
		}
		fin.close();
	}
	if (_ns.empty() || _fs.empty()) {
		printf("Error : Mesh_init : Object Load Error\n");
		exit(1);
		return;
	}
	_numFaces = _fs.size() / 3u;
	_numNodes = _ns.size() / 3u;
	moveCenter(center, scale);
	buildAdjacency();
	computeNormal();

	CUDA_CHECK(cudaDeviceSynchronize());
	printf("Num of Faces: %d, Num of Vertices: %d, %f ms\n", _numFaces, _numNodes, (CNOW - timer) / 10000.);
}
void Mesh::loadObj(const char* filename, REAL3 center, REAL scale) {
	CUDA_CHECK(cudaDeviceSynchronize());
	ctimer timer = CNOW;

	_fs.clear();
	_ns.clear();

	bool flag = true;
	ifstream fin;
	fin.open(filename);
	if (fin.is_open()) {
		while (!fin.eof()) {
			string head;
			fin >> head;
			if (head.length() > 1)
				continue;
			if (head[0] == 'v') {
				REAL3 x;
				fin >> x.x >> x.y >> x.z;
				_ns.push_back(x.x);
				_ns.push_back(x.y);
				_ns.push_back(x.z);
				if (flag) {
					_aabb._min = _aabb._max = x;
					flag = false;
				}
				else addAABB(_aabb, x);
			}
			else if (head[0] == 'f') {
				uint3 x;
				fin >> x.x >> x.y >> x.z;
				_fs.push_back(x.x - 1u);
				_fs.push_back(x.y - 1u);
				_fs.push_back(x.z - 1u);
			}
		}
		fin.close();
	}
	if (_ns.empty() || _fs.empty()) {
		printf("Error : Mesh_init : Object Load Error\n");
		exit(1);
		return;
	}
	_numFaces = _fs.size() / 3u;
	_numNodes = _ns.size() / 3u;
	moveCenter(center, scale);
	buildAdjacency();
	computeNormal();

	CUDA_CHECK(cudaDeviceSynchronize());
	printf("Num of Faces: %d, Num of Vertices: %d, %f ms\n", _numFaces, _numNodes, (CNOW - timer) / 10000.);
}
void Mesh::loadObj(const char* filename) {
	CUDA_CHECK(cudaDeviceSynchronize());
	ctimer timer = CNOW;

	_fs.clear();
	_ns.clear();

	bool flag = true;
	ifstream fin;
	fin.open(filename);
	if (fin.is_open()) {
		while (!fin.eof()) {
			string head;
			fin >> head;
			if (head.length() > 1)
				continue;
			if (head[0] == 'v') {
				REAL3 x;
				fin >> x.x >> x.y >> x.z;
				_ns.push_back(x.x);
				_ns.push_back(x.y);
				_ns.push_back(x.z);
				if (flag) {
					_aabb._min = _aabb._max = x;
					flag = false;
				}
				else addAABB(_aabb, x);
			}
			else if (head[0] == 'f') {
				uint3 x;
				fin >> x.x >> x.y >> x.z;
				_fs.push_back(x.x - 1u);
				_fs.push_back(x.y - 1u);
				_fs.push_back(x.z - 1u);
			}
		}
		fin.close();
	}
	if (_ns.empty() || _fs.empty()) {
		printf("Error : Mesh_init : Object Load Error\n");
		exit(1);
		return;
	}
	_numFaces = _fs.size() / 3u;
	_numNodes = _ns.size() / 3u;
	buildAdjacency();
	computeNormal();

	CUDA_CHECK(cudaDeviceSynchronize());
	printf("Num of Faces: %d, Num of Vertices: %d, %f ms\n", _numFaces, _numNodes, (CNOW - timer) / 10000.);
}

void Mesh::moveCenter(REAL3 center, REAL3 scale) {
	REAL3 size = _aabb._max - _aabb._min;
	REAL3 max_length = make_REAL3(scale.x / size.x, scale.y / size.y, scale.z / size.z);

	REAL3 prevCenter = (_aabb._min + _aabb._max) * (REAL)0.5;

	bool flag = false;
	uint vlen = _ns.size();
	for (uint i = 0u; i < vlen; i += 3u) {
		REAL3 pos = make_REAL3(_ns[i], _ns[i + 1u], _ns[i + 2u]);
		REAL3 grad = pos - prevCenter;
		grad *= max_length;
		pos = center + grad;
		_ns[i] = pos.x;
		_ns[i + 1u] = pos.y;
		_ns[i + 2u] = pos.z;
		if (flag) addAABB(_aabb, pos);
		else {
			_aabb._min = _aabb._max = pos;
			flag = true;
		}
	}
}
void Mesh::moveCenter(REAL3 center, REAL scale) {
	REAL3 size = _aabb._max - _aabb._min;
	REAL max_length = size.x;
	if (max_length < size.y)
		max_length = size.y;
	if (max_length < size.z)
		max_length = size.z;
	max_length = 2.0 * scale / max_length;

	REAL3 prevCenter = (_aabb._min + _aabb._max) * (REAL)0.5;

	bool flag = false;
	uint vlen = _ns.size();
	for (uint i = 0u; i < vlen; i += 3u) {
		REAL3 pos = make_REAL3(_ns[i], _ns[i + 1u], _ns[i + 2u]);
		REAL3 grad = pos - prevCenter;
		grad *= max_length;
		pos = center + grad;
		_ns[i] = pos.x;
		_ns[i + 1u] = pos.y;
		_ns[i + 2u] = pos.z;
		if (flag) addAABB(_aabb, pos);
		else {
			_aabb._min = _aabb._max = pos;
			flag = true;
		}
	}
}
void Mesh::buildAdjacency(void)
{
	vector<set<uint>> nbFs(_numNodes);
	vector<set<uint>> nbEs(_numNodes);
	vector<set<uint>> nbNs(_numNodes);
	_es.clear();
	_EnbFs.clear();
	_EnbNs.clear();

	for (uint i = 0u; i < _numFaces; i++) {
		uint ino = i * 3u;
		uint ino0 = _fs[ino + 0u];
		uint ino1 = _fs[ino + 1u];
		uint ino2 = _fs[ino + 2u];
		nbFs[ino0].insert(i);
		nbFs[ino1].insert(i);
		nbFs[ino2].insert(i);
		nbNs[ino0].insert(ino1);
		nbNs[ino0].insert(ino2);
		nbNs[ino1].insert(ino2);
		nbNs[ino1].insert(ino0);
		nbNs[ino2].insert(ino0);
		nbNs[ino2].insert(ino1);
	}
	_nbFs.init(nbFs);
	_nbNs.init(nbNs);

	for (uint i = 0u; i < _numNodes; i++) {
		for (auto inbv : nbNs[i]) {
			if (i < inbv) {
				_es.push_back(i);
				_es.push_back(inbv);
			}
		}
	}
	_numEdges = _es.size() >> 1u;

	for (uint i = 0u; i < _numEdges; i++) {
		uint ino = i << 1u;
		uint ino0 = _es[ino + 0u];
		uint ino1 = _es[ino + 1u];
		nbEs[ino0].insert(i);
		nbEs[ino1].insert(i);
	}
	_nbEs.init(nbEs);

	for (uint i = 0u; i < _es.size(); i += 2) {
		uint ino0 = _es[i + 0u];
		uint ino1 = _es[i + 1u];

		uint EnbF[2] = { 0xffffffff, 0xffffffff };
		uint EnbN[2] = { 0xffffffff, 0xffffffff };
		uint num = 0u;
		for (auto nbf0 : nbFs[ino0]) {
			for (auto nbf1 : nbFs[ino1]) {
				if (nbf0 == nbf1) {
					EnbF[num] = nbf0;

					for (uint j = 0u; j < 3u; j++) {
						uint ino = _fs[nbf0 * 3u + j];
						if (ino != ino0 && ino != ino1) {
							EnbN[num] = ino;
							break;
						}
					}
					num++;
					break;
				}
			}
			if (num == 2u) break;
		}
		_EnbFs.push_back(EnbF[0]);
		_EnbFs.push_back(EnbF[1]);
		_EnbNs.push_back(EnbN[0]);
		_EnbNs.push_back(EnbN[1]);
	}
}
void Mesh::computeNormal(void)
{
	_fnorms.resize(_numFaces * 3u);
	_vnorms.clear();
	_vnorms.resize(_numNodes * 3u, 0.0);

	for (uint i = 0u; i < _numFaces; i++) {
		uint ino = i * 3u;
		uint ino0 = _fs[ino + 0u] * 3u;
		uint ino1 = _fs[ino + 1u] * 3u;
		uint ino2 = _fs[ino + 2u] * 3u;
		REAL3 a = make_REAL3(_ns[ino0 + 0u], _ns[ino0 + 1u], _ns[ino0 + 2u]);
		REAL3 b = make_REAL3(_ns[ino1 + 0u], _ns[ino1 + 1u], _ns[ino1 + 2u]);
		REAL3 c = make_REAL3(_ns[ino2 + 0u], _ns[ino2 + 1u], _ns[ino2 + 2u]);
		REAL3 norm = Cross(a - b, a - c);
		Normalize(norm);
		_fnorms[ino + 0u] = norm.x;
		_fnorms[ino + 1u] = norm.y;
		_fnorms[ino + 2u] = norm.z;
		REAL radian = AngleBetweenVectors(a - b, a - c);
		_vnorms[ino0 + 0u] += norm.x * radian;
		_vnorms[ino0 + 1u] += norm.y * radian;
		_vnorms[ino0 + 2u] += norm.z * radian;
		radian = AngleBetweenVectors(b - a, b - c);
		_vnorms[ino1 + 0u] += norm.x * radian;
		_vnorms[ino1 + 1u] += norm.y * radian;
		_vnorms[ino1 + 2u] += norm.z * radian;
		radian = AngleBetweenVectors(c - a, c - b);
		_vnorms[ino2 + 0u] += norm.x * radian;
		_vnorms[ino2 + 1u] += norm.y * radian;
		_vnorms[ino2 + 2u] += norm.z * radian;
	}

	for (uint i = 0u; i < _numNodes; i++) {
		uint ino = i * 3u;
		REAL3 norm = make_REAL3(_vnorms[ino + 0u], _vnorms[ino + 1u], _vnorms[ino + 2u]);
		Normalize(norm);
		_vnorms[ino + 0u] = norm.x;
		_vnorms[ino + 1u] = norm.y;
		_vnorms[ino + 2u] = norm.z;
	}
}
void Mesh::rotate(REAL3 degree)
{
	degree.x *= M_PI * 0.00555555555555555555555555555556;
	degree.y *= M_PI * 0.00555555555555555555555555555556;
	degree.z *= M_PI * 0.00555555555555555555555555555556;

	REAL cx = cos(degree.x);
	REAL sx = sin(degree.x);
	REAL cy = cos(degree.y);
	REAL sy = -sin(degree.y);
	REAL cz = cos(degree.z);
	REAL sz = sin(degree.z);
	REAL3 center = (_aabb._min + _aabb._max) * (REAL)0.5;

	REAL3 n, pn;
	for (uint i = 0u; i < _ns.size(); i += 3u) {
		n.x = _ns[i + 0u];
		n.y = _ns[i + 1u];
		n.z = _ns[i + 2u];
		n -= center;

		pn.x = n.x * cz * cy + n.y * (cz * sy * sx - sz * cx) + n.z * (cz * sy * cx + sz * sx);
		pn.y = n.x * sz * cy + n.y * (sz * sy * sx + cz * cx) + n.z * (sz * sy * cx - cz * sx);
		pn.z = n.x * -sy + n.y * cy * sx + n.z * cy * cx;

		pn += center;

		_ns[i + 0u] = pn.x;
		_ns[i + 1u] = pn.y;
		_ns[i + 2u] = pn.z;
	}

	computeNormal();
}