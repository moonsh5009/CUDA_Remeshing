#include <Windows.h>
#include <stdio.h>

#include "System.h"

int _stride = 1;
int _frame = 0;
int _width = WINDOW_WIDTH;
int _height = WINDOW_HEIGHT;

float _zoom = -2.5;
float _tx = 0;
float _ty = 0;
float _rotx = 0;
float _roty = 0;
int _lastx = 0;
int _lasty = 0;

float _fovy = 45.0f;
float _zNear = 0.1f;
float _zFar = 100.f;

unsigned char _buttons[3] = { 0 };
bool _simulation = false;
char _FPS_str[100];

int _rayx = 0;
int _rayy = 0;
M_Ray _ray;
bool _rayCasting = false;

vector<int> _cutxs;
vector<int> _cutys;
uint _numCuts = 0u;
bool _tearing = false;

float4 purple = make_float4(0.0f, 0.44705882352941176470588235294118f, 0.66666666666666666666666666666667f, 1.0f);
float4 blue = make_float4(0.0f, 0.44705882352941176470588235294118f, 0.66666666666666666666666666666667f, 1.0f);
float4 pink = make_float4(1.0f, 180.f / 255.f, 210.f / 255.f, 1.0f);
float4 yellow = make_float4(1.0f, 1.0f, 0.8f, 1.0f);
float4 gray = make_float4(0.2f, 0.2f, 0.2f, 1.0f);
float4 white = make_float4(1.0f, 1.0f, 1.0f, 1.0f);

System* _system = nullptr;
Mesh* _mesh = nullptr;

#define SCREEN_CAPTURE
//#define SAVE_SIMULATION

M_Ray getRay(int x, int y) {
	M_Ray ray;

	float rotX = _rotx * M_PI / 180.f;
	float rotY = _roty * M_PI / 180.f;

	float cosX = cosf(rotX);
	float sinX = -sinf(rotX);
	float cosY = cosf(rotY);
	float sinY = -sinf(rotY);

	float tx = -_tx;
	float ty = -_ty;
	float tz = -_zoom;

	ray._pos.x = tx * cosY + (ty * sinX + tz * cosX) * sinY;
	ray._pos.y = ty * cosX - tz * sinX;
	ray._pos.z = -tx * sinY + (ty * sinX + tz * cosX) * cosY;

	/*float rx = _zNear / cosf(_fovy * M_PI / 360.f * (float)_width / (float)_height);
	float ry = _zNear / cosf(_fovy * M_PI / 360.f);
	float wx = (float)x / (float)_width * 2.0 - 1.0f;
	float wy = (float)y / (float)_height * 2.0 - 1.0f;
	float cosX2 = _zNear / ((ry - _zNear) * fabsf(wy) + _zNear);
	float cosY2 = _zNear / ((rx - _zNear) * fabsf(wx) + _zNear);
	float sinX2 = sqrtf(max(1.f - cosX2 * cosX2, 0.0));
	float sinY2 = sqrtf(max(1.f - cosY2 * cosY2, 0.0));
	if (wy > 0.f) sinX2 = -sinX2;
	if (wx > 0.f) sinY2 = -sinY2;*/
	float wx = ((float)x / (float)_width * 2.0 - 1.0f) * _fovy * M_PI / 360.f * (float)_width / (float)_height;
	float wy = ((float)y / (float)_height * 2.0 - 1.0f) * _fovy * M_PI / 360.f;
	float cosX2 = cosf(wy);
	float sinX2 = -sinf(wy);
	float cosY2 = cosf(wx);
	float sinY2 = -sinf(wx);

	tx = -cosX2 * sinY2;
	ty = sinX2;
	tz = -cosX2 * cosY2;

	ray._dir.x = tx * cosY + (ty * sinX + tz * cosX) * sinY;
	ray._dir.y = ty * cosX - tz * sinX;
	ray._dir.z = -tx * sinY + (ty * sinX + tz * cosX) * cosY;

	Normalize(ray._dir);
	return ray;
}
void DrawRay(const M_Ray& ray) {
	float3 p0 = ray._pos + ray._dir * 0.5f;
	float3 p1 = ray._pos + ray._dir * 4.f;

	glLineWidth(2.f);
	glPointSize(5.f);

	glBegin(GL_LINES);
	glVertex3fv(&p0.x);
	glVertex3fv(&p1.x);
	glEnd();

	glColor3f(1.f, 0.f, 0.f);
	glPushMatrix();
	glTranslatef(p0.x, p0.y, p0.z);
	glutSolidSphere(0.01, 10, 10);
	glPopMatrix();

	glColor3f(0.f, 0.f, 1.f);
	glPushMatrix();
	glTranslatef(p1.x, p1.y, p1.z);
	glutSolidSphere(0.01, 10, 10);
	glPopMatrix();
}
void DrawCutLine(void)
{
	glColor3f(1, 0, 0);
	glDisable(GL_DEPTH_TEST);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0.0, (double)_width, 0.0, (double)_height, -1.0, 1.0);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	glLineWidth(5.0);
	glBegin(GL_LINES);
	for (int i = 0; i < _cutxs.size(); i += 2) {
		glVertex2d(_cutxs[i], _height - _cutys[i]);
		glVertex2d(_cutxs[i + 1], _height - _cutys[i + 1]);
	}
	glEnd();
	glLineWidth(1.0);

	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glEnable(GL_DEPTH_TEST);
}
void compCutting(void) {
	if (!_numCuts)
		return;
	REAL3 n0, n1;
	M_Ray ray0, ray1;
	CutPlane plane;
	plane._v = make_REAL3(0.0, 0.0, 1.0);
	plane._vMax = FLT_MAX;
	plane._vMin = -FLT_MAX;
	for (int i = 0; i < _cutxs.size(); i += 2) {
		ray0 = getRay(_cutxs[i], _cutys[i]);
		ray1 = getRay(_cutxs[i + 1], _cutys[i + 1]);
		n0.x = (REAL)(ray0._pos.x + ray0._dir.x * (-ray0._pos.z / ray0._dir.z));
		n0.y = (REAL)(ray0._pos.y + ray0._dir.y * (-ray0._pos.z / ray0._dir.z));
		n0.z = (REAL)(ray0._pos.z + ray0._dir.z * (-ray0._pos.z / ray0._dir.z));
		n1.x = (REAL)(ray1._pos.x + ray1._dir.x * (-ray1._pos.z / ray1._dir.z));
		n1.y = (REAL)(ray1._pos.y + ray1._dir.y * (-ray1._pos.z / ray1._dir.z));
		n1.z = (REAL)(ray1._pos.z + ray1._dir.z * (-ray1._pos.z / ray1._dir.z));

		plane._u = n1 - n0;
		Normalize(plane._u);
		plane._u.z = 0.0;
		plane._uMin = min(Dot(plane._u, n0), Dot(plane._u, n1));
		plane._uMax = max(Dot(plane._u, n0), Dot(plane._u, n1));
		plane._norm = make_REAL3(-plane._u.y, plane._u.x, 0.0);
		plane._pos = Dot(plane._norm, n0);

		_system->h_cutPlanes.push_back(plane);
	}
	_cutxs.clear();
	_cutys.clear();
	_numCuts = 0u;
	_system->cutting();
}

void DrawText(float x, float y, const char* text, void* font = NULL)
{
	glColor3f(0, 0, 0);
	glDisable(GL_DEPTH_TEST);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0.0, (double)_width, 0.0, (double)_height, -1.0, 1.0);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	if (font == NULL) {
		font = GLUT_BITMAP_9_BY_15;
	}

	size_t len = strlen(text);

	glRasterPos2f(x, y);
	for (const char* letter = text; letter < text + len; letter++) {
		if (*letter == '\n') {
			y -= 12.0f;
			glRasterPos2f(x, y);
		}
		glutBitmapCharacter(font, *letter);
	}

	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glEnable(GL_DEPTH_TEST);
}
tuple<Mesh*, Mesh*> LoadClothAndAvatar(char* cloth_file, char* avatar_file, double scale)
{
	Mesh* cloth, * avatar;
	cloth = new Mesh(cloth_file);
	avatar = new Mesh(avatar_file);

	AABB aabb;
	setAABB(aabb, cloth->_aabb);
	addAABB(aabb, avatar->_aabb);

	REAL3 size = aabb._max - aabb._min;
	REAL max_length = size.x;
	if (max_length < size.y)
		max_length = size.y;
	if (max_length < size.z)
		max_length = size.z;
	max_length = 2.0 * scale / max_length;

	REAL3 prevCenter = (aabb._min + aabb._max) * (REAL)0.5;
	REAL3 center = make_REAL3(0.0, 0.0, 0.0);

	bool flag = false;
	uint vlen = cloth->_ns.size();
	for (uint i = 0u; i < vlen; i += 3u) {
		REAL3 pos = make_REAL3(cloth->_ns[i], cloth->_ns[i + 1u], cloth->_ns[i + 2u]);
		REAL3 grad = pos - prevCenter;
		grad *= max_length;
		pos = center + grad;
		cloth->_ns[i] = pos.x;
		cloth->_ns[i + 1u] = pos.y;
		cloth->_ns[i + 2u] = pos.z;
		if (flag) addAABB(aabb, pos);
		else {
			aabb._min = aabb._max = pos;
			flag = true;
		}
	}
	vlen = avatar->_ns.size();
	flag = false;
	for (uint i = 0u; i < vlen; i += 3u) {
		REAL3 pos = make_REAL3(avatar->_ns[i], avatar->_ns[i + 1u], avatar->_ns[i + 2u]);
		REAL3 grad = pos - prevCenter;
		grad *= max_length;
		pos = center + grad;
		avatar->_ns[i] = pos.x;
		avatar->_ns[i + 1u] = pos.y;
		avatar->_ns[i + 2u] = pos.z;
		if (flag) addAABB(aabb, pos);
		else {
			aabb._min = aabb._max = pos;
			flag = true;
		}
	}

	return make_tuple(cloth, avatar);
}

void SetView(double zoom, double tx, double ty, double rotx, double roty)
{
	_zoom = zoom;
	_tx = tx;
	_ty = ty;
	_rotx = rotx;
	_roty = roty;
}
void SetView(void)
{
	SetView(-3.050000, 0.000000, 0.000000, 0.000000, 0.000000);
	//SetView(-2.900000, 0.000000, -0.000000, 20.500000, 0.000000);
}
void Init(void)
{
	// Cut lines
	/*{
		int2 a = make_int2(400, 300);
		REAL r = 0.0;
		int n = 8;
		_tearing = true;
		for (int i = 0; i < n; i++) {
			int2 b = make_int2((int)(cos(r) * 180.0), (int)(sin(r) * 180.0));
			printf("%d, %d\n", b.x, b.y);
			_cutxs.push_back(a.x + b.x);
			_cutxs.push_back(a.x - b.x);
			_cutys.push_back(a.y + b.y);
			_cutys.push_back(a.y - b.y);
			_numCuts++;
			r += M_PI / (REAL)n;
		}
	}*/

	glEnable(GL_DEPTH_TEST);

	//_system = new System(make_REAL3(0.0, -0.098, 0.0), 0.005);
	//_system = new System(make_REAL3(0.0, -0.0, 0.0), 0.005);
	_system = new System(make_REAL3(0.0, -9.8, 0.0), 0.005);
	//_system->_thickness = clothRadius;

	REAL clothFriction = 0.3;
	REAL obstacleFriction = 0.3;
	REAL boundaryFriction = 0.3;

	REAL sphDensity = 1000.0;
	REAL clothDensity = 800.0;
	REAL clothSolidFraction = 0.4;

	REAL sphViscosity = 0.01;
	REAL clothViscosity = 0.01;
	REAL obstacleViscosity = 0.01;
	REAL boundaryViscosity = 0.01;

	REAL obstacleMass = 1.0;

	REAL boundaryRadius = 0.02;

#if QUALITY==0
	REAL clothRadius = 0.0012;
	REAL obstacleRadius = 0.0012;
#elif QUALITY==1
	REAL clothRadius = 0.0012;
	REAL obstacleRadius = 0.0012;
#else
	REAL clothRadius = 0.0012;
	REAL obstacleRadius = 0.0012;
#endif

#if QUALITY==0
	_system->_subStep = 1u;
#elif QUALITY==1
	_system->_subStep = 1u;
#else
	_system->_subStep = 1u;
#endif

	REAL clothSurfaceTension = 4.2;
	REAL obstacleSurfaceTension = 4.2;
	REAL boundarySurfaceTension = 4.2;

	_mesh = new Mesh("../obj/cube.obj", make_REAL3(0.0), _system->_boundary._max - _system->_boundary._min);
	_system->addObstacle(_mesh, 1.0, boundaryFriction, make_REAL3(0.0), make_REAL3(0.0),
		boundaryRadius, boundaryViscosity, boundarySurfaceTension,
		make_float4(1.0f, 1.0f, 1.0f, 0.0f), make_float4(1.0f, 1.0f, 1.0f, 0.0f));

#if QUALITY==0
	//_mesh = new Mesh("../obj/LR_cloth.obj", make_REAL3(0.0, -1.4, 0.0));
	_mesh = new Mesh("../obj/LR_cloth.obj", make_REAL3(0.0, 0.0, 0.0));
#elif QUALITY==1
	//_mesh = new Mesh("../obj/MR_cloth.obj", make_REAL3(0.0, -1.4, 0.0));
	_mesh = new Mesh("../obj/MR_cloth.obj", make_REAL3(0.0, 0.0, 0.0));
#else
	_mesh = new Mesh("../obj/HR_cloth.obj", make_REAL3(0.0, 0.0, 0.0));
#endif

	//_mesh->rotate(make_REAL3(-90.0, 0.0, 0.0));
	_system->addCloth(_mesh, clothFriction,
		clothRadius, clothDensity, sphDensity,
		clothSolidFraction, clothViscosity, clothSurfaceTension,
		pink, yellow);

	/*_mesh = new Mesh("../obj/bunny.obj", make_REAL3(0.0, -1.2, 0.0), 0.48);
	_system->addObstacle(_mesh, obstacleMass, obstacleFriction,
		make_REAL3(0.0, 0.0, 0.0), make_REAL3(0.0, 0.0, 0.0),
		obstacleRadius, obstacleViscosity, obstacleSurfaceTension, gray, gray);*/

}

void FPS(void)
{
	static float framesPerSecond = 0.0f;
	static float lastTime = 0.0f;
	float currentTime = GetTickCount() * 0.001f;
	++framesPerSecond;
	if (currentTime - lastTime > 1.0f) {
		lastTime = currentTime;
		sprintf(_FPS_str, "FPS : %d", (int)framesPerSecond);
		framesPerSecond = 0;
	}
}
void Darw(void)
{
	glutReshapeWindow(_width, _height);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glShadeModel(GL_SMOOTH);
	char text[100];

	if (_system)
		_system->draw();

	glDisable(GL_LIGHTING);
	DrawCutLine();

	if (_system) {
		sprintf(text, "Number of triangles : %d", _system->numFaces());
		DrawText(10.0f, 560.0f, text);
	}
	DrawText(10.0f, 520.0f, _FPS_str);
	sprintf(text, "Frame : %d", _frame);
	DrawText(10.0f, 500.0f, text);
}
void Capture(int endFrame)
{
#ifdef SCREEN_CAPTURE
	if (_frame == 0 || _frame % _stride == 0) {
		static int index = 0;
		char filename[100];
		sprintf_s(filename, "../capture/capture-%d.bmp", index);
		BITMAPFILEHEADER bf;
		BITMAPINFOHEADER bi;
		unsigned char* image = (unsigned char*)malloc(sizeof(unsigned char) * _width * _height * 3);
		FILE* file;
		fopen_s(&file, filename, "wb");
		if (image != NULL) {
			if (file != NULL) {
				glReadPixels(0, 0, _width, _height, 0x80E0, GL_UNSIGNED_BYTE, image);
				memset(&bf, 0, sizeof(bf));
				memset(&bi, 0, sizeof(bi));
				bf.bfType = 'MB';
				bf.bfSize = sizeof(bf) + sizeof(bi) + _width * _height * 3;
				bf.bfOffBits = sizeof(bf) + sizeof(bi);
				bi.biSize = sizeof(bi);
				bi.biWidth = _width;
				bi.biHeight = _height;
				bi.biPlanes = 1;
				bi.biBitCount = 24;
				bi.biSizeImage = _width * _height * 3;
				fwrite(&bf, sizeof(bf), 1, file);
				fwrite(&bi, sizeof(bi), 1, file);
				fwrite(image, sizeof(unsigned char), _height * _width * 3, file);
				fclose(file);
			}
			free(image);
		}
		if (index == endFrame) {
			exit(0);
		}
		index++;
	}
#endif
}
void SaveSimulation(int endFrame)
{
#ifdef SAVE_SIMULATION
	if (_frame == 0 || _frame % _stride == 0) {
		static int index = 0;
		char text[100];
		sprintf_s(text, "simulation\\scene%d\\scene%d-%d.txt", SCENE, SCENE, index);

		ofstream fout(text);
		if (fout.fail())
		{
			std::cerr << "Error!" << std::endl;
		}
		else {
			uint numNodes = 0u;
			for (uint i = 0u; i < _system->_cloths->_numFaces; i++) {
				uint ino0 = _system->_cloths->h_fs[i * 3u + 0u];
				uint ino1 = _system->_cloths->h_fs[i * 3u + 1u];
				uint ino2 = _system->_cloths->h_fs[i * 3u + 2u];
				sprintf(text, "f %d %d %d", ino0 + 1u, ino1 + 1u, ino2 + 1u);
				fout << text << endl;
			}
			for (uint i = 0u; i < _system->_obstacles->_numFaces; i++) {
				uint ino0 = _system->_obstacles->h_fs[i * 3u + 0u] + _system->_cloths->_numNodes;
				uint ino1 = _system->_obstacles->h_fs[i * 3u + 1u] + _system->_cloths->_numNodes;
				uint ino2 = _system->_obstacles->h_fs[i * 3u + 2u] + _system->_cloths->_numNodes;
				sprintf(text, "f %d %d %d", ino0 + 1u, ino1 + 1u, ino2 + 1u);
				fout << text << endl;
			}
			for (uint i = 0u; i < _system->_cloths->_numNodes; i++) {
				float x = _system->_cloths->h_ns[i * 3u + 0u];
				float y = _system->_cloths->h_ns[i * 3u + 1u];
				float z = _system->_cloths->h_ns[i * 3u + 2u];

				uint phase = _system->_cloths->h_nodePhases[i];

				float frontColor[4];
				float backColor[4];

				float s = powf(1.f - min((1.0 - _system->_cloths->h_restSolidFractions[i]) * _system->_cloths->h_ss[i], 1.f), 1.0f);

				for (int j = 0; j < 3; j++) {
					frontColor[j] = (&_system->_cloths->h_frontColors[phase].x)[j] * s;
					backColor[j] = (&_system->_cloths->h_backColors[phase].x)[j] * s;
				}
				frontColor[3] = _system->_cloths->h_frontColors[phase].w;
				backColor[3] = _system->_cloths->h_backColors[phase].w;

				sprintf(text, "v %f %f %f %f %f %f %f %f %f %f %f", x, y, z,
					frontColor[0], frontColor[1], frontColor[2], frontColor[3],
					backColor[0], backColor[1], backColor[2], backColor[3]);
				fout << text << endl;
			}
			for (uint i = 0u; i < _system->_obstacles->_numNodes; i++) {
				float x = _system->_obstacles->h_ns[i * 3u + 0u];
				float y = _system->_obstacles->h_ns[i * 3u + 1u];
				float z = _system->_obstacles->h_ns[i * 3u + 2u];

				uint phase = _system->_obstacles->h_nodePhases[i];

				float frontColor[4];
				float backColor[4];

				for (int j = 0; j < 4; j++) {
					frontColor[j] = (&_system->_obstacles->h_frontColors[phase].x)[j];
					backColor[j] = (&_system->_obstacles->h_backColors[phase].x)[j];
				}

				sprintf(text, "v %f %f %f %f %f %f %f %f %f %f %f", x, y, z,
					frontColor[0], frontColor[1], frontColor[2], frontColor[3],
					backColor[0], backColor[1], backColor[2], backColor[3]);
				fout << text << endl;
			}
			for (uint i = 0u; i < _system->_sphParticles->_numParticles; i++) {
				float x = _system->_sphParticles->h_xs[i * 3u + 0u];
				float y = _system->_sphParticles->h_xs[i * 3u + 1u];
				float z = _system->_sphParticles->h_xs[i * 3u + 2u];

				uint phase = _system->_sphParticles->h_phases[i];
				float radius = _system->_sphParticles->h_radii[phase];
				radius *= S3TO1(_system->_sphParticles->h_ss[i]);

				sprintf(text, "p %f %f %f %f", x, y, z, radius);
				fout << text << endl;
			}
		}
		fout.close();
		if (index == endFrame) {
			exit(0);
		}
		index++;
	}
#endif
}

void Update(void)
{
	if (_simulation) {
		if (_rayCasting)
			_system->moveNode(_ray);
		if (_tearing) {
			compCutting();
		}
#if SCENE==0

#if 0
		uint rotateFrame = 1000u;
		uint idleFrame = 50u;
		uint totalFrame = (rotateFrame * 2u + idleFrame) / _stride;
		REAL move = 0.3 / (REAL)rotateFrame;
		REAL3 degree = make_REAL3(4.0, 0.0, 0.0);
		_system->_cloths->rotateFixed(degree, degree * -1.0, move, -move, _system->_invdt);
		if (_frame < rotateFrame) {
			_system->_cloths->rotateFixed(degree, degree * -1.0, move, -move, _system->_invdt);
		}
		else if (_frame < rotateFrame * 2u) {
			_system->_cloths->rotateFixed(degree * -1.0, degree, -move, move, _system->_invdt);
		}
		else {
			_system->_cloths->moveFixed(make_REAL3(0.0));
		}
#else
		uint totalFrame = 5000;
		//_system->_cloths->moveFixed(make_REAL3(-1.4, 0.0, 0.0), make_REAL3(1.4, 0.0, 0.0));
#endif

		SaveSimulation(totalFrame);
		Capture(totalFrame);

		if (_system)
			_system->simulation();
#elif SCENE==1
		SaveSimulation(700 / _stride);
		Capture(700 / _stride);
		if (_system)
			_system->simulation();
#elif SCENE==2
		SaveSimulation(1600 / _stride);
		Capture(1600 / _stride);
		if (_frame == 300)
			_system->spawnSPHParticle(1);

		if (_system)
			_system->simulation();
#elif SCENE==3
		SaveSimulation(400 / _stride);
		Capture(400 / _stride);
		if (_system)
			_system->simulation();
#elif SCENE==4
		if (_system) {
			/*uint idleFrame = 0;
			uint moveFrame = 0;
			uint wetFrame = 0;
			uint rotateFrame = 800u;*/
			uint idleFrame = 50u;
			uint moveFrame = 300u;
			uint wetFrame = 200u;
			uint rotateFrame = 800u;
			uint rotateFrame2 = 200u;
			REAL3 moveVel = make_REAL3(0.0, -(_system->_boundary._max.y - _system->_boundary._min.y) * 0.4 / (REAL)moveFrame * _system->_invdt, 0.0);
			REAL3 degree = make_REAL3(540.0 / (REAL)rotateFrame, 0.0, 0.0);
			REAL move = 0.2 / (REAL)rotateFrame;

			SaveSimulation((idleFrame * 3u + moveFrame * 2u + wetFrame * 3u + rotateFrame2 + rotateFrame * 2u) / _stride);
			Capture((idleFrame * 3u + moveFrame * 2u + wetFrame * 3u + rotateFrame2 + rotateFrame * 2u) / _stride);

			if (_frame < idleFrame) {
			}
			else if (_frame < idleFrame + moveFrame) {
				_system->_cloths->moveFixed(moveVel);
			}
			else if (_frame < idleFrame + moveFrame + wetFrame) {
				_system->_cloths->moveFixed(make_REAL3(0.0));
			}
			else if (_frame < idleFrame + moveFrame * 2u + wetFrame) {
				_system->_cloths->moveFixed(moveVel * -1.0);
			}
			else if (_frame < idleFrame * 2u + moveFrame * 2u + wetFrame) {
				_system->_cloths->moveFixed(make_REAL3(0.0));
			}
			else if (_frame < idleFrame * 2u + moveFrame * 2u + wetFrame + rotateFrame2) {
				_system->_cloths->rotateFixed(
					make_REAL3(90.0 / (REAL)rotateFrame2, 0.0, 0.0), make_REAL3(90.0 / (REAL)rotateFrame2, 0.0, 0.0),
					0.0, 0.0, _system->_invdt);
			}
			else if (_frame < idleFrame * 3u + moveFrame * 2u + wetFrame + rotateFrame2) {
				_system->_cloths->moveFixed(make_REAL3(0.0));
			}
			else if (_frame < idleFrame * 3u + moveFrame * 2u + wetFrame + rotateFrame2 + rotateFrame) {
				_system->_cloths->rotateFixed(degree, degree * -1.0, move, -move, _system->_invdt);
			}
			else if (_frame < idleFrame * 3u + moveFrame * 2u + wetFrame * 2u + rotateFrame2 + rotateFrame) {
				_system->_cloths->moveFixed(make_REAL3(0.0));
			}
			else if (_frame < idleFrame * 3u + moveFrame * 2u + wetFrame * 2u + rotateFrame2 + rotateFrame * 2u) {
				_system->_cloths->rotateFixed(degree * -1.0, degree, -move, move, _system->_invdt);
			}
			else {
				_system->_cloths->moveFixed(make_REAL3(0.0));
			}

			_system->simulation();
		}
#endif
		_frame++;
	}
	::glutPostRedisplay();
}

void Display(void)
{
	glClearColor(0.8980392156862745f, 0.9490196078431373f, 1.0f, 1.0f);
	//glClearColor(0.96f, 1.f, 0.98f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1.1f, 4.0f);
	glLoadIdentity();

	SetView();

	glTranslatef(_tx, _ty, _zoom);
	glRotatef(_rotx, 1, 0, 0);
	glRotatef(_roty, 0, 1, 0);

	//DrawRay(_ray);

	//glTranslatef(-0.5f, -0.5f, -0.5f);
	Darw();
	FPS();
	glutSwapBuffers();
}

void Reshape(int w, int h)
{
	if (w == 0) {
		h = 1;
	}
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(_fovy, (float)w / h, _zNear, _zFar);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void Motion(int x, int y)
{
	int diffx = x - _lastx;
	int diffy = y - _lasty;
	_lastx = x;
	_lasty = y;
	if (!_rayCasting && !_tearing) {
		if (_buttons[2]) {
			_zoom += (float)0.05f * diffx;
		}
		else if (_buttons[0]) {
			_rotx += (float)0.5f * diffy;
			_roty += (float)0.5f * diffx;
		}
		else if (_buttons[1]) {
			_tx += (float)0.03f * diffx;
			_ty -= (float)0.03f * diffy;
		}
	}
	else if (_tearing) {
		if (_buttons[0]) {
			_cutxs.back() = x;
			_cutys.back() = y;
		}
	}
	else if (_rayCasting) {
		_rayx = x;
		_rayy = y;
		_ray = getRay(_rayx, _rayy);
		//_system->moveNode(_ray);
	}
	
	glutPostRedisplay();
}

void Mouse(int button, int state, int x, int y)
{
	_lastx = x;
	_lasty = y;
	switch (button)
	{
	case GLUT_LEFT_BUTTON:
		_buttons[0] = ((GLUT_DOWN == state) ? 1 : 0);
		if (_tearing) {
			if (GLUT_DOWN == state) {
				_cutxs.push_back(x);
				_cutys.push_back(y);
				_cutxs.push_back(x);
				_cutys.push_back(y);
			}
			else if (GLUT_UP == state) {
				_numCuts++;
			}
		}
		else if (_rayCasting) {
			if (GLUT_DOWN == state) {
				_rayx = x;
				_rayy = y;
				_ray = getRay(_rayx, _rayy);
				_system->clickNode(_ray, 0.1f, 10.f);
			}
			else if (GLUT_UP == state) {
				_system->clickOff();
			}
		}
		break;
	case GLUT_MIDDLE_BUTTON:
		_buttons[1] = ((GLUT_DOWN == state) ? 1 : 0);
		if (_rayCasting)
			_rayCasting = false;
		break;
	case GLUT_RIGHT_BUTTON:
		_buttons[2] = ((GLUT_DOWN == state) ? 1 : 0);
		if (_rayCasting)
			_rayCasting = false;
		break;
	default:
		break;
	}
	glutPostRedisplay();
}

void SpecialInput(int key, int x, int y)
{
	glutPostRedisplay();
}

void Keyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'q':
	case 'Q':
		exit(0);
	case ' ':
		_simulation = !_simulation;
		break;
	case 'r':
	case 'R':
		_system->reset();
		break;
	case 'c':
	case 'C':
		printf("%f, %f, %f, %f, %f\n", _zoom, _tx, _ty, _rotx, _roty);
		break;
	case 'a':
	case 'A':
		_rayCasting = !_rayCasting;
		break;
	case 't':
	case 'T':
		if (_tearing) {
			compCutting();
			_cutxs.clear();
			_cutys.clear();
			_numCuts = 0u;
		}
		_tearing = !_tearing;
		break;
	case 'd':
	case 'D':
		_system->_cloths->_bvh->_test++;
		printf("%d\n", _system->_cloths->_bvh->_test);
		//CollisionSolver::Debug();
		break;
	}
	glutPostRedisplay();
}

int main(int argc, char** argv)
{
	cudaDeviceReset();
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowSize(_width, _height);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("Cloth Simulator");
	glutDisplayFunc(Display);
	glutReshapeFunc(Reshape);
	glutIdleFunc(Update);
	glutMouseFunc(Mouse);
	glutMotionFunc(Motion);
	glutKeyboardFunc(Keyboard);
	glutSpecialFunc(SpecialInput);
	Init();
	glutMainLoop();
}
//#include <Windows.h>
//#include <stdio.h>
//
//#include "System.h"
//
//int _stride = 1;
//int _frame = 0;
//int _width = WINDOW_WIDTH;
//int _height = WINDOW_HEIGHT;
//float _zoom = -2.5;
//float _rotx = 0;
//float _roty = 0;
//float _tx = 0;
//float _ty = 0;
//int _lastx = 0;
//int _lasty = 0;
//float _fovy = 45.0f;
//
//unsigned char _buttons[3] = { 0 };
//bool _simulation = false;
//char _FPS_str[100];
//
//int _rayx = 0;
//int _rayy = 0;
//M_Ray _ray;
//bool _rayCasting = false;
//
//float4 purple = make_float4(0.0f, 0.44705882352941176470588235294118f, 0.66666666666666666666666666666667f, 1.0f);
//float4 blue = make_float4(0.0f, 0.44705882352941176470588235294118f, 0.66666666666666666666666666666667f, 1.0f);
//float4 pink = make_float4(1.0f, 180.f / 255.f, 210.f / 255.f, 1.0f);
//float4 yellow = make_float4(1.0f, 1.0f, 0.8f, 1.0f);
//float4 gray = make_float4(0.2f, 0.2f, 0.2f, 1.0f);
//float4 white = make_float4(1.0f, 1.0f, 1.0f, 1.0f);
//
//System* _system = nullptr;
//Mesh* _mesh = nullptr;
//
//#define SCREEN_CAPTURE
////#define SAVE_SIMULATION
//
//M_Ray getRay(int x, int y) {
//	M_Ray ray;
//
//	float rotX = _rotx * M_PI / 180.f;
//	float rotY = _roty * M_PI / 180.f;
//
//	float cosX = cosf(rotX);
//	float sinX = -sinf(rotX);
//	float cosY = cosf(rotY);
//	float sinY = -sinf(rotY);
//
//	float tx = -_tx;
//	float ty = -_ty;
//	float tz = -_zoom;
//
//	ray._center.x = tx * cosY + (ty * sinX + tz * cosX) * sinY;
//	ray._center.y = ty * cosX - tz * sinX;
//	ray._center.z = -tx * sinY + (ty * sinX + tz * cosX) * cosY;
//
//	float fovy = _fovy * M_PI / 180.f;
//	float wx = ((float)x / (float)_width * 2.0 - 1.0f) * fovy;
//	float wy = 0.f;// ((float)y / (float)_height * 2.0 - 1.0f)* fovy;
//
//	cosX = cosf(wy);
//	sinX = -sinf(wy);
//	cosY = cosf(wx);
//	sinY = -sinf(wx);
//	
//	tx = -cosX * sinY;
//	ty = sinX;
//	tz = -cosX * cosY;
//
//	cosX = cosf(rotX);
//	sinX = -sinf(rotX);
//	cosY = cosf(rotY + wx);
//	sinY = -sinf(rotY + wx);
//
//	ray._center.x = tx * cosY + (ty * sinX + tz * cosX) * sinY;
//	ray._center.y = ty * cosX - tz * sinX;
//	ray._center.z = -tx * sinY + (ty * sinX + tz * cosX) * cosY;
//
//	Normalize(ray._normal);
//	printf("%f\n", wx, wy);
//	return ray;
//}
//void DrawRay(const M_Ray& ray) {
//	float3 p0 = ray._center + ray._normal * 2.f;
//	float3 p1 = ray._center + ray._normal * 4.f;
//
//	glLineWidth(2.f);
//	glPointSize(5.f);
//
//	glBegin(GL_LINES);
//	glVertex3fv(&p0.x);
//	glVertex3fv(&p1.x);
//	glEnd();
//
//	glColor3f(1.f, 0.f, 0.f);
//	glPushMatrix();
//	glTranslatef(p0.x, p0.y, p0.z);
//	glutSolidSphere(0.1, 10, 10);
//	glPopMatrix();
//
//	glColor3f(0.f, 0.f, 1.f);
//	glPushMatrix();
//	glTranslatef(p1.x, p1.y, p1.z);
//	glutSolidSphere(0.1, 10, 10);
//	glPopMatrix();
//}
//
//void DrawText(float x, float y, const char* text, void* font = NULL)
//{
//	glColor3f(0, 0, 0);
//	glDisable(GL_DEPTH_TEST);
//
//	glMatrixMode(GL_PROJECTION);
//	glPushMatrix();
//	glLoadIdentity();
//	glOrtho(0.0, (double)_width, 0.0, (double)_height, -1.0, 1.0);
//
//	glMatrixMode(GL_MODELVIEW);
//	glPushMatrix();
//	glLoadIdentity();
//
//	if (font == NULL) {
//		font = GLUT_BITMAP_9_BY_15;
//	}
//
//	size_t len = strlen(text);
//
//	glRasterPos2f(x, y);
//	for (const char* letter = text; letter < text + len; letter++) {
//		if (*letter == '\n') {
//			y -= 12.0f;
//			glRasterPos2f(x, y);
//		}
//		glutBitmapCharacter(font, *letter);
//	}
//
//	glPopMatrix();
//	glMatrixMode(GL_PROJECTION);
//	glPopMatrix();
//	glMatrixMode(GL_MODELVIEW);
//	glEnable(GL_DEPTH_TEST);
//}
//tuple<Mesh*, Mesh*> LoadClothAndAvatar(char *cloth_file, char *avatar_file, double scale)
//{
//	Mesh *cloth, *avatar;
//	cloth = new Mesh(cloth_file);
//	avatar = new Mesh(avatar_file);
//
//	AABB aabb;
//	setAABB(aabb, cloth->_aabb);
//	addAABB(aabb, avatar->_aabb);
//
//	REAL3 size = aabb._max - aabb._min;
//	REAL max_length = size.x;
//	if (max_length < size.y)
//		max_length = size.y;
//	if (max_length < size.z)
//		max_length = size.z;
//	max_length = 2.0 * scale / max_length;
//
//	REAL3 prevCenter = (aabb._min + aabb._max) * (REAL)0.5;
//	REAL3 center = make_REAL3(0.0, 0.0, 0.0);
//
//	bool flag = false;
//	uint vlen = cloth->_ns.size();
//	for (uint i = 0u; i < vlen; i += 3u) {
//		REAL3 pos = make_REAL3(cloth->_ns[i], cloth->_ns[i + 1u], cloth->_ns[i + 2u]);
//		REAL3 grad = pos - prevCenter;
//		grad *= max_length;
//		pos = center + grad;
//		cloth->_ns[i] = pos.x;
//		cloth->_ns[i + 1u] = pos.y;
//		cloth->_ns[i + 2u] = pos.z;
//		if (flag) addAABB(aabb, pos);
//		else {
//			aabb._min = aabb._max = pos;
//			flag = true;
//		}
//	}
//	vlen = avatar->_ns.size();
//	flag = false;
//	for (uint i = 0u; i < vlen; i += 3u) {
//		REAL3 pos = make_REAL3(avatar->_ns[i], avatar->_ns[i + 1u], avatar->_ns[i + 2u]);
//		REAL3 grad = pos - prevCenter;
//		grad *= max_length;
//		pos = center + grad;
//		avatar->_ns[i] = pos.x;
//		avatar->_ns[i + 1u] = pos.y;
//		avatar->_ns[i + 2u] = pos.z;
//		if (flag) addAABB(aabb, pos);
//		else {
//			aabb._min = aabb._max = pos;
//			flag = true;
//		}
//	}
//
//	return make_tuple(cloth, avatar);
//}
//
//void SetView(double zoom, double tx, double ty, double rotx, double roty)
//{
//	_zoom = zoom;
//	_tx = tx;
//	_ty = ty;
//	_rotx = rotx;
//	_roty = roty;
//}
//void SetView(void)
//{
//#if SCENE==0
//	//----< Drop Boundary Collision >-----------------
//	//SetView(-2.000000, 0.060000, 1.020000, 34.500000, 18.000999);
//	//SetView(-3.899997, -1.020000, 0.480000, 4.500000, 183.001007);
//
//	//----< Bunny Collision >-----------------
//	//SetView(-1.700000, 0.000000, 0.960000, 36.000000, 56.500999);
//	//SetView(-0.600000, -0.030000, 0.630000, 48.000000, -127.998993);
//
//	//----< dragon Collision >-----------------
//	//SetView(-1.750000, 0.030000, 0.840000, 31.000000, 15.500999);
//
//	//----< Sphere Collision >-----------------
//	//SetView(-1.750000, 0.000000, 0.930000, 35.500000, 26.500999);
//	//SetView(-1.100000, 0.000000, 1.289999, 0.000000, 0.001000);
//
//	//----< Complex Collision >-----------------
//	//SetView(-3.200000, -0.000000, 0.750000, 34.500000, 0.000000);
//	//SetView(-2.500000, -0.000000, 1.100000, 34.000000, 0.000000);
//
//	//----< Avatar Collision >-----------------
//	//SetView(-0.950001, -0.000000, -0.390000, 22.000000, 33.500999);
//	//SetView(-0.950001, -0.000000, -0.390000, 22.000000, 153.500999);
//
//	//----< Stress Collision >-----------------
//	//SetView(-2.500000, 0.000000, 0.000000, 7.000000, 0.000000);
//
//	//----< Water Drop >-----------------
//	//SetView(-2.900000, 0.000000, 0.000000, 30.000000, 0.000000);
//	//SetView(-2.750000, 0.000000, 0.690000, 28.000000, 000000);
//	//SetView(-2.650000, 0.000000, 00.000000, -42.000000, 0.000000);
//
//	//----< Cloth Drop >-----------------
//	//SetView(-3.500000, 0.030000, 1.020000, 29.500000, 37.500999);
//
//	//----< Sphere Drop >-----------------
//	//SetView(-2.100000, 0.150000, 0.560000, 24.000000, 29.000999);
//
//	//SetView(-0.600000, -0.000000, 0.990000, 51.500000, 0.000000);
//#elif SCENE==1
//	//SetView(-2.750000, 0.000000, 0.690000, 28.000000, 0.000000);
//	//SetView(-2.750000, 0.000000, 0.690000, 20.000000, 0.000000);
//	SetView(-1.650000, 0.000000, 00.000000, -42.000000, 0.000000);
//#elif SCENE==2
//	SetView(-2.350000, 0.000000, 0.400000, 25.000000, 45.000000);
//#elif SCENE==3
//	//SetView(-3.250000, 0.000000, 0.000000, 16.000000, 12.500999);
//#elif SCENE==4
//	SetView(-3.650000, 0.000000, 0.400000, 18.000000, 0.000000);
//#endif
//}
//void Init(void)
//{
//	glEnable(GL_DEPTH_TEST);
//
//	//_system = new System(make_REAL3(0.0, -0.098, 0.0), 0.01);
//	//_system = new System(make_REAL3(0.0, -0.0, 0.0), 0.01);
//	_system = new System(make_REAL3(0.0, -9.8, 0.0), 0.005);
//	//_system = new System(make_REAL3(0.0, -9.8, 0.0), 0.01);
//	//_system->_
//  = clothRadius;
//
//	REAL clothFriction = 0.6;
//	REAL obstacleFriction = 0.6;
//	REAL boundaryFriction = 0.6;
//
//	REAL sphDensity = 1000.0;
//	REAL clothDensity = 800.0;
//	REAL clothSolidFraction = 0.4;
//
//	REAL sphViscosity = 0.01;
//	REAL clothViscosity = 0.01;
//	REAL obstacleViscosity = 0.01;
//	REAL boundaryViscosity = 0.01;
//
//	REAL obstacleMass = 1.0;
//
//#if QUALITY==0
//	REAL sphRadius = 0.012;
//	REAL clothRadius = 0.012;
//	REAL obstacleRadius = 0.012; //max(sphRadius, clothRadius);
//	REAL boundaryRadius = 0.012; //max(sphRadius, clothRadius);
//#elif QUALITY==1
//	REAL sphRadius = 0.0068;
//	REAL clothRadius = 0.0068;
//	REAL obstacleRadius = 0.0068; //max(sphRadius, clothRadius);
//	REAL boundaryRadius = 0.0068; //max(sphRadius, clothRadius);
//#else
//	REAL sphRadius = 0.004;
//	REAL clothRadius = 0.004;
//	REAL obstacleRadius = 0.004; //max(sphRadius, clothRadius);
//	REAL boundaryRadius = 0.004; //max(sphRadius, clothRadius);
//#endif
//
//#if SCENE==0
//#if QUALITY==0
//	_system->_subStep = 1u;
//#elif QUALITY==1
//	_system->_subStep = 1u;
//#else
//	_system->_subStep = 1u;
//#endif
//
//	REAL sphSurfaceTension = 0.4;
//	REAL clothSurfaceTension = 4.2;
//	REAL obstacleSurfaceTension = 4.2;
//	REAL boundarySurfaceTension = 4.2;
//
//	_system->addSPHModel(sphRadius, sphDensity, sphViscosity, sphSurfaceTension, blue);
//
//	_mesh = new Mesh("../obj/cube.obj", make_REAL3(0.0), _system->_boundary._max - _system->_boundary._min);
//	_system->addObstacle(_mesh, 1.0, boundaryFriction, make_REAL3(0.0), make_REAL3(0.0),
//		boundaryRadius, boundaryViscosity, boundarySurfaceTension,
//		make_float4(1.0f, 1.0f, 1.0f, 0.0f), make_float4(1.0f, 1.0f, 1.0f, 0.0f));
//
//#if QUALITY==0
//	_mesh = new Mesh("../obj/LR_cloth.obj", make_REAL3(0.0, -10.4, 0.0));
//	//_mesh = new Mesh("../obj/LR_cloth.obj", make_REAL3(0.0, 0.0, 0.0));
//#elif QUALITY==1
//	_mesh = new Mesh("../obj/MR_cloth.obj", make_REAL3(0.0, -1.4, 0.0));
//	//_mesh = new Mesh("../obj/MR_cloth.obj", make_REAL3(0.0, 0.0, 0.0));
//#else
//	_mesh = new Mesh("../obj/HR_cloth.obj", make_REAL3(0.0, 0.0, 0.0));
//#endif
//	_mesh->rotate(make_REAL3(-90.0, 0.0, 0.0));
//	_system->addCloth(_mesh, clothFriction,
//		clothRadius, clothDensity, sphDensity,
//		clothSolidFraction, clothViscosity, clothSurfaceTension,
//		pink, yellow);
//
//	/*_mesh = new Mesh("../obj/bunny.obj", make_REAL3(0.0, -1.2, 0.0), 0.48);
//	_system->addObstacle(_mesh, obstacleMass, obstacleFriction,
//		make_REAL3(0.0, 0.0, 0.0), make_REAL3(0.0, 0.0, 0.0),
//		obstacleRadius, obstacleViscosity, obstacleSurfaceTension, gray, gray);*/
//
//#elif SCENE==1
//
//#if QUALITY==0
//	_system->_subStep = 1u;
//#elif QUALITY==1
//	_system->_subStep = 1u;
//#else
//	_system->_subStep = 2u;
//#endif
//
//	/*REAL sphSurfaceTension = 0.4;
//	REAL clothSurfaceTension = 4.2;
//	REAL obstacleSurfaceTension = 4.2;
//	REAL boundarySurfaceTension = 4.2;*/
//	REAL sphSurfaceTension = 1.0;
//	REAL clothSurfaceTension = 8.0;
//	REAL obstacleSurfaceTension = 8.0;
//	REAL boundarySurfaceTension = 8.0;
//
//	_system->addSPHModel(sphRadius, sphDensity, sphViscosity, sphSurfaceTension, blue);
//
//	_mesh = new Mesh("../obj/cube.obj", make_REAL3(0.0), _system->_boundary._max - _system->_boundary._min);
//	_system->addObstacle(_mesh, 1.0, boundaryFriction, make_REAL3(0.0), make_REAL3(0.0),
//		boundaryRadius, boundaryViscosity, boundarySurfaceTension,
//		make_float4(1.0f, 1.0f, 1.0f, 0.0f), make_float4(1.0f, 1.0f, 1.0f, 0.0f));
//
//#if QUALITY==0 
//	_mesh = new Mesh("../obj/LR_cloth.obj", make_REAL3(0.0, 0.0, 0.0));
//#elif QUALITY==1
//	_mesh = new Mesh("../obj/MR_cloth.obj", make_REAL3(0.0, 0.0, 0.0));
//#else
//	_mesh = new Mesh("../obj/HR_cloth.obj", make_REAL3(0.0, 0.0, 0.0));
//#endif
//	_mesh->rotate(make_REAL3(-90.0, 0.0, 0.0));
//	_system->addCloth(_mesh, clothFriction,
//		clothRadius, clothDensity, sphDensity,
//		clothSolidFraction, clothViscosity, clothSurfaceTension,
//		pink, yellow);
//
//	_system->spawnSPHParticle(1);
//
//#elif SCENE==2
//
//#if QUALITY==0
//	_system->_subStep = 1u;
//#elif QUALITY==1
//	_system->_subStep = 1u;
//#else
//	_system->_subStep = 2u;
//#endif
//
//	REAL sphSurfaceTension = 0.4;
//	REAL clothSurfaceTension = 4.2;
//	REAL obstacleSurfaceTension = 4.2;
//	REAL boundarySurfaceTension = 4.2;
//	/*REAL sphSurfaceTension = 1.0;
//	REAL clothSurfaceTension = 8.0;
//	REAL obstacleSurfaceTension = 8.0;
//	REAL boundarySurfaceTension = 8.0;*/
//
//	_system->addSPHModel(sphRadius, sphDensity, sphViscosity, sphSurfaceTension, blue);
//
//#if QUALITY==0
//	_mesh = new Mesh("../obj/LR_cloth.obj", make_REAL3(0.0, 0.6, 0.0));
//#elif QUALITY==1
//	_mesh = new Mesh("../obj/MR_cloth.obj", make_REAL3(0.0, 0.6, 0.0));
//#else
//	_mesh = new Mesh("../obj/HR_cloth.obj", make_REAL3(0.0, 0.6, 0.0));
//#endif
//	_mesh->rotate(make_REAL3(-90.0, 0.0, 0.0));
//	_system->addCloth(_mesh, clothFriction,
//		clothRadius, clothDensity, sphDensity,
//		clothSolidFraction, clothViscosity, clothSurfaceTension,
//		pink, yellow);
//
//	_mesh = new Mesh("../obj/bunny.obj", make_REAL3(0.0, 0.0, 0.0), 0.3);
//	_system->addObstacle(_mesh, obstacleMass, obstacleFriction,
//		make_REAL3(0.0, 0.0, 0.0), make_REAL3(0.0, 0.2, 0.0),
//		obstacleRadius, obstacleViscosity, obstacleSurfaceTension, gray, gray);
//#elif SCENE==3
//
//#if QUALITY==0
//	_system->_subStep = 1u;
//#elif QUALITY==1
//	_system->_subStep = 1u;
//#else
//	_system->_subStep = 2u;
//#endif
//
//	REAL sphSurfaceTension = 0.2;
//	REAL clothSurfaceTension = 1.8;
//	REAL obstacleSurfaceTension = 1.8;
//	REAL boundarySurfaceTension = 1.8;
//
//	_system->addSPHModel(sphRadius, sphDensity, sphViscosity, sphSurfaceTension, blue);
//
//	_mesh = new Mesh("../obj/cube.obj", make_REAL3(0.0), _system->_boundary._max - _system->_boundary._min);
//	_system->addObstacle(_mesh, 1.0, boundaryFriction, make_REAL3(0.0), make_REAL3(0.0),
//		boundaryRadius, boundaryViscosity, boundarySurfaceTension,
//		make_float4(1.0f, 1.0f, 1.0f, 0.0f), make_float4(1.0f, 1.0f, 1.0f, 0.0f));
//
//#if QUALITY==0
//	_mesh = new Mesh("../obj/LR_cloth.obj", make_REAL3(0.0, 0.0, 0.0));
//#elif QUALITY==1
//	_mesh = new Mesh("../obj/MR_cloth.obj", make_REAL3(0.0, 0.0, 0.0));
//#else
//	_mesh = new Mesh("../obj/HR_cloth.obj", make_REAL3(0.0, 0.0, 0.0));
//#endif
//	//_mesh->rotate(make_REAL3(-90.0, 0.0, 0.0));
//	_system->addCloth(_mesh, clothFriction,
//		clothRadius, clothDensity, sphDensity,
//		clothSolidFraction, clothViscosity, clothSurfaceTension,
//		pink, yellow);
//
//	_system->spawnSPHParticle(1);
//#elif SCENE==4
//
//#if QUALITY==0
//	_system->_subStep = 2u;
//#elif QUALITY==1
//	_system->_subStep = 2u;
//#else
//	_system->_subStep = 4u;
//#endif
//
//
//	/*REAL sphSurfaceTension = 0.4;
//	REAL clothSurfaceTension = 4.2;
//	REAL obstacleSurfaceTension = 4.2;
//	REAL boundarySurfaceTension = 4.2;*/
//
//	REAL sphSurfaceTension = 1.0;
//	REAL clothSurfaceTension = 8.0;
//	REAL obstacleSurfaceTension = 8.0;
//	REAL boundarySurfaceTension = 8.0;
//
//	_system->addSPHModel(sphRadius, sphDensity, sphViscosity, sphSurfaceTension, blue);
//
//	_mesh = new Mesh("../obj/cube.obj", make_REAL3(0.0), _system->_boundary._max - _system->_boundary._min);
//	_system->addObstacle(_mesh, 1.0, boundaryFriction, make_REAL3(0.0), make_REAL3(0.0),
//		boundaryRadius, boundaryViscosity, boundarySurfaceTension,
//		make_float4(1.0f, 1.0f, 1.0f, 0.0f), make_float4(1.0f, 1.0f, 1.0f, 0.0f));
//
//#if QUALITY==0
//	_mesh = new Mesh("../obj/LR_cloth.obj", make_REAL3(0.0, 0.0, 0.0));
//#elif QUALITY==1
//	_mesh = new Mesh("../obj/MR_cloth.obj", make_REAL3(0.0, 0.0, 0.0));
//#else
//	_mesh = new Mesh("../obj/HR_cloth.obj", make_REAL3(0.0, 0.0, 0.0));
//#endif
//	_mesh->rotate(make_REAL3(-90.0, 0.0, 0.0));
//	_system->addCloth(_mesh, clothFriction,
//		clothRadius, clothDensity, sphDensity,
//		clothSolidFraction, clothViscosity, clothSurfaceTension,
//		pink, yellow);
//
//	_system->spawnSPHParticle(4);
//#endif
//}
//
//void FPS(void)
//{
//	static float framesPerSecond = 0.0f;
//	static float lastTime = 0.0f;
//	float currentTime = GetTickCount() * 0.001f;
//	++framesPerSecond;
//	if (currentTime - lastTime > 1.0f) {
//		lastTime = currentTime;
//		sprintf(_FPS_str, "FPS : %d", (int)framesPerSecond);
//		framesPerSecond = 0;
//	}
//}
//void Darw(void)
//{
//	glutReshapeWindow(_width, _height);
//	glEnable(GL_LIGHTING);
//	glEnable(GL_LIGHT0);
//	glShadeModel(GL_SMOOTH);
//	char text[100];
//
//	if (_system)
//		_system->draw();
//
//	glDisable(GL_LIGHTING);
//	//DrawText(10.0f, 580.0f, "Projective Dynamics, ACM TOG 2014");
//
//	glDisable(GL_LIGHTING);
//	if (_system) {
//		sprintf(text, "Number of triangles : %d", _system->numFaces());
//		DrawText(10.0f, 560.0f, text);
//		//sprintf(text, "Number of particles : %d", _system->numParticles());
//		sprintf(text, "SPH Particle: %d, Pore Particle : %d, Boundary Particle: %d", 
//			_system->_sphParticles->_numParticles, _system->_poreParticles->_numParticles,_system->_boundaryParticles->_numParticles);
//		DrawText(10.0f, 540.0f, text);
//	}
//	DrawText(10.0f, 520.0f, _FPS_str);
//	sprintf(text, "Frame : %d", _frame);
//	DrawText(10.0f, 500.0f, text);
//}
//void Capture(int endFrame)
//{
//	if (_frame == 0 || _frame % _stride == 0) {
//		static int index = 0;
//		char filename[100];
//		sprintf_s(filename, "capture\\capture-%d.bmp", index);
//		BITMAPFILEHEADER bf;
//		BITMAPINFOHEADER bi;
//		unsigned char* image = (unsigned char*)malloc(sizeof(unsigned char) * _width * _height * 3);
//		FILE* file;
//		fopen_s(&file, filename, "wb");
//		if (image != NULL) {
//			if (file != NULL) {
//				glReadPixels(0, 0, _width, _height, 0x80E0, GL_UNSIGNED_BYTE, image);
//				memset(&bf, 0, sizeof(bf));
//				memset(&bi, 0, sizeof(bi));
//				bf.bfType = 'MB';
//				bf.bfSize = sizeof(bf) + sizeof(bi) + _width * _height * 3;
//				bf.bfOffBits = sizeof(bf) + sizeof(bi);
//				bi.biSize = sizeof(bi);
//				bi.biWidth = _width;
//				bi.biHeight = _height;
//				bi.biPlanes = 1;
//				bi.biBitCount = 24;
//				bi.biSizeImage = _width * _height * 3;
//				fwrite(&bf, sizeof(bf), 1, file);
//				fwrite(&bi, sizeof(bi), 1, file);
//				fwrite(image, sizeof(unsigned char), _height * _width * 3, file);
//				fclose(file);
//			}
//			free(image);
//		}
//		if (index == endFrame) {
//			exit(0);
//		}
//		index++;
//	}
//}
//void SaveSimulation(int endFrame)
//{
//	if (_frame == 0 || _frame % _stride == 0) {
//		static int index = 0;
//		char text[100];
//		sprintf_s(text, "simulation\\scene%d\\scene%d-%d.txt", SCENE, SCENE, index);
//
//		ofstream fout(text);
//		if (fout.fail())
//		{
//		    std::cerr << "Error!" << std::endl;
//		}
//		else {
//			uint numNodes = 0u;
//			for (uint i = 0u; i < _system->_cloths->_numFaces; i++) {
//				uint ino0 = _system->_cloths->h_fs[i * 3u + 0u];
//				uint ino1 = _system->_cloths->h_fs[i * 3u + 1u];
//				uint ino2 = _system->_cloths->h_fs[i * 3u + 2u];
//				sprintf(text, "f %d %d %d", ino0 + 1u, ino1 + 1u, ino2 + 1u);
//				fout << text << endl;
//			}
//			for (uint i = 0u; i < _system->_obstacles->_numFaces; i++) {
//				uint ino0 = _system->_obstacles->h_fs[i * 3u + 0u] + _system->_cloths->_numNodes;
//				uint ino1 = _system->_obstacles->h_fs[i * 3u + 1u] + _system->_cloths->_numNodes;
//				uint ino2 = _system->_obstacles->h_fs[i * 3u + 2u] + _system->_cloths->_numNodes;
//				sprintf(text, "f %d %d %d", ino0 + 1u, ino1 + 1u, ino2 + 1u);
//				fout << text << endl;
//			}
//			for (uint i = 0u; i < _system->_cloths->_numNodes; i++) {
//				float x = _system->_cloths->h_ns[i * 3u + 0u];
//				float y = _system->_cloths->h_ns[i * 3u + 1u];
//				float z = _system->_cloths->h_ns[i * 3u + 2u];
//
//				uint phase = _system->_cloths->h_nodePhases[i];
//
//				float frontColor[4];
//				float backColor[4];
//
//				float s = powf(1.f - min((1.0 - _system->_cloths->h_restSolidFractions[i]) * _system->_cloths->h_ss[i], 1.f), 1.0f);
//
//				for (int j = 0; j < 3; j++) {
//					frontColor[j] = (&_system->_cloths->h_frontColors[phase].x)[j] * s;
//					backColor[j] = (&_system->_cloths->h_backColors[phase].x)[j] * s;
//				}
//				frontColor[3] = _system->_cloths->h_frontColors[phase].w;
//				backColor[3] = _system->_cloths->h_backColors[phase].w;
//
//				sprintf(text, "v %f %f %f %f %f %f %f %f %f %f %f", x, y, z,
//					frontColor[0], frontColor[1], frontColor[2], frontColor[3],
//					backColor[0], backColor[1], backColor[2], backColor[3]);
//				fout << text << endl;
//			}
//			for (uint i = 0u; i < _system->_obstacles->_numNodes; i++) {
//				float x = _system->_obstacles->h_ns[i * 3u + 0u];
//				float y = _system->_obstacles->h_ns[i * 3u + 1u];
//				float z = _system->_obstacles->h_ns[i * 3u + 2u];
//
//				uint phase = _system->_obstacles->h_nodePhases[i];
//
//				float frontColor[4];
//				float backColor[4];
//
//				for (int j = 0; j < 4; j++) {
//					frontColor[j] = (&_system->_obstacles->h_frontColors[phase].x)[j];
//					backColor[j] = (&_system->_obstacles->h_backColors[phase].x)[j];
//				}
//
//				sprintf(text, "v %f %f %f %f %f %f %f %f %f %f %f", x, y, z,
//					frontColor[0], frontColor[1], frontColor[2], frontColor[3],
//					backColor[0], backColor[1], backColor[2], backColor[3]);
//				fout << text << endl;
//			}
//			for (uint i = 0u; i < _system->_sphParticles->_numParticles; i++) {
//				float x = _system->_sphParticles->h_xs[i * 3u + 0u];
//				float y = _system->_sphParticles->h_xs[i * 3u + 1u];
//				float z = _system->_sphParticles->h_xs[i * 3u + 2u];
//
//				uint phase = _system->_sphParticles->h_phases[i];
//				float radius = _system->_sphParticles->h_radii[phase];
//				radius *= S3TO1(_system->_sphParticles->h_ss[i]);
//
//				sprintf(text, "p %f %f %f %f", x, y, z, radius);
//				fout << text << endl;
//			}
//		}
//		fout.close();
//		if (index == endFrame) {
//			exit(0);
//		}
//		index++;
//	}
//}
//void Update(void)
//{
//#if SCENE==0
//	if (_simulation) {
//#ifdef SAVE_SIMULATION
//		SaveSimulation(2050);
//#endif
//#ifdef SCREEN_CAPTURE
//		Capture(2050);
//#endif
//		if (_system)
//			_system->simulation();
//		_frame++;
//}
//#elif SCENE==1
//	if (_simulation) {
//#ifdef SAVE_SIMULATION
//		SaveSimulation(700 / _stride);
//#endif
//#ifdef SCREEN_CAPTURE
//		Capture(700 / _stride);
//#endif
//		if (_system)
//			_system->simulation();
//		_frame++;
//	}
//#elif SCENE==2
//	if (_simulation) {
//#ifdef SAVE_SIMULATION
//		SaveSimulation(1600 / _stride);
//#endif
//#ifdef SCREEN_CAPTURE
//		Capture(1600 / _stride);
//#endif
//		if (_frame == 300)
//			_system->spawnSPHParticle(1);
//
//		if (_system)
//			_system->simulation();
//		_frame++;
//	}
//#elif SCENE==3
//	if (_simulation) {
//#ifdef SAVE_SIMULATION
//		SaveSimulation(400 / _stride);
//#endif
//#ifdef SCREEN_CAPTURE
//		Capture(400 / _stride);
//#endif
//		if (_system)
//			_system->simulation();
//		_frame++;
//	}
//#elif SCENE==4
//	if (_simulation) {
//		if (_system) {
//			/*uint idleFrame = 0;
//			uint moveFrame = 0;
//			uint wetFrame = 0;
//			uint rotateFrame = 800u;*/
//			uint idleFrame = 50u;
//			uint moveFrame = 300u;
//			uint wetFrame = 200u;
//			uint rotateFrame = 800u;
//			uint rotateFrame2 = 200u;
//			REAL3 moveVel = make_REAL3(0.0, -(_system->_boundary._max.y - _system->_boundary._min.y) * 0.4 / (REAL)moveFrame * _system->_invdt, 0.0);
//			REAL3 degree = make_REAL3(540.0 / (REAL)rotateFrame, 0.0, 0.0);
//			REAL move = 0.2 / (REAL)rotateFrame;
//
//#ifdef SAVE_SIMULATION
//			SaveSimulation((idleFrame * 3u + moveFrame * 2u + wetFrame * 3u + rotateFrame2 + rotateFrame * 2u) / _stride);
//#endif
//#ifdef SCREEN_CAPTURE
//			Capture((idleFrame * 3u + moveFrame * 2u + wetFrame * 3u + rotateFrame2 + rotateFrame * 2u) / _stride);
//#endif
//
//			if (_frame < idleFrame) {
//			}
//			else if (_frame < idleFrame + moveFrame) {
//				_system->_cloths->moveFixed(moveVel);
//			}
//			else if (_frame < idleFrame + moveFrame + wetFrame) {
//				_system->_cloths->moveFixed(make_REAL3(0.0));
//			}
//			else if (_frame < idleFrame + moveFrame * 2u + wetFrame) {
//				_system->_cloths->moveFixed(moveVel * -1.0);
//			}
//			else if (_frame < idleFrame * 2u + moveFrame * 2u + wetFrame) {
//				_system->_cloths->moveFixed(make_REAL3(0.0));
//			}
//			else if (_frame < idleFrame * 2u + moveFrame * 2u + wetFrame + rotateFrame2) {
//				_system->_cloths->rotateFixed(
//					make_REAL3(90.0 / (REAL)rotateFrame2, 0.0, 0.0), make_REAL3(90.0 / (REAL)rotateFrame2, 0.0, 0.0), 
//					0.0, 0.0, _system->_invdt);
//			}
//			else if (_frame < idleFrame * 3u + moveFrame * 2u + wetFrame + rotateFrame2) {
//				_system->_cloths->moveFixed(make_REAL3(0.0));
//			}
//			else if (_frame < idleFrame * 3u + moveFrame * 2u + wetFrame + rotateFrame2 + rotateFrame) {
//				_system->_cloths->rotateFixed(degree, degree * -1.0, move, -move, _system->_invdt);
//			}
//			else if (_frame < idleFrame * 3u + moveFrame * 2u + wetFrame * 2u + rotateFrame2 + rotateFrame) {
//				_system->_cloths->moveFixed(make_REAL3(0.0));
//			}
//			else if (_frame < idleFrame * 3u + moveFrame * 2u + wetFrame * 2u + rotateFrame2 + rotateFrame * 2u) {
//				_system->_cloths->rotateFixed(degree * -1.0, degree, -move, move, _system->_invdt);
//			}
//			else {
//				_system->_cloths->moveFixed(make_REAL3(0.0));
//			}
//
//			_system->simulation();
//		}
//		_frame++;
//	}
//#endif
//
//	::glutPostRedisplay();
//}
//
//void Display(void)
//{
//	glClearColor(0.8980392156862745f, 0.9490196078431373f, 1.0f, 1.0f);
//	//glClearColor(0.96f, 1.f, 0.98f, 1.0f);
//	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//	glEnable(GL_POLYGON_OFFSET_FILL);
//	glPolygonOffset(1.1f, 4.0f);
//	glLoadIdentity();
//
//	SetView();
//
//	glTranslatef(_tx, _ty, _zoom);
//	glRotatef(_rotx, 1, 0, 0);
//	glRotatef(_roty, 0, 1, 0);
//
//	_ray = getRay(_rayx, _rayy);
//	DrawRay(_ray);
//
//	//glTranslatef(-0.5f, -0.5f, -0.5f);
//	Darw();
//	FPS();
//	glutSwapBuffers();
//}
//
//void Reshape(int w, int h)
//{
//	if (w == 0) {
//		h = 1;
//	}
//	glViewport(0, 0, w, h);
//	glMatrixMode(GL_PROJECTION);
//	glLoadIdentity();
//	gluPerspective(_fovy, (float)w / h, 0.1, 100);
//	glMatrixMode(GL_MODELVIEW);
//	glLoadIdentity();
//}
//
//void Motion(int x, int y)
//{
//	int diffx = x - _lastx;
//	int diffy = y - _lasty;
//	_lastx = x;
//	_lasty = y;
//
//	if (_buttons[2]) {
//		_zoom += (float)0.05f * diffx;
//	}
//	else if (_buttons[0]) {
//		_rotx += (float)0.5f * diffy;
//		_roty += (float)0.5f * diffx;
//	}
//	else if (_buttons[1]) {
//		_tx += (float)0.03f * diffx;
//		_ty -= (float)0.03f * diffy;
//	}
//	glutPostRedisplay();
//}
//
//void Mouse(int button, int state, int x, int y)
//{
//	_lastx = x;
//	_lasty = y;
//	if (_rayCasting) {
//		_rayx = x;
//		_rayy = y;
//		//_ray = getRay(_rayx, _rayy);
//	}
//	switch (button)
//	{
//	case GLUT_LEFT_BUTTON:
//		_buttons[0] = ((GLUT_DOWN == state) ? 1 : 0);
//		break;
//	case GLUT_MIDDLE_BUTTON:
//		_buttons[1] = ((GLUT_DOWN == state) ? 1 : 0);
//		break;
//	case GLUT_RIGHT_BUTTON:
//		_buttons[2] = ((GLUT_DOWN == state) ? 1 : 0);
//		break;
//	default:
//		break;
//	}
//	glutPostRedisplay();
//}
//
//void SpecialInput(int key, int x, int y)
//{
//	glutPostRedisplay();
//}
//
//void Keyboard(unsigned char key, int x, int y)
//{
//	switch (key)
//	{
//	case 'q':
//	case 'Q':
//		exit(0);
//	case ' ':
//		_simulation = !_simulation;
//		break;
//	case 'r':
//	case 'R':
//		_system->reset();
//		break;
//	case 'c':
//	case 'C':
//		printf("%f, %f, %f, %f, %f\n", _zoom, _tx, _ty, _rotx, _roty);
//		break;
//	case 't':
//	case 'T':
//		_rayCasting = !_rayCasting;
//		break;
//	case 'd':
//	case 'D':
//		_system->_cloths->_bvh->_test++;
//		printf("%d\n", _system->_cloths->_bvh->_test);
//		//CollisionSolver::Debug();
//		break;
//	case '0':
//	case ')':
//		if (_system)
//			_system->spawnSPHParticle(0);
//		break;
//	case '1':
//	case '!':
//		if (_system)
//			_system->spawnSPHParticle(1);
//		break;
//	case '2':
//	case '@':
//		if (_system)
//			_system->spawnSPHParticle(2);
//		break;
//	case '3':
//	case '#':
//		if (_system)
//			_system->spawnSPHParticle(3);
//		break;
//	case '4':
//	case '$':
//		if (_system)
//			_system->spawnSPHParticle(4);
//		break;
//	case '5':
//	case '%':
//		if (_system)
//			_system->spawnSPHParticle(5);
//		break;
//	case '6':
//	case '^':
//		if (_system)
//			_system->spawnSPHParticle(6);
//		break;
//	}
//	glutPostRedisplay();
//}
//
//int main(int argc, char** argv)
//{
//	cudaDeviceReset();
//	glutInit(&argc, argv);
//	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
//	glutInitWindowSize(_width, _height);
//	glutInitWindowPosition(100, 100);
//	glutCreateWindow("Cloth Simulator");
//	glutDisplayFunc(Display);
//	glutReshapeFunc(Reshape);
//	glutIdleFunc(Update);
//	glutMouseFunc(Mouse);
//	glutMotionFunc(Motion);
//	glutKeyboardFunc(Keyboard);
//	glutSpecialFunc(SpecialInput);
//	Init();
//	glutMainLoop();
//}