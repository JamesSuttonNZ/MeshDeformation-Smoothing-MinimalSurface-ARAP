//---------------------------------------------------------------------------
//
// Copyright (c) 2016 Taehyun Rhee, Joshua Scott, Ben Allen
//
// This software is provided 'as-is' for assignment of COMP308 in ECS,
// Victoria University of Wellington, without any express or implied warranty.
// In no event will the authors be held liable for any damages arising from
// the use of this software.
//
// The contents of this file may not be copied or duplicated in any form
// without the prior permission of its owner.
//
//----------------------------------------------------------------------------

#include <cmath>
#include <iostream> // input/output streams
#include <fstream>  // file streams
#include <sstream>  // string streams
#include <string>
#include <stdexcept>
#include <vector>
#include <set>
#include <map>

#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include "cgra_math.h"
#include "geometry.h"

using namespace std;
using namespace cgra;

Geometry::Geometry(string filename, vec3 trans, vec3 rot, vec3 sca, float uv, Material* mat) {
	m_filename = filename;
	uvScale = uv;
	readOBJ(filename);
	if (m_triangles.size() > 0) {
		createDisplayListPoly();
		createDisplayListWire();
	}

	rotation = rot;
	translation = trans;
	scale = sca;

	if (mat == NULL) {
		// default material
		material = new Material("default", vec4(1), vec4(1), vec4(1), 1);
	}
	else {
		material = mat;
	}

	m_points_source = m_points;
}

Geometry::~Geometry() { }

void Geometry::readOBJ(string filename) {

	// Make sure our geometry information is cleared
	m_points.clear();
	m_uvs.clear();
	m_normals.clear();
	m_triangles.clear();
	m_quadsG.clear();

	// Load dummy points because OBJ indexing starts at 1 not 0
	m_points.push_back(vec3(0, 0, 0));
	m_uvs.push_back(vec2(0, 0));
	m_normals.push_back(vec3(0, 0, 1));


	ifstream objFile(filename);

	if (!objFile.is_open()) {
		cerr << "Error reading " << filename << endl;
		throw runtime_error("Error :: could not open file.");
	}

	cout << "Reading file " << filename << endl;

	// good() means that failbit, badbit and eofbit are all not set
	while (objFile.good()) {

		// Pull out line from file
		string line;
		std::getline(objFile, line);
		istringstream objLine(line);

		// Pull out mode from line
		string mode;
		objLine >> mode;

		// Reading like this means whitespace at the start of the line is fine
		// attempting to read from an empty string/line will set the failbit
		if (!objLine.fail()) {

			if (mode == "v") {
				vec3 v;
				objLine >> v.x >> v.y >> v.z;
				m_points.push_back(v);

			}
			else if (mode == "vn") {
				vec3 vn;
				objLine >> vn.x >> vn.y >> vn.z;
				m_normals.push_back(vn);

			}
			else if (mode == "vt") {
				vec2 vt;
				objLine >> vt.x >> vt.y;
				m_uvs.push_back(vt * uvScale);

			}
			else if (mode == "f") {

				vector<vertex> verts;
				while (objLine.good()) {
					vertex v;

					// Assignment code (assumes you have all of v/vt/vn for each vertex)
					if (m_uvs.size() > 1 && m_normals.size() > 1) {
						objLine >> v.p;		// Scan in position index
						objLine.ignore(1);	// Ignore the '/' character
						objLine >> v.t;		// Scan in uv (texture coord) index
						objLine.ignore(1);	// Ignore the '/' character
						objLine >> v.n;		// Scan in normal index
					}
					else if (m_uvs.size() > 1 && m_normals.size() == 1) {
						objLine >> v.p;		// Scan in position index
						objLine.ignore(1);	// Ignore the '/' character
						objLine >> v.t;		// Scan in normal index
					}
					else if (m_uvs.size() == 1 && m_normals.size() > 1) {
						objLine >> v.p;		// Scan in position index
						objLine.ignore(2);	// Ignore the '//' character
						objLine >> v.n;		// Scan in normal index
					}
					else {
						objLine >> v.p;		// Scan in position index
					}
					if (v.p != 0 || v.t != 0 || v.n != 0) {
						verts.push_back(v);
					}
				}



				// IFF we have 3 verticies, construct a triangle
				if (verts.size() == 3) {
					triangle tri;
					tri.v[0] = verts[0];
					tri.v[1] = verts[1];
					tri.v[2] = verts[2];
					m_triangles.push_back(tri);

				}

				if (verts.size() == 4) {
					quadG q;
					q.v[0] = verts[0];
					q.v[1] = verts[1];
					q.v[2] = verts[2];
					q.v[2] = verts[3];
					m_quadsG.push_back(q);
				}
			}
		}
	}

	cout << "Reading OBJ file is DONE." << endl;
	cout << m_points.size() - 1 << " points" << endl;
	cout << m_uvs.size() - 1 << " uv coords" << endl;
	cout << m_normals.size() - 1 << " normals" << endl;
	cout << m_triangles.size() << " faces" << endl;
	cout << m_quadsG.size() << " quads" << endl;

	// If we didn't have any normals, create them
	if (m_normals.size() <= 1) createNormals();

	//createTangents();
}

void Geometry::createNormals() {
	std::vector<cgra::vec3> normals; //triangle face normals
	for (int i = 0; i < m_triangles.size(); i++) {
		vec3 v0 = m_points[m_triangles[i].v[0].p];
		vec3 v1 = m_points[m_triangles[i].v[1].p];
		vec3 v2 = m_points[m_triangles[i].v[2].p];

		vec3 deltaPos1 = v1 - v0;
		vec3 deltaPos2 = v2 - v0;

		vec3 normal(cross(deltaPos1, deltaPos2));
		normals.push_back(normal);

		m_triangles[i].v[0].n = m_triangles[i].v[0].p;	//set n as p because need normal for every vertex/point
		m_triangles[i].v[1].n = m_triangles[i].v[1].p;
		m_triangles[i].v[2].n = m_triangles[i].v[2].p;

		m_normals.push_back(vec3(0, 0, 0));	//create 0 normal for each point/vertex
		m_normals.push_back(vec3(0, 0, 0));
		m_normals.push_back(vec3(0, 0, 0));

	}


	std::vector<cgra::vec3> normalsQ; //quad face normals
	for (int i = 0; i < m_quadsG.size(); i++) {
		vec3 v0 = m_points[m_quadsG[i].v[0].p];
		vec3 v1 = m_points[m_quadsG[i].v[1].p];
		vec3 v2 = m_points[m_quadsG[i].v[2].p];

		vec3 deltaPos1 = v1 - v0;
		vec3 deltaPos2 = v2 - v0;

		vec3 normal(cross(deltaPos1, deltaPos2));
		normalsQ.push_back(normal);

		m_quadsG[i].v[0].n = m_quadsG[i].v[0].p;	//set n as p because need normal for every vertex/point
		m_quadsG[i].v[1].n = m_quadsG[i].v[1].p;
		m_quadsG[i].v[2].n = m_quadsG[i].v[2].p;
		m_quadsG[i].v[3].n = m_quadsG[i].v[3].p;

		m_normals.push_back(vec3(0, 0, 0));	//create 0 normal for each point/vertex
		m_normals.push_back(vec3(0, 0, 0));
		m_normals.push_back(vec3(0, 0, 0));
		m_normals.push_back(vec3(0, 0, 0));

	}

	//tri
	for (int j = 0; j < m_triangles.size(); j++) {
		for (int k = 0; k < 3; k++) {
			m_normals[m_triangles[j].v[k].n] += normals[j]; //sum vertex normal with face normal
		}
	}
	for (int j = 0; j < m_triangles.size(); j++) {
		for (int k = 0; k < 3; k++) {
			normalize(m_normals[m_triangles[j].v[k].n]);
		}
	}

	//quads
	for (int j = 0; j < m_quadsG.size(); j++) {
		for (int k = 0; k < 4; k++) {
			m_normals[m_quadsG[j].v[k].n] += normalsQ[j]; //sum vertex normal with face normal
		}
	}
	for (int j = 0; j < m_quadsG.size(); j++) {
		for (int k = 0; k < 4; k++) {
			normalize(m_normals[m_quadsG[j].v[k].n]);
		}
	}
}

void Geometry::createTangents() {

	std::vector<cgra::vec3> tangents; //triangle face normals
	for (int i = 0; i < m_triangles.size(); i++) {

		vec3 v0 = m_points[m_triangles[i].v[0].p];
		vec3 v1 = m_points[m_triangles[i].v[1].p];
		vec3 v2 = m_points[m_triangles[i].v[2].p];

		vec2 uv0 = m_uvs[m_triangles[i].v[0].t];
		vec2 uv1 = m_uvs[m_triangles[i].v[1].t];
		vec2 uv2 = m_uvs[m_triangles[i].v[2].t];

		vec3 deltaPos1 = v1 - v0;
		vec3 deltaPos2 = v2 - v0;

		vec2 deltaUV1 = uv1 - uv0;
		vec2 deltaUV2 = uv2 - uv0;

		float r = 1.0f / (deltaUV1.x * deltaUV2.y - deltaUV1.y * deltaUV2.x);
		vec3 tangent = (deltaPos1 * deltaUV2.y - deltaPos2 * deltaUV1.y) * r;
		tangents.push_back(tangent);

		m_triangles[i].v[0].tang = m_triangles[i].v[0].p;	//set n as p because need normal for every vertex/point
		m_triangles[i].v[1].tang = m_triangles[i].v[1].p;
		m_triangles[i].v[2].tang = m_triangles[i].v[2].p;

		m_tangents.push_back(vec3(0, 0, 0));	//create 0 normal for each point/vertex
		m_tangents.push_back(vec3(0, 0, 0));
		m_tangents.push_back(vec3(0, 0, 0));
	}

	std::vector<cgra::vec3> tangentsQ; //quad face normals
	for (int i = 0; i < m_quadsG.size(); i++) {

		vec3 v0 = m_points[m_quadsG[i].v[0].p];
		vec3 v1 = m_points[m_quadsG[i].v[1].p];
		vec3 v2 = m_points[m_quadsG[i].v[2].p];

		vec2 uv0 = m_uvs[m_quadsG[i].v[0].t];
		vec2 uv1 = m_uvs[m_quadsG[i].v[1].t];
		vec2 uv2 = m_uvs[m_quadsG[i].v[2].t];

		vec3 deltaPos1 = v1 - v0;
		vec3 deltaPos2 = v2 - v0;

		vec2 deltaUV1 = uv1 - uv0;
		vec2 deltaUV2 = uv2 - uv0;

		float r = 1.0f / (deltaUV1.x * deltaUV2.y - deltaUV1.y * deltaUV2.x);
		vec3 tangent = (deltaPos1 * deltaUV2.y - deltaPos2 * deltaUV1.y) * r;
		tangentsQ.push_back(tangent);

		m_quadsG[i].v[0].tang = m_quadsG[i].v[0].p;	//set n as p because need normal for every vertex/point
		m_quadsG[i].v[1].tang = m_quadsG[i].v[1].p;
		m_quadsG[i].v[2].tang = m_quadsG[i].v[2].p;
		m_quadsG[i].v[3].tang = m_quadsG[i].v[3].p;

		m_tangents.push_back(vec3(0, 0, 0));	//create 0 normal for each point/vertex
		m_tangents.push_back(vec3(0, 0, 0));
		m_tangents.push_back(vec3(0, 0, 0));
		m_tangents.push_back(vec3(0, 0, 0));
	}
	for (int j = 0; j < m_triangles.size(); j++) {
		for (int k = 0; k < 3; k++) {
			m_tangents[m_triangles[j].v[k].tang] += tangents[j]; //sum vertex normal with face normal
		}
	}
	for (int j = 0; j < m_quadsG.size(); j++) {
		for (int k = 0; k < 4; k++) {
			m_tangents[m_quadsG[j].v[k].tang] += tangentsQ[j]; //sum vertex normal with face normal
		}
	}
}

void Geometry::createVertex(vertex vert) {
	glNormal3f(m_normals[vert.n][0], m_normals[vert.n][1], m_normals[vert.n][2]);
	glTexCoord2f(m_uvs[vert.t][0], m_uvs[vert.t][1]);
	//glColor3f(m_tangents[vert.tang][0], m_tangents[vert.tang][1], m_tangents[vert.tang][2]);
	glVertex3f(m_points[vert.p][0], m_points[vert.p][1], m_points[vert.p][2]);
}

void Geometry::createDisplayListPoly() {
	// Delete old list if there is one
	if (m_displayListPoly) glDeleteLists(m_displayListPoly, 1);

	// Create a new list
	//cout << "Creating Poly Geometry" << endl;
	m_displayListPoly = glGenLists(1);
	glNewList(m_displayListPoly, GL_COMPILE);
	glBegin(GL_TRIANGLES);
	glColor3f(1.0f, 0.0f, 0.0f);
	for (triangle t : m_triangles) {

		//v0
		createVertex(t.v[0]);

		//v1
		createVertex(t.v[1]);

		//v2
		createVertex(t.v[2]);
	}
	glBegin(GL_QUADS);
	for (quadG q : m_quadsG) {
		//v0
		createVertex(q.v[0]);

		//v1
		createVertex(q.v[1]);

		//v2
		createVertex(q.v[2]);

		//v3
		createVertex(q.v[3]);
	}
	glEnd();
	glEndList();
	//cout << "Finished creating Poly Geometry" << endl;
}

void Geometry::createLine(vertex v1, vertex v2) {
	// v1
	glNormal3f(m_normals[v1.n][0], m_normals[v1.n][1], m_normals[v1.n][2]);
	glTexCoord2f(m_uvs[v1.t][0], m_uvs[v1.t][1]);
	glVertex3f(m_points[v1.p][0], m_points[v1.p][1], m_points[v1.p][2]);
	// v2
	glNormal3f(m_normals[v2.n][0], m_normals[v2.n][1], m_normals[v2.n][2]);
	glTexCoord2f(m_uvs[v2.t][0], m_uvs[v2.t][1]);
	glVertex3f(m_points[v2.p][0], m_points[v2.p][1], m_points[v2.p][2]);
}

void Geometry::createDisplayListWire() {
	// Delete old list if there is one
	if (m_displayListWire) glDeleteLists(m_displayListWire, 1);

	// Create a new list
	//cout << "Creating Wire Geometry" << endl;
	m_displayListWire = glGenLists(1);
	glNewList(m_displayListWire, GL_COMPILE);
	glBegin(GL_LINES);
	for (triangle t : m_triangles) {
		//join v0 to v1
		createLine(t.v[0], t.v[1]);
		//join v1 to v2
		createLine(t.v[1], t.v[2]);
		//join v2  to v0
		createLine(t.v[2], t.v[0]);
	}
	glEnd();
	glEndList();
	//cout << "Finished creating Wire Geometry" << endl;
}

void Geometry::render(GLuint shader) {

	material->setShaderMaterial(shader);

	glPushMatrix();
	glMatrixMode(GL_MODELVIEW);

	

	glTranslatef(translation[0], translation[1], translation[2]);

	glRotatef(rotation[0], 1.0f, 0.0f, 0.0f);
	glRotatef(rotation[1], 0.0f, 1.0f, 0.0f);
	glRotatef(rotation[2], 0.0f, 0.0f, 1.0f);
	glScalef(scale[0], scale[1], scale[2]);
	
	
	//render wireframe or mesh
	if (m_wireFrameOn) {
		glShadeModel(GL_SMOOTH);
		glCallList(m_displayListWire);
	}
	else {
		glShadeModel(GL_SMOOTH);
		glCallList(m_displayListPoly);
	}
	glPopMatrix();
}

void Geometry::toggleWireFrame() {
	m_wireFrameOn = !m_wireFrameOn;
}

void Geometry::initNeighbours() {

	neighbours.resize(m_points.size() - 1);

	//loop through triangles and add neighbours to set for each vertex
	for (triangle t : m_triangles) {

		neighbours[t.v[0].p - 1].insert(t.v[1].p);
		neighbours[t.v[0].p - 1].insert(t.v[2].p);

		neighbours[t.v[1].p - 1].insert(t.v[0].p);
		neighbours[t.v[1].p - 1].insert(t.v[2].p);

		neighbours[t.v[2].p - 1].insert(t.v[0].p);
		neighbours[t.v[2].p - 1].insert(t.v[1].p);

	}
	//loop through quads and add neighbours to set for each vertex
	for (quadG q : m_quadsG) {

		neighbours[q.v[0].p - 1].insert(q.v[1].p);
		neighbours[q.v[0].p - 1].insert(q.v[2].p);

		neighbours[q.v[1].p - 1].insert(q.v[0].p);
		neighbours[q.v[1].p - 1].insert(q.v[2].p);

		neighbours[q.v[2].p - 1].insert(q.v[1].p);
		neighbours[q.v[2].p - 1].insert(q.v[3].p);

		neighbours[q.v[3].p - 1].insert(q.v[2].p);
		neighbours[q.v[3].p - 1].insert(q.v[0].p);
	}
}

void Geometry::initLaplacianUniform() {

	cout << "Initialising Uniform Laplacian" << endl;

	//init Symmetric Laplacian matrix
	Lw_Uniform.resize(m_points.size() - 1, m_points.size() - 1);

	//init Mass matrix
	M_Uniform.resize(m_points.size() - 1, m_points.size() - 1);

	//set laplacian matrix values
	for (int i = 0; i < neighbours.size(); i++) {

		Lw_Uniform.insert(i, i) = double(neighbours[i].size());
		M_Uniform.insert(i, i) = double(neighbours[i].size());

		for (int j : neighbours[i]) {
			Lw_Uniform.insert(i, j - 1) = -1;
		}
	}

	L_Uniform = M_Uniform.cwiseInverse() * Lw_Uniform;
}

//calculate cotan weight for neighbour vertex and make it negative
double calcCotanWeight(vec3 vi, vec3 vj, std::vector<vec3> ab) {

	double weight = 0;

	for (vec3 vc : ab) {
		vec3 u = vi - vc;
		vec3 v = vj - vc;

		//float cosAlpha = dot(normalize(u), normalize(v));
		//float alpha = acos(cosAlpha);
		//float cotanAlpha = cos(alpha) / sin(alpha);

		double cotanAlpha = dot(normalize(u), normalize(v)) / (length(cross(normalize(u), normalize(v))));

		weight -= 0.5 * cotanAlpha;
	}

	return weight;
}

//circumcenter of triangle
vec2 circumcenter(vec2 a, vec2 b, vec2 c) {

	double d = 2 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
	double ux = ((a.x * a.x + a.y * a.y) * (b.y - c.y) + (b.x * b.x + b.y * b.y) * (c.y - a.y) + (c.x * c.x + c.y * c.y) * (a.y - b.y)) / d;
	double uy = ((a.x * a.x + a.y * a.y) * (c.x - b.x) + (b.x * b.x + b.y * b.y) * (a.x - c.x) + (c.x * c.x + c.y * c.y) * (b.x - a.x)) / d;

	return vec2(ux, uy);
}

//Voronoi Area for a single segment (convert to 2d and calculate)
double calcVoronoiSegmentArea(vec3 vi, vec3 vj, std::vector<vec3> ab) {

	//calc lengths of edges
	double vijL = std::abs(length(vj - vi));
	double viaL = std::abs(length(ab[0] - vi));

	//convert to 2d
	vec2 v1 = vec2(0, 0);
	vec2 v2 = vec2(vijL, 0);

	//length of cross product triangle A
	double l = length(cross(normalize(vj - vi), normalize(ab[0] - vi)));

	//this fixes a bug where 1 was returning greater than 1 for some reason, don't have time to figure out why so this is a temp fix
	if (l > 1) {
		l = 1;
	}

	//angle of triangle a
	double theta = asin(l);

	//point alpha in 2d
	vec2 v3 = vec2(viaL * cos(theta), viaL * sin(theta));

	//points for voronoi segment
	vec2 cA, cB, v4;

	if (theta < math::pi() / 2) {
		cA = circumcenter(v1, v2, v3);
	}
	else {
		cA = (v2 + v3) / 2;
	}

	if (ab.size() > 1) {

		double vibL = std::abs(length(ab[1] - vi));

		//length of cross product of triangle b
		double l2 = length(cross(normalize(vj - vi), normalize(ab[1] - vi)));

		//this fixes a bug where 1 was returning greater than 1 for some reason, don't have time to figure out why so this is a temp fix
		if (l2 > 1) {
			l2 = 1;
		}

		//angle of triangle b
		double theta2 = asin(l2);

		//point beta in 2d
		v4 = vec2(vibL * cos(-theta2), vibL * sin(-theta2));

		if (theta2 < math::pi() / 2) {
			cB = circumcenter(v1, v2, v4);
		}
		else {
			cB = (v2 + v4) / 2;
		}
	}
	else {
		cB = (v1 + v3) / 2;
	}

	double area = 0.5 * ((cA[0] - v1[0]) * (cB[1] - v1[1]) - (cB[0] - v1[0]) * (cA[1] - v1[1]));

	return abs(area);
}

void Geometry::initLaplacianCotan() {

	cout << "Initialising Cotangent Laplacian" << endl;

	//init Symmetric Laplacian matrix
	Lw_Cotan.resize(m_points.size() - 1, m_points.size() - 1);

	//init Mass matrix
	M_Cotan.resize(m_points.size() - 1, m_points.size() - 1);

	//calculate cotan weights
	for (int i = 1; i < m_points.size(); i++) {

		double voronoiArea = 0;

		//for sum of weights
		double w = 0;

		//set Vi
		vec3 vi = m_points[i];
		//get neighbours of Vi
		std::set<int> ni = neighbours[i - 1];

		//loop through all neighbours of Vi
		for (int n : ni) {

			//set Vj
			vec3 vj = m_points[n];
			//get neighbours of Vj
			std::set<int> nj = neighbours[n - 1];

			//for alpha and beta points
			std::vector<vec3> ab;

			//loop through neighbours of Vj
			for (int n2 : nj) {

				//if vertices is neighbour of Vi and Vj then use to calculate cotan weights
				if (ni.find(n2) != ni.end()) {

					ab.push_back(m_points[n2]);

				}
			}

			//set neighbour cotan weight
			Lw_Cotan.insert(i - 1, n - 1) = calcCotanWeight(vi, vj, ab);

			//add weight to total weight
			w += calcCotanWeight(vi, vj, ab);

			//Add area of voronoi segment to total voronoi vertex area
			voronoiArea += calcVoronoiSegmentArea(vi, vj, ab);
		}

		//set diagonal of Laplacian to sum of cotan weight
		Lw_Cotan.insert(i - 1, i - 1) = abs(w);
		//set diagonal of Mass Matrix to Voronoi Vertex Area
		M_Cotan.insert(i - 1, i - 1) = voronoiArea;
	}

	L_Cotan = M_Cotan.cwiseInverse() * Lw_Cotan;
}

//Sets the edges of cubes as features with 10x the weight, CGRA409 A2 Challenge
void Geometry::featurePreserve(Eigen::SparseMatrix<double>* M) {
	Eigen::SparseMatrix<double> F(m_points.size() - 1, m_points.size() - 1);
	F.setIdentity();
	double weightF = 10;

	//set edge vertices of cube as features
	for (int i = 1; i < m_points.size(); i++) {

		if (m_points[i].x >= 1 || m_points[i].x <= -1) {
			if (m_points[i].y >= 1 || m_points[i].y <= -1 || m_points[i].z >= 1 || m_points[i].z <= -1) {
				F.coeffRef(i - 1, i - 1) = weightF;
			}
		}
		if (m_points[i].y >= 1 || m_points[i].y <= -1) {
			if (m_points[i].x >= 1 || m_points[i].x <= -1 || m_points[i].z >= 1 || m_points[i].z <= -1) {
				F.coeffRef(i - 1, i - 1) = weightF;
			}
		}
		if (m_points[i].z >= 1 || m_points[i].z <= -1) {
			if (m_points[i].x >= 1 || m_points[i].x <= -1 || m_points[i].y >= 1 || m_points[i].y <= -1) {
				F.coeffRef(i - 1, i - 1) = weightF;
			}
		}
	}
	*M = F * *M;
}

//Smooth a mesh using Uniform Weights, CGRA409 A2 Core
void Geometry::smooth(float w, int extraIter, bool useCotan, bool featurePre) {

	//set initial time
	float time = glfwGetTime();

	Eigen::SparseMatrix<double> Lw, M;

	if (useCotan) {
		Lw = Lw_Cotan;
		M = M_Cotan;
	}
	else {
		Lw = Lw_Uniform;
		M = M_Uniform;
	}

	//feature preservation, second part of challenge. A bit of a hacky implementation, currently only for cube
	if (featurePre) {
		featurePreserve(&M);
	}

	//matrix of current vertex positions
	Eigen::MatrixXd p = Eigen::MatrixXd::Zero(m_points.size() - 1, 3);
	for (int k = 1; k < m_points.size(); k++){
		p(k - 1, 0) = m_points[k].x;
		p(k - 1, 1) = m_points[k].y;
		p(k - 1, 2) = m_points[k].z;
	}

	//solve new positions of vertices
	Eigen::SimplicialLLT<Eigen::SparseMatrix<double> > solver;
	solver.compute(Lw * M.cwiseInverse() *Lw + w * M);
	Eigen::MatrixXd pNew = solver.solve(w * M * p);

	//do extra iterations based on function parameter
	for (int l = 0; l < extraIter; l++) {
		pNew = solver.solve(w * M * pNew);
	}

	//new vertex positions
	for (int m = 1; m < m_points.size(); m++) {
		m_points[m].x = pNew(m - 1, 0);
		m_points[m].y = pNew(m - 1, 1);
		m_points[m].z = pNew(m - 1, 2);
	}

	//print how long the function took
	float currentTime = glfwGetTime();
	if (useCotan) {
		cout << "Finished Smoothing (Cotan Weights), ";
	}
	else {
		cout << "Finished Smoothing (Uniform Weights), ";
	}
	cout << "Time: " << currentTime - time << endl;
}

//Minimal Surface using Lagrange Multipliers and Uniform weights CGRA409 A2 Completion
void Geometry::minimalSurface(std::vector<int> constraints, bool useCotan) {

	//set initial time
	float time = glfwGetTime();

	Eigen::SparseMatrix<double> LHSKnown;
	if (useCotan) {
		LHSKnown = L_Cotan;
	}
	else {
		LHSKnown = L_Uniform;
	}

	LHSKnown.conservativeResize(m_points.size() - 1 + constraints.size(), m_points.size() - 1 + constraints.size());

	Eigen::MatrixXd RHS = Eigen::MatrixXd::Zero(m_points.size() - 1 + constraints.size(), 3);

	for (int i = 0; i < constraints.size(); i++) {
		
		LHSKnown.insert(m_points.size() - 1 + i, constraints[i]-1) = 1;
		LHSKnown.insert(constraints[i]-1, m_points.size() - 1 + i) = 1;

		for (int j = 0; j < 3; j++) {
			RHS(m_points.size() - 1 + i, j) = m_points[constraints[i]][j];
		}

	}

	Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
	solver.compute(LHSKnown);
	Eigen::MatrixXd LHSUnknown = solver.solve(RHS);
	
	//new vertex positions
	for (int k = 1; k < m_points.size(); k++) {
		m_points[k].x = LHSUnknown(k - 1, 0);
		m_points[k].y = LHSUnknown(k - 1, 1);
		m_points[k].z = LHSUnknown(k - 1, 2);
	}

	//print how long the function took
	float currentTime = glfwGetTime();
	if (useCotan) {
		cout << "Finished Minimal Surface (Cotan Weights), ";
	}
	else {
		cout << "Finished Minimal Surface (Uniform Weights), ";
	}
	cout << "Time: " << currentTime - time << endl;
}

std::vector<Eigen::Matrix3d> Geometry::ARAP_Local(bool useCotan) {

	//set initial time
	float time = glfwGetTime();

	std::vector<Eigen::Matrix3d> R;

	Eigen::MatrixXd V_old, V_new, D;
	vec3 pi, pj, qi, qj;
	Eigen::Vector3d Vpi, Vpj, Vqi, Vqj, vi_old, vi_new;
	std::set<int> n;

	Eigen::Matrix3d Cov, U, V, Ri;

	for (int i = 1; i < m_points.size(); i++) {

		pi = m_points_source[i];
		Vpi = Eigen::Vector3d(pi[0], pi[1], pi[2]);
		qi = m_points[i];
		Vqi = Eigen::Vector3d(qi[0], qi[1], qi[2]);

		n = neighbours[i - 1];

		V_old = Eigen::MatrixXd::Zero(3, n.size());
		V_new = Eigen::MatrixXd::Zero(3, n.size());
		D = Eigen::MatrixXd::Zero(n.size(), n.size());

		int col = 0;

		for (int index : n) {
			pj = m_points_source[index];
			Vpj = Eigen::Vector3d(pj[0], pj[1], pj[2]);
			qj = m_points[index];
			Vqj = Eigen::Vector3d(qj[0], qj[1], qj[2]);

			vi_old = Vpj - Vpi;
			vi_new = Vqj - Vqi;

			V_old.col(col) = vi_old;
			V_new.col(col) = vi_new;

			if (useCotan) {
				D(col, col) = -Lw_Cotan.coeff(i - 1, index - 1);
			}
			else {
				D(col, col) = -Lw_Uniform.coeff(i - 1, index - 1);
			}

			col++;
		}

		Cov = V_old * D * V_new.transpose();

		Eigen::JacobiSVD<Eigen::Matrix3d> svd(Cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
		U = svd.matrixU();
		V = svd.matrixV();
		Ri = V * U.transpose();

		if (Ri.determinant() < 0) {
			U.col(2) *= -1;
			Ri = V * U.transpose();
		}

		R.push_back(Ri);
	}

	return R;
}

Eigen::SparseMatrix<double> Geometry::ARAP_GlobalSubstitution(std::vector<int> constraints, mat4 transform, bool useCotan, Eigen::SimplicialLLT<Eigen::SparseMatrix<double>>& solver) {

	//set initial time
	float time = glfwGetTime();

	Eigen::SparseMatrix<double> L;
	if (useCotan) {
		L = Lw_Cotan;
	}
	else {
		L = Lw_Uniform;
	}

	//matrix of current vertex positions
	Eigen::MatrixXd pX = Eigen::MatrixXd::Zero(m_points.size() - 1, 1);
	Eigen::MatrixXd pY = Eigen::MatrixXd::Zero(m_points.size() - 1, 1);
	Eigen::MatrixXd pZ = Eigen::MatrixXd::Zero(m_points.size() - 1, 1);

	for (int k = 1; k < m_points.size(); k++) {
		pX(k - 1, 0) = m_points[k].x;
		pY(k - 1, 0) = m_points[k].y;
		pZ(k - 1, 0) = m_points[k].z;
	}

	Eigen::MatrixXd bX = L * pX;
	Eigen::MatrixXd bY = L * pY;
	Eigen::MatrixXd bZ = L * pZ;

	for (int k = 0; k < constraints.size(); k++) {

		if (constraints[k] == 0 || constraints[k] == 2) {
			
			vec4 v = vec4(m_points[k + 1], 1);
			if (constraints[k] == 2) {
				v = v * transform;
			}
			
			bX -= v[0] * L.col(k);
			bY -= v[1] * L.col(k);
			bZ -= v[2] * L.col(k);
			
			L.coeffRef(k, k) = 1;

			for (int j : neighbours[k]) {
				L.coeffRef(k, j - 1) = 0;
				L.coeffRef(j - 1, k) = 0;
			}

			bX(k, 0) = v[0];
			bY(k, 0) = v[1];
			bZ(k, 0) = v[2];
		}
	}

	solver.compute(L);
	Eigen::MatrixXd pXNew = solver.solve(bX);
	Eigen::MatrixXd pYNew = solver.solve(bY);
	Eigen::MatrixXd pZNew = solver.solve(bZ);

	//new vertex positions
	for (int k = 1; k < m_points.size(); k++) {
		m_points[k].x = pXNew(k - 1, 0);
		m_points[k].y = pYNew(k - 1, 0);
		m_points[k].z = pZNew(k - 1, 0);
	}

	//print how long the function took
	float currentTime = glfwGetTime();
	if (useCotan) {
		cout << "Finished ARAP Global Step Substitution (Cotan Weights), ";
	}
	else {
		cout << "Finished ARAP Global Step Substitution (Uniform Weights), ";
	}
	cout << "Time: " << currentTime - time << endl;
	return L;
}

void Geometry::ARAP_LocalAndGlobalSubstitution(std::vector<int> constraints, cgra::mat4 transform, bool useCotan, int iterations, Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> &solver) {

	//set initial time
	float time = glfwGetTime();

	std::vector<Eigen::Matrix3d> R;

	Eigen::MatrixXd pXNew, pYNew, pZNew;

	Eigen::Vector3d bi, Vpi, Vpj;
	vec3 pi, pj;
	double Wij;
	std::set<int> n;

	//matrix of current vertex positions
	Eigen::MatrixXd bX = Eigen::MatrixXd::Zero(m_points.size() - 1, 1);
	Eigen::MatrixXd bY = Eigen::MatrixXd::Zero(m_points.size() - 1, 1);
	Eigen::MatrixXd bZ = Eigen::MatrixXd::Zero(m_points.size() - 1, 1);

	for (int iter = 0; iter < iterations; iter++) {

		cout << "Iteration: " << iter + 1 << endl;

		Eigen::SparseMatrix<double> L;

		if (useCotan) {
			L = Lw_Cotan;
		}
		else {
			L = Lw_Uniform;
		}

		R = ARAP_Local(useCotan);

		for (int i = 1; i < m_points.size(); i++) {

			bi = Eigen::Vector3d::Zero();

			pi = m_points_source[i];
			Vpi = Eigen::Vector3d(pi[0], pi[1], pi[2]);

			n = neighbours[i - 1];

			for (int index : n) {

				pj = m_points_source[index];

				Vpj = Eigen::Vector3d(pj[0], pj[1], pj[2]);

				Wij = -L.coeff(i - 1, index - 1);

				bi += (Wij / 2) * (R[i - 1] + R[index - 1]) * (Vpi - Vpj);
			}

			bX(i - 1, 0) = bi[0];
			bY(i - 1, 0) = bi[1];
			bZ(i - 1, 0) = bi[2];
		}

		for (int k = 0; k < constraints.size(); k++) {

			if (constraints[k] == 0 || constraints[k] == 2) {

				vec4 v = vec4(m_points_source[k + 1], 1);
				if (constraints[k] == 2) {
					v = v * transform;
				}

				bX -= v.x * L.col(k);
				bY -= v.y * L.col(k);
				bZ -= v.z * L.col(k);

				L.coeffRef(k, k) = 1;

				for (int j : neighbours[k]) {
					L.coeffRef(k, j - 1) = 0;
					L.coeffRef(j - 1, k) = 0;
				}

				bX(k, 0) = v.x;
				bY(k, 0) = v.y;
				bZ(k, 0) = v.z;
			}
		}

		
		pXNew = solver.solve(bX);
		pYNew = solver.solve(bY);
		pZNew = solver.solve(bZ);

		//new vertex positions
		for (int k = 1; k < m_points.size(); k++) {
			m_points[k].x = pXNew(k - 1, 0);
			m_points[k].y = pYNew(k - 1, 0);
			m_points[k].z = pZNew(k - 1, 0);
		}
	}

	//print how long the function took
	float currentTime = glfwGetTime();
	if (useCotan) {
		cout << "Finished ARAP Local + Global Step Substitution " << iterations << " Iterations (Cotan Weights), ";
	}
	else {
		cout << "Finished ARAP Local + Global Step Substitution " << iterations << " Iterations (Uniform Weights), ";
	}
	cout << "Time: " << currentTime - time << endl;

}

void Geometry::ARAP_GlobalLagrange(std::vector<int> constraints, mat4 transform, bool useCotan, Eigen::SparseLU<Eigen::SparseMatrix<double>>& solver, Eigen::MatrixXd& LHSUnknown, Eigen::MatrixXd& RHS) {

	//set initial time
	float time = glfwGetTime();

	//matrix of current vertex positions
	Eigen::MatrixXd P = Eigen::MatrixXd::Zero(m_points.size() - 1, 3);

	for (int k = 1; k < m_points.size(); k++) {
		P(k - 1, 0) = m_points[k].x;
		P(k - 1, 1) = m_points[k].y;
		P(k - 1, 2) = m_points[k].z;
	}

	Eigen::SparseMatrix<double> LHSKnown;
	if (useCotan) {
		LHSKnown = Lw_Cotan;
	}
	else {
		LHSKnown = Lw_Uniform;
	}

	RHS = LHSKnown * P;

	std::map<int, vec4> c;

	for (int k = 0; k < constraints.size(); k++) {

		if (constraints[k] == 0 || constraints[k] == 2) {

			vec4 v = vec4(m_points_source[k + 1], 1);
			if (constraints[k] == 2) {
				v = v * transform;
			}

			c[k] = v;

		}
	}

	LHSKnown.conservativeResize(LHSKnown.rows() + c.size(), LHSKnown.cols() + c.size());
	RHS.conservativeResize(RHS.rows() + c.size(), RHS.cols());

	int x = 0;
	map<int, vec4>::iterator it;
	for (it = c.begin(); it != c.end(); it++) {
		LHSKnown.coeffRef(m_points.size() - 1 + x, it->first) = 1;
		LHSKnown.coeffRef(it->first, m_points.size() - 1 + x) = 1;

		for (int j = 0; j < 3; j++) {
			RHS(m_points.size() - 1 + x, j) = it->second[j];
		}
		x++;
	}

	solver.compute(LHSKnown);
	LHSUnknown = solver.solve(RHS);

	//new vertex positions
	for (int k = 1; k < m_points.size(); k++) {
		m_points[k].x = LHSUnknown(k - 1, 0);
		m_points[k].y = LHSUnknown(k - 1, 1);
		m_points[k].z = LHSUnknown(k - 1, 2);
	}

	//print how long the function took
	float currentTime = glfwGetTime();
	if (useCotan) {
		cout << "Finished ARAP Global Step Lagrange (Cotan Weights), ";
	}
	else {
		cout << "Finished ARAP Global Step Lagrange (Uniform Weights), ";
	}
	cout << "Time: " << currentTime - time << endl;
}

void Geometry::ARAP_LocalAndGlobalLagrange(std::vector<int> constraints, mat4 transform, bool useCotan, int iterations, Eigen::SparseLU<Eigen::SparseMatrix<double>>& solver, Eigen::MatrixXd& LHSUnknown, Eigen::MatrixXd& RHS) {

	std::vector<Eigen::Matrix3d> R;
	Eigen::Vector3d bi, Vpi, Vpj;
	vec3 pi, pj;
	std::set<int> n;
	double Wij;

	//set initial time
	float time = glfwGetTime();

	for (int iter = 0; iter < iterations; iter++) {

		cout << "Iteration: " << iter + 1 << endl;

		//calculate rotations - local step
		R = ARAP_Local(useCotan);

		//recalculate b
		for (int i = 1; i < m_points.size(); i++) {

			bi = Eigen::Vector3d::Zero();

			pi = m_points_source[i];
			Vpi = Eigen::Vector3d(pi[0], pi[1], pi[2]);

			n = neighbours[i - 1];

			for (int index : n) {

				pj = m_points_source[index];
				Vpj = Eigen::Vector3d(pj[0], pj[1], pj[2]);

				if (useCotan) {
					Wij = -Lw_Cotan.coeff(i - 1, index - 1);
				}
				else{
					Wij = -Lw_Uniform.coeff(i - 1, index - 1);
				}

				bi += (Wij / 2) * (R[i - 1] + R[index - 1]) * (Vpi - Vpj);
			}

			RHS.row(i - 1) = bi;
		}

		LHSUnknown = solver.solve(RHS);

		//new vertex positions
		for (int k = 1; k < m_points.size(); k++) {
			m_points[k].x = LHSUnknown(k - 1, 0);
			m_points[k].y = LHSUnknown(k - 1, 1);
			m_points[k].z = LHSUnknown(k - 1, 2);
		}
	}

	//print how long the function took
	float currentTime = glfwGetTime();
	if (useCotan) {
		cout << "Finished ARAP Local + Global Step Lagrange " << iterations << " Iterations (Cotan Weights), ";
	}
	else {
		cout << "Finished ARAP Local + Global Step Lagrange " << iterations << " Iterations (Uniform Weights), ";
	}
	cout << "Time: " << currentTime - time << endl;
}

std::vector<cgra::vec3> Geometry::getPoints() {
	return m_points;
}

void Geometry::setNeighbours(std::vector<std::set<int>> n) {
	neighbours = n;
}

void Geometry::setLaplacianUniform(Eigen::SparseMatrix<double> Lw, Eigen::SparseMatrix<double> M, Eigen::SparseMatrix<double> L) {
	Lw_Uniform = Lw;
	M_Uniform = M;
	L_Uniform = L;
}

void Geometry::setLaplacianCotan(Eigen::SparseMatrix<double> Lw, Eigen::SparseMatrix<double> M, Eigen::SparseMatrix<double> L) {
	Lw_Cotan = Lw;
	M_Cotan = M;
	L_Cotan = L;
}

void Geometry::loadPoints(std::vector<cgra::vec3> points) {
	m_points = points;
}

void Geometry::updateMesh() {
	//recalculate normals
	createNormals();
	//recreate display lists
	createDisplayListPoly();
	createDisplayListWire();
}