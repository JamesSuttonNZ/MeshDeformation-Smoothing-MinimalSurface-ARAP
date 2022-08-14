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

#pragma once

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <Eigen/Sparse>

#include "cgra_math.h"
#include "GL/glew.h"
#include "GLFW/glfw3.h"
#include "material.h"

struct vertex {
	int p = 0; // index for point in m_points
	int t = 0; // index for uv in m_uvs
	int n = 0; // index for normal in m_normals
	int tang = 0; // index for tang in m_tangents
};

struct triangle {
	vertex v[3]; //requires 3 verticies
};

struct quadG {
	vertex v[4]; //requires 3 verticies
};

class Geometry {
private:

	// Feilds for storing raw obj information
	std::string m_filename;

	std::vector<cgra::vec3> m_points;	// Current Point list
	std::vector<cgra::vec3> m_points_source;	// ARAP - Initial Points
	std::vector<cgra::vec2> m_uvs;		// Texture Coordinate list
	std::vector<cgra::vec3> m_normals;	// Normal list
	std::vector<cgra::vec3> m_tangents;	// Tangent list
	//std::vector<cgra::vec3> m_bitangents;	// Bitangent list
	std::vector<triangle> m_triangles;	// Triangle/Face list
	std::vector<quadG> m_quadsG;	// Triangle/Face list

	bool m_wireFrameOn = false;

	// IDs for the display list to render
	GLuint m_displayListPoly = 0; // DisplayList for Polygon
	GLuint m_displayListWire = 0; // DisplayList for Wireframe

	void readOBJ(std::string);

	
	void createVertex(vertex vert);
	void createLine(vertex v1, vertex v2);

public:

	cgra::vec3 translation;
	cgra::vec3 rotation;
	cgra::vec3 scale;
	float uvScale;
	Material* material;

	std::vector<std::set<int>> neighbours;

	Eigen::SparseMatrix<double> Lw_Uniform;
	Eigen::SparseMatrix<double> M_Uniform;
	Eigen::SparseMatrix<double> L_Uniform;

	Eigen::SparseMatrix<double> Lw_Cotan;
	Eigen::SparseMatrix<double> M_Cotan;
	Eigen::SparseMatrix<double> L_Cotan;

	Geometry(std::string, cgra::vec3 trans, cgra::vec3 rot, cgra::vec3 sca, float uv, Material* mat);
	~Geometry();

	void render(GLuint shader);
	void toggleWireFrame();
	void createNormals();
	void createTangents();
	void createDisplayListPoly();
	void createDisplayListWire();

	void initNeighbours();
	void initLaplacianUniform();
	void initLaplacianCotan();
	void loadPoints(std::vector<cgra::vec3> points);
	void updateMesh();

	void smooth(float w, int extraIter, bool useCotan, bool featurePreserve);
	void minimalSurface(std::vector<int> constraints, bool useCotan);

	Eigen::SparseMatrix<double> ARAP_GlobalSubstitution(std::vector<int> constraints, cgra::mat4 transform, bool useCotan, Eigen::SimplicialLLT<Eigen::SparseMatrix<double>>& solver);
	void ARAP_LocalAndGlobalSubstitution(std::vector<int> constraints, cgra::mat4 transform, bool useCotan, int iterations, Eigen::SimplicialLLT<Eigen::SparseMatrix<double>>& solver);

	void ARAP_GlobalLagrange(std::vector<int> constraints, cgra::mat4 transform, bool useCotan, Eigen::SparseLU<Eigen::SparseMatrix<double>> &solver, Eigen::MatrixXd &LHSUnknown, Eigen::MatrixXd &RHS);
	void ARAP_LocalAndGlobalLagrange(std::vector<int> constraints, cgra::mat4 transform, bool useCotan, int iterations, Eigen::SparseLU<Eigen::SparseMatrix<double>>& solver, Eigen::MatrixXd& LHSUnknown, Eigen::MatrixXd& RHS);


	std::vector<Eigen::Matrix3d> ARAP_Local(bool useCotan);
	
	void featurePreserve(Eigen::SparseMatrix<double>* M);
	std::vector<cgra::vec3> getPoints();

	void setNeighbours(std::vector<std::set<int>> neighbours);
	void setLaplacianUniform(Eigen::SparseMatrix<double> Lw, Eigen::SparseMatrix<double> M, Eigen::SparseMatrix<double> L);
	void setLaplacianCotan(Eigen::SparseMatrix<double> Lw, Eigen::SparseMatrix<double> M, Eigen::SparseMatrix<double> L);
};
