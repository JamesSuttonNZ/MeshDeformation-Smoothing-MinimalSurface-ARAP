#include <cmath>
#include <iostream> // input/output streams
#include <fstream>  // file streams
#include <sstream>  // string streams
#include <string>
#include <stdexcept>
#include <vector>
#include <set>
#include <Eigen/Sparse>
#include <Eigen/Dense>

#include "cgra_math.h"
#include "scene.h"
#include "camera.h"
#include "light.h"
#include "material.h"

using namespace std;
using namespace cgra;

int iterations_uniform = 100;
int iterations_cotangent = 100;
int iterations_uniform_laplacian_editing = 100;
int iterations_cotangent_laplacian_editing = 100;

Scene::Scene(std::vector<Geometry*> objs, std::vector<Light*> ls, Geometry* lem) {
	objects = objs;
	lights = ls;
	lap_edit_mesh = lem;
}

std::vector<int> Scene::readConstraints(string filename) {

	std::vector<int> constraints;

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
		
		//skip comments
		if (objLine.peek() != 35) {

			int c;
			objLine >> c;

			// Reading like this means whitespace at the start of the line is fine
			// attempting to read from an empty string/line will set the failbit
			if (!objLine.fail()) {

				constraints.push_back(c);
			}
		}
	}
	return constraints;
	
}

cgra::mat4 Scene::readTransform(string filename) {

	std::vector<vec4> data;

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

		if (objLine.peek() != 35) {
			vec4 v;
			objLine >> v[0];

			// Reading like this means whitespace at the start of the line is fine
			// attempting to read from an empty string/line will set the failbit
			if (!objLine.fail()) {
				objLine >> v[1] >> v[2] >> v[3];
				data.push_back(v);
			}
		}
	}

	return mat4(data[0], data[1], data[2], data[3]);;
}

void Scene::initLaplacian() {
	objects[1]->initNeighbours();
	objects[1]->initLaplacianUniform();
	objects[1]->initLaplacianCotan();

	//set laplacian for other objects
	for (int i = 2; i < objects.size(); i++) {
		objects[i]->setNeighbours(objects[1]->neighbours);
		objects[i]->setLaplacianUniform(objects[1]->Lw_Uniform, objects[1]->M_Uniform, objects[1]->L_Uniform);
		objects[i]->setLaplacianCotan(objects[1]->Lw_Cotan, objects[1]->M_Cotan, objects[1]->L_Cotan);
	}
}

void Scene::initARAP_Lagrange(std::vector<int> constraints, mat4 transform, int iterUniform, int iterCotan, int iterUniformLap, int iterCotanLap) {

	//with global step initial guess
	//uniform
	Eigen::SparseLU<Eigen::SparseMatrix<double>> solver1;
	Eigen::MatrixXd LHSUnknown1, RHS1;
	objects[1]->ARAP_GlobalLagrange(constraints, transform, false, solver1, LHSUnknown1, RHS1);
	objects[3]->ARAP_LocalAndGlobalLagrange(constraints, transform, false, iterUniform, solver1, LHSUnknown1, RHS1);
	//cotan
	Eigen::SparseLU<Eigen::SparseMatrix<double>> solver2;
	Eigen::MatrixXd LHSUnknown2, RHS2;
	objects[2]->ARAP_GlobalLagrange(constraints, transform, true, solver2, LHSUnknown2, RHS2);
	objects[4]->ARAP_LocalAndGlobalLagrange(constraints, transform, true, iterCotan, solver2, LHSUnknown2, RHS2);

	//with laplacian editing initial step
	//uniform
	objects[5]->loadPoints(lap_edit_mesh->getPoints());
	Eigen::SparseLU<Eigen::SparseMatrix<double>> solver3;
	Eigen::MatrixXd LHSUnknown3, RHS3;
	objects[7]->ARAP_GlobalLagrange(constraints, transform, false, solver3, LHSUnknown3, RHS3);
	objects[7]->loadPoints(lap_edit_mesh->getPoints());
	objects[7]->ARAP_LocalAndGlobalLagrange(constraints, transform, false, iterUniformLap, solver3, LHSUnknown3, RHS3);
	//cotan
	objects[6]->loadPoints(lap_edit_mesh->getPoints());
	Eigen::SparseLU<Eigen::SparseMatrix<double>> solver4;
	Eigen::MatrixXd LHSUnknown4, RHS4;
	objects[8]->ARAP_GlobalLagrange(constraints, transform, true, solver4, LHSUnknown4, RHS4);
	objects[8]->loadPoints(lap_edit_mesh->getPoints());
	objects[8]->ARAP_LocalAndGlobalLagrange(constraints, transform, true, iterCotanLap, solver4, LHSUnknown4, RHS4);
}

void Scene::initARAP_Substitution(std::vector<int> constraints, mat4 transform, int iterUniform, int iterCotan, int iterUniformLap, int iterCotanLap) {
	
	//with global step initial guess
	//uniform
	Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver1;
	objects[1]->ARAP_GlobalSubstitution(constraints, transform, false, solver1);
	objects[3]->ARAP_LocalAndGlobalSubstitution(constraints, transform, false, iterUniform, solver1);
	//cotan
	Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver2;
	objects[2]->ARAP_GlobalSubstitution(constraints, transform, true, solver2);
	objects[4]->ARAP_LocalAndGlobalSubstitution(constraints, transform, true, iterCotan, solver2);

	//with laplacian editing initial step
	//uniform
	objects[5]->loadPoints(lap_edit_mesh->getPoints());
	Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver3;
	objects[7]->ARAP_GlobalSubstitution(constraints, transform, false, solver3);
	objects[7]->loadPoints(lap_edit_mesh->getPoints());
	objects[7]->ARAP_LocalAndGlobalSubstitution(constraints, transform, false, iterUniformLap, solver3);
	//cotan
	objects[6]->loadPoints(lap_edit_mesh->getPoints());
	Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver4;
	objects[8]->ARAP_GlobalSubstitution(constraints, transform, true, solver4);
	objects[8]->loadPoints(lap_edit_mesh->getPoints());
	objects[8]->ARAP_LocalAndGlobalSubstitution(constraints, transform, true, iterCotanLap, solver4);
}


Scene Scene::barScene() {
	 
	std::vector<Geometry*> objs;
	std::vector<Light*> ls;

	//create light
	ls.push_back(new DirectionalLight(GL_LIGHT0, vec3(0,0,-1), vec4(1), vec4(0.8), vec4(0.2)));

	//create material
	Material* red = new Material("Red", vec4(1.0f, 0.0f, 0.0f, 1.0f), vec4(1.0f, 0.0f, 0.0f, 1.0f), vec4(1.0f, 0.0f, 0.0f, 1.0f), 100.0f);
	Material* cyan = new Material("Cyan", vec4(0.0f, 1.0f, 1.0f, 1.0f), vec4(0.0f, 1.0f, 1.0f, 1.0f), vec4(0.0f, 1.0f, 1.0f, 1.0f), 100.0f);
	Material* green = new Material("Green", vec4(0.0f, 1.0f, 0.0f, 1.0f), vec4(0.0f, 1.0f, 0.0f, 1.0f), vec4(0.0f, 1.0f, 0.0f, 1.0f), 100.0f);
	Material* yellow = new Material("Yellow", vec4(1.0f, 1.0f, 0.0f, 1.0f), vec4(1.0f, 1.0f, 0.0f, 1.0f), vec4(1.0f, 1.0f, 0.0f, 1.0f), 100.0f);

	//create objects
	objs.push_back(new Geometry("./Assets/ARAP/02-bar-twist/00-bar-original.obj", vec3(0, -2, -20), vec3(-90, 0, 0), vec3(0.1), 1, red));
	objs.push_back(new Geometry("./Assets/ARAP/02-bar-twist/00-bar-original.obj", vec3(-3, -2, -20), vec3(-90, 0, 0), vec3(0.1), 1, green));
	objs.push_back(new Geometry("./Assets/ARAP/02-bar-twist/00-bar-original.obj", vec3(3, -2, -20), vec3(-90, 0, 0), vec3(0.1), 1, yellow));
	objs.push_back(new Geometry("./Assets/ARAP/02-bar-twist/00-bar-original.obj", vec3(-6, -2, -20), vec3(-90, 0, 0), vec3(0.1), 1, green));
	objs.push_back(new Geometry("./Assets/ARAP/02-bar-twist/00-bar-original.obj", vec3(6, -2, -20), vec3(-90, 0, 0), vec3(0.1), 1, yellow));
	objs.push_back(new Geometry("./Assets/ARAP/02-bar-twist/00-bar-original.obj", vec3(-3, -11, -20), vec3(-90, 0, 0), vec3(0.1), 1, cyan));
	objs.push_back(new Geometry("./Assets/ARAP/02-bar-twist/00-bar-original.obj", vec3(3, -11, -20), vec3(-90, 0, 0), vec3(0.1), 1, cyan));
	objs.push_back(new Geometry("./Assets/ARAP/02-bar-twist/00-bar-original.obj", vec3(-6, -11, -20), vec3(-90, 0, 0), vec3(0.1), 1, green));
	objs.push_back(new Geometry("./Assets/ARAP/02-bar-twist/00-bar-original.obj", vec3(6, -11, -20), vec3(-90, 0, 0), vec3(0.1), 1, yellow));

	Geometry* lap_edit_mesh = new Geometry("./Assets/ARAP/02-bar-twist/04-bar-laplacian_editing.obj", vec3(0), vec3(-90, 0, 0), vec3(0.1), 1, NULL);

	return Scene(objs, ls, lap_edit_mesh);
}

void Scene::initBarScene(bool lagrange) {

	cout << "Initialising Bar Scene..." << endl;

	std::vector<int> constraints = readConstraints("./Assets/ARAP/02-bar-twist/bar.sel");
	mat4 transform = readTransform("./Assets/ARAP/02-bar-twist/bar.def");

	initLaplacian();

	if (lagrange) {
		initARAP_Lagrange(constraints, transform, iterations_uniform, iterations_cotangent, iterations_uniform_laplacian_editing, iterations_cotangent_laplacian_editing);
	}
	else {
		initARAP_Substitution(constraints, transform, iterations_uniform, iterations_cotangent, iterations_uniform_laplacian_editing, iterations_cotangent_laplacian_editing);
	}

	updateObjects();

	init = true;
}

Scene Scene::cylinderScene() {

	std::vector<Geometry*> objs;
	std::vector<Light*> ls;

	//create light
	ls.push_back(new DirectionalLight(GL_LIGHT0, vec3(0, 0, -1), vec4(1), vec4(0.8), vec4(0.2)));

	//create material
	Material* red = new Material("Red", vec4(1.0f, 0.0f, 0.0f, 1.0f), vec4(1.0f, 0.0f, 0.0f, 1.0f), vec4(1.0f, 0.0f, 0.0f, 1.0f), 100.0f);
	Material* cyan = new Material("Cyan", vec4(0.0f, 1.0f, 1.0f, 1.0f), vec4(0.0f, 1.0f, 1.0f, 1.0f), vec4(0.0f, 1.0f, 1.0f, 1.0f), 100.0f);
	Material* green = new Material("Green", vec4(0.0f, 1.0f, 0.0f, 1.0f), vec4(0.0f, 1.0f, 0.0f, 1.0f), vec4(0.0f, 1.0f, 0.0f, 1.0f), 100.0f);
	Material* yellow = new Material("Yellow", vec4(1.0f, 1.0f, 0.0f, 1.0f), vec4(1.0f, 1.0f, 0.0f, 1.0f), vec4(1.0f, 1.0f, 0.0f, 1.0f), 100.0f);

	/*objs.push_back(new Geometry("./Assets/cubeSimple.obj", vec3(0, 0, -100), vec3(0, 0, 0), vec3(10), 1, emerald));
	objs[0]->smooth(1);*/
	objs.push_back(new Geometry("./Assets/ARAP/01-cylinder-bend/00-cylinder-original.obj", vec3(0, 5, -20), vec3(90, 0, 0), vec3(0.5), 1, red));
	objs.push_back(new Geometry("./Assets/ARAP/01-cylinder-bend/00-cylinder-original.obj", vec3(-2, 5, -20), vec3(90, 0, 0), vec3(0.5), 1, green));
	objs.push_back(new Geometry("./Assets/ARAP/01-cylinder-bend/00-cylinder-original.obj", vec3(4, 5, -20), vec3(90, 0, 0), vec3(0.5), 1, yellow));
	objs.push_back(new Geometry("./Assets/ARAP/01-cylinder-bend/00-cylinder-original.obj", vec3(-6, 5, -20), vec3(90, 0, 0), vec3(0.5), 1, green));
	objs.push_back(new Geometry("./Assets/ARAP/01-cylinder-bend/00-cylinder-original.obj", vec3(8, 5, -20), vec3(90, 0, 0), vec3(0.5), 1, yellow));
	objs.push_back(new Geometry("./Assets/ARAP/01-cylinder-bend/00-cylinder-original.obj", vec3(-2, -3, -20), vec3(90, 0, 0), vec3(0.5), 1, cyan));
	objs.push_back(new Geometry("./Assets/ARAP/01-cylinder-bend/00-cylinder-original.obj", vec3(4, -3, -20), vec3(90, 0, 0), vec3(0.5), 1, cyan));
	objs.push_back(new Geometry("./Assets/ARAP/01-cylinder-bend/00-cylinder-original.obj", vec3(-6, -3, -20), vec3(90, 0, 0), vec3(0.5), 1, green));
	objs.push_back(new Geometry("./Assets/ARAP/01-cylinder-bend/00-cylinder-original.obj", vec3(8, -3, -20), vec3(90, 0, 0), vec3(0.5), 1, yellow));

	Geometry* lap_edit_mesh = new Geometry("./Assets/ARAP/01-cylinder-bend/04-cylinder-laplacian_editing.obj", vec3(0), vec3(90, 0, 0), vec3(0.5), 1, NULL);

	return Scene(objs, ls, lap_edit_mesh);
}

void Scene::initCylinderScene(bool lagrange) {

	cout << "Initialising Cylinder Scene..." << endl;

	std::vector<int> constraints = readConstraints("./Assets/ARAP/01-cylinder-bend/cylinder.sel");
	mat4 transform = readTransform("./Assets/ARAP/01-cylinder-bend/cylinder.def");

	initLaplacian();

	if (lagrange) {
		initARAP_Lagrange(constraints, transform, iterations_uniform, iterations_cotangent, iterations_uniform_laplacian_editing, iterations_cotangent_laplacian_editing);
	}
	else {
		initARAP_Substitution(constraints, transform, iterations_uniform, iterations_cotangent, iterations_uniform_laplacian_editing, iterations_cotangent_laplacian_editing);
	}

	updateObjects();

	init = true;
}

Scene Scene::knubbelScene() {

	std::vector<Geometry*> objs;
	std::vector<Light*> ls;

	//create light
	ls.push_back(new DirectionalLight(GL_LIGHT0, vec3(0, 0, -1), vec4(1), vec4(0.8), vec4(0.2)));

	//create material
	Material* red = new Material("Red", vec4(1.0f, 0.0f, 0.0f, 1.0f), vec4(1.0f, 0.0f, 0.0f, 1.0f), vec4(1.0f, 0.0f, 0.0f, 1.0f), 100.0f);
	Material* cyan = new Material("Cyan", vec4(0.0f, 1.0f, 1.0f, 1.0f), vec4(0.0f, 1.0f, 1.0f, 1.0f), vec4(0.0f, 1.0f, 1.0f, 1.0f), 100.0f);
	Material* green = new Material("Green", vec4(0.0f, 1.0f, 0.0f, 1.0f), vec4(0.0f, 1.0f, 0.0f, 1.0f), vec4(0.0f, 1.0f, 0.0f, 1.0f), 100.0f);
	Material* yellow = new Material("Yellow", vec4(1.0f, 1.0f, 0.0f, 1.0f), vec4(1.0f, 1.0f, 0.0f, 1.0f), vec4(1.0f, 1.0f, 0.0f, 1.0f), 100.0f);

	//create objects
	objs.push_back(new Geometry("./Assets/ARAP/00-plane-lift/00-knubbel-original.obj", vec3(-1, 0, -20), vec3(0, 0, 0), vec3(0.025), 1, red));
	objs.push_back(new Geometry("./Assets/ARAP/00-plane-lift/00-knubbel-original.obj", vec3(-4, 0, -20), vec3(0, 0, 0), vec3(0.025), 1, green));
	objs.push_back(new Geometry("./Assets/ARAP/00-plane-lift/00-knubbel-original.obj", vec3(2, 0, -20), vec3(0, 0, 0), vec3(0.025), 1, yellow));
	objs.push_back(new Geometry("./Assets/ARAP/00-plane-lift/00-knubbel-original.obj", vec3(-7, 0, -20), vec3(0, 0, 0), vec3(0.025), 1, green));
	objs.push_back(new Geometry("./Assets/ARAP/00-plane-lift/00-knubbel-original.obj", vec3(5, 0, -20), vec3(0, 0, 0), vec3(0.025), 1, yellow));
	objs.push_back(new Geometry("./Assets/ARAP/00-plane-lift/00-knubbel-original.obj", vec3(-4, -3, -20), vec3(0, 0, 0), vec3(0.025), 1, cyan));
	objs.push_back(new Geometry("./Assets/ARAP/00-plane-lift/00-knubbel-original.obj", vec3(2, -3, -20), vec3(0, 0, 0), vec3(0.025), 1, cyan));
	objs.push_back(new Geometry("./Assets/ARAP/00-plane-lift/00-knubbel-original.obj", vec3(-7, -3, -20), vec3(0, 0, 0), vec3(0.025), 1, green));
	objs.push_back(new Geometry("./Assets/ARAP/00-plane-lift/00-knubbel-original.obj", vec3(5, -3, -20), vec3(0, 0, 0), vec3(0.025), 1, yellow));

	Geometry* lap_edit_mesh = new Geometry("./Assets/ARAP/00-plane-lift/04-knubbel-laplacian_editing.obj", vec3(0), vec3(0, 0, 0), vec3(0.025), 1, NULL);

	return Scene(objs, ls, lap_edit_mesh);
}

void Scene::initKnubbelScene(bool lagrange) {

	cout << "Initialising Knubbel Scene (this one takes a while) ..." << endl;

	std::vector<int> constraints = readConstraints("./Assets/ARAP/00-plane-lift/knubbel.sel");
	mat4 transform = readTransform("./Assets/ARAP/00-plane-lift/knubbel.def");

	initLaplacian();

	if (lagrange) {
		initARAP_Lagrange(constraints, transform, iterations_uniform, iterations_cotangent, iterations_uniform_laplacian_editing, iterations_cotangent_laplacian_editing);
	}
	else {
		initARAP_Substitution(constraints, transform, iterations_uniform, iterations_cotangent, iterations_uniform_laplacian_editing, iterations_cotangent_laplacian_editing);
	}

	updateObjects();

	init = true;
}

Scene Scene::cactusScene() {

	std::vector<Geometry*> objs;
	std::vector<Light*> ls;

	//create light
	ls.push_back(new DirectionalLight(GL_LIGHT0, vec3(0, 0, -1), vec4(1), vec4(0.8), vec4(0.2)));

	//create material
	Material* red = new Material("Red", vec4(1.0f, 0.0f, 0.0f, 1.0f), vec4(1.0f, 0.0f, 0.0f, 1.0f), vec4(1.0f, 0.0f, 0.0f, 1.0f), 100.0f);
	Material* cyan = new Material("Cyan", vec4(0.0f, 1.0f, 1.0f, 1.0f), vec4(0.0f, 1.0f, 1.0f, 1.0f), vec4(0.0f, 1.0f, 1.0f, 1.0f), 100.0f);
	Material* green = new Material("Green", vec4(0.0f, 1.0f, 0.0f, 1.0f), vec4(0.0f, 1.0f, 0.0f, 1.0f), vec4(0.0f, 1.0f, 0.0f, 1.0f), 100.0f);
	Material* yellow = new Material("Yellow", vec4(1.0f, 1.0f, 0.0f, 1.0f), vec4(1.0f, 1.0f, 0.0f, 1.0f), vec4(1.0f, 1.0f, 0.0f, 1.0f), 100.0f);

	// create objects
	objs.push_back(new Geometry("./Assets/ARAP/03-cactus-bend/00-cactus-original.obj", vec3(-2.5, 0, -25), vec3(-90, 0, -90), vec3(6), 1, red));
	objs.push_back(new Geometry("./Assets/ARAP/03-cactus-bend/00-cactus-original.obj", vec3(-7.5, 0, -25), vec3(-90, 0, -90), vec3(6), 1, green));
	objs.push_back(new Geometry("./Assets/ARAP/03-cactus-bend/00-cactus-original.obj", vec3(0.5, 0, -25), vec3(-90, 0, -90), vec3(6), 1, yellow));
	objs.push_back(new Geometry("./Assets/ARAP/03-cactus-bend/00-cactus-original.obj", vec3(-11.5, 0, -25), vec3(-90, 0, -90), vec3(6), 1, green));
	objs.push_back(new Geometry("./Assets/ARAP/03-cactus-bend/00-cactus-original.obj", vec3(4.5, 0, -25), vec3(-90, 0, -90), vec3(6), 1, yellow));
	objs.push_back(new Geometry("./Assets/ARAP/03-cactus-bend/00-cactus-original.obj", vec3(-7.5, -6, -25), vec3(-90, 0, -90), vec3(6), 1, cyan));
	objs.push_back(new Geometry("./Assets/ARAP/03-cactus-bend/00-cactus-original.obj", vec3(0.5, -6, -25), vec3(-90, 0, -90), vec3(6), 1, cyan));
	objs.push_back(new Geometry("./Assets/ARAP/03-cactus-bend/00-cactus-original.obj", vec3(-11.5, -6, -25), vec3(-90, 0, -90), vec3(6), 1, green));
	objs.push_back(new Geometry("./Assets/ARAP/03-cactus-bend/00-cactus-original.obj", vec3(4.5, -6, -25), vec3(-90, 0, -90), vec3(6), 1, yellow));

	Geometry* lap_edit_mesh = new Geometry("./Assets/ARAP/03-cactus-bend/04-cactus-laplacian_editing.obj", vec3(0), vec3(-90, 0, -90), vec3(6), 1, NULL);

	return Scene(objs, ls, lap_edit_mesh);
}

void Scene::initCactusScene(bool lagrange) {

	cout << "Initialising Cactus Scene..." << endl;

	std::vector<int> constraints = readConstraints("./Assets/ARAP/03-cactus-bend/cactus.sel");
	mat4 transform = readTransform("./Assets/ARAP/03-cactus-bend/cactus.def");

	initLaplacian();

	if (lagrange) {
		initARAP_Lagrange(constraints, transform, iterations_uniform, iterations_cotangent, iterations_uniform_laplacian_editing, iterations_cotangent_laplacian_editing);
	}
	else {
		initARAP_Substitution(constraints, transform, iterations_uniform, iterations_cotangent, iterations_uniform_laplacian_editing, iterations_cotangent_laplacian_editing);
	}

	updateObjects();

	init = true;
}

Scene Scene::meshFairingScene() {

	std::vector<Geometry*> objs;
	std::vector<Light*> ls;

	//create light
	ls.push_back(new DirectionalLight(GL_LIGHT0, vec3(-1), vec4(1), vec4(0.8), vec4(0.2)));

	//create material
	Material* red = new Material("Red", vec4(1.0f, 0.0f, 0.0f, 1.0f), vec4(1.0f, 0.0f, 0.0f, 1.0f), vec4(1.0f, 0.0f, 0.0f, 1.0f), 100.0f);
	Material* green = new Material("Green", vec4(0.0f, 1.0f, 0.0f, 1.0f), vec4(0.0f, 1.0f, 0.0f, 1.0f), vec4(0.0f, 1.0f, 0.0f, 1.0f), 100.0f);
	Material* yellow = new Material("Yellow", vec4(1.0f, 1.0f, 0.0f, 1.0f), vec4(1.0f, 1.0f, 0.0f, 1.0f), vec4(1.0f, 1.0f, 0.0f, 1.0f), 100.0f);

	//create objects
	objs.push_back(new Geometry("./Assets/cube16.obj", vec3(0, 5, -20), vec3(0, 0, 0), vec3(2), 1, red));
	objs.push_back(new Geometry("./Assets/cube16.obj", vec3(-5, 0, -20), vec3(0, 0, 0), vec3(2), 1, green));
	objs.push_back(new Geometry("./Assets/cube16.obj", vec3(-0, 0, -20), vec3(0, 0, 0), vec3(2), 1, green));
	objs.push_back(new Geometry("./Assets/cube16.obj", vec3(5, 0, -20), vec3(0, 0, 0), vec3(2), 1, green));
	objs.push_back(new Geometry("./Assets/cube16.obj", vec3(-5, -5, -20), vec3(0, 0, 0), vec3(2), 1, yellow));
	objs.push_back(new Geometry("./Assets/cube16.obj", vec3(0, -5, -20), vec3(0, 0, 0), vec3(2), 1, yellow));
	objs.push_back(new Geometry("./Assets/cube16.obj", vec3(5, -5, -20), vec3(0, 0, 0), vec3(2), 1, yellow));

	//test objects
	//objs.push_back(new Geometry("./Assets/cube2.obj", vec3(-7.5, 0, -20), vec3(0, 0, 0), vec3(1), 1, emerald));
	//objs.push_back(new Geometry("./Assets/cube2.obj", vec3(-4.5, 0, -20), vec3(0, 0, 0), vec3(1), 1, emerald));
	//objs.push_back(new Geometry("./Assets/cube2.obj", vec3(-1.5, 0, -20), vec3(0, 0, 0), vec3(1), 1, emerald));
	//objs.push_back(new Geometry("./Assets/cube2.obj", vec3(1.5, 0, -20), vec3(0, 0, 0), vec3(1), 1, emerald));
	//objs.push_back(new Geometry("./Assets/cube2.obj", vec3(4.5, 0, -20), vec3(0, 0, 0), vec3(1), 1, emerald));
	//objs.push_back(new Geometry("./Assets/cube2.obj", vec3(7.5, 0, -20), vec3(0, 0, 0), vec3(1), 1, emerald));
	//objs.push_back(new Geometry("./Assets/cube2.obj", vec3(7.5, 0, -20), vec3(0, 0, 0), vec3(1), 1, emerald));

	return Scene(objs, ls, NULL);
}

void Scene::initMeshFairingScene() {

	cout << "Initialising Cube16 Scene..." << endl;

	initLaplacian();

	//smooth uniform
	objects[1]->smooth(0.0002, 0, false, false);
	//smooth uniform and preserve edges
	objects[2]->smooth(0.0002, 0, false, true);

	//smooth cotangent
	objects[4]->smooth(10, 0, true, false);

	//smooth cotangent and preserve edges
	objects[5]->smooth(10, 0, true, true);

	//minimal surface
	std::vector<int> constraints = { 1,2,3,4,5,6,7,8 };
	//uniform
	objects[3]->minimalSurface(constraints, false);
	//cotan
	objects[6]->minimalSurface(constraints, true);

	updateObjects();

	init = true;
}

void Scene::render(GLuint shader) {

	// set shader
	glUseProgram(shader);
	
	// position lights
	for (auto &light : lights) {
		light->positionLight();
	}

	// render scene objects
	for (auto &obj : objects) {
		obj->render(shader);
	}
}

void Scene::toggleWireFrame() {
	for (Geometry* g : objects) {
		g->toggleWireFrame();
	}
}

void Scene::updateObjects() {
	for (int i = 1; i < objects.size(); i++) {
		objects[i]->updateMesh();
	}
}