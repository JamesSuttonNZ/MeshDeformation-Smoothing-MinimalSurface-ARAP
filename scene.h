#pragma once

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "camera.h"
#include "geometry.h"
#include "light.h"

#include "cgra_math.h"
#include "GL/glew.h"
#include "GLFW/glfw3.h"

class Scene {

public:

	// list of parametric objects in scene
	std::vector<Geometry*> objects;
	// laplacian editing mesh
	Geometry* lap_edit_mesh;

	// list of lights in scene
	std::vector<Light*> lights;

	bool init = false;

	void render(GLuint shader);
	void toggleWireFrame();

	Scene(std::vector<Geometry*> objects, std::vector<Light*> lights, Geometry* lap_edit_mesh);

	//setup scenes
	static Scene barScene();
	static Scene cylinderScene();
	static Scene knubbelScene();
	static Scene cactusScene();
	static Scene meshFairingScene();

	//calculate arap
	void initBarScene(bool lagrange);
	void initCylinderScene(bool lagrange);
	void initKnubbelScene(bool lagrange);
	void initCactusScene(bool lagrange);
	void initMeshFairingScene();

	void initLaplacian();
	void initARAP_Lagrange(std::vector<int> constraints, mat4 transform, int iterUniform, int iterCotan, int iterUniformLap, int iterCotanLap);
	void initARAP_Substitution(std::vector<int> constraints, mat4 transform, int iterUniform, int iterCotan, int iterUniformLap, int iterCotanLap);
	void updateObjects();

	std::vector<int> readConstraints(std::string filename);
	cgra::mat4 readTransform(std::string filename);
};