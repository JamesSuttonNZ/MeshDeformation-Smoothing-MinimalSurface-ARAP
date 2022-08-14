#pragma once

#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "cgra_math.h"
#include "GL/glew.h"
#include "GLFW/glfw3.h"

class Skybox {

private:
	GLuint m_displayListPoly = 0; // DisplayList for Polygon
	void createDisplayListPoly();

public:
	Skybox();
	void renderSkybox();
};