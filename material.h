#pragma once

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "cgra_math.h"
#include "GL/glew.h"
#include "GLFW/glfw3.h"

class Material {
public:
	std::string name;
	cgra::vec4 ambient;
	cgra::vec4 diffuse;
	cgra::vec4 specular;
	float shininess;

	void setShaderMaterial(GLuint shader);
	Material(std::string name, cgra::vec4 ambient, cgra::vec4 diffuse, cgra::vec4 specular, float shininess);
};