#pragma once

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "cgra_math.h"
#include "GL/glew.h"
#include "GLFW/glfw3.h"

class Object {

	bool intersect(cgra::vec3 rayOrigin, cgra::vec3 rayDir, float& t);

};