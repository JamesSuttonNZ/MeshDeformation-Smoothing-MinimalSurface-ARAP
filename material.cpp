#include <cmath>
#include <iostream> // input/output streams
#include <fstream>  // file streams
#include <sstream>  // string streams
#include <string>
#include <stdexcept>
#include <vector>

#include "cgra_math.h"
#include "material.h"

using namespace std;
using namespace cgra;

Material::Material(string n, vec4 amb, vec4 diff, vec4 spec, float shin) {
	name = n;
	ambient = amb;
	diffuse = diff;
	specular = spec;
	shininess = shin;
}

void Material::setShaderMaterial(GLuint shader) {
	glUniform4f(glGetUniformLocation(shader, "material.ambient"), ambient[0], ambient[1], ambient[2], ambient[3]);
	glUniform4f(glGetUniformLocation(shader, "material.diffuse"), diffuse[0], diffuse[1], diffuse[2], diffuse[3]);
	glUniform4f(glGetUniformLocation(shader, "material.specular"), specular[0], specular[1], specular[2], specular[3]);
	glUniform1f(glGetUniformLocation(shader, "material.shininess"), shininess);
}