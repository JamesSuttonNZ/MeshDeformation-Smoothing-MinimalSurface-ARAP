#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "light.h"

#include "cgra_math.h"
#include "GL/glew.h"
#include "GLFW/glfw3.h"

using namespace std;
using namespace cgra;

float* Light::toArray(vec3 v) {
	float a[] = { v[0], v[1], v[2] };
	return a;
}

float* Light::toArray(vec4 v) {
	float a[] = { v[0], v[1], v[2], v[3] };
	return a;
}

void Light::toggleLight() {
	if (on) {
		glLightfv(light, GL_DIFFUSE, off);
		glLightfv(light, GL_SPECULAR, off);
		glLightfv(light, GL_AMBIENT, off);
		on = !on;
		glDisable(light);
	}
	else {
		glLightfv(light, GL_DIFFUSE, toArray(diffuse));
		glLightfv(light, GL_SPECULAR, toArray(specular));
		glLightfv(light, GL_AMBIENT, toArray(ambient));
		on = !on;
		glEnable(light);
	}
	
}

void Light::positionLight() {
	glLightfv(light, GL_POSITION, toArray(position));
}

void Light::setPos(vec3 p) {
	position = vec4(p,1);
}

DirectionalLight::DirectionalLight(GLenum l, vec3 dir, vec4 diff, vec4 spec, vec4 amb) {

	light = l;

	position = vec4(dir, 0);
	diffuse = diff;
	specular = spec;
	ambient = amb;

	setupDirectionaLlight();
}

void DirectionalLight::setupDirectionaLlight() {

	glLightfv(light, GL_POSITION, toArray(position));
	glLightfv(light, GL_DIFFUSE, toArray(diffuse));
	glLightfv(light, GL_SPECULAR, toArray(specular));
	glLightfv(light, GL_AMBIENT, toArray(ambient));
	glEnable(light);
}

void DirectionalLight::setPos(vec3 p) {
	position = vec4(p, 0);
}

void DirectionalLight::illuminate(vec3& hitPoint, vec3& lightDir, vec3& lightIntensity) {
	lightDir = normalize(vec3(position[0], position[1], position[2]));
	lightIntensity = vec3(1);
}

PointLight::PointLight(GLenum l, vec3 pos, vec4 diff, vec4 spec, vec4 amb) {
	light = l;

	position = vec4(pos, 1);
	diffuse = diff;
	specular = spec;
	ambient = amb;

	setupPointLight();
}

void PointLight::setupPointLight() {

	glLightfv(light, GL_POSITION, toArray(position));
	glLightfv(light, GL_DIFFUSE, toArray(diffuse));
	glLightfv(light, GL_SPECULAR, toArray(specular));
	glLightfv(light, GL_AMBIENT, toArray(ambient));
	glEnable(light);
}

void PointLight::illuminate(vec3& hitPoint, vec3& lightDir, vec3& lightIntensity) {
	lightDir = normalize((vec3(position[0], position[1], position[2]) - hitPoint));
	//float r2 = sqrt((lightDir[0]*lightDir[0]) + (lightDir[1] * lightDir[1]) + (lightDir[2] * lightDir[2])); // norm
	//distance = sqrt(r2);
	//lightDir.x /= distance, lightDir.y /= distance, lightDir.z /= distance;
	// avoid division by 0
	//lightIntensity = vec3(1) / (4 * math::pi() * r2);
}

SpotLight::SpotLight(GLenum l, vec3 pos, vec3 dir, float a, vec4 diff, vec4 spec, vec4 amb) {

	light = l;

	position = vec4(pos, 1);
	direction = dir;
	angle = a;
	diffuse = diff;
	specular = spec;
	ambient = amb;

	setupSpotLight();
}

void SpotLight::setupSpotLight() {

	GLfloat exp = 2;
	GLfloat atten = 0.001;

	glLightfv(light, GL_POSITION, toArray(position));
	glLightfv(light, GL_SPOT_DIRECTION, toArray(direction));
	glLightfv(light, GL_SPOT_EXPONENT, &exp);
	glLightf(light, GL_SPOT_CUTOFF, angle);
	glLightfv(light, GL_LINEAR_ATTENUATION, &atten);

	glLightfv(light, GL_AMBIENT, toArray(ambient));
	glLightfv(light, GL_DIFFUSE, toArray(diffuse));
	glLightfv(light, GL_SPECULAR, toArray(specular));

	glEnable(light);
}

void SpotLight::illuminate(vec3& hitPoint, vec3& lightDir, vec3& lightIntensity) {

}