#pragma once

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "cgra_math.h"
#include "GL/glew.h"
#include "GLFW/glfw3.h"

using namespace cgra;

class Light {
public:

	GLenum light;

	cgra::vec4 position;

	cgra::vec4 diffuse;
	cgra::vec4 specular;
	cgra::vec4 ambient;

	bool on = true;
	float off[4] = { 0.0f, 0.0f, 0.0f, 0.0f };

	float* toArray(cgra::vec3 v);
	float* toArray(cgra::vec4 v);

	void toggleLight();
	
	void positionLight(); // pos light in scene
	virtual void setPos(cgra::vec3 p);

	virtual void illuminate(vec3& hitPoint, vec3& lightDir, vec3& lightIntensity) = 0;
};


class DirectionalLight : public Light {
public:

	void setPos(cgra::vec3 p) override;
	void setupDirectionaLlight();
	DirectionalLight(GLenum light, cgra::vec3 dir, cgra::vec4 diff, cgra::vec4 spec, cgra::vec4 amb);

	void illuminate(vec3& hitPoint, vec3& lightDir, vec3& lightIntensity) override;
};


class PointLight : public Light {
public:

	void setupPointLight();
	PointLight(GLenum light, cgra::vec3 pos, cgra::vec4 diff, cgra::vec4 spec, cgra::vec4 amb);

	void illuminate(vec3& hitPoint, vec3& lightDir, vec3& lightIntensity) override;
};


class SpotLight : public Light {
private:

	cgra::vec3 direction;
	float angle;

public:

	void setupSpotLight();
	SpotLight(GLenum light, cgra::vec3 pos, cgra::vec3 dir, float a, cgra::vec4 diff, cgra::vec4 spec, cgra::vec4 amb);

	void illuminate(vec3& hitPoint, vec3& lightDir, vec3& lightIntensity) override;
};