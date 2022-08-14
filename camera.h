#pragma once

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "cgra_math.h"
#include "GL/glew.h"
#include "GLFW/glfw3.h"

class Camera {

public:

	cgra::vec3 cameraPos;
	cgra::vec3 cameraTarget;
	cgra::vec3 cameraDirection;
	cgra::vec3 worldUp;
	cgra::vec3 cameraUp;
	cgra::vec3 cameraRight;
	cgra::vec3 cameraFront;

	float cameraSpeed;

	// Projection values
	float g_fovy = 50.0;
	float g_znear = 0.1;
	float g_zfar = 1000.0;
	float zoomSpeed = 2.0f;

	Camera(cgra::vec3 camPos);

	void setupCamera(int width, int height);
	void moveCamera(GLFWwindow* window, float deltaTime);
	void rotateCamera(float yaw, float pitch);
	void zoomCamera(double yoffset);
	cgra::vec3 getCameraDirection();
	cgra::vec3 getCameraPos();
	float getFov();
	cgra::mat4 getViewMatrix();
};