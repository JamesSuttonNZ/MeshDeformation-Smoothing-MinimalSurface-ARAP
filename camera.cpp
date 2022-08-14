#include <cmath>
#include <iostream> // input/output streams
#include <fstream>  // file streams
#include <sstream>  // string streams
#include <string>
#include <stdexcept>
#include <vector>

#include "cgra_math.h"
#include "material.h"
#include "camera.h"

using namespace std;
using namespace cgra;

Camera::Camera(vec3 camPos) {

	cameraPos = camPos;

	cameraFront = vec3(0.0f, 0.0f, -1.0f);
		
	cameraDirection = cameraFront;
	
	worldUp = vec3(0.0f, 1.0f, 0.0f);

	cameraRight = normalize(cross(cameraDirection, worldUp));

	cameraUp = cross(cameraRight, cameraDirection);

	//initial camera speed
	cameraSpeed = 20.0f;
}

// Sets up where the camera is in the scene
//
void Camera::setupCamera(int width, int height) {
	// Set up the projection matrix
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(g_fovy, width / float(height), g_znear, g_zfar);

	// Set up the view part of the model view matrix
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	vec3 lookAt = cameraPos + cameraDirection;

	gluLookAt(cameraPos[0], cameraPos[1], cameraPos[2],// position of camera
		lookAt[0], lookAt[1], lookAt[2], // position to look at
		0.0, 1.0, 0.0);// up relative to camera
}

// moves the camera
//
void Camera::moveCamera(GLFWwindow* window, float deltaTime) {

	//calculate speed
	float speed = cameraSpeed * deltaTime;

	cameraRight = normalize(cross(cameraDirection, worldUp));

	cameraUp = cross(cameraRight, cameraDirection);

	// camera forward
	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
		cameraPos += speed * cameraDirection;
	}
	// camera back
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
		cameraPos -= speed * cameraDirection;
	}
	// camera left
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
		cameraPos -= normalize(cross(cameraDirection, cameraUp)) * speed;
	}
	// camera right
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
		cameraPos += normalize(cross(cameraDirection, cameraUp)) * speed;
	}
	// camera up
	if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) {
		cameraPos += speed * cameraUp;
	}
	// camera down
	if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) {
		cameraPos -= speed * cameraUp;
	}
}

// rotates the camera
//
void Camera::rotateCamera(float yaw, float pitch) {
	vec3 direction;
	direction.x = sin(radians(yaw)) * cos(radians(pitch));
	direction.y = -sin(radians(pitch));
	direction.z = -cos(radians(yaw)) * cos(radians(pitch));
	cameraDirection = normalize(direction);
}

// zooms the camera
//
void Camera::zoomCamera(double yoffset) {
	g_fovy -= (float)yoffset * zoomSpeed;
	if (g_fovy < 1.0f)
		g_fovy = 1.0f;
	if (g_fovy > 50.0f)
		g_fovy = 50.0f;
}

// get camera front direction
//
vec3 Camera::getCameraDirection() {
	return cameraDirection;
}

// get camera pos
//
vec3 Camera::getCameraPos() {
	return cameraPos;
}

// get current fov
//
float Camera::getFov() {
	return g_fovy;
}

mat4 Camera::getViewMatrix() {

	cameraDirection = normalize(cameraDirection);

	cameraRight = normalize(cross(cameraDirection, worldUp));

	cameraUp = normalize(cross(cameraRight, cameraDirection));

	return mat4( vec4(cameraRight, 0) , vec4(cameraUp, 0) , vec4(-cameraDirection, 0) , vec4(cameraPos, 1));
}