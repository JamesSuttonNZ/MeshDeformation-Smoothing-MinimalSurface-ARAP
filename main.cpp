#include <stdio.h>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <stdexcept>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "cgra_math.h"
#include "cgra_geometry.h"
#include "geometry.h"
#include "simple_shader.h"
#include "simple_image.h"
#include "skybox.h"
#include "camera.h"
#include "scene.h"

using namespace std;
using namespace cgra;

// Window
GLFWwindow* g_window;
static const int w_width = 1000;
static const int w_height = 1000;

// scene (objects and lights
Scene* currentScene;
std::vector<Scene> scenes;

// delta time
float deltaTime = 0.0f;	// Time between current frame and last frame
float lastFrame = 0.0f; // Time of last frame

// Camera
Camera* camera = nullptr;

// Mouse controlled Camera values
bool g_leftMouseDown = false;
vec2 g_mousePosition;
float sensitivity = 0.3f;
float g_pitch = 0;
float g_yaw = 0;

// textures
GLuint* textures = new GLuint[2];
GLuint* texturesNormal = new GLuint[1];
GLuint* texturesSpecular = new GLuint[1];

// shaders
GLuint g_screenShader = 0;
GLuint g_skyboxShader = 0;
GLuint g_phongTexShader = 0;
GLuint g_phongShader = 0;


// Mouse Button callback
// Called for mouse movement event on since the last glfwPollEvents
//
void cursorPosCallback(GLFWwindow* win, double xpos, double ypos) {
		
	// for rotating camera with mouse
	if (g_leftMouseDown) {
		g_yaw -= (g_mousePosition.x - xpos) * sensitivity;
		g_pitch -= (g_mousePosition.y - ypos) * sensitivity;

		if (g_pitch > 89.0f)
			g_pitch = 89.0f;
		if (g_pitch < -89.0f)
			g_pitch = -89.0f;

		camera->rotateCamera(g_yaw, g_pitch);
	}
	g_mousePosition = vec2(xpos, ypos);
}

// Mouse Button callback
// Called for mouse button event on since the last glfwPollEvents
//
void mouseButtonCallback(GLFWwindow* win, int button, int action, int mods) {
	// rotate camera when left mouse button pressed
	if (button == GLFW_MOUSE_BUTTON_LEFT)
		g_leftMouseDown = (action == GLFW_PRESS);
}

// Scroll callback
// Called for scroll event on since the last glfwPollEvents
//
void scrollCallback(GLFWwindow* win, double xoffset, double yoffset) {

	// zoom camera on scoll
	camera->zoomCamera(yoffset);
}

// Keyboard callback
// Called for every key event on since the last glfwPollEvents
//
void keyCallback(GLFWwindow* win, int key, int scancode, int action, int mods) {

	// switch scene (only calculate mesh fairing when first switching to the scene to avoid long program start times)
	if (key == GLFW_KEY_1 && action == GLFW_PRESS) {
		currentScene = &scenes[0];
		cout << "Scene: Bar Scene Lagrange" << endl;
	}
	if (key == GLFW_KEY_2 && action == GLFW_PRESS) {
		currentScene = &scenes[1];
		if (!currentScene->init) { currentScene->initCylinderScene(true); }
		cout << "Scene: Cylinder Scene Lagrange" << endl;
	}
	if (key == GLFW_KEY_3 && action == GLFW_PRESS) {
		currentScene = &scenes[2];
		if (!currentScene->init) { currentScene->initKnubbelScene(true); }
		cout << "Scene: Knubbel Scene Lagrange" << endl;
	}
	if (key == GLFW_KEY_4 && action == GLFW_PRESS) {
		currentScene = &scenes[3];
		if (!currentScene->init) { currentScene->initCactusScene(true); }
		cout << "Scene: Cactus Scene Lagrange" << endl;
	}
	if (key == GLFW_KEY_5 && action == GLFW_PRESS) {
		currentScene = &scenes[4];
		if (!currentScene->init) { currentScene->initBarScene(false); }
		cout << "Scene: Bar Scene Substitution" << endl;
	}
	if (key == GLFW_KEY_6 && action == GLFW_PRESS) {
		currentScene = &scenes[5];
		if (!currentScene->init) { currentScene->initCylinderScene(false); }
		cout << "Scene: Cylinder Scene Substitution" << endl;
	}
	if (key == GLFW_KEY_7 && action == GLFW_PRESS) {
		currentScene = &scenes[6];
		if (!currentScene->init) { currentScene->initKnubbelScene(false); }
		cout << "Scene: Knubbel Scene Substitution" << endl;
	}
	if (key == GLFW_KEY_8 && action == GLFW_PRESS) {
		currentScene = &scenes[7];
		if (!currentScene->init) { currentScene->initCactusScene(false); }
		cout << "Scene: Cactus Scene Substitution" << endl;
	}
	if (key == GLFW_KEY_0 && action == GLFW_PRESS) {
		currentScene = &scenes[8];
		if (!currentScene->init) { currentScene->initMeshFairingScene(); }
		cout << "Scene: Mesh Fairing Scene" << endl;
	}
	if (key == GLFW_KEY_F && action == GLFW_PRESS) {
		currentScene->toggleWireFrame();
	}
}

void loadTexture(GLuint texture, const char* filename)
{
	cout << filename << endl;
	Image tex(filename);
	cout << "test" << endl;
	glBindTexture(GL_TEXTURE_2D, texture);

	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);

	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

	gluBuild2DMipmaps(GL_TEXTURE_2D, 3, tex.w, tex.h, tex.glFormat(), GL_UNSIGNED_BYTE, tex.dataPointer());
}

void loadCubemap(GLuint texture, const char* filename)
{
	Image tex(filename);

	glBindTexture(GL_TEXTURE_CUBE_MAP, texture);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

	glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

	Image right = tex.subsection(tex.w / 2, tex.h / 3, tex.w / 4, tex.h / 3);
	gluBuild2DMipmaps(GL_TEXTURE_CUBE_MAP_POSITIVE_X, 3, right.w, right.h, right.glFormat(), GL_UNSIGNED_BYTE, right.dataPointer());

	Image left = tex.subsection(0, tex.h / 3, tex.w / 4, tex.h / 3);
	gluBuild2DMipmaps(GL_TEXTURE_CUBE_MAP_NEGATIVE_X, 3, left.w, left.h, left.glFormat(), GL_UNSIGNED_BYTE, left.dataPointer());

	Image top = tex.subsection(tex.w / 4, 0, tex.w / 4, tex.h / 3);
	gluBuild2DMipmaps(GL_TEXTURE_CUBE_MAP_POSITIVE_Y, 3, top.w, top.h, top.glFormat(), GL_UNSIGNED_BYTE, top.dataPointer());

	Image bottom = tex.subsection(tex.w / 4, (tex.h / 3) * 2, tex.w / 4, tex.h / 3);
	gluBuild2DMipmaps(GL_TEXTURE_CUBE_MAP_NEGATIVE_Y, 3, bottom.w, bottom.h, bottom.glFormat(), GL_UNSIGNED_BYTE, bottom.dataPointer());

	Image front = tex.subsection(tex.w / 4, tex.h / 3, tex.w / 4, tex.h / 3);
	gluBuild2DMipmaps(GL_TEXTURE_CUBE_MAP_POSITIVE_Z, 3, front.w, front.h, front.glFormat(), GL_UNSIGNED_BYTE, front.dataPointer());

	Image back = tex.subsection((tex.w / 4) * 3, tex.h / 3, tex.w / 4, tex.h / 3);
	gluBuild2DMipmaps(GL_TEXTURE_CUBE_MAP_NEGATIVE_Z, 3, back.w, back.h, back.glFormat(), GL_UNSIGNED_BYTE, back.dataPointer());
}

// An example of how to load a texure from a hardcoded location
//
void initTexture() {

	glActiveTexture(GL_TEXTURE0); // Use slot 0, need to use GL_TEXTURE1 ... etc if using more than one texture PER OBJECT
	glGenTextures(2, textures); // Generate texture ID

	loadTexture(textures[0], "./textures/fine_wood.jpg");
	loadCubemap(textures[1], "./textures/skybox/skybox.png");

	glActiveTexture(GL_TEXTURE1);
	glGenTextures(1, texturesSpecular);
	loadTexture(texturesSpecular[0], "./textures/fine_wood_spec.jpg");

	glActiveTexture(GL_TEXTURE2); // Use slot 0, need to use GL_TEXTURE1 ... etc if using more than one texture PER OBJECT
	glGenTextures(1, texturesNormal); // Generate texture ID

	loadTexture(texturesNormal[0], "./textures/fine_wood_normal.jpg");

	
}

// Load Shaders
//
void initShader() {

	g_screenShader = makeShaderProgramFromFile({ GL_VERTEX_SHADER, GL_FRAGMENT_SHADER }, { "./shaders/screenShader.vert", "./shaders/screenShader.frag" });
	g_skyboxShader = makeShaderProgramFromFile({ GL_VERTEX_SHADER, GL_FRAGMENT_SHADER }, { "./shaders/shaderSkybox.vert", "./shaders/shaderSkybox.frag" });
	g_phongTexShader = makeShaderProgramFromFile({ GL_VERTEX_SHADER, GL_FRAGMENT_SHADER }, { "./shaders/shaderPhongTex.vert", "./shaders/shaderPhongTex.frag" });
	g_phongShader = makeShaderProgramFromFile({ GL_VERTEX_SHADER, GL_FRAGMENT_SHADER }, { "./shaders/shaderPhong.vert", "./shaders/shaderPhong.frag" });
	
}

// process user input (called every frame)
//
void processInput(GLFWwindow* window)
{
	// close window when escape ky pressed
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);

	// set light direction to current view direction when space bar pressed
	if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {

		// update light direction
		currentScene->lights[0]->setPos(camera->getCameraDirection());
		//cout << "light direction set to: " << camera->getCameraDirection() << endl;
	}

	camera->moveCamera(window, deltaTime);
}

// Render Scene (Rasterized)
//
void render(int width, int height) {

	// Set viewport to be the whole window
	glViewport(0, 0, width, height);

	// Grey/Blueish background
	glClearColor(0.3f, 0.3f, 0.4f, 1.0f);

	// clear buffers
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

	// camera
	camera->setupCamera(width, height);

	// Enable flags for normal rendering
	glEnable(GL_TEXTURE_2D);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_NORMALIZE);

	currentScene->render(g_phongShader);

	// Disable flags for cleanup (optional)
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glDisable(GL_NORMALIZE);
	
}

// main function
//
int main(){

	// Initialize the GLFW library
	if (!glfwInit()) {
		cerr << "Error: Could not initialize GLFW" << endl;
		abort(); // Unrecoverable error
	}

	// Get the version for GLFW for later
	int glfwMajor, glfwMinor, glfwRevision;
	glfwGetVersion(&glfwMajor, &glfwMinor, &glfwRevision);

	// Create a windowed mode window and its OpenGL context
	g_window = glfwCreateWindow(w_width, w_height, "ARAP", nullptr, nullptr);
	if (!g_window) {
		cerr << "Error: Could not create GLFW window" << endl;
		abort(); // Unrecoverable error
	}

	// Make the g_window's context is current. If we have multiple windows we will need to switch contexts
	glfwMakeContextCurrent(g_window);

	// Initialize GLEW, must be done after making a GL context current (glfwMakeContextCurrent in this case)
	glewExperimental = GL_TRUE; // required for full GLEW functionality for OpenGL 3.0+
	GLenum err = glewInit();
	if (GLEW_OK != err) { // Problem: glewInit failed, something is seriously wrong.
		cerr << "Error: " << glewGetErrorString(err) << endl;
		abort(); // Unrecoverable error
	}

	// Print out our OpenGL verisions
	cout << "Using OpenGL " << glGetString(GL_VERSION) << endl;
	cout << "Using GLEW " << glewGetString(GLEW_VERSION) << endl;
	cout << "Using GLFW " << glfwMajor << "." << glfwMinor << "." << glfwRevision << endl;

	// Attach input callbacks to g_window
	glfwSetCursorPosCallback(g_window, cursorPosCallback);
	glfwSetMouseButtonCallback(g_window, mouseButtonCallback);
	glfwSetScrollCallback(g_window, scrollCallback);
	glfwSetKeyCallback(g_window, keyCallback);

	// create camera
	camera = new Camera(vec3(0.0f));

	// Initialize Scenes
	scenes.push_back(Scene::barScene());
	scenes.push_back(Scene::cylinderScene());
	scenes.push_back(Scene::knubbelScene());
	scenes.push_back(Scene::cactusScene());
	scenes.push_back(Scene::barScene());
	scenes.push_back(Scene::cylinderScene());
	scenes.push_back(Scene::knubbelScene());
	scenes.push_back(Scene::cactusScene());
	scenes.push_back(Scene::meshFairingScene());
	
	scenes[0].initBarScene(true);
	currentScene = &scenes[0];
	
	initTexture();
	initShader();

	// Loop until the user closes the window
	while (!glfwWindowShouldClose(g_window)) {

		// input
		processInput(g_window);

		// Make sure we draw to the WHOLE window
		int width, height;
		glfwGetFramebufferSize(g_window, &width, &height);

		// Main Render
		render(width, height);

		// calculate delta time
		float currentFrame = glfwGetTime();
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;
		
		// Swap front and back buffers
		glfwSwapBuffers(g_window);

		// Poll for and process events
		glfwPollEvents();
	}

	glfwTerminate();
}