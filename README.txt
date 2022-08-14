I created the program in visual studio 2019 and used the precompiled windows binaries for GLEW and GLFW. I also haven't used CMake so i'm not sure what will be required to get it to run on other machines.

How to use the Program:

When running the program it first shows the scene objects in rasterized mode.
In this mode you can move the Camera:

'W' - Forward
'S' - Back
'A' - Left
'D' - Right
'E' - Up
'Q' - Down

Left clicking the screen and moving the mouse rotates the camera.

Change fov (zoom in/out) with the mouse scroll wheel.

'Space Bar' - sets the light direction to the current camera direction

'T' - Toggles Anti-Aliasing on and off

You can also switch scenes with the number keys:

'1' - Parametric Spheres Scene
'2' - Reflective Teapot Scene
'3' - Glass Bunny Scene

!!!When you have the scene setup the way you want it press the 'R' key to turn on Ray Tracing.

Pressing 'R' again turns ray tracing off but doesn't take effect until the current frame render has finished because polling for input while ray tracing slowed it down too much.