//---------------------------------------------------------------------------
//
// Copyright (c) 2015 Taehyun Rhee, Joshua Scott, Ben Allen
//
// This software is provided 'as-is' for assignment of COMP308 in ECS,
// Victoria University of Wellington, without any express or implied warranty. 
// In no event will the authors be held liable for any damages arising from
// the use of this software.
//
// The contents of this file may not be copied or duplicated in any form
// without the prior permission of its owner.
//
//----------------------------------------------------------------------------

#version 120

// Constant across both shaders
uniform sampler2D texture0;
uniform float elapsedTime;

// Values passed in from the vertex shader
varying vec3 vNormal;
varying vec3 vPosition;
varying vec2 vTextureCoord0;
varying vec3 vWorldPosition;

vec3 rainbow(float x);

void main() {

	// Can do all sorts of cool stuff here
	//vec3 color = texture2D(texture0, vTextureCoord0).rgb;
	vec3 color = rainbow(fract(vWorldPosition.x + elapsedTime));

	// IMPORTANT tell OpenGL what the final color of the fragment is (vec4)
	gl_FragColor = vec4(color, 1);
}

vec3 rainbow(float x) {
float r = max(max(0.33-x, (0.33-abs(1.0-x))), 0);
float g = max(0.33-abs(0.33-x), 0);
float b = max(0.33-abs(0.67-x), 0);
return normalize(vec3(r, g, b));
}
