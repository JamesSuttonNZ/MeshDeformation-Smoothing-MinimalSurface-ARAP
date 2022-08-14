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

const float pi = 3.14159265;

// Constant across both shaders
uniform sampler2D texture0;
uniform float elapsedTime;

// Values to pass to the fragment shader
varying vec3 vNormal;
varying vec3 vPosition;
varying vec2 vTextureCoord0;

void main() {

	// Work out an oscillating coefficient
	float fraction = fract(elapsedTime); // Value in [0,1)
	float expansion_coeff = sin(fraction * 2 * pi)/2;
	vec4 new_position = vec4(gl_Vertex.xyz + (expansion_coeff *
	gl_Normal), 1.0);
	vTextureCoord0 = gl_MultiTexCoord0.xy;
	gl_Position = gl_ModelViewProjectionMatrix * new_position;

}
