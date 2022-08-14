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
//uniform sampler2D texture0;
uniform vec2 waterBoundA;
uniform vec2 waterBoundB;
uniform float offset;

varying vec2 waterCoord;

// Values to pass to the fragment shader
varying vec3 vNormal;
varying vec3 vPosition;
varying vec2 vTextureCoord0;


void main() {

    waterCoord = gl_Vertex.xz;
    waterCoord -= waterBoundA;
    waterCoord.x /= waterBoundB.x - waterBoundA.x;
    waterCoord.y /= waterBoundB.y - waterBoundA.y;

    waterCoord += offset*(0.7, 0.3);

    waterCoord *= 30;

	// Transform and pass on the normal/position/texture to fragment shader
    vNormal = normalize(gl_NormalMatrix * vec3(0, 1, 0));
    vPosition = vec3(gl_ModelViewMatrix * gl_Vertex);
    vTextureCoord0 = gl_MultiTexCoord0.xy;

	// IMPORTANT tell OpenGL where the vertex is
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
}