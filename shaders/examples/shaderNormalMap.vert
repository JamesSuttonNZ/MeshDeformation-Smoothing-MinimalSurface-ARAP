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

#version 130

uniform sampler2D texture0;
uniform sampler2D textureNormal;
uniform sampler2D textureShadowMap;
uniform mat4 lightSpaceMatrix;

varying vec3 vNormal;
varying vec3 vPosition;
varying vec4 vPositionLightSpace;
varying vec2 vTextureCoord0;
varying mat3 toTangentSpace;

void main() {

	vTextureCoord0 = gl_MultiTexCoord0.xy;

	vec3 n = normalize (gl_NormalMatrix * gl_Normal);
	vec3 t = normalize (gl_NormalMatrix * gl_Color.xyz);
	vec3 b = normalize (gl_NormalMatrix * cross(n,t));
	
	toTangentSpace = mat3(
		t.x, b.x, n.x,
		t.y, b.y, n.y,
		t.z, b.z, n.z
	);
	
	// Transform and pass on the normal/position/texture to fragment shader
	vNormal = normalize(gl_NormalMatrix * gl_Normal);
	vPosition = vec3(gl_ModelViewMatrix * gl_Vertex);
	vPositionLightSpace = lightSpaceMatrix*vec4(vPosition, 1.0);

	// IMPORTANT tell OpenGL where the vertex is
	//gl_Position = lightSpaceMatrix*(gl_ModelViewMatrix)* gl_Vertex;
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;

}

