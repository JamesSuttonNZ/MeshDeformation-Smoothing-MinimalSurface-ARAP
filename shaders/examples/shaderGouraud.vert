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

uniform sampler2D texture0;

varying vec3 vColor;

// Define a function to help us
vec3 colorFromLight(int, vec3, vec3);

void main() {
	vec3 viewspace_position = (gl_ModelViewMatrix * gl_Vertex).xyz;
	vec3 viewspace_normal = gl_NormalMatrix * gl_Normal;
	vColor = colorFromLight(0, viewspace_position, viewspace_normal);
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
}

vec3 colorFromLight(int light_idx, vec3 viewspace_position, vec3 viewspace_normal) {
	vec3 light_viewspace_position = gl_LightSource[light_idx].position.xyz;
	vec3 light_direction = normalize(light_viewspace_position - viewspace_position);
	
	// Ambient
	vec3 ambient = gl_LightSource[light_idx].ambient.rgb *
	gl_FrontMaterial.ambient.rgb;
	
	// Diffuse
	float s_dot_n = max(dot(light_direction, viewspace_normal), 0.0);
	vec3 diffuse = gl_LightSource[light_idx].diffuse.rgb * gl_FrontMaterial.diffuse.rgb * s_dot_n;
	
	return ambient + diffuse;
}


