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

struct Material {
	vec4 ambient;
	vec4 diffuse;
	vec4 specular;
	float shininess;
};

uniform Material material;

// Values passed in from the vertex shader
varying vec3 vNormal;
varying vec3 vPosition;

#define MAX_LIGHTS 2 

void main() {
	
	vec4 result = vec4(0.0);
	
	vec4 lightPos;
	
	for (int i=0;i<MAX_LIGHTS;i++)
	{
		// ambient
		vec4 ambient = gl_LightSource[i].ambient * material.ambient;
		
		// diffuse
		vec3 norm = normalize(vNormal);
		lightPos = gl_LightSource[i].position;
		
		vec3 lightDir;
		
		// directional light
		if(lightPos.w == 0){
			lightDir = normalize(-lightPos.xyz);
		}
		// point light
		else{
			lightDir = normalize(lightPos.xyz - vPosition);
		}
		
		float diff = max(dot(norm, lightDir), 0);
		vec4 diffuse = gl_LightSource[i].diffuse * (diff * material.diffuse);
		
		//specular
		vec3 viewDir = normalize(-vPosition);
		vec3 reflectDir = reflect(-lightDir, norm);
		float spec = pow(max(dot(viewDir, reflectDir), 0), material.shininess);
		vec4 specular = gl_LightSource[i].specular * (spec * material.specular);
		
		result += ambient + diffuse + specular;
	}
	
	gl_FragColor = result;
}
