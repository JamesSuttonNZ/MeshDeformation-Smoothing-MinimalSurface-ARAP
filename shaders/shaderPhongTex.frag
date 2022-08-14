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

// material (texture)
struct Material {
	sampler2D diffuse;
	sampler2D specular;
	float shininess;
};

uniform Material material;

// Values passed in from the vertex shader
varying vec3 vNormal;
varying vec3 vPosition;
varying vec2 vTextureCoord0;

#define MAX_LIGHTS 2

void main() {

	vec4 result = vec4(0.0);
	
	for (int i=0;i<MAX_LIGHTS;i++)
	{
		vec4 diffuseTex = vec4(texture(material.diffuse, vTextureCoord0));
		vec4 specularTex = vec4(texture(material.specular, vTextureCoord0));

		// ambient
		vec4 ambient = gl_LightSource[i].ambient * diffuseTex;
		
		// diffuse
		vec3 norm = normalize(vNormal);
		vec4 lightPos = gl_LightSource[i].position;
		
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
		vec4 diffuse = gl_LightSource[i].diffuse * (diff * diffuseTex);
		
		//specular
		vec3 viewDir = normalize(-vPosition);
		vec3 reflectDir = reflect(-lightDir, norm);
		float spec = pow(max(dot(viewDir, reflectDir), 0), material.shininess);
		vec4 specular = gl_LightSource[i].specular * (spec * specularTex);
		
		result += ambient + diffuse + specular;
	}	
		
	gl_FragColor = result;
}
