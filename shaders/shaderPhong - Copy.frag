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

// Values passed in from the vertex shader
varying vec3 vNormal;
varying vec3 vPosition;
varying vec2 vTextureCoord0;

#define MAX_LIGHTS 3 

void main() {
	
	vec3 N = normalize(vNormal);
	vec4 finalColor = vec4(0.0, 0.0, 0.0, 0.0);

	for (int i=0;i<MAX_LIGHTS;i++)
	{
		vec3 spotDir = -normalize(gl_LightSource[i].spotDirection);
		
		vec3 L;
		if(gl_LightSource[i].position.w == 0){
			L = normalize(gl_LightSource[i].position.xyz);
		}
		else{
			L = normalize(gl_LightSource[i].position.xyz - vPosition);
		}
		vec3 E = normalize(-vPosition); // we are in Eye Coordinates, so EyePos is (0,0,0) 
		vec3 R = normalize(-reflect(L,N));
		
		float angle = acos(dot(L,spotDir));
		
		if(degrees(angle) <= gl_LightSource[i].spotCutoff){

		//calculate Ambient Term: 
		vec4 Iamb = gl_FrontLightProduct[i].ambient; 
		//calculate Diffuse Term: 
		vec4 Idiff = gl_FrontLightProduct[i].diffuse * max(dot(N,L), 0.0);
		Idiff = clamp(Idiff, 0.0, 1.0); 

		// calculate Specular Term:
		vec4 Ispec = gl_FrontLightProduct[i].specular 
			 * pow(max(dot(R,E),0.0),0.3*gl_FrontMaterial.shininess);
		Ispec = clamp(Ispec, 0.0, 1.0); 

		finalColor += Iamb + Idiff + Ispec;
		}
	}

	// write Total Color: 
	gl_FragColor = gl_FrontLightModelProduct.sceneColor + finalColor; 
}
