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
uniform sampler2D textureNormal;

// Values passed in from the vertex shader
varying vec3 vNormal;
varying vec3 vPosition;
varying vec2 vTextureCoord0;
varying mat3 toTangentSpace;

#define MAX_LIGHTS 3

void main() {

	vec3 color = texture2D(texture0, vTextureCoord0).rgb;

	//bump map normal
	vec3 N = texture2D(textureNormal, vTextureCoord0).rgb;
	vec3 norm = vec3(N.x*2 - 1, N.y*2 - 1, N.z*2 - 1);
	norm = norm * toTangentSpace;

	vec4 finalColor = vec4(0.0, 0.0, 0.0, 0.0);

	for (int i=0;i<MAX_LIGHTS;i++)
	{
		vec3 spotDir = -normalize(gl_LightSource[i].spotDirection);

		vec3 light;
		if(gl_LightSource[i].position.w == 0){
			light = normalize(gl_LightSource[i].position.xyz);
		}
		else{
			light = normalize(gl_LightSource[i].position.xyz - vPosition);
		}
		vec3 eye = normalize(-vPosition); // we are in Eye Coordinates, so EyePos is (0,0,0)
		vec3 refl = normalize(-reflect(light,norm));

		float angle = acos(dot(light,spotDir));

		if(degrees(angle) <= gl_LightSource[i].spotCutoff){

		//calculate Ambient Term:
		vec4 Iamb = gl_FrontLightProduct[i].ambient;
		//calculate Diffuse Term:
		vec4 Idiff = gl_FrontLightProduct[i].diffuse * max(dot(norm,light), 0.0);
		Idiff = clamp(Idiff, 0.0, 1.0);

		// calculate Specular Term:
		vec4 Ispec = gl_FrontLightProduct[i].specular
			 * pow(max(dot(refl,eye),0.0),0.3*gl_FrontMaterial.shininess);
		Ispec = clamp(Ispec, 0.0, 1.0);

		finalColor += Iamb + Idiff + Ispec;
		}
	}

	// write Total Color:
	gl_FragColor = gl_FrontLightModelProduct.sceneColor + vec4(color, 1)*finalColor;
	//gl_FragColor = vec4(abs(norm), 1);
	//gl_FragColor = vec4(N, 1);
}