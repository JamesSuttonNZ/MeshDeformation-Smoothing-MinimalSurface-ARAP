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

// Constant across both shaders
uniform sampler2D texture0;
uniform sampler2D textureNormal;
uniform sampler2D textureShadowMap;
uniform sampler2D textureSpecular;
uniform mat4 lightSpaceMatrix;

// Values passed in from the vertex shader
varying vec3 vNormal;
varying vec3 vPosition;
varying vec4 vPositionLightSpace;
varying vec2 vTextureCoord0;
varying mat3 toTangentSpace;

#define MAX_LIGHTS 1

float ShadowCalculation(vec4 positionLightSpace, vec3 norm, vec3 light)
{
    vec3 projCoords = positionLightSpace.xyz / positionLightSpace.w;
	projCoords = projCoords * 0.5 + 0.5;
	float closestDepth = texture2D(textureShadowMap, projCoords.xy).r;
	float currentDepth = projCoords.z;
	float bias = max(0.000005 * (1.0 - dot(norm, light)), 0.0000005); 
	float shadow = 0.0;
	vec2 texelSize = 1.0 / textureSize(textureShadowMap, 0);
	for(int x = -1; x <= 1; ++x)
	{
		for(int y = -1; y <= 1; ++y)
		{
			float pcfDepth = texture2D(textureShadowMap, projCoords.xy + vec2(x, y) * texelSize).r; 
			shadow += currentDepth - bias > pcfDepth ? 1.0 : 0.0;        
		}    
	}
	shadow /= 9.0;
	if(projCoords.z > 1.0){
		shadow = 0.0;
	}
	
	return shadow;
}

void main() {

	vec3 color = texture2D(texture0, vTextureCoord0).rgb;

	//bump map normal
	vec3 N = texture2D(textureNormal, vTextureCoord0).rgb;
	vec3 norm = vec3(N.x*2 - 1, N.y*2 - 1, N.z*2 - 1);
	norm = norm * toTangentSpace;

	vec4 S = texture2D(textureSpecular, vTextureCoord0).rgba;
	vec4 spec = vec4(S.x*2 - 1, S.y*2 - 1, S.z*2 - 1, S.w);
	
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
		
		Ispec = Ispec*spec;

		float shadow = ShadowCalculation(vPositionLightSpace, norm, light);    
		
		finalColor += (Iamb + (1.0 - shadow) * (Idiff + Ispec))*vec4(color, 1);
		}
	}

	// write Total Color:
	gl_FragColor = gl_FrontLightModelProduct.sceneColor + finalColor;
	//gl_FragColor = vec4(abs(norm), 1);
	//gl_FragColor = vec4(N, 1);
}