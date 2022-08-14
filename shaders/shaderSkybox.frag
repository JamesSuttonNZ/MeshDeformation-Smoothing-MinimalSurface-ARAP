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

uniform samplerCube texture0;

varying vec3 vPosition;
varying vec3 vTextureCoord0;

void main()
{    

	//vec3 color = textureCube(texture0, vTextureCoord0).rgb;
    //gl_FragColor = vec4(color.x, color.y, color.z, 1);
	gl_FragColor = textureCube(texture0, vTextureCoord0);
}