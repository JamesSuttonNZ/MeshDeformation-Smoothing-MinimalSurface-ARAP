#version 120

uniform sampler2D waterGradientMap;
uniform float offset;

varying vec2 waterCoord;
varying float reflectionAmount;

// Values passed in from the vertex shader
varying vec3 vNormal;
varying vec3 vPosition;
varying vec2 vTextureCoord0;

void main() {
    vec3 viewDirection = (gl_ModelViewMatrix * vec4(0, 0, -1, 1)).xyz;
    vec2 gradient = texture2D(waterGradientMap, waterCoord).rg - vec2(0.5, 0.5);

    gradient *= cos((offset+sqrt(waterCoord.x*waterCoord.x + waterCoord.y*waterCoord.y)/300)*200);

    vec3 normal = gl_NormalMatrix * normalize(vec3(gradient.x, 1, gradient.y));

    float incident = 1 - dot(normal, normalize(-vPosition));

    gl_FragColor = vec4(0, 1, 1, incident*incident);
    //gl_FragColor = vec4(gradient.x, gradient.y, 0, 1);
}