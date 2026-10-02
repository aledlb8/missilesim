#version 430 core
out vec4 FragColor;

in vec3 TexCoords;

#include "sky.glsl"

void main(){
    vec3 direction = normalize(TexCoords);
    vec3 color = skyRadiance(direction, true);

    // Sun disc (~0.5 deg), aligned with the directional light so shadows
    // point away from it. HDR values well above 1.0 let bloom build the corona.
    // Cloud in front of the sun dims the disc.
    float cosSun = dot(direction, skyToSun());
    float disc = smoothstep(0.99989, 0.99996, cosSun);
    float cover = skyClouds(direction).a;
    color += skySunColor * disc * 80.0 * (1.0 - cover * 0.9);

    FragColor = vec4(color, 1.0);
}
