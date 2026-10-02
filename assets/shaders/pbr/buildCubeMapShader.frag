#version 460 core
out vec4 FragColor;

in vec3 localPos;

uniform sampler2D equirectangularMap;
// True: fill the cube from the procedural sky instead of an HDR image.
uniform bool proceduralSky;

#include "sky.glsl"

//Mapping from spherical to cubemap function
const vec2 invAtan = vec2(0.1591, 0.3183);
vec2 SampleSphericalMap(vec3 v){
    vec2 uv = vec2(atan(v.z, v.x), asin(v.y));
    uv *= invAtan;
    uv += 0.5;
    return uv;
}

void main(){
    vec3 direction = normalize(localPos);
    if (proceduralSky) {
        // No sun disc: the directional light already carries it.
        FragColor = vec4(skyRadiance(direction, true), 1.0);
        return;
    }

    vec2 uv = SampleSphericalMap(direction);
    vec3 color = texture(equirectangularMap, uv).rgb;
    FragColor = vec4(color, 1.0);
}
