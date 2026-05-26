#version 430 core
out vec4 FragColor;

in vec3 TexCoords;

uniform samplerCube environmentMap;

void main(){
    vec3 direction = normalize(TexCoords);
    vec3 envColor = texture(environmentMap, direction).rgb;
    vec3 hazeColor = vec3(0.52, 0.63, 0.74);
    float skyAmount = smoothstep(-0.12, 0.32, direction.y);
    float horizonHaze = 1.0 - smoothstep(0.05, 0.55, abs(direction.y));
    envColor = mix(hazeColor, envColor, skyAmount);
    envColor = mix(envColor, hazeColor, horizonHaze * 0.35);
    FragColor = vec4(envColor, 1.0);
}
