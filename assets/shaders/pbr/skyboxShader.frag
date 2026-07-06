#version 430 core
out vec4 FragColor;

in vec3 TexCoords;

uniform samplerCube environmentMap;
uniform vec3 sunDirection;  // light travel direction (sun -> scene)
uniform vec3 sunColor;      // strength-scaled sun colour

void main(){
    vec3 direction = normalize(TexCoords);
    vec3 envColor = texture(environmentMap, direction).rgb;

    // Linear-space haze, kept in sync with the scene fogColor in
    // PBRPipeline::bindPBRUniforms (sRGB 0.52/0.63/0.74 linearized).
    vec3 hazeColor = vec3(0.237, 0.362, 0.516);
    float skyAmount = smoothstep(-0.12, 0.32, direction.y);
    float horizonHaze = 1.0 - smoothstep(0.05, 0.55, abs(direction.y));
    envColor = mix(hazeColor, envColor, skyAmount);
    envColor = mix(envColor, hazeColor, horizonHaze * 0.35);

    // Sun disc (~0.5 deg) + tight forward-scatter glow, aligned with the
    // directional light so shadows point away from it. HDR values well
    // above 1.0 let bloom build the corona.
    vec3 toSun = normalize(-sunDirection);
    float cosSun = dot(direction, toSun);
    float disc = smoothstep(0.99989, 0.99996, cosSun);
    float glow = pow(max(cosSun, 0.0), 350.0);
    envColor += sunColor * (disc * 80.0 + glow * 4.0);

    FragColor = vec4(envColor, 1.0);
}
