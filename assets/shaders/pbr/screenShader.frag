#version 430 core
out vec4 FragColor;

in vec2 TexCoords;

uniform sampler2D screenTexture;
uniform sampler2D bloomBlur;
uniform float exposure;
uniform float bloomStrength;

// ACES filmic tone mapping, Stephen Hill's fitted approximation.
// Matrices are the sRGB<->ACES transforms folded into the RRT+ODT fit;
// GLSL mat3 constructors below are column-major.
const mat3 ACESInputMat = mat3(
    0.59719, 0.07600, 0.02840,
    0.35458, 0.90834, 0.13383,
    0.04823, 0.01566, 0.83777);

const mat3 ACESOutputMat = mat3(
     1.60475, -0.10208, -0.00327,
    -0.53108,  1.10813, -0.07276,
    -0.07367, -0.00605,  1.07602);

vec3 RRTAndODTFit(vec3 v){
    vec3 a = v * (v + 0.0245786) - 0.000090537;
    vec3 b = v * (0.983729 * v + 0.4329510) + 0.238081;
    return a / b;
}

vec3 acesFitted(vec3 color){
    color = ACESInputMat * color;
    color = RRTAndODTFit(color);
    color = ACESOutputMat * color;
    return clamp(color, 0.0, 1.0);
}

// Linear -> sRGB. The default framebuffer is not GL_FRAMEBUFFER_SRGB
// (ImGui and debug primitives draw to it afterwards), so encode here.
vec3 srgbEncode(vec3 c){
    vec3 lo = 12.92 * c;
    vec3 hi = 1.055 * pow(c, vec3(1.0 / 2.4)) - 0.055;
    return mix(lo, hi, step(vec3(0.0031308), c));
}

void main(){
    vec3 hdrCol = texture(screenTexture, TexCoords).rgb;
    vec3 bloomCol = texture(bloomBlur, TexCoords).rgb;
    vec3 hdr = hdrCol + bloomStrength * bloomCol;

    vec3 mapped = acesFitted(hdr * exposure);
    FragColor = vec4(srgbEncode(mapped), 1.0);
}
