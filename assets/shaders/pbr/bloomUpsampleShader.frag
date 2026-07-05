#version 430 core
out vec4 FragColor;

in vec2 TexCoords;

uniform sampler2D srcTexture;
// Sample radius in UV space of the destination mip; blended additively
// (GL_ONE, GL_ONE) into the next larger mip by the caller.
uniform float filterRadius;

// 9-tap 3x3 tent filter upsample.
void main(){
    float x = filterRadius;
    float y = filterRadius;

    vec3 a = texture(srcTexture, TexCoords + vec2(-x,  y)).rgb;
    vec3 b = texture(srcTexture, TexCoords + vec2( 0,  y)).rgb;
    vec3 c = texture(srcTexture, TexCoords + vec2( x,  y)).rgb;
    vec3 d = texture(srcTexture, TexCoords + vec2(-x,  0)).rgb;
    vec3 e = texture(srcTexture, TexCoords).rgb;
    vec3 f = texture(srcTexture, TexCoords + vec2( x,  0)).rgb;
    vec3 g = texture(srcTexture, TexCoords + vec2(-x, -y)).rgb;
    vec3 h = texture(srcTexture, TexCoords + vec2( 0, -y)).rgb;
    vec3 i = texture(srcTexture, TexCoords + vec2( x, -y)).rgb;

    vec3 result = e * 4.0;
    result += (b + d + f + h) * 2.0;
    result += (a + c + g + i);
    result *= 1.0 / 16.0;

    FragColor = vec4(result, 1.0);
}
