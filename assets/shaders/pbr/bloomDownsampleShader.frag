#version 430 core
out vec4 FragColor;

in vec2 TexCoords;

uniform sampler2D srcTexture;
// First downsample out of the scene: apply the bright-pass prefilter and a
// Karis average so single hot pixels (spark fireflies, MSAA resolve speckle)
// don't flicker their way through the whole chain.
uniform bool firstPass;

const vec3 kLuma = vec3(0.2126, 0.7152, 0.0722);

vec3 karisAverage4(vec3 a, vec3 b, vec3 c, vec3 d){
    float wa = 1.0 / (1.0 + dot(a, kLuma));
    float wb = 1.0 / (1.0 + dot(b, kLuma));
    float wc = 1.0 / (1.0 + dot(c, kLuma));
    float wd = 1.0 / (1.0 + dot(d, kLuma));
    return (a * wa + b * wb + c * wc + d * wd) / (wa + wb + wc + wd);
}

// Soft-knee bright pass: only HDR energy above ~threshold feeds the bloom.
vec3 prefilter(vec3 c){
    const float threshold = 1.0;
    const float knee = 0.5;
    float br = max(c.r, max(c.g, c.b));
    float soft = clamp(br - threshold + knee, 0.0, 2.0 * knee);
    soft = soft * soft / (4.0 * knee);
    float w = max(soft, br - threshold) / max(br, 1e-4);
    return c * w;
}

// 13-tap downsample (Jimenez, "Next Generation Post Processing in CoD:AW").
void main(){
    vec2 t = 1.0 / vec2(textureSize(srcTexture, 0));
    float x = t.x;
    float y = t.y;

    vec3 a = texture(srcTexture, TexCoords + vec2(-2.0 * x,  2.0 * y)).rgb;
    vec3 b = texture(srcTexture, TexCoords + vec2( 0.0,      2.0 * y)).rgb;
    vec3 c = texture(srcTexture, TexCoords + vec2( 2.0 * x,  2.0 * y)).rgb;
    vec3 d = texture(srcTexture, TexCoords + vec2(-2.0 * x,  0.0)).rgb;
    vec3 e = texture(srcTexture, TexCoords).rgb;
    vec3 f = texture(srcTexture, TexCoords + vec2( 2.0 * x,  0.0)).rgb;
    vec3 g = texture(srcTexture, TexCoords + vec2(-2.0 * x, -2.0 * y)).rgb;
    vec3 h = texture(srcTexture, TexCoords + vec2( 0.0,     -2.0 * y)).rgb;
    vec3 i = texture(srcTexture, TexCoords + vec2( 2.0 * x, -2.0 * y)).rgb;
    vec3 j = texture(srcTexture, TexCoords + vec2(-x,  y)).rgb;
    vec3 k = texture(srcTexture, TexCoords + vec2( x,  y)).rgb;
    vec3 l = texture(srcTexture, TexCoords + vec2(-x, -y)).rgb;
    vec3 m = texture(srcTexture, TexCoords + vec2( x, -y)).rgb;

    vec3 result;
    if (firstPass) {
        vec3 g0 = karisAverage4(j, k, l, m) * 0.5;
        vec3 g1 = karisAverage4(a, b, d, e) * 0.125;
        vec3 g2 = karisAverage4(b, c, e, f) * 0.125;
        vec3 g3 = karisAverage4(d, e, g, h) * 0.125;
        vec3 g4 = karisAverage4(e, f, h, i) * 0.125;
        result = prefilter(g0 + g1 + g2 + g3 + g4);
    } else {
        result  = e * 0.125;
        result += (a + c + g + i) * 0.03125;
        result += (b + d + f + h) * 0.0625;
        result += (j + k + l + m) * 0.125;
    }

    FragColor = vec4(result, 1.0);
}
