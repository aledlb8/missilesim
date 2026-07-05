#version 430 core
out vec4 FragColor;

in vec2 TexCoords;

uniform sampler2D screenTexture;

// Bright-pass with a quadratic soft knee below the threshold so bloom
// fades in smoothly instead of popping on pixels crossing 1.0.
void main(){
    vec3 color = texture(screenTexture, TexCoords).rgb;
    float brightness = dot(color, vec3(0.2126, 0.7152, 0.0722));

    const float threshold = 1.0;
    const float knee = 0.5;
    float soft = clamp(brightness - threshold + knee, 0.0, 2.0 * knee);
    soft = soft * soft / (4.0 * knee);
    float weight = max(soft, brightness - threshold) / max(brightness, 1e-4);

    FragColor = vec4(color * weight, 1.0);
}
