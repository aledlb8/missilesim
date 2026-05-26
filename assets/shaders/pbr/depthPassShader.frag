#version 430 core

in vec2 TexCoords;

uniform sampler2D albedo;
uniform bool alphaTest;

void main(){
    if(alphaTest){
        float alpha = texture(albedo, TexCoords).a;
        if(alpha < 0.5){
            discard;
        }
    }
}
