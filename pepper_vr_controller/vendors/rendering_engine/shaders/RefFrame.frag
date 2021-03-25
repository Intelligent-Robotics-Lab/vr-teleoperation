#version 330 core

out vec4 color;

in vec3 posColor;

void main(){
   color = vec4(posColor, 1.0);
}