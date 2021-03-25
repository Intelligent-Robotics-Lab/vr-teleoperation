#version 330 core

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aTexCoord;

out vec2 texCoord;
out vec3 normal;
out vec3 fragPos;

uniform mat4 u_Model;
uniform mat4 u_View;
uniform mat4 u_Projection;

void main(){                                           
   gl_Position = u_Projection * u_View * u_Model * vec4(aPos, 1.0);
   fragPos = vec3(u_Model * vec4(aPos, 1.0));
   texCoord = aTexCoord;
   normal = mat3(transpose(inverse(u_Model))) * aNormal;
}