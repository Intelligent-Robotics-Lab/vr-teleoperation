#version 330 core

out vec4 color;

in vec2 texCoord;
in vec3 normal;
in vec3 fragPos;

uniform sampler2D inTexture;

uniform vec3 u_LightPos;
uniform float u_AmbientStrength;
uniform vec3 u_LightColor;
uniform vec3 u_ViewPos;
uniform float u_SpecularStrength;

void main(){
   vec3 ambient = u_AmbientStrength * u_LightColor;

   vec3 norm = normalize(normal);
   vec3 lightDir = normalize(u_LightPos - fragPos);
   float diff = max(dot(norm, lightDir), 0.0);
   vec3 diffuse = diff * u_LightColor;

   vec3 viewDir = normalize(u_ViewPos - fragPos);
   vec3 reflectDir = reflect(-lightDir, norm);
   float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);
   vec3 specular = u_SpecularStrength * spec * u_LightColor;

   vec4 texColor = texture(inTexture, texCoord);
   color = texColor * vec4((diffuse + ambient + specular), 1.0);
}