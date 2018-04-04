#version 430
in vec3 Color;
in vec3 FragPos;
out vec4 color;
uniform mat4 modelMat;
uniform mat4 viewMat;
uniform mat4 mv_matrix;
uniform mat4 proj_matrix;
uniform vec3 col;
uniform vec3 lightColor;
uniform vec3 lightPos;
void main(void)
{
    color = vec4(Color, 1.0);
}