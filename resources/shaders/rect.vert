#version 430
layout (location=0) in vec3 position;
layout (location=1) in vec3 color;
uniform mat4 viewMat;
uniform mat4 modelMat;
uniform mat4 proj_matrix;
out vec3 Color;
out vec3 FragPos;
void main(void)
{
    gl_Position = proj_matrix * viewMat * modelMat * vec4(position,1.0);
    Color = color;
    FragPos = vec3(modelMat * vec4(position, 1.0f));
}