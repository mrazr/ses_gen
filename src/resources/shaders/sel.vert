#version 430
layout (location=0) in vec3 position;
layout (location=1) in vec3 normal;

uniform mat4 modelMat;
uniform mat4 viewMat;
uniform mat4 mv_matrix;
uniform mat4 proj_matrix;
uniform int atomID;

out vec3 FragPos;
void main(void)
{
    gl_Position = proj_matrix * viewMat * modelMat * vec4(position,1.0);
    FragPos = vec3(modelMat * vec4(position, 1.0f));
}