#version 430
in vec3 FragPos;

out int id;
uniform mat4 modelMat;
uniform mat4 viewMat;
uniform mat4 mv_matrix;
uniform mat4 proj_matrix;

uniform int start;
uniform int end;
uniform int globalOffset;
uniform isamplerBuffer u_offset_Tex;
flat in int atomID;

void main(void)
{
	id = atomID;
}