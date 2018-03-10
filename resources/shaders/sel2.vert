#version 430
layout (location=0) in vec3 position;
layout (location=1) in vec3 normal;

uniform mat4 modelMat;
uniform mat4 viewMat;
uniform mat4 mv_matrix;
uniform mat4 proj_matrix;
//uniform int atomID;
uniform int start;
uniform int end;
uniform int globalOffset;
uniform isamplerBuffer u_offset_Tex;
out vec3 FragPos;
flat out int atomID;
void main(void)
{
	atomID = -42;
    gl_Position = proj_matrix * viewMat * modelMat * vec4(position,1.0);
    FragPos = vec3(modelMat * vec4(position, 1.0f));
    /*for (int i = start; i < end; ++i){
    	if (gl_VertexID + globalOffset < texelFetch(u_offset_Tex, i).r){
    		atomID = i + start;
    		break;
    	}
    }*/
    //atomID = end;
    atomID = gl_VertexID + globalOffset;
}