#version 430
layout (location=0) in vec3 position;
layout (location=1) in vec3 normal;
uniform mat4 viewMat;
uniform mat4 modelMat;
uniform mat4 projMat;
uniform int selectedMeshStart[20];
uniform int selectedMeshEnd[20];
uniform int selectedMeshCount;
uniform vec3 normalColor;
uniform vec3 selectedColor;
out vec3 Normal;
out vec3 FragPos;
out vec3 baseColor;
void main(void)
{
    gl_Position = projMat * viewMat * modelMat * vec4(position,1.0);
    Normal = normal;
    FragPos = vec3(modelMat * vec4(position, 1.0f));
    baseColor = normalColor;
    for (int i = 0; i < selectedMeshCount; ++i){
    	if (gl_VertexID >= selectedMeshStart[i] && gl_VertexID < selectedMeshEnd[i]){
    		baseColor = selectedColor;
    	}
    }
}