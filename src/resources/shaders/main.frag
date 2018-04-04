#version 430
in vec3 Normal;
in vec3 FragPos;
in vec3 baseColor;
out vec4 color;
uniform mat4 modelMat;
uniform mat4 viewMat;
uniform mat4 mv_matrix;
uniform mat4 projMat;
uniform mat4 mvInverse;
uniform vec3 col;
uniform float alpha;
uniform vec3 lightColor;
uniform vec3 lightPos;
uniform float ambientStrength;
void main(void)
{
    //float ambientStrength = 0.5f;
    vec3 ambient = ambientStrength * lightColor;
    vec3 normal2 = (mvInverse * vec4(Normal,0.0)).xyz;

    //vec3 truePOS = (viewMat * vec4(FragPos, 1.0)).xyz;

    vec3 lightDir = normalize(lightPos - FragPos);
    float diff = max(dot(lightDir, normal2), 0.0f);
    vec3 diffuse = lightColor * diff;
    vec3 result = (ambient + diffuse) * baseColor;
    color = vec4(result, alpha);
}