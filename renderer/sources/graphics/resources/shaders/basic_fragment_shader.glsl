#version 330 core


in vec3 vertexColor;
out vec4 FragColor;

uniform vec4 u_color;

void main()
{
    FragColor  = vec4(vertexColor,u_color.w);
};
