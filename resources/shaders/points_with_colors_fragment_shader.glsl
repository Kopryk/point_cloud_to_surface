#version 330 core

out vec4 FragColor;

uniform vec4 u_color;

in vec3 fragmentColor; 

void main()
{
    FragColor  = vec4(fragmentColor.r, fragmentColor.g, fragmentColor.b, u_color.a);
};
