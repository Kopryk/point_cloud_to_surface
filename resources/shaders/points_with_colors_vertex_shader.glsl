#version 330 core

layout(location=0) in vec3 position;
layout(location=1) in vec3 color;

uniform mat4 u_mvp;

out vec3 fragmentColor; 

void main()
{
    
    gl_Position = u_mvp * vec4(position.x, position.y, position.z, 1.0);
    fragmentColor = color;
}
