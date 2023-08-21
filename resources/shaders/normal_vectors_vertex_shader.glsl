
#version 330 core

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;

out vec3 outStartPoint;
out vec3 outEndPoint;

uniform float lineLength;
uniform mat4 u_mvp;

void main()
{
    
    outStartPoint = (u_mvp * vec4(inPosition, 1.0)).xyz;
    outEndPoint = (u_mvp * vec4(inPosition + inNormal * lineLength, 1.0)).xyz;

    gl_Position = vec4(outEndPoint, 1.0);
}
