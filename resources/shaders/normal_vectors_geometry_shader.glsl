#version 330 core

layout(points) in;
layout(line_strip, max_vertices = 2) out;

in vec3 outStartPoint[];
in vec3 outEndPoint[];

void main()
{
    gl_Position = vec4(outStartPoint[0], 1.0);
    EmitVertex();
    gl_Position = vec4(outEndPoint[0], 1.0);
    EmitVertex();
    EndPrimitive();
}