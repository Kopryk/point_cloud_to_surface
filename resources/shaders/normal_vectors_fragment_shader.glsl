#version 330 core

uniform vec4 lineColor;

out vec4 fragColor;

void main()
{

    fragColor = vec4(lineColor.x,lineColor.y, lineColor.z, 1.0f);
}