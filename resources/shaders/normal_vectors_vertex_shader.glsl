
#version 330 core

layout(location = 0) in vec3 inPosition;

uniform mat4 u_mvp;

void main() {
    // Calculate the transformed start point
    vec4 transformedStartPoint = u_mvp * vec4(inPosition, 1.0f);
    
    // Pass the start point to the next stage
    gl_Position = transformedStartPoint;
}