
#version 330 core

layout(location=0) in vec3 position;
//layout(location=1) in vec3 color;

uniform mat4 u_model;
uniform mat4 u_view;
uniform mat4 u_projection;
uniform mat4 u_mvp;

out vec3 vertexColor;

void main()
{
    
    //gl_Position = u_projection * u_view * u_model * u_position;
    gl_Position = u_mvp * vec4(position.x, position.y, position.z, 1.0);;
    vertexColor = vec3(1.0f, 0.5f, 0.2f);

}
