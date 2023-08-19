#pragma once
#include <GLFW/glfw3.h>

struct VertexElementGL
{
    GLuint index;
    GLint count;
    GLenum type;
    GLboolean isNormalized;
    GLsizei stride;
    size_t offset;
    size_t sizeInBytes;
};

