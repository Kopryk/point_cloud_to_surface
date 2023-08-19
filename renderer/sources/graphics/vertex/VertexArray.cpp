#include "VertexArray.h"

VertexArray::VertexArray()
{
    glGenVertexArrays(1, &m_vertexArrayObject);
}

void VertexArray::bind() const
{
    glBindVertexArray(m_vertexArrayObject);
}

void VertexArray::unbind() const
{
    glBindVertexArray(0);
}

void VertexArray::setNumberPrimitives(GLint n)
{
    m_nPrimitives = n;
}

GLint VertexArray::getNumberPrimitives() const
{
    return m_nPrimitives;
}

