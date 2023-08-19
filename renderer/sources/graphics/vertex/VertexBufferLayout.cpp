#include "VertexBufferLayout.h"
#include "VertexElementGL.h"


const std::vector<VertexElementGL>& VertexBufferLayout::getElements() const
{
    return m_elements;
}

