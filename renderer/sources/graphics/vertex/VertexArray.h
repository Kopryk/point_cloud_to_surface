#pragma once
#include "VertexBuffer.h"
#include "VertexBufferLayout.h"
#include "VertexElementGL.h"

class VertexArray
{
public:
    VertexArray();
    ~VertexArray() = default;
    VertexArray(const VertexArray&) = delete;
    VertexArray(VertexArray&&) = delete;
    VertexArray& operator=(const VertexArray&) = delete;
    VertexArray& operator=(VertexArray&&) = delete;

    void bind() const;
    void unbind() const;

    template<typename T>
    void linkVertexBuffer(const VertexBuffer<T>& vertexBuffer, const VertexBufferLayout& vertexBufferLayout);

    void setNumberPrimitives(GLint n);
    [[nodiscard]] GLint getNumberPrimitives() const;

private:
    GLuint m_vertexArrayObject =0;
    GLint m_nPrimitives =0;
};

template <typename T>
void VertexArray::linkVertexBuffer(const VertexBuffer<T>& vertexBuffer, const VertexBufferLayout& vertexBufferLayout)
{
    const auto& layouts = vertexBufferLayout.getElements();

    vertexBuffer.bind();
    vertexBuffer.setData();

    for (const auto& [index, count, type, isNormalized, stride, offset, sizeInBytes] : layouts)
    {
        glVertexAttribPointer(index, count, type, isNormalized, stride, reinterpret_cast<void*>(offset));
        glEnableVertexAttribArray(index);
    }

    setNumberPrimitives(vertexBuffer.getCount());

    vertexBuffer.unbind();
}

