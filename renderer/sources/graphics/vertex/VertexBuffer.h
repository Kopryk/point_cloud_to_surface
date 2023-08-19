#pragma once

#include <gl/glew.h>
#include <vector>

template <typename T>
class VertexBuffer
{
public:
    explicit VertexBuffer(std::vector<T>& data);
    ~VertexBuffer() = default;
    VertexBuffer(const VertexBuffer&) = delete;
    VertexBuffer(VertexBuffer&&) = delete;
    VertexBuffer& operator= (const VertexBuffer&) = delete;
    VertexBuffer& operator= (VertexBuffer&&) = delete;

    void bind() const;
    void unbind() const;
    void setData() const;
    [[nodiscard]] uint32_t getCount() const;

private:
    GLuint m_vertexBufferObject;
    std::vector<T> m_data;
};

template <typename T>
VertexBuffer<T>::VertexBuffer(std::vector<T>& data) : m_data(data)
{
    glGenBuffers(1, &m_vertexBufferObject);
}

template <typename T>
void VertexBuffer<T>::bind() const
{
    glBindBuffer(GL_ARRAY_BUFFER, m_vertexBufferObject);
}

template <typename T>
void VertexBuffer<T>::unbind() const
{
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

template <typename T>
void VertexBuffer<T>::setData() const
{
    auto sizeInBytes = m_data.size() * sizeof(T);
    glBufferData(GL_ARRAY_BUFFER, sizeInBytes, m_data.data(), GL_STATIC_DRAW);
}

template <typename T>
uint32_t VertexBuffer<T>::getCount() const
{
    return static_cast<uint32_t>(m_data.size());
}
