#pragma once

#include <gl/glew.h>

class FrameBuffer
{
public:

    FrameBuffer() = default;
    ~FrameBuffer();
    FrameBuffer(const FrameBuffer&) = delete;
    FrameBuffer(FrameBuffer&&) = delete;
    FrameBuffer& operator=(const FrameBuffer&) = delete;
    FrameBuffer& operator= (FrameBuffer&&) = delete;

    void init(int width, int height);
    void bind() const;
    void unbind() const;
    int getWidth() { return m_width; }
    int getHeight() { return m_height; }

    void setWidth(int width) { m_width = width; }
    void setHeight(int height) { m_height = height; }


    [[nodiscard]] auto getColorTexture() const { return m_colorTexture; }
    [[nodiscard]] auto getDepthTexture() const { return m_depthTexture; }

private:

    GLuint m_frameBufferObject;
    GLuint m_colorTexture;
    GLuint m_depthTexture;
    int m_width;
    int m_height;
};

inline FrameBuffer::~FrameBuffer()
{
    glDeleteFramebuffers(1, &m_frameBufferObject);
}

inline void FrameBuffer::init(int width, int height)
{
    m_width = width;
    m_height = height;

    glCreateFramebuffers(1, &m_frameBufferObject);
    glBindFramebuffer(GL_FRAMEBUFFER, m_frameBufferObject);

    glCreateTextures(GL_TEXTURE_2D, 1, &m_colorTexture);
    glBindTexture(GL_TEXTURE_2D, m_colorTexture);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, m_width, m_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);

    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_colorTexture, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

inline void FrameBuffer::bind() const
{
    glBindFramebuffer(GL_FRAMEBUFFER, m_frameBufferObject);
}

inline void FrameBuffer::unbind() const
{
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}
