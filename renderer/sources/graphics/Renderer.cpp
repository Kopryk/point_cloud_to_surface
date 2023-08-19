#include "Renderer.h"

#include "Shader.h"
#include "vertex/VertexArray.h"

Renderer::Renderer()
{
    m_clearColor[0] = 0.2f;
    m_clearColor[1] = 0.3f;
    m_clearColor[2] = 0.3f;
    m_clearColor[3] = 1.0f;

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void Renderer::draw(VertexArray& vertexArray, Shader& shader)  const  // TODO glDrawArrays just for cloudPoint
{
    shader.bind();
    vertexArray.bind();
    glDrawArrays(GL_POINTS, 0, vertexArray.getNumberPrimitives());
}

void Renderer::clear() const
{
    //glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClearColor(m_clearColor[0], m_clearColor[1], m_clearColor[2], m_clearColor[3]);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::setClearColor(float clearColor[4])
{
    m_clearColor[0] = clearColor[0];
    m_clearColor[1] = clearColor[1];
    m_clearColor[2] = clearColor[2];
    m_clearColor[3] = clearColor[3];
}
