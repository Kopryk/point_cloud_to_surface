#include "MeshPoints.h"

#include "../Camera.h"
#include "../Renderer.h"
#include "../Shader.h"
#include "../vertex/VertexArray.h"


MeshPoints::MeshPoints(std::vector<VertexType>& points, std::string_view name) : MeshBase(name), m_points(points)
{

}


void MeshPoints::draw()
{
    m_renderer->clear();
    m_shader->bind();
    updateUniforms();

    m_renderer->draw(*m_vertexArray, *m_shader);
    printDebugInfo();
}

void MeshPoints::updateUniforms()
{
    //// TODO remove
    //auto m_viewCopy = glm::rotate(m_view, glm::radians(m_rotationView), glm::vec3(0.0f, 1.0f, 0.0f));

    //auto cameraMovement = glm::vec3(m_modelPosition[0], m_modelPosition[1], m_modelPosition[2]);
    //m_viewCopy = glm::translate(m_viewCopy, cameraMovement);

    m_mvp = m_camera->getViewProjection() * m_model;

    // m_shader->setUniform("u_model", m_model);
    // m_shader->setUniform("u_view", m_view);
    // m_shader->setUniform("u_projection", m_projection);
    m_shader->setUniform("u_color", m_color);
    m_shader->setUniform("u_mvp", m_mvp);
}

void MeshPoints::initializeShader()
{

    m_shader = new Shader(R"(resources/shaders/basic_vertex_shader.glsl)",
        R"(resources/shaders/basic_fragment_shader.glsl)");
}

void MeshPoints::initMvp()
{
    m_model = glm::mat4(1.0f);
    m_mvp = glm::mat4(1.0f);
    m_color = glm::vec4(0.f, 0.f, 0.5f, 1.f);

    m_model = glm::rotate(m_model, glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    //m_model = glm::translate(m_model, glm::vec3(-1.0f, 0.0f, 0.0f));
    m_model = glm::scale(m_model, glm::vec3(0.01f));

    m_mvp = m_camera->getViewProjection() * m_model;

}

void MeshPoints::initVertexBuffer()
{
    m_vertexArray = new VertexArray();
    m_vertexArray->bind();
    m_vertexBuffer = new VertexBuffer<VertexType>(m_points);

    m_vertexBufferLayout = new VertexBufferLayout();

    // positions
    m_vertexBufferLayout->addElement<VertexType>();
    // colors
    m_vertexBufferLayout->addElement<VertexType>();

    m_vertexArray->linkVertexBuffer(*m_vertexBuffer, *m_vertexBufferLayout);
}