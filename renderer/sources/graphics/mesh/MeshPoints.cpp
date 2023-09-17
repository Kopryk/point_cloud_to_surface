#include "MeshPoints.h"

#include "../Camera.h"
#include "../Renderer.h"
#include "../Shader.h"
#include "../vertex/VertexArray.h"
#include "../utils/RenderUtils.h"

MeshPoints::MeshPoints(std::vector<Vertex4<float>>& points, std::string_view name) : MeshBase(name), m_points(points)
{

}


void MeshPoints::draw()
{
	//    m_renderer->clear(); // TODO move it  moved
	m_shader->bind();
	RenderErrors::checkError();
	updateUniforms();
	RenderErrors::checkError();
	m_renderer->drawPoints(*m_vertexArray, *m_shader);
	RenderErrors::checkError();
	printDebugInfo();
	m_shader->unbind();
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

	m_shader = new Shader(R"(C:/Users/s1560/Desktop/magisterka_projekt/point_cloud_to_surface/resources/shaders/basic_vertex_shader.glsl)",
		R"(C:/Users/s1560/Desktop/magisterka_projekt/point_cloud_to_surface/resources/shaders/basic_fragment_shader.glsl)");
}

void MeshPoints::initMvp()
{
	m_model = glm::mat4(1.0f);
	m_mvp = glm::mat4(1.0f);
	m_color = glm::vec4(1.f, 0.f, 0.0f, 1.f);

	m_model = glm::rotate(m_model, glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
	m_model = glm::translate(m_model, glm::vec3(-1.0f, 0.0f, 0.0f));
	m_model = glm::scale(m_model, glm::vec3(1.f));


	m_mvp = m_camera->getViewProjection() * m_model;

}

void MeshPoints::initVertexBuffer()
{
	m_vertexArray = new VertexArray();
	m_vertexArray->bind();
	m_vertexBuffer = new VertexBuffer<Vertex4<float>>(m_points);

	m_vertexBufferLayout = new VertexBufferLayout();

	// positions
	m_vertexBufferLayout->addElement<Vertex4<float>>();
	// colors
	//  m_vertexBufferLayout->addElement<Vertex4<float>>();

	m_vertexArray->linkVertexBuffer(*m_vertexBuffer, *m_vertexBufferLayout);
	m_vertexArray->unbind();
}
