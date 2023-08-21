#include "MeshLines.h"

#include "../Camera.h"
#include "../Renderer.h"
#include "../Shader.h"
#include "../vertex/VertexArray.h"
#include "../utils/RenderUtils.h"

MeshLines::MeshLines(std::vector<Vertex4<float>>& points, std::vector<Vertex4<float>>& normals, std::string_view name) : MeshBase(name), m_points(points), m_normals(normals)
{

}


void MeshLines::draw()
{

	m_shader->bind();
	RenderErrors::checkError();

	updateUniforms();
	RenderErrors::checkError();

	m_vertexBuffer->bind();
	m_vertexBufferNormals->bind();

	m_renderer->drawLines(*m_vertexArray, *m_shader);
	RenderErrors::checkError();

	printDebugInfo();


	m_vertexBuffer->unbind();
	m_vertexBufferNormals->unbind();


	m_shader->unbind();
	RenderErrors::checkError();


}

void MeshLines::updateUniforms()
{
	//// TODO remove
	//auto m_viewCopy = glm::rotate(m_view, glm::radians(m_rotationView), glm::vec3(0.0f, 1.0f, 0.0f));

	//auto cameraMovement = glm::vec3(m_modelPosition[0], m_modelPosition[1], m_modelPosition[2]);
	//m_viewCopy = glm::translate(m_viewCopy, cameraMovement);

	m_mvp = m_camera->getViewProjection() * m_model;

	// vertex shader
	// uniform float lineLength;
	// uniform mat4 u_mvp;

	// fragment shader

	// uniform vec3 lineColor;
	RenderErrors::checkError();

	m_shader->setUniform("lineColor", m_color);
	RenderErrors::checkError();

	m_shader->setUniform("u_mvp", m_mvp);
	RenderErrors::checkError();

	m_shader->setUniform("lineLength", lineLength);
	RenderErrors::checkError();

}

void MeshLines::initializeShader()
{

	m_shader = new Shader(R"(C:/Users/s1560/Desktop/magisterka_projekt/point_cloud_to_surface/resources/shaders/normal_vectors_vertex_shader.glsl)",
		R"(C:/Users/s1560/Desktop/magisterka_projekt/point_cloud_to_surface/resources/shaders/normal_vectors_fragment_shader.glsl)",
		R"(C:/Users/s1560/Desktop/magisterka_projekt/point_cloud_to_surface/resources/shaders/normal_vectors_geometry_shader.glsl)");
}

void MeshLines::initMvp()
{
	m_model = glm::mat4(1.0f);
	m_mvp = glm::mat4(1.0f);
	m_color = glm::vec4(0.f, 0.f, 0.5f, 1.f);

	m_model = glm::rotate(m_model, glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
	//m_model = glm::translate(m_model, glm::vec3(-1.0f, 0.0f, 0.0f));
	m_model = glm::scale(m_model, glm::vec3(10.0f));

	m_mvp = m_camera->getViewProjection() * m_model;

}

void MeshLines::initVertexBuffer()
{
	m_vertexArray = new VertexArray();
	m_vertexArray->bind();

	// positions
	m_vertexBuffer = new VertexBuffer<Vertex4<float>>(m_points);
	m_vertexBufferLayout = new VertexBufferLayout();

	m_vertexBufferLayout->addElement<Vertex4<float>>();
	m_vertexArray->linkVertexBuffer(*m_vertexBuffer, *m_vertexBufferLayout);

	// normals
	m_vertexBufferNormals = new VertexBuffer<Vertex4<float>>(m_normals);
	auto startingIndex = 1u;    // since it will be bound to 1 VAO
	m_vertexBufferNormalsLayout = new VertexBufferLayout(startingIndex);

	m_vertexBufferNormalsLayout->addElement<Vertex4<float>>();
	m_vertexArray->linkVertexBuffer(*m_vertexBufferNormals, *m_vertexBufferNormalsLayout);

	// override numver of primitivies   = number of vertices
	m_vertexArray->setNumberPrimitives(m_vertexBuffer->getCount());

	m_vertexArray->unbind();


}
