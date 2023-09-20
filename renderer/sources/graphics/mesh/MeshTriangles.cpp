#include "MeshTriangles.h"

#include "../Camera.h"
#include "../Renderer.h"
#include "../Shader.h"
#include "../vertex/VertexArray.h"
#include "../utils/RenderUtils.h"

MeshTriangles::MeshTriangles(std::vector<Vertex4<float>>* triangleVerticles, std::string_view name) : MeshBase(name), m_triangleVerticles(triangleVerticles)
{

}


void MeshTriangles::draw()
{

	
	if (selectedMode == 0 || selectedMode == 2) {

		glEnable(GL_POLYGON_OFFSET_FILL);
		glPolygonOffset(1.0f, 1.0f);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

		//    m_renderer->clear(); // TODO move it  moved
		m_shader->bind();
		RenderErrors::checkError();
		updateUniforms();
		RenderErrors::checkError();
		m_renderer->drawTriangles(*m_vertexArray, *m_shader);
		RenderErrors::checkError();
		printDebugInfo();
		m_shader->unbind();
		glDisable(GL_POLYGON_OFFSET_FILL);
	}

	if (selectedMode == 1 || selectedMode == 2) {
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		m_shader->bind();
		RenderErrors::checkError();
		updateUniforms();
		RenderErrors::checkError();
		m_renderer->drawTriangles(*m_vertexArray, *m_shader);
		RenderErrors::checkError();
		printDebugInfo();
		m_shader->unbind();
	}


}

void MeshTriangles::updateUniforms()
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
	RenderErrors::checkError();
}

void MeshTriangles::initializeShader()
{

	m_shader = new Shader(R"(C:/Users/s1560/Desktop/magisterka_projekt/point_cloud_to_surface/resources/shaders/triangles_vertex_shader.glsl)",
		R"(C:/Users/s1560/Desktop/magisterka_projekt/point_cloud_to_surface/resources/shaders/triangles_fragment_shader.glsl)");
}

void MeshTriangles::initMvp()
{
	m_model = glm::mat4(1.0f);
	m_mvp = glm::mat4(1.0f);
	m_color = glm::vec4(0.0f, 1.f, 0.0f, 0.5f);

	m_model = glm::rotate(m_model, glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
	m_model = glm::translate(m_model, glm::vec3(-1.0f, 0.0f, 0.0f));
	m_model = glm::scale(m_model, glm::vec3(0.05f));


	m_mvp = m_camera->getViewProjection() * m_model;

}

void MeshTriangles::initVertexBuffer()
{
	std::cout << " m_triangleVerticles size=" << m_triangleVerticles->size() << std::endl;
	m_vertexArray = new VertexArray();
	m_vertexArray->bind();
	RenderErrors::checkError();
	m_vertexBuffer = new VertexBuffer<Vertex4<float>>(m_triangleVerticles);
	m_vertexBufferLayout = new VertexBufferLayout();
	RenderErrors::checkError();
	// positions
	m_vertexBufferLayout->addElement<Vertex4<float>>();
	// colors
	//  m_vertexBufferLayout->addElement<Vertex4<float>>();
	RenderErrors::checkError();
	m_vertexArray->linkVertexBuffer(*m_vertexBuffer, *m_vertexBufferLayout, 0);
	RenderErrors::checkError();
	m_vertexArray->unbind();
	RenderErrors::checkError();
}
