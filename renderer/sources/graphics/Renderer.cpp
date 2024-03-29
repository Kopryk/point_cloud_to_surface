#include "Renderer.h"

#include "Shader.h"
#include "vertex/VertexArray.h"

#include "utils/RenderUtils.h"
Renderer::Renderer()
{
	m_clearColor[0] = 0.2f;
	m_clearColor[1] = 0.3f;
	m_clearColor[2] = 0.3f;
	m_clearColor[3] = 1.0f;

	glEnable(GL_DEPTH_TEST);
	//glDisable(GL_CULL_FACE);

	glEnable(GL_BLEND);
	//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	//glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

}

void Renderer::drawTriangles(VertexArray& vertexArray, Shader& shader)  const
{
	RenderErrors::checkError();
	shader.bind();
	RenderErrors::checkError();
	vertexArray.bind();
	RenderErrors::checkError();
	glDrawArrays(GL_TRIANGLES, 0, vertexArray.getNumberPrimitives());
	RenderErrors::checkError();
	vertexArray.unbind();
	RenderErrors::checkError();
	shader.unbind();
	RenderErrors::checkError();

}


void Renderer::drawPoints(VertexArray& vertexArray, Shader& shader)  const  // TODO glDrawArrays just for cloudPoint
{
	shader.bind();
	vertexArray.bind();
	glDrawArrays(GL_POINTS, 0, vertexArray.getNumberPrimitives());
	vertexArray.unbind();
	shader.unbind();

}

void Renderer::drawLines(VertexArray& vertexArray, Shader& shader)  const  // since lines are generated by geometry shader
{
	RenderErrors::checkError();

	shader.bind();
	RenderErrors::checkError();

	vertexArray.bind();
	RenderErrors::checkError();

	glDrawArrays(GL_POINTS, 0, vertexArray.getNumberPrimitives());
	RenderErrors::checkError();

	vertexArray.unbind();
	shader.unbind();

}


void Renderer::clear() const
{
	//glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
	glClearColor(m_clearColor[0], m_clearColor[1], m_clearColor[2], m_clearColor[3]);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


}

void Renderer::setClearColor(float clearColor[4])
{
	m_clearColor[0] = clearColor[0];
	m_clearColor[1] = clearColor[1];
	m_clearColor[2] = clearColor[2];
	m_clearColor[3] = clearColor[3];
}
