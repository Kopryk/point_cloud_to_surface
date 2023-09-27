#pragma once


#include "../vertex/VertexBuffer.h"
#include "../../utils/sources/utils.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>


class Camera;
class Shader;
class VertexBufferLayout;
class VertexArray;
class Renderer;

class MeshBase
{
public:
	explicit  MeshBase(std::string_view name = "base", std::vector<Vertex4<float>>* data = nullptr);
	virtual  ~MeshBase();
	MeshBase(const MeshBase&) = delete;
	MeshBase(MeshBase&&) = delete;
	MeshBase& operator=(const MeshBase&) = delete;
	MeshBase& operator=(MeshBase&&) = delete;

	virtual void draw() = 0;
	virtual void init(Camera* camera, std::string& pathToShaderDir);
	virtual void updateModel(float modelPosition[]);

	Renderer* getRenderer() { return m_renderer; }

	void setColor(Vertex4<float> color);
	Vertex4<float> getColor();

	std::string m_name;

	bool isActive() {
		return active;
	}

	virtual bool isPoint() {
		return false;
	}


	virtual bool isTriangle() {
		return false;
	}

	float& getPointSize() {
		return pointSize;
	}

	virtual bool hasOriginalColors() {
		return false;
	}


	std::string pathToShaderDir = "";

	float pointSize = 1.0f;
	bool active = true;
	void changeActivity() { std::cout << " active = " << active << std::endl;  active = !active; }
	int selectedMode = 0;
	std::vector<Vertex4<float>>* data;

protected:

	Camera* m_camera;
	Shader* m_shader = nullptr; // TODO
	Renderer* m_renderer = nullptr;   // TODO use unique_ptr 
	glm::mat4 m_model = glm::mat4(1.0f);

	glm::vec4 m_color = glm::vec4(1.0f);
	glm::mat4 m_mvp = glm::mat4(1.0f);// TODO link all matrices to just one
	float lineLenght = 1.0f;
	float m_modelPosition[3] = { 0.0f,0.0f,0.0f };

	VertexArray* m_vertexArray = nullptr;
	VertexBuffer<Vertex4<float>>* m_vertexBuffer = nullptr;
	VertexBufferLayout* m_vertexBufferLayout = nullptr;
	//VertexIndices* m_vertexIndices;

	virtual void updateUniforms() = 0;
	virtual void initializeShader() = 0;
	virtual void initMvp() = 0;
	virtual void initVertexBuffer() = 0;
	void printDebugInfo() const;

};
