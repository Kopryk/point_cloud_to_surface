#pragma once
#include "MeshBase.h"
#include "../../utils/sources/utils.h"

class MeshLines final :
	public MeshBase
{
public:
	explicit MeshLines(std::vector<Vertex4<float>>& points, std::vector<Vertex4<float>>& normals, std::string_view name);
	~MeshLines() override = default;
	MeshLines(const MeshLines&) = delete;
	MeshLines(MeshLines&&) = delete;
	MeshLines& operator=(const MeshLines&) = delete;
	MeshLines& operator=(MeshLines&&) = delete;


	void draw() override;

private:
	void updateUniforms() override;
	void initializeShader() override;
	void initMvp() override;
	void initVertexBuffer() override;

	std::vector<Vertex4<float>> m_points;
	std::vector<Vertex4<float>> m_normals;

	float lineLength = 0.1f;


	VertexBuffer<Vertex4<float>>* m_vertexBufferNormals = nullptr;
	VertexBufferLayout* m_vertexBufferNormalsLayout = nullptr;


};

