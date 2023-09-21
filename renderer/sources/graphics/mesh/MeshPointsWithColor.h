#pragma once
#include "MeshBase.h"
#include "../../utils/sources/utils.h"

class MeshPointsWithColor final :
	public MeshBase
{
public:
	explicit MeshPointsWithColor(std::vector<Vertex4<float>>* points, std::vector<Vertex4<float>>* colors, std::string_view name);
	~MeshPointsWithColor() override = default;
	MeshPointsWithColor(const MeshPointsWithColor&) = delete;
	MeshPointsWithColor(MeshPointsWithColor&&) = delete;
	MeshPointsWithColor& operator=(const MeshPointsWithColor&) = delete;
	MeshPointsWithColor& operator=(MeshPointsWithColor&&) = delete;

	void draw() override;
	virtual bool isPoint() override { return true; }

	virtual bool hasOriginalColors() override {
		return true;
	}


	VertexBuffer<Vertex4<float>>* m_vertexBufferColors = nullptr;


private:
	void updateUniforms() override;
	void initializeShader() override;
	void initMvp() override;
	void initVertexBuffer() override;

	std::vector<Vertex4<float>>* m_colors;
};

