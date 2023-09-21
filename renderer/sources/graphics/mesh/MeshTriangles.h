#pragma once
#include "MeshBase.h"
#include "../../utils/sources/utils.h"

class MeshTriangles final :
	public MeshBase
{
public:
	explicit MeshTriangles(std::vector<Vertex4<float>>* triangleVerticles, std::string_view name);
	~MeshTriangles() override = default;
	MeshTriangles(const MeshTriangles&) = delete;
	MeshTriangles(MeshTriangles&&) = delete;
	MeshTriangles& operator=(const MeshTriangles&) = delete;
	MeshTriangles& operator=(MeshTriangles&&) = delete;

	void draw() override;

	virtual bool isTriangle() override { return true; }

private:
	void updateUniforms() override;
	void initializeShader() override;
	void initMvp() override;
	void initVertexBuffer() override;
};

