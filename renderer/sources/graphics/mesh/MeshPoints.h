#pragma once
#include "MeshBase.h"
#include "../../utils/sources/utils.h"

class MeshPoints final :
	public MeshBase
{
public:
	explicit MeshPoints(std::vector<Vertex4<float>>* points, std::string_view name);
	~MeshPoints() override = default;
	MeshPoints(const MeshPoints&) = delete;
	MeshPoints(MeshPoints&&) = delete;
	MeshPoints& operator=(const MeshPoints&) = delete;
	MeshPoints& operator=(MeshPoints&&) = delete;

	void draw() override;
	virtual bool isPoint() override { return true; }


private:
	void updateUniforms() override;
	void initializeShader() override;
	void initMvp() override;
	void initVertexBuffer() override;


};

