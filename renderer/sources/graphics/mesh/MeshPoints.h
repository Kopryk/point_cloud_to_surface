#pragma once
#include "MeshBase.h"

class MeshPoints final :
    public MeshBase
{
public:
    explicit MeshPoints(std::vector<VertexType>& points, std::string_view name);
    ~MeshPoints() override = default;
    MeshPoints(const MeshPoints&) = delete;
    MeshPoints(MeshPoints&&) = delete;
    MeshPoints& operator=(const MeshPoints&) = delete;
    MeshPoints& operator=(MeshPoints&&) = delete;

    void draw() override;

private:
    void updateUniforms() override;
    void initializeShader() override;
    void initMvp() override;
    void initVertexBuffer() override;

    std::vector<VertexType> m_points;
};

