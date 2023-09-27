#pragma once

#include "Display.h"
#include <vector>
#include "../../utils/sources/utils.h"
#include <memory>
#include <string>

// clang-format off
#include <gl/glew.h>
#include <GLFW/glfw3.h>
// clang-format on


class Camera;
class MeshBase;

class GraphicsApplication
{
public:

	static GraphicsApplication& get()
	{
		static GraphicsApplication instance{};
		return instance;
	}

	GraphicsApplication(const GraphicsApplication&) = delete;
	GraphicsApplication& operator=(const GraphicsApplication&) = delete;
	~GraphicsApplication();

	void init();
	void initRawPointCloud();
	void initCalculatedSurface(std::vector<Vertex4<float>>* surface);
	void mainLoop();
	void saveAsObj(std::vector < Vertex4<float>>& points, const std::string& name);

private:
	GraphicsApplication() = default;
	std::vector<MeshBase*> m_meshes;
	Display* m_display = nullptr;
	Camera* m_camera = nullptr;

	std::vector<std::unique_ptr<PointCloudData>> data;
	uint32_t numberOfMeshes = 0;
	uint32_t m_activeMesh = 0;
	uint32_t lastClickedMesh = 0;

	PointCloudOptimizationMode optimizationMode = PointCloudOptimizationMode::None;
	SurfaceReconstructionMode reconstructionMode = SurfaceReconstructionMode::GreedyProjectionTriangulation;
	uint32_t depthOctree = 8;
	uint32_t gridResolution = 128;
	float gridSizeInPercent = 1.0;
	float dilationVoxelSizeInPercent = 1.0;
	uint32_t dilationIteration = 2;

};

