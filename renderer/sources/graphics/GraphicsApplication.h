#pragma once

#include "Display.h"
#include <vector>
#include "../../utils/sources/utils.h"
#include <memory>

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



private:
	GraphicsApplication() = default;
	std::vector<MeshBase*> m_meshes;
	Display* m_display = nullptr;
	Camera* m_camera = nullptr;

	std::vector<std::unique_ptr<PointCloudData>> data;
	uint32_t numberOfMeshes = 0;
	uint32_t m_activeMesh = 0;
	uint32_t lastClickedMesh = 0;

};

