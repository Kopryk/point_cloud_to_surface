#pragma once

#include "Display.h"
#include <vector>
#include "../../utils/sources/utils.h"

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
	void init(std::vector<Vertex4<float>>& points);
	void init(std::vector<Vertex4<float>>& points, std::vector<Vertex4<float>>& normals);
	void initWithTriangles(std::vector<Vertex4<float>>& triangleVerticles);
	void initWithTrianglesWithPoints(std::vector<Vertex4<float>>& triangleVerticles, std::vector<Vertex4<float>>& points);
	


	void mainLoop();
	std::vector<Vertex4<float>> points; // TODO it should be removed
	std::vector<Vertex4<float>> normals; // TODO it should be removed
	std::vector<Vertex4<float>> triangleVerticles; // TODO it should be removed


private:
	GraphicsApplication() = default;
	std::vector<MeshBase*> m_meshes;
	Display* m_display = nullptr;
	Camera* m_camera = nullptr;
	void createMeshes(Camera* camera);
	void createMeshes2(Camera* camera);
	void createMeshesTriangle(Camera* camera);
	void createMeshesTriangleWithPoints(Camera* camera);

	
	uint32_t numberOfMeshes=0;
	//std::map<uint32_t, MeshBase*> meshesId{}; // copy of pointers, dont delete memory
	uint32_t m_activeMesh = 0;
	uint32_t lastClickedMesh = 0;

};

