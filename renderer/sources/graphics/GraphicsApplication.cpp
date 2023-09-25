#include "GraphicsApplication.h"

#include "FileDialog.h"
#include "mesh/MeshPoints.h"
#include "mesh/MeshLines.h"
#include "mesh/MeshPointsWithColor.h"
#include "utils/RenderUtils.h"
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include "mesh/MeshTriangles.h"
#include "../../surface_reconstruction/sources/point_cloud_library/point_cloud_library.h"
#include <iostream>

#include "TaskManager.h"
#include "Camera.h"
#include "FrameBuffer.h"
#include "Renderer.h"

#include <chrono>
#include <iomanip>
#include <sstream>

GraphicsApplication::~GraphicsApplication()
{
	m_meshes.clear();
}

void GraphicsApplication::init()
{
	m_display = &Display::get();
	m_display->init();
	m_camera = &Camera::get();
	m_camera->init(m_display->getWidth(), m_display->getHeight());

	m_activeMesh = 0;
}

void GraphicsApplication::initRawPointCloud()
{

	if (data.back()->containsPointsWithColors) {

		m_activeMesh += 1;
		std::string pointsMeshName = "environment_" + std::to_string(this->numberOfMeshes);
		auto pointsMesh2 = new MeshPointsWithColor(&data.back()->environment.points, &data.back()->environment.colors, pointsMeshName);
		pointsMesh2->init(m_camera);
		m_meshes.emplace_back(pointsMesh2);
		numberOfMeshes++;

		m_activeMesh += 1;
		pointsMeshName = "buildings_" + std::to_string(this->numberOfMeshes);
		auto pointsMesh = new MeshPointsWithColor(&data.back()->buildings.points, &data.back()->buildings.colors, pointsMeshName);
		pointsMesh->init(m_camera);
		m_meshes.emplace_back(pointsMesh);
		numberOfMeshes++;

	}
	else if (data.back()->containsPoints) {
		std::string pointsMeshName = "point_cloud_" + std::to_string(this->numberOfMeshes);
		m_activeMesh += 1;
		auto pointsMesh = new MeshPoints(&data.back()->buildings.points, pointsMeshName);
		pointsMesh->init(m_camera);
		m_meshes.emplace_back(pointsMesh);
		numberOfMeshes++;
	}

}

void GraphicsApplication::initCalculatedSurface(std::vector<Vertex4<float>>* surface)
{
	m_activeMesh += 1;

	std::string trianglesMeshName = "surface_" + std::to_string(this->numberOfMeshes);
	auto trianglesMesh = new MeshTriangles(surface, trianglesMeshName);

	trianglesMesh->init(m_camera);

	m_meshes.emplace_back(trianglesMesh);

	numberOfMeshes++;
}


void ShowDockSpace(bool* p_open)
{
	static bool opt_fullscreen_persistant = true;
	bool opt_fullscreen = opt_fullscreen_persistant;
	static ImGuiDockNodeFlags dockspace_flags = ImGuiDockNodeFlags_None;

	ImGuiWindowFlags window_flags = ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoDocking;
	if (opt_fullscreen)
	{
		ImGuiViewport* viewport = ImGui::GetMainViewport();
		ImGui::SetNextWindowPos(viewport->Pos);
		ImGui::SetNextWindowSize(viewport->Size);
		ImGui::SetNextWindowViewport(viewport->ID);
		ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
		ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
		window_flags |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
		window_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;
	}

	if (dockspace_flags & ImGuiDockNodeFlags_PassthruCentralNode)
		window_flags |= ImGuiWindowFlags_NoBackground;

	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
	ImGui::Begin("###DockSpace", p_open, window_flags);
	ImGui::PopStyleVar();

	if (opt_fullscreen)
		ImGui::PopStyleVar(2);

	// DockSpace
	ImGuiIO& io = ImGui::GetIO();
	if (io.ConfigFlags & ImGuiConfigFlags_DockingEnable)
	{
		ImGuiID dockspace_id = ImGui::GetID("MyDockSpace");
		ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), dockspace_flags);
	}
	else
	{
		//ShowDockingDisabledMessage();
	}

	ImGui::End();
}


void DebugMessageCallback(GLenum source, GLenum type, GLuint id, GLenum severity,
	GLsizei length, const GLchar* message, const void* userParam) {
	std::cerr << "OpenGL Debug Message: " << message << std::endl;
}


void GraphicsApplication::mainLoop()
{
	init();

	PointCloudLibrary pcl;
	TaskManager taskManager(&pcl);
	FrameBuffer frameBuffer;

	frameBuffer.init(m_display->getRenderWindowWidth(), m_display->getRenderWindowHeight());

	// Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();
	io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
	io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;

	// Setup Platform/Renderer bindings
	ImGui_ImplGlfw_InitForOpenGL(m_display->getWindow(), true);
	ImGui_ImplOpenGL3_Init("#version 330");//TODO
	ImGui::StyleColorsDark();

	static bool dockspaceOpen = true;
	static bool fullscreen = true;

	glEnable(GL_DEBUG_OUTPUT);
	glDebugMessageCallback(DebugMessageCallback, nullptr);


	while (!glfwWindowShouldClose(m_display->getWindow()))
	{
		// feed inputs to dear imgui, start new frame
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		// clear errors
		RenderErrors::clearError();
		// input


		ShowDockSpace(&dockspaceOpen);
		ImGui::Begin("Infos");
		static bool render = false;
		if (ImGui::Button("Open viewport"))
		{
			render = !render;
		}

		if (ImGui::Button("Open pointCloud"))
		{

			taskManager.startLoadPoints();
			taskManager.isLoadPointsResultReceived = false;
		}

		if (taskManager.isLoadPointsResultReceived == false) {
			if (taskManager.isJobDone()) {
				taskManager.isLoadPointsResultReceived = true;
				auto result = taskManager.getResults();
				if (result != nullptr &&
					(result->buildings.points.size() > 0 || result->environment.points.size() > 0)) {

					this->data.push_back(std::move(result));
					initRawPointCloud();
				}

			}
		}

		ImGui::End();

		// update camera
		m_camera->update();

		if (!m_meshes.empty() && render)
		{
			RenderErrors::checkError();
			frameBuffer.bind();
			m_meshes[0]->getRenderer()->clear();
			for (auto i = 0u; i < m_activeMesh; i++) {
				RenderErrors::checkError();

				if (m_meshes[i]->isActive()) {
					m_meshes[i]->draw();
				}

			}
			RenderErrors::checkError();
			frameBuffer.unbind();
			RenderErrors::checkError();
			ImGui::Begin("Window");
			{
				ImGui::BeginChild("Window renderer");
				ImVec2 childSize = ImGui::GetContentRegionAvail();

				frameBuffer.setWidth(childSize.x);
				frameBuffer.setHeight(childSize.y);


				uint32_t colorTexture = frameBuffer.getColorTexture();
				ImGui::Image(reinterpret_cast<void*>(static_cast<intptr_t>(colorTexture)), ImVec2{ static_cast<float>(frameBuffer.getWidth()),static_cast<float>(frameBuffer.getHeight()) }, ImVec2{ 0,1 }, ImVec2{ 1,0 });
				ImGui::EndChild();
			}
			ImGui::End();

			ImGui::Begin("Meshes");
			{

				for (auto i = 0u; i < m_meshes.size(); i++)
				{
					if (ImGui::Selectable(m_meshes[i]->m_name.c_str())) {

						lastClickedMesh = i;
						std::cout << "Clicked on: " << m_meshes[i]->m_name.c_str() << std::endl;
					}
				}
			}
			ImGui::End();

			// render GUI
			ImGui::Begin("Camera menu");
			{
				ImGui::BeginChild("menu");
				ImGui::SliderFloat("fov", &m_camera->getFovRef(), 0.0f, 90.0f);
				ImGui::SliderFloat("near", &m_camera->getNearRef(), 0.0f, 90.0f);
				ImGui::SliderFloat("far", &m_camera->getFarRef(), 0.0f, 120.0f);
				ImGui::SliderFloat("camera speed", &m_camera->getCameraSpeedRef(), 0.01f, 1.0f);
				ImGui::SliderFloat("scroll speed", &m_camera->getScrollSpeedRef(), 1.0f, 10.0f);
				ImGui::EndChild();
			}
			ImGui::End();

			ImGui::Begin("Frames info");
			ImGui::Text("Application average: \n%.3f ms/frame \n(%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
			ImGui::End();

			ImGui::Begin("Point Cloud");
			{
				ImGui::BeginChild("Point Cloud editor");
				static float* clearColor = m_meshes[0]->getRenderer()->getClearColor();
				ImGui::ColorEdit4("Background color", clearColor);
				ImGui::EndChild();
			}
			ImGui::End();

			ImGui::Begin("Mesh");
			{
				ImGui::BeginChild("Mesh editor");
				Vertex4<float> color = m_meshes[lastClickedMesh]->getColor();
				if (m_meshes[lastClickedMesh]->hasOriginalColors() == false)
				{
					ImGui::ColorEdit3("Color", color.data);
				}

				ImGui::SliderFloat("alpha", &color.a, 0.0f, 1.0f);
				m_meshes[lastClickedMesh]->setColor(color);
				ImGui::Checkbox("Visible", &m_meshes[lastClickedMesh]->active);


				if (m_meshes[lastClickedMesh]->isPoint()) {
					auto& pointSize = m_meshes[lastClickedMesh]->getPointSize();
					ImGui::SliderFloat("pointSize", &pointSize, 1.0f, 10.0f);
					glPointSize(pointSize);

					static bool useGridFilter = true;
					ImGui::Checkbox("UseGridFilter", &useGridFilter);

					static float gridSizeInPercent = 1.0;
					ImGui::SliderFloat("gridSizeInPercent", &gridSizeInPercent, 0.1f, 10.0f);

					static float neighbourRangeInPercent = 2.0;
					ImGui::SliderFloat("neighbourRangeInPercent", &neighbourRangeInPercent, 0.1f, 25.0f);

					if (ImGui::Button("Calculate Surface"))
					{
						taskManager.startSurfaceReconstruction(m_meshes[lastClickedMesh]->data, useGridFilter, gridSizeInPercent / 100.0, neighbourRangeInPercent / 100.0);
						taskManager.isSurfaceReconstructionResultReceived = false;
					}

					if (taskManager.isSurfaceReconstructionResultReceived == false) {
						if (taskManager.isJobDone()) {
							taskManager.isSurfaceReconstructionResultReceived = true;
							auto result = taskManager.getResults();
							if (result != nullptr && result->surface.size() > 0) {
								this->data.push_back(std::move(result));

								initCalculatedSurface(&this->data.back()->surface);
							}
						}
					}
				}

				if (m_meshes[lastClickedMesh]->isTriangle()) {
					static const char* items[] = { "Fill triangles", "Only lines", "Fill + lines" };
					auto& selectedItem = m_meshes[lastClickedMesh]->selectedMode;
					ImGui::Combo("Select Option", &selectedItem, items, IM_ARRAYSIZE(items));

					if (ImGui::Button("Save model as .obj"))
					{
						saveAsObj(*m_meshes[lastClickedMesh]->data, m_meshes[lastClickedMesh]->m_name);

					}

				}


				ImGui::EndChild();
			}
			ImGui::End();

		}

		RenderErrors::checkError();

		// render dear imgui into screen
		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
		{
			GLFWwindow* backup_current_context = glfwGetCurrentContext();
			ImGui::UpdatePlatformWindows();
			ImGui::RenderPlatformWindowsDefault();
			glfwMakeContextCurrent(backup_current_context);
		}

		// check and call events and swap the buffers
		glfwSwapBuffers(m_display->getWindow());
		glfwPollEvents();
	}

	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
}

void GraphicsApplication::saveAsObj(std::vector < Vertex4<float>>& points, const std::string& name)
{
	auto now = std::chrono::system_clock::now();
	auto now_timet_t = std::chrono::system_clock::to_time_t(now);

	std::stringstream ss;
	ss << std::put_time(std::localtime(&now_timet_t), "%d-%m-%Y-%X");
	std::string filename = name + "_" + ss.str();

	std::replace(filename.begin(), filename.end(), ':', '-');

	filename += ".obj";

	std::ofstream file(filename);

	if (!file.is_open()) {
		std::cerr << "Failed to open file: " << filename << std::endl;
		return;
	}

	for (auto& p : points) {
		file << "v " << p.x << " " << p.y << " " << p.z << "\n";
	}

	for (auto i = 0u; i < points.size(); i += 3) {
		file << "f " << (i + 1) << " " << (i + 2) << " " << (i + 3) << "\n";
	}
}
