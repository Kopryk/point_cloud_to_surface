#include "GraphicsApplication.h"

#include "FileDialog.h"
#include "mesh/MeshPoints.h"
#include "mesh/MeshLines.h"
#include "utils/RenderUtils.h"
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <iostream>

#include "Camera.h"
#include "FrameBuffer.h"
#include "Renderer.h"

GraphicsApplication::~GraphicsApplication()
{
	m_meshes.clear();
}

void GraphicsApplication::init(std::vector<Vertex4<float>>& points)
{
	this->points = points;
	m_display = &Display::get();
	m_display->init();
	m_camera = &Camera::get();
	m_camera->init(m_display->getWidth(), m_display->getHeight());

	m_activeMesh = 1;
	createMeshes(m_camera);
}


void GraphicsApplication::init(std::vector<Vertex4<float>>& points, std::vector<Vertex4<float>>& normals)
{
	this->points = points;
	this->normals = normals;

	m_display = &Display::get();
	m_display->init();
	m_camera = &Camera::get();
	m_camera->init(m_display->getWidth(), m_display->getHeight());

	m_activeMesh = 2;
	createMeshes2(m_camera);
}

void ShowDockSpace(bool* p_open)
{
	static bool opt_fullscreen_persistant = true;
	bool opt_fullscreen = opt_fullscreen_persistant;
	static ImGuiDockNodeFlags dockspace_flags = ImGuiDockNodeFlags_None;

	// We are using the ImGuiWindowFlags_NoDocking flag to make the parent window not dockable into,
	// because it would be confusing to have two docking targets within each others.
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

	// When using ImGuiDockNodeFlags_PassthruCentralNode, DockSpace() will render our background 
	// and handle the pass-thru hole, so we ask Begin() to not render a background.
	if (dockspace_flags & ImGuiDockNodeFlags_PassthruCentralNode)
		window_flags |= ImGuiWindowFlags_NoBackground;

	// Important: note that we proceed even if Begin() returns false (aka window is collapsed).
	// This is because we want to keep our DockSpace() active. If a DockSpace() is inactive,
	// all active windows docked into it will lose their parent and become undocked.
	// We cannot preserve the docking relationship between an active window and an inactive docking, otherwise
	// any change of dockspace/settings would lead to windows being stuck in limbo and never being visible.
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
			// TODO OPEN FILEDIALOG
			/*if (auto result = FileDialog::openFileDialog())
			{
				render = !render;
			}*/
		}
		ImGui::End();

		// update camera
		m_camera->update();


		if (!m_meshes.empty() && render)
		{
			RenderErrors::checkError();
			frameBuffer.bind();

			// TODO it should check if mesh is active instead of simple looping over all meshes
			// now only last meshes can be disabled
			m_meshes[0]->getRenderer()->clear();
			for (auto i = 0u; i < m_activeMesh; i++) {
				RenderErrors::checkError();

				m_meshes[i]->draw();


			}
			RenderErrors::checkError();
			frameBuffer.unbind();
			RenderErrors::checkError();
			ImGui::Begin("Window");
			{
				ImGui::BeginChild("Window renderer");
				uint32_t colorTexture = frameBuffer.getColorTexture();
				ImGui::Image(reinterpret_cast<void*>(static_cast<intptr_t>(colorTexture)), ImVec2{ static_cast<float>(frameBuffer.getWidth()),static_cast<float>(frameBuffer.getHeight()) }, ImVec2{ 0,1 }, ImVec2{ 1,0 });
				ImGui::EndChild();
			}
			ImGui::End();

			//// render your GUI
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
				static Vertex4<float> color = m_meshes[0]->getColor();
				ImGui::ColorEdit3("Color", color.data);
				m_meshes[0]->setColor(color);
				ImGui::SliderFloat("alpha", &color.a, 0.0f, 1.0f);
				static float pointSize = 1.0f;
				ImGui::SliderFloat("pointSize", &pointSize, 1.0f, 10.0f);
				glPointSize(pointSize);
				glLineWidth(pointSize);
				ImGui::EndChild();
			}
			ImGui::End();
		}

		RenderErrors::checkError();



		// Render dear imgui into screen
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

void GraphicsApplication::createMeshes(Camera* camera)
{
	// points cloud
	m_meshes.emplace_back(new MeshPoints(points, "points cloud"));

	for (auto& mesh : m_meshes)
	{
		mesh->init(camera);
	}
}

void GraphicsApplication::createMeshes2(Camera* camera)
{
	// points cloud
	m_meshes.emplace_back(new MeshPoints(points, "points cloud"));
	m_meshes.emplace_back(new MeshLines(points, normals, "points cloud"));


	for (auto& mesh : m_meshes)
	{
		mesh->init(camera);
	}
}

