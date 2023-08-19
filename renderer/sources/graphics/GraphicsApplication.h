#pragma once

#include "../core/utils/Vertex.h"
#include "Display.h"
#include <vector>

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
    void init(std::vector<VertexType>& points);
    void mainLoop();
    std::vector<VertexType> m_points; // TODO it should be removed


private:
    GraphicsApplication() = default;
    std::vector<MeshBase*> m_meshes;
    Display* m_display = nullptr;
    Camera* m_camera = nullptr;
    void createMeshes(Camera* camera);

    uint32_t m_activeMesh = 0;


};

