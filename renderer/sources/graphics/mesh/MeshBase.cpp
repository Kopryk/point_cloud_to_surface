#include "MeshBase.h"

#include "../Renderer.h"
#include "../Shader.h"
#include "../vertex/VertexArray.h"

#ifdef _DEBUG
#include <iostream>
#endif

MeshBase::MeshBase(std::string_view name) : m_renderer(new Renderer()), m_name(name), m_camera(nullptr)
{

}

MeshBase::~MeshBase()
{
    delete m_vertexArray;
    delete m_vertexBuffer;
    delete m_vertexBufferLayout;
    delete m_renderer;
    delete m_shader;
}

void MeshBase::init(Camera *camera)
{
    m_camera = camera;

    initializeShader();
    initMvp();
    initVertexBuffer();
}

void MeshBase::updateModel(float modelPosition[])
{
    for(uint32_t i=0; i<3; ++i)
    {
        m_modelPosition[i] = modelPosition[i];
    }
}

void MeshBase::setColor(Vertex4<float> color)
{
    m_color.r = color.r;
    m_color.g = color.g;
    m_color.b = color.b;
    m_color.a = color.a;
}

Vertex4<float> MeshBase::getColor()
{
    Vertex4<float> color{ m_color.r, m_color.g, m_color.b, m_color.a };
    return color;
}


void MeshBase::printDebugInfo() const
{
    //#ifdef  _DEBUG
    //    std::cout << "MeshBase :" << m_name << " rendered\n";
    //#endif

}
