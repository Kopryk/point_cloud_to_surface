#pragma once


#include "../vertex/VertexBuffer.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>


class Camera;
class Shader;
class VertexBufferLayout;
class VertexArray;
class Renderer;

class MeshBase
{
public:
    explicit  MeshBase(std::string_view name = "base");
    virtual  ~MeshBase();
    MeshBase(const MeshBase&) = delete;
    MeshBase(MeshBase&&) = delete;
    MeshBase& operator=(const MeshBase&) = delete;
    MeshBase& operator=(MeshBase&&) = delete;

    virtual void draw() = 0;
    virtual void init(Camera* camera);
    virtual void updateModel(float modelPosition[]);

    Renderer* getRenderer() { return m_renderer; }

    void setColor(Vertex4<float> color);
    Vertex4<float> getColor();


protected:

    Camera* m_camera;

    std::string m_name;
    Shader* m_shader = nullptr; // TODO
    Renderer* m_renderer = nullptr;   // TODO use unique_ptr 
    glm::mat4 m_model = glm::mat4(1.0f);

    glm::vec4 m_color = glm::vec4(1.0f);
    glm::mat4 m_mvp = glm::mat4(1.0f);// TODO link all matrices to just one
    float m_modelPosition[3] = { 0.0f,0.0f,0.0f };

    VertexArray* m_vertexArray = nullptr;
    VertexBuffer<VertexType>* m_vertexBuffer = nullptr;
    VertexBufferLayout* m_vertexBufferLayout = nullptr;
    //VertexIndices* m_vertexIndices;

    virtual void updateUniforms() = 0;
    virtual void initializeShader() = 0;
    virtual void initMvp() = 0;
    virtual void initVertexBuffer() = 0;
    void printDebugInfo() const;

};
