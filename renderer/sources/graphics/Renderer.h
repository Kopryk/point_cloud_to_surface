#pragma once

class Shader;
class VertexArray;

class Renderer
{
public:
    Renderer();
    ~Renderer() = default;
    Renderer(const Renderer&) = delete;
    Renderer(Renderer&&) = delete;
    Renderer& operator=(const Renderer&) = delete;
    Renderer& operator=(Renderer&&) = delete;

    void drawTriangles(VertexArray& vertexArray, Shader& shader) const;
    void drawPoints(VertexArray& vertexArray, Shader& shader) const;
    void drawLines(VertexArray& vertexArray, Shader& shader) const;

    void clear() const;

    void setClearColor(float clearColor[4]);
    float* getClearColor() { return m_clearColor; }

private:

    float m_clearColor[4];
};

