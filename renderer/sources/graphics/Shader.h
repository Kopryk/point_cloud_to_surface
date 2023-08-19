#pragma once

// clang-format off
#include <gl/glew.h>
#include <GLFW/glfw3.h>
// clang-format on

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <map>
#include <string_view>

class Shader
{
public:
    Shader(std::string_view vertexShaderPath, std::string_view fragmentShaderPath);
    ~Shader();
    Shader(const Shader& shader) = delete;
    Shader(Shader && shader) = delete;
    Shader& operator=(const Shader& shader) = delete;
    Shader& operator=(Shader&& shader) = delete;

    GLint getVertexShaderId();
    GLint getFragmentShaderId();

    void bind() const;
    void unbind() const;

    template<typename T>
    void setUniform(const std::string_view name, T data);

private:
    GLint m_vertexShaderId;
    GLint m_fragmentShaderId;
    GLint compileShader(std::string_view shaderSource, GLenum shaderType);
    std::string getSourceShader(std::string_view shaderPath);
    char m_errorLog[512];
    GLuint m_program;
    uint32_t getUniformLocation(std::string_view name);

    std::map<std::string_view, uint32_t> m_uniformLocationsCache;
};

template <typename T>
void Shader::setUniform(const std::string_view name, T data)
{
    static_assert(false);
}

template<>
inline void Shader::setUniform<glm::vec4>(const std::string_view name, glm::vec4 data)
{

    auto location = getUniformLocation(name);
    glUniform4fv(location, 1, glm::value_ptr(data));
}

template<>
inline void Shader::setUniform<glm::mat4>(const std::string_view name, glm::mat4 data)
{
    auto location = getUniformLocation(name);
    glUniformMatrix4fv(location, 1, GL_FALSE, glm::value_ptr(data));
}
