#include "Shader.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

Shader::Shader(std::string_view vertexShaderPath, std::string_view fragmentShaderPath)
{
	m_program = glCreateProgram();

	auto vertexShaderSource = getSourceShader(vertexShaderPath);
	auto fragmentShaderSource = getSourceShader(fragmentShaderPath);

	m_vertexShaderId = compileShader(vertexShaderSource, GL_VERTEX_SHADER);
	m_fragmentShaderId = compileShader(fragmentShaderSource, GL_FRAGMENT_SHADER);

	glAttachShader(m_program, m_vertexShaderId);
	glAttachShader(m_program, m_fragmentShaderId);
	glLinkProgram(m_program);
	glValidateProgram(m_program);
	glDeleteShader(m_vertexShaderId);
	glDeleteShader(m_fragmentShaderId);
}

Shader::Shader(std::string_view vertexShaderPath, std::string_view fragmentShaderPath, std::string_view geometryShaderPath)
{
	m_program = glCreateProgram();

	auto vertexShaderSource = getSourceShader(vertexShaderPath);
	auto fragmentShaderSource = getSourceShader(fragmentShaderPath);
	auto geometryShaderSource = getSourceShader(geometryShaderPath);

	m_vertexShaderId = compileShader(vertexShaderSource, GL_VERTEX_SHADER);
	m_fragmentShaderId = compileShader(fragmentShaderSource, GL_FRAGMENT_SHADER);
	m_geometryShaderId = compileShader(geometryShaderSource, GL_GEOMETRY_SHADER);

	glAttachShader(m_program, m_vertexShaderId);
	glAttachShader(m_program, m_fragmentShaderId);
	glAttachShader(m_program, m_geometryShaderId);

	glLinkProgram(m_program);
	glValidateProgram(m_program);
	glDeleteShader(m_vertexShaderId);
	glDeleteShader(m_fragmentShaderId);
	glDeleteShader(m_geometryShaderId);

}

Shader::~Shader()
{
	glDeleteProgram(m_program);
}

GLint Shader::getVertexShaderId()
{
	return m_vertexShaderId;
}

GLint Shader::getFragmentShaderId()
{
	return m_fragmentShaderId;
}

void Shader::bind() const
{
	glUseProgram(m_program);
}

void Shader::unbind() const
{
	glUseProgram(0);
}

GLint Shader::compileShader(std::string_view shaderSource, GLenum  shaderType)
{
	auto id = glCreateShader(shaderType);
	auto src = shaderSource.data();
	glShaderSource(id, 1, &src, nullptr);
	glCompileShader(id);

	GLint result;
	if (glGetShaderiv(id, GL_COMPILE_STATUS, &result); result == GL_FALSE) {
		std::cout << "Compile shader failed";
		glGetShaderInfoLog(id, 512, nullptr, m_errorLog);
		std::cout << m_errorLog << "\n";
	}
	return id;
}

std::string Shader::getSourceShader(std::string_view shaderPath)
{
	std::ifstream shaderSourceFile(shaderPath.data());
	std::string line = "";
	std::stringstream  shaderSource;

	while (std::getline(shaderSourceFile, line))
	{
		shaderSource << line << "\n";
	}
	return shaderSource.str();
}

uint32_t Shader::getUniformLocation(std::string_view name)
{
	// check if cached location
	if (m_uniformLocationsCache.find(name) != m_uniformLocationsCache.end())
	{
		return m_uniformLocationsCache.at(name);
	}

	auto location = glGetUniformLocation(m_program, name.data());

	if (location == -1)
	{
		_ASSERT(false);
	}
	// update cache
	m_uniformLocationsCache[name] = location;
	return location;
}
