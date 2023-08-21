#pragma once
#include "VertexElementGL.h"

#include "../../utils/sources/utils.h"
#include <vector>

struct VertexElementGL;

class VertexBufferLayout
{
public:
	VertexBufferLayout() = default;
	~VertexBufferLayout() = default;
	VertexBufferLayout(const VertexBufferLayout&) = delete;
	VertexBufferLayout(VertexBufferLayout&&) = delete;
	VertexBufferLayout& operator=(const VertexBufferLayout&) = delete;
	VertexBufferLayout& operator=(VertexBufferLayout&&) = delete;

	template<typename T>
	void addElement(int count);

	template<typename T>
	void addElement();

	[[nodiscard]] const std::vector<VertexElementGL>& getElements() const;

private:
	std::vector<VertexElementGL> m_elements;
	size_t m_offset = 0;

};

template <typename T>
void VertexBufferLayout::addElement(int count)
{
	//static_assert(false); // TODO
}

template <typename T>
void VertexBufferLayout::addElement()
{
	int count = 0;
	int sizeInBytes = 0;
	const auto index = static_cast<GLuint>(m_elements.size());

	int stride = 0;

	/*if constexpr (std::is_same<T, Vertex6<float>>::value)
	{
		stride = 6 * sizeof(float);
		count = 3;
		sizeInBytes = 6 * sizeof(float);


	}*/
	if constexpr (std::is_same<T, Vertex4<float>>::value)
	{
		// IT MIGHT BE WRONG
		stride = 4 * sizeof(float);
		count = 3;
		sizeInBytes = 4 * sizeof(float);

	}

	m_elements.emplace_back(VertexElementGL{ index, count, GL_FLOAT, true, stride, m_offset,static_cast<size_t>(sizeInBytes) });
	m_offset += count * sizeof(float);
}

