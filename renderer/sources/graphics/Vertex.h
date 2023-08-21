#pragma once

class Shader;
class VertexArray;

template<class T>
class Vertex4 {

	union {
		{T x, y, z, w;
		T tab[4];
		}
	};
};

