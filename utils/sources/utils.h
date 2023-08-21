#pragma once 

namespace UTILS {


	// all classes should be stateless ( have only static helper functions )
	class Helper {
	public:



	};



}


template <class T>
class Vertex4 {
public:
	union {
		struct {
			T x, y, z, w;
		};
		struct {
			T r, g, b, a;
		};
		T data[4];
	};
	T& operator[](size_t index) {
		return data[index];
	}

	const T& operator[](size_t index) const {
		return data[index];
	}

	
};

