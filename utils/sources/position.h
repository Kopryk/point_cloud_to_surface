#pragma once 
#include <type_traits>
#include <memory>
#include <vector>

namespace UTILS {

	using PositionType = float;

	template<typename T>
	struct Position {
		T x;
		T y;
		T z;
	};

}

