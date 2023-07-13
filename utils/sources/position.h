#pragma once 
#include <type_traits>

namespace UTILS {

	using PositionType = float;

	template<typename T>
	struct Position {
		T x;
		T y;
		T z;
	};



}

