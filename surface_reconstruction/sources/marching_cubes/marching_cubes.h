#pragma once
#include <vector>
#include "../../utils/sources/position.h"

namespace SR {
	using namespace UTILS;

	struct BoundingBox {

		Position<PositionType> min;
		Position<PositionType> max;
	};

	class MarchingCubes {
	public:
		MarchingCubes() = default;

		void test();
		void setPoints(std::vector < Position<PositionType>>* points);
		BoundingBox findBoundingBox();
		

	private:
		std::vector < Position<PositionType>>* points_ = { nullptr };

	};
}