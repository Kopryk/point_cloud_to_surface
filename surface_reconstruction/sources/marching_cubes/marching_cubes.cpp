#include "marching_cubes.h"
#include <limits>
#include <algorithm>
#include <ctime>
#include <execution>
#include <random>

void SR::MarchingCubes::test()
{
	this->points_ = new std::vector<Position<PositionType>>;

	constexpr size_t size = 1ull *  1024;

	points_->reserve(size);

	std::random_device random_device;
	std::mt19937 engine{ random_device() };
	std::uniform_real_distribution<PositionType> distribution(0.0f, 10000.0f);


	for (auto i = 0ull; i < size; i++) {
		Position < PositionType> position;
		position.x = distribution(engine);
		position.y = distribution(engine);
		position.z = distribution(engine);

		points_->push_back(position);
	}

	auto boundingBox = findBoundingBox();

	return;
}

void SR::MarchingCubes::setPoints(std::vector<UTILS::Position<UTILS::PositionType>>* points)
{
	points_ = points;
}

// iterate over all points
// and find max and min for x,y,z values
SR::BoundingBox SR::MarchingCubes::findBoundingBox()
{
	BoundingBox boundingBox{};

	using PositionFloat = Position<float>;
	using PositionDouble = Position<double>;

	if constexpr (!(std::is_same_v< PositionType, float> || std::is_same_v< PositionType, double>)) {
		// wrong type, we expect only float or double
		abort();
	}

	// find min max

	auto [positionMinX, positionMaxX] = std::minmax_element(std::execution::par, points_->cbegin(), points_->cend(), [](const Position<PositionType>& a, const Position<PositionType>& b)
		{
			if (b.x > a.x) {
				return true;
			}
			return false;
		});


	auto [positionMinY, positionMaxY] = std::minmax_element(std::execution::par, points_->cbegin(), points_->cend(), [](const Position<PositionType>& a, const Position<PositionType>& b)
		{
			if (b.y > a.y) {
				return true;
			}
			return false;
		});


	auto [positionMinZ, positionMaxZ] = std::minmax_element(std::execution::par, points_->cbegin(), points_->cend(), [](const Position<PositionType>& a, const Position<PositionType>& b)
		{
			if (b.z > a.z) {
				return true;
			}
			return false;
		});

	// initalize bounding box
	boundingBox.max.x = positionMaxX->x;
	boundingBox.max.y = positionMaxY->y;
	boundingBox.max.z = positionMaxZ->z;
	boundingBox.min.x = positionMinX->x;
	boundingBox.min.y = positionMinY->y;
	boundingBox.min.z = positionMinZ->z;

	return boundingBox;
}
