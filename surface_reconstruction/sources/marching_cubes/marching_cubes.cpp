#include "marching_cubes.h"
#include <limits>
#include <algorithm>
#include <ctime>

void SR::MarchingCubes::test()
{
	this->points_ = new std::vector<Position<PositionType>>;

	srand(time(NULL));

	for (auto i = 0u; i < 100; i++) {
		Position < PositionType> position;
		position.x = static_cast<float>(rand() % RAND_MAX);
		position.y = static_cast<float>(rand() % RAND_MAX);
		position.z = static_cast<float>(rand() % RAND_MAX);

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
// and find max, min x,y,z values
// which is bounding box
SR::BoundingBox SR::MarchingCubes::findBoundingBox()
{
	BoundingBox boundingBox{};

	using PositionFloat = Position<float>;
	using PositionDouble = Position<double>;

	if constexpr (!(std::is_same_v< PositionType, float> || std::is_same_v< PositionType, double>)) {
		// wrong type, we expect only float or double
		abort();
	}

	// initalize bounding box
	boundingBox.max.x = std::numeric_limits<PositionType>::min();
	boundingBox.max.y = std::numeric_limits<PositionType>::min();
	boundingBox.max.z = std::numeric_limits<PositionType>::min();
	boundingBox.min.x = std::numeric_limits<PositionType>::max();
	boundingBox.min.y = std::numeric_limits<PositionType>::max();
	boundingBox.min.z = std::numeric_limits<PositionType>::max();


	// find min max

	boundingBox.max.x = std::max_element(points_->begin(), points_->end(), [](Position<PositionType>& a, Position<PositionType>& b)
		{
			if (b.x > a.x) {
				return true;
			}
			return false;
		})->x;

	boundingBox.max.y = std::max_element(points_->begin(), points_->end(), [](Position<PositionType>& a, Position<PositionType>& b)
		{
			if (b.y > a.y) {
				return true;
			}
			return false;
		})->y;

	boundingBox.max.z = std::max_element(points_->begin(), points_->end(), [](Position<PositionType>& a, Position<PositionType>& b)
		{
			if (b.z > a.z) {
				return true;
			}
			return false;
		})->z;

	boundingBox.min.x = std::min_element(points_->begin(), points_->end(), [](Position<PositionType>& a, Position<PositionType>& b)
		{
			if (b.x > a.x) {
				return true;
			}
			return false;
		})->x;

	boundingBox.min.y = std::min_element(points_->begin(), points_->end(), [](Position<PositionType>& a, Position<PositionType>& b)
		{
			if (b.y > a.y) {
				return true;
			}
			return false;
		})->y;

	boundingBox.min.z = std::min_element(points_->begin(), points_->end(), [](Position<PositionType>& a, Position<PositionType>& b)
		{
			if (b.z > a.z) {
				return true;
			}
			return false;
		})->z;



	return boundingBox;
}
