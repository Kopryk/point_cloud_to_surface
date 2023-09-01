#include "octree.h"

void Octree::insert(Point& point, uint32_t depth)
{
	if (isPointInsideBoundingBox(point) == false) {
		return;
	}

	if (depth < maxDepth && points.size() >= maxPoints)
	{
		if (children[0] == nullptr)
		{
			subdivide();
		}

		for (Point& existingPoint : points) {	// if there is too much points, move all points to leafs
			for (auto i = 0u; i < maxChildren; i++) {
				if (children[i]->isPointInsideBoundingBox(existingPoint)) {
					children[i]->insert(existingPoint, depth + 1);
					break;
				}
			}
		}
		points.clear();

		for (auto i = 0u; i < maxChildren; i++) {
			children[i]->insert(point, depth + 1);	// it will actually add a point only to one chilldren 
		}
	}
	else {
		points.push_back(point);
	}


}

bool Octree::isPointInsideBoundingBox(Point& point)
{

	return (point.x >= boundingBox.min.x && point.x <= boundingBox.max.x) &&
		(point.y >= boundingBox.min.y && point.y <= boundingBox.max.y) &&
		(point.z >= boundingBox.min.z && point.z <= boundingBox.max.z);

}

void Octree::subdivide()
{

	for (int i = 0; i < maxChildren; i++) {

		auto xMax = boundingBox.max.x;
		auto xMin = boundingBox.min.x;
		auto yMax = boundingBox.max.y;
		auto yMin = boundingBox.min.y;
		auto zMax = boundingBox.max.z;
		auto zMin = boundingBox.min.z;

		BoundingBox newBoundingBox{};


		bool is1BitSet = (i & 1);
		bool is2BitSet = (i & 2);
		bool is4BitSet = (i & 4);

		newBoundingBox.max.x = is1BitSet ? (xMin + xMax) / 2 : xMax;
		newBoundingBox.min.x = is1BitSet ? (xMin + xMax) / 2 : xMin;

		newBoundingBox.max.y = is2BitSet ? (yMin + yMax) / 2 : yMax;
		newBoundingBox.min.y = is2BitSet ? (yMin + yMax) / 2 : yMin;

		newBoundingBox.max.z = is4BitSet ? (zMin + zMax) / 2 : zMax;
		newBoundingBox.min.z = is4BitSet ? (zMin + zMax) / 2 : zMin;

		children[i] = std::make_unique<Octree>(newBoundingBox);
	}

}
