#include "Octree.h"
#include <iostream>


void OctreeNode::insert(Point& point, uint32_t depth)
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

bool OctreeNode::isPointInsideBoundingBox(Point& point)
{

	return (point.x >= boundingBox.min.x && point.x <= boundingBox.max.x) &&
		(point.y >= boundingBox.min.y && point.y <= boundingBox.max.y) &&
		(point.z >= boundingBox.min.z && point.z <= boundingBox.max.z);

}

std::vector<Point> OctreeNode::findNeigborsInRadius(Point p) {

	float radius = 0.01f;

	// check neighbors size if in range (minNeighbors, maxNeighbors)
	// if to big decrese radius
	// if to small increase radius

	auto maxIterations = 1'000'000u;
	while (maxIterations--) {
		auto neighbors = queryRadius(p, radius);
		if (neighbors.size() >= this->minNeighbors && neighbors.size() <= this->maxNeighbors) {
			return neighbors;
		}

		if (neighbors.size() > this->maxNeighbors) {
			radius = radius * 0.9;
		}

		if (neighbors.size() < this->minNeighbors) {
			radius = radius * 2;
		}
	}

	if (maxIterations == 0) {
		std::cout << __LINE__ << "findNeigborsInRadius ERROR: MAX_ITERATIONS!" << std::endl;
	}


	return {};
}

inline Eigen::Vector3f OctreeNode::computeNormalForPoint(OctreeNode& octreeNode, Point p) {
	auto neighbors = octreeNode.findNeigborsInRadius(p);

	Eigen::Vector3f centroid(0, 0, 0);
	for (auto& neighbor : neighbors) {
		centroid += Eigen::Vector3f(neighbor.x, neighbor.y, neighbor.z);
	}
	centroid /= neighbors.size();

	auto covariance = Eigen::Matrix3f::Zero();
	for (auto& neighbor : neighbors) {
		Eigen::Vector3f neighbor3f(neighbor.x, neighbor.y, neighbor.z);
		Eigen::Vector3f diff = neighbor3f - centroid;
		covariance += diff * diff.transpose();
	}
	covariance /= neighbors.size();

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solve(covariance);

	// Principal Component Analysis(PCA)
	// eigenvector of covariance matrix corresponding to its smallest eigenvalue is estimated normal vector.

	Eigen::Vector3f normal = solve.eigenvectors().col(0);

	return normal;
}

inline void OctreeNode::computeNormalForAllPoints(OctreeNode& octreeNode) {
	if (octreeNode.isLeaf()) {
		for (auto& point : octreeNode.points) {
			// calculate normals and set to point
			Eigen::Vector3f normal = computeNormalForPoint(octreeNode, point);
			point.nx = normal.x();
			point.ny = normal.y();
			point.nz = normal.z();
		}
	}
	else {
		for (int i = 0; i < maxChildren; i++) {
			if (octreeNode.children[i]) {
				computeNormalForAllPoints(*octreeNode.children[i]);

			}
		}
	}

}

inline bool OctreeNode::isLeaf() {
	for (auto i = 0u; i < maxChildren; i++) {
		if (children[i]) {
			return false;
		}
	}
	return true;
}

std::vector<Point> OctreeNode::queryRadius(Point p, float radius)
{

	std::vector<Point> neighbourhoodPoints;
	// check if the sphere intersects current octree node's bounding box
	if (!intersectsSphere(p, radius)) {
		return neighbourhoodPoints;	// return empty
	}

	for (const auto& point : points) {
		if (distance(point, p) <= radius) {
			neighbourhoodPoints.push_back(point);
		}
	}
	for (int i = 0; i < maxChildren; i++) {
		if (children[i]) {
			auto childPoints = children[i]->queryRadius(p, radius);
			neighbourhoodPoints.insert(std::end(neighbourhoodPoints), std::begin(childPoints), std::end(childPoints));
		}
	}

	return neighbourhoodPoints;

}

float OctreeNode::distance(const Point& a, const Point& b)
{

	float dx = a.x - b.x;
	float dy = a.y - b.y;
	float dz = a.z - b.z;
	return std::sqrt(dx * dx + dy * dy + dz * dz);


}

bool OctreeNode::intersectsSphere(const Point& sphereCenter, float radius)
{
	{

		float squaredDistance = 0.0f;

		if (sphereCenter.x < boundingBox.min.x) {
			float dx = sphereCenter.x - boundingBox.min.x;
			squaredDistance += dx * dx;
		}
		else if (sphereCenter.x > boundingBox.max.x) {
			float dx = sphereCenter.x - boundingBox.max.x;
			squaredDistance += dx * dx;
		}

		if (sphereCenter.y < boundingBox.min.y) {
			float dy = sphereCenter.y - boundingBox.min.y;
			squaredDistance += dy * dy;
		}
		else if (sphereCenter.y > boundingBox.max.y) {
			float dy = sphereCenter.y - boundingBox.max.y;
			squaredDistance += dy * dy;
		}

		if (sphereCenter.z < boundingBox.min.z) {
			float dz = sphereCenter.z - boundingBox.min.z;
			squaredDistance += dz * dz;
		}
		else if (sphereCenter.z > boundingBox.max.z) {
			float dz = sphereCenter.z - boundingBox.max.z;
			squaredDistance += dz * dz;
		}

		return squaredDistance <= (radius * radius);
	}
}

void OctreeNode::subdivide()
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

		children[i] = std::make_unique<OctreeNode>(newBoundingBox);
	}

}
