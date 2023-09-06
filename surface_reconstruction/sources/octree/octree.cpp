#include "octree.h"
#include <iostream>
#include "index_octree.h"
#include <chrono>
#include <thread>
#include "corner.h"
#include "corner_scalar_cache.h"
#include "../marching_cubes_from_octree/triangle.h"
#include "../marching_cubes_from_octree/marching_cubes_from_octree.h"

void OctreeNode::insert(Point& point, uint32_t depth)
{

	if (isPointInsideBoundingBox(point) == false) {
		return;
	}

	// if  node has reached  maximum points and hasn't reached maximum depth, split it
	if (depth < maxDepth && points.size() >= maxPoints)
	{

		// if leaf split it
		if (this->isLeaf()) {
			subdivide();
		}

		// move existing points to children
		for (Point& existingPoint : points) {
			for (auto i = 0u; i < maxChildren; i++) {
				if (children[i]->isPointInsideBoundingBox(existingPoint)) {
					children[i]->insert(existingPoint, depth + 1);
					break;
				}
			}
		}
		points.clear();  // remove points after moving them to children
	}

	if (this->isLeaf()) {
		points.push_back(point);
	}
	else {
		for (auto i = 0u; i < maxChildren; i++) {
			if (children[i]->isPointInsideBoundingBox(point)) {
				children[i]->insert(point, depth + 1);
				return;
			}
		}

	}


}

bool OctreeNode::isPointInsideBoundingBox(Point& point)
{

	return (point.x >= boundingBox.min.x && point.x <= boundingBox.max.x) &&
		(point.y >= boundingBox.min.y && point.y <= boundingBox.max.y) &&
		(point.z >= boundingBox.min.z && point.z <= boundingBox.max.z);

}

std::vector<Point> OctreeNode::findNeigborsInRadius(Point p) {
	using namespace std::chrono_literals;

	//radius = 0.01f;

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
			radius = radius * 2.0;
		}
	}

	if (maxIterations == 0) {
		std::cout << __LINE__ << "findNeigborsInRadius ERROR: MAX_ITERATIONS!" << std::endl;
		std::this_thread::sleep_for(2000ms);
	}


	return {};
}

Eigen::Vector3f OctreeNode::computeNormalForPoint(OctreeNode& octreeNode, Point p) {
	auto neighbors = octreeNode.findNeigborsInRadius(p);

	Eigen::Vector3f centroid(0, 0, 0);
	for (auto& neighbor : neighbors) {
		centroid += Eigen::Vector3f(neighbor.x, neighbor.y, neighbor.z);
	}
	centroid /= neighbors.size();

	Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
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

void OctreeNode::computeNormalForAllPoints(OctreeNode& octreeNode, OctreeNode* root) {
	if (octreeNode.isLeaf()) {
		for (auto& point : octreeNode.points) {
			// calculate normals and set to point
			Eigen::Vector3f normal = computeNormalForPoint(*root, point);
			point.nx = normal.x();
			point.ny = normal.y();
			point.nz = normal.z();
		}
	}
	else {
		for (int i = 0; i < maxChildren; i++) {
			if (octreeNode.children[i]) {
				computeNormalForAllPoints(*octreeNode.children[i], root);

			}
		}
	}
}

uint32_t OctreeNode::getNumberOfNodesInOctree()
{
	if (nAllPoints == 0) {
		std::cout << __LINE__ << "ERROR: nAllPoints =0 \n";
		using namespace std::chrono_literals;
		std::this_thread::sleep_for(2000ms);
	}
	return nAllPoints;
}

std::vector<Triangle> OctreeNode::solvePoissonProblem(OctreeNode* root) {


	computeNormalForAllPoints(*root, root);
	IndexOctreeHelper indexer{};
	indexer.indexOctree(root);

	Eigen::SparseMatrix<double> A;
	Eigen::VectorXd b, x;
	setupPoissonSystem(root, A, b, indexer);

	Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower | Eigen::Upper> solver;
	solver.setMaxIterations(1000);
	solver.setTolerance(0.1); // fixme

	solver.compute(A);

	if (solver.info() != Eigen::Success) {
		std::cout << __LINE__ << "compute failed\n";
		return {};
	}


	x = solver.solve(b);

	std::cout << "Number of iterations : " << solver.iterations() << std::endl;
	std::cout << "Estimated error: " << solver.error() << std::endl;


	if (solver.info() != Eigen::Success) {
		std::cout << __LINE__ << "solve failed error: " << solver.info() << std::endl;
		return {};
	 }

	auto allNodes = indexer.getAllNodesMap();

	for (auto& [node, index] : allNodes) {
		node->scalarValue = x(index);
		node->containsScalarValue = true;
		std::cout << "index= " << index << "scalar value = " << node->scalarValue << "\n";
	}

	// optional
	// check if helps
	// root->averageMissingLeafScalarValues(root);
	// better option
	int numberOfLeavesWithScalar = countLeafsWithScalar(root);
	std::cout << "numberOfLeavesWithScalar = " << numberOfLeavesWithScalar << std::endl;
	propagateScalarValues(root);
	std::cout << "numberOfLeavesWithScalar = " << numberOfLeavesWithScalar << std::endl;

	numberOfLeavesWithScalar = countLeafsWithScalar(root);

	// calculate cache - not needed here but w/e
	// remove later

	CornerScalarCache cornerScalarCache{};


	for (auto& [node, index] : allNodes) {

		std::cout << "node = " << node << std::endl;
		std::cout << "node scalar value = " << node->scalarValue << std::endl;

		for (int i = static_cast<int>(Corner::BOTTOM_FRONT_LEFT);
			i <= static_cast<int>(Corner::TOP_BACK_RIGHT);
			i++) {

			Corner currentCorner = static_cast<Corner>(i);

			auto scalarValue = cornerScalarCache.getScalarValueForCorner(node, currentCorner);
			std::cout << "corner i=" << i << " scalarValue = " << scalarValue << std::endl;
		}
	}

	auto nSignChanges = countNodesWithSignChange(root, cornerScalarCache);
	std::cout << nSignChanges << std::endl;

	MarchingCubesFromOctree marchingCubesFromOctree{};


	std::vector<Triangle> result{};


	for (auto& [node, index] :  allNodes) {
		if (node->isLeaf()) {
			std::vector<Triangle> triangles = marchingCubesFromOctree.extractTrianglesForNode(node, cornerScalarCache);
			for (const auto& tri : triangles) {

				// todo fix me
				result.push_back(tri);

				std::cout << "Triangle:\n";

				for (int i = 0; i < 3; i++) {
					Vector3 vertex = tri.getVertex(i);
					std::cout << "  Vertex " << i + 1 << ": ("
						<< vertex.x << ", "
						<< vertex.y << ", "
						<< vertex.z << ")\n";
				}

				std::cout << "--------------------------------------\n";
			}
		}
	}
	std::cout << "SolvePoissonProblem passed!\n";

	return result;
}


Eigen::VectorXd OctreeNode::computeDivergenceForAllPoints(OctreeNode* rootNode) {
	std::vector<Point> allPoints = getAllPoints();

	auto n = allPoints.size();
	Eigen::VectorXd divergenceValues(n);

	for (auto i = 0u; i < n; i++) {
		divergenceValues(i) = computeDivergence(rootNode, allPoints[i]);
	}

	return divergenceValues;
}

std::vector<Point> OctreeNode::getAllPoints() {
	std::vector<Point> allPoints;

	if (isLeaf()) {
		return points;
	}
	else {
		for (auto i = 0u; i < maxChildren; i++) {
			if (children[i]) {
				std::vector<Point> childPoints = children[i]->getAllPoints();
				allPoints.insert(allPoints.end(), childPoints.cbegin(), childPoints.cend());
			}
		}
	}

	// assumption
	// calculate all points only once
	nAllPoints = static_cast<uint32_t>(allPoints.size());
	return allPoints;
}

bool OctreeNode::isLeaf() {
	for (auto i = 0u; i < maxChildren; i++) {
		if (children[i] != nullptr) {
			return false;
		}
	}
	return true;
}

std::vector<Point> OctreeNode::queryRadius(Point p, double radius)
{

	std::vector<Point> neighbourhoodPoints;
	// check if the sphere intersects current octree node's bounding box
	if (!intersectsSphere(p, radius)) {
		return neighbourhoodPoints;	// return empty
	}

	constexpr double epsilon = 1e-5;

	for (const auto& point : points) {
		double dist = distance(point, p);
		if (dist <= radius && dist > epsilon) {
			neighbourhoodPoints.push_back(point);
		}
	}

	for (auto i = 0u; i < maxChildren; i++) {
		if (children[i]) {
			auto childPoints = children[i]->queryRadius(p, radius);
			neighbourhoodPoints.insert(std::end(neighbourhoodPoints), std::begin(childPoints), std::end(childPoints));
		}
	}

	return neighbourhoodPoints;

}

double OctreeNode::distance(const Point& a, const Point& b)
{
	double dx = a.x - b.x;
	double dy = a.y - b.y;
	double dz = a.z - b.z;
	return std::sqrt(dx * dx + dy * dy + dz * dz);

}

bool OctreeNode::intersectsSphere(const Point& sphereCenter, double radius)
{
	auto squaredDistance = 0.0;

	if (sphereCenter.x < boundingBox.min.x) {
		double dx = sphereCenter.x - boundingBox.min.x;
		squaredDistance += dx * dx;
	}
	else if (sphereCenter.x > boundingBox.max.x) {
		double dx = sphereCenter.x - boundingBox.max.x;
		squaredDistance += dx * dx;
	}

	if (sphereCenter.y < boundingBox.min.y) {
		double dy = sphereCenter.y - boundingBox.min.y;
		squaredDistance += dy * dy;
	}
	else if (sphereCenter.y > boundingBox.max.y) {
		double dy = sphereCenter.y - boundingBox.max.y;
		squaredDistance += dy * dy;
	}

	if (sphereCenter.z < boundingBox.min.z) {
		double dz = sphereCenter.z - boundingBox.min.z;
		squaredDistance += dz * dz;
	}
	else if (sphereCenter.z > boundingBox.max.z) {
		double dz = sphereCenter.z - boundingBox.max.z;
		squaredDistance += dz * dz;
	}

	return squaredDistance <= (radius * radius);

}

double OctreeNode::computeDivergence(OctreeNode* node, const Point& point) {
	auto neighbors = node->findNeigborsInRadius(point);

	auto divergence = 0.0;

	for (auto& neighbor : neighbors) {
		// calculate vector from point to neighbor
		Eigen::Vector3f direction(neighbor.x - point.x, neighbor.y - point.y, neighbor.z - point.z);
		direction.normalize(); // make it a unit vector

		// dot product with neighbor's normal
		Eigen::Vector3f normal(neighbor.nx, neighbor.ny, neighbor.nz);
		divergence += direction.dot(normal);
	}

	// average divergence with number of neighbors
	divergence /= neighbors.size();

	return divergence;
}

int OctreeNode::countLeafsWithScalar(OctreeNode* node) {
	if (node == nullptr)
	{
		return 0;
	}

	if (node->isLeaf()) {
		return node->containsScalarValue ? 1 : 0;
	}

	int count = 0;
	for (auto& child : node->children) {
		count += countLeafsWithScalar(child.get());
	}

	return count;
}


void OctreeNode::propagateScalarValues(OctreeNode* rootNode, int maxIterations) {
	int iteration = 0;
	while (iteration < maxIterations && fillLeafScalars(rootNode)) {
		iteration++;
	};
}


std::optional<double> OctreeNode::interpolateFromNeighbors(OctreeNode* node) {
	int dx[] = { -1, 1, 0, 0, 0, 0 };
	int dy[] = { 0, 0, -1, 1, 0, 0 };
	int dz[] = { 0, 0, 0, 0, -1, 1 };

	double sum = 0.0;
	auto count = 0u;

	for (int i = 0; i < 6; i++) {
		OctreeNode* neighbor = node->getNeighborInDirection(dx[i], dy[i], dz[i]);
		if (neighbor && neighbor->isLeaf() && neighbor->containsScalarValue) {
			sum += neighbor->scalarValue;
			count++;
		}
	}

	if (count > 0) {
		return sum / count;
	}
	else {
		return std::nullopt;
	}
}


bool OctreeNode::fillLeafScalars(OctreeNode* node) {
	if (node == nullptr) {
		return false;
	}

	bool updated = false;

	if (node->isLeaf() && !node->containsScalarValue) {
		std::optional<double> interpolatedValue = interpolateFromNeighbors(node);
		if (interpolatedValue.has_value()) {
			node->scalarValue = interpolatedValue.value();
			node->containsScalarValue = true;
			updated = true;

			std::cout << "updatedd scalar value = " <<node->scalarValue << std::endl;
		}
	}
	else {
		for (auto& child : node->children) {
			updated |= fillLeafScalars(child.get());
		}
	}

	return updated;
}



void OctreeNode::setDirectionForCorner(Corner corner, int& dx, int& dy, int& dz) {
	switch (corner) {
	case Corner::BOTTOM_FRONT_LEFT:
		dx = -1; dy = -1; dz = -1;
		break;

	case Corner::BOTTOM_FRONT_RIGHT:
		dx = 1; dy = -1; dz = -1;
		break;

	case Corner::BOTTOM_BACK_LEFT:
		dx = -1; dy = -1; dz = 1;
		break;

	case Corner::BOTTOM_BACK_RIGHT:
		dx = 1; dy = -1; dz = 1;
		break;

	case Corner::TOP_FRONT_LEFT:
		dx = -1; dy = 1; dz = -1;
		break;

	case Corner::TOP_FRONT_RIGHT:
		dx = 1; dy = 1; dz = -1;
		break;

	case Corner::TOP_BACK_LEFT:
		dx = -1; dy = 1; dz = 1;
		break;

	case Corner::TOP_BACK_RIGHT:
		dx = 1; dy = 1; dz = 1;
		break;
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

		bool is1BitSet = (i & 0b1);
		bool is2BitSet = (i & 0b10) >> 1;
		bool is4BitSet = (i & 0b100) >> 2;

		newBoundingBox.min.x = is1BitSet ? (xMin + xMax) / 2 : xMin;
		newBoundingBox.max.x = is1BitSet ? xMax : (xMin + xMax) / 2;

		newBoundingBox.min.y = is2BitSet ? (yMin + yMax) / 2 : yMin;
		newBoundingBox.max.y = is2BitSet ? yMax : (yMin + yMax) / 2;

		newBoundingBox.min.z = is4BitSet ? (zMin + zMax) / 2 : zMin;
		newBoundingBox.max.z = is4BitSet ? zMax : (zMin + zMax) / 2;

		std::cout << "bounding box\nmin.x = " << newBoundingBox.min.x << " max.x = " << newBoundingBox.max.x << "\n";
		std::cout << "min.y = " << newBoundingBox.min.y << " max.y = " << newBoundingBox.max.y << "\n";
		std::cout << "min.z = " << newBoundingBox.min.z << " max.z = " << newBoundingBox.max.z << "\n";
		std::cout << std::endl;

		children[i] = std::make_unique<OctreeNode>(newBoundingBox);
		children[i]->parent = this;
	}
}

std::vector<OctreeNode*> OctreeNode::getNeighbors(OctreeNode* node, std::unordered_map<OctreeNode*, int>& nodeToIndex) {
	std::vector<OctreeNode*> neighbors;

	double epsilon = 1e-5;

	for (auto& pair : nodeToIndex) {
		OctreeNode* potentialNeighbor = pair.first;

		if (potentialNeighbor == node) {
			continue;
		}

		// check if bounding boxes share face, edge, corner
		bool xOverlap = (node->boundingBox.min.x - epsilon <= potentialNeighbor->boundingBox.max.x) &&
			(node->boundingBox.max.x + epsilon >= potentialNeighbor->boundingBox.min.x);

		bool yOverlap = (node->boundingBox.min.y - epsilon <= potentialNeighbor->boundingBox.max.y) &&
			(node->boundingBox.max.y + epsilon >= potentialNeighbor->boundingBox.min.y);

		bool zOverlap = (node->boundingBox.min.z - epsilon <= potentialNeighbor->boundingBox.max.z) &&
			(node->boundingBox.max.z + epsilon >= potentialNeighbor->boundingBox.min.z);

		if (xOverlap && yOverlap && zOverlap && potentialNeighbor->points.size() > 0) {
			neighbors.push_back(potentialNeighbor);
		}
	}

	return neighbors;
}


void OctreeNode::setupPoissonSystem(OctreeNode* root, Eigen::SparseMatrix<double>& A, Eigen::VectorXd& b, IndexOctreeHelper& indexer) {


	auto& allNodesMap = indexer.getAllNodesMap();

	auto n = allNodesMap.size();
	A = Eigen::SparseMatrix<double>(n, n);
	b = Eigen::VectorXd(n);

	std::vector<Eigen::Triplet<double>> coefficients;

	for (const auto& pair : allNodesMap) {
		OctreeNode* node = pair.first;
		int index = pair.second;

		std::vector<OctreeNode*> neighbors = getNeighbors(node, allNodesMap);

		coefficients.push_back(Eigen::Triplet<double>(index, index, neighbors.size()));

		for (OctreeNode* neighbor : neighbors) {
			int neighborIndex = allNodesMap.at(neighbor);
			coefficients.push_back(Eigen::Triplet<double>(index, neighborIndex, -1));
		}

		b(index) = computeDivergenceForNode(node, root);
	}

	A.setFromTriplets(coefficients.begin(), coefficients.end());
}

double OctreeNode::computeDivergenceForNode(OctreeNode* node, OctreeNode* root) {

	double totalDivergence = 0.0;

	// sum divergences of all points inside node
	for (const auto& point : node->points) {
		totalDivergence += computeDivergence(root, point);
	}

	// calculate average divergence
	return totalDivergence / node->points.size();
}


bool OctreeNode::intersects(const BoundingBox& a, const BoundingBox& b) {
	return (a.min.x <= b.max.x && a.max.x >= b.min.x) &&
		(a.min.y <= b.max.y && a.max.y >= b.min.y) &&
		(a.min.z <= b.max.z && a.max.z >= b.min.z);
}

BoundingBox OctreeNode::shiftedBoundingBox(const BoundingBox& box, int dx, int dy, int dz) {
	BoundingBox shiftedBox = box;

	shiftedBox.min.x += dx;
	shiftedBox.min.y += dy;
	shiftedBox.min.z += dz;

	shiftedBox.max.x += dx;
	shiftedBox.max.y += dy;
	shiftedBox.max.z += dz;

	return shiftedBox;
}

OctreeNode* OctreeNode::getNeighborInDirection(int dx, int dy, int dz) {
	BoundingBox shiftedBox = shiftedBoundingBox(this->boundingBox, dx, dy, dz);


	if (parent) {
		for (auto& sameLevelNodes : parent->children) {
			if (sameLevelNodes.get() != this && intersects(shiftedBox, sameLevelNodes->boundingBox)) {
				if (sameLevelNodes->isLeaf())
				{
					return sameLevelNodes.get();
				}
			}
		}
	}

	// go to parent and repeat
	if (parent) {
		return parent->getNeighborInDirection(dx, dy, dz);
	}

	// no neighbor in the  direction
	return nullptr;
}


double OctreeNode::getCornerValue(OctreeNode* node, Corner corner) {

	double sum = node->scalarValue;
	auto count = 1u;

	int dx = 0, dy = 0, dz = 0;
	setDirectionForCorner(corner, dx, dy, dz);


	OctreeNode* dxNeighbor = node->getNeighborInDirection(dx, 0, 0);
	if (dxNeighbor && dxNeighbor->containsScalarValue) {
		sum += dxNeighbor->scalarValue;
		count++;
	}

	OctreeNode* dyNeighbor = node->getNeighborInDirection(0, dy, 0);
	if (dyNeighbor && dyNeighbor->containsScalarValue) {
		sum += dyNeighbor->scalarValue;
		count++;
	}

	OctreeNode* dzNeighbor = node->getNeighborInDirection(0, 0, dz);
	if (dzNeighbor && dzNeighbor->containsScalarValue) {
		sum += dzNeighbor->scalarValue;
		count++;
	}


	OctreeNode* dxyNeighbor = node->getNeighborInDirection(dx, dy, 0);
	if (dxyNeighbor && dxyNeighbor->containsScalarValue) {
		sum += dxyNeighbor->scalarValue;
		count++;
	}

	OctreeNode* dxzNeighbor = node->getNeighborInDirection(dx, 0, dz);
	if (dxzNeighbor && dxzNeighbor->containsScalarValue) {
		sum += dxzNeighbor->scalarValue;
		count++;
	}

	OctreeNode* dyzNeighbor = node->getNeighborInDirection(0, dy, dz);
	if (dyzNeighbor && dyzNeighbor->containsScalarValue) {
		sum += dyzNeighbor->scalarValue;
		count++;
	}

	return sum / count;
}

void OctreeNode::averageMissingLeafScalarValues(OctreeNode* node) {

	if (node == nullptr || node->isLeaf())
	{
		return;
	}

	double sum = 0.0;
	int countWithScalars = 0;


	for (auto& child : node->children) {
		if (child && child->isLeaf() && child->containsScalarValue) {
			sum += child->scalarValue;
			countWithScalars++;
		}
	}


	if (countWithScalars == 0) {
		for (auto& child : node->children) {
			averageMissingLeafScalarValues(child.get());
		}
		return;
	}

	double average = sum / countWithScalars;


	for (auto& child : node->children) {
		if (child && child->isLeaf() && !child->containsScalarValue) {
			child->scalarValue = average;
			child->containsScalarValue = true;

		}
		else if (!child->isLeaf()) {

			averageMissingLeafScalarValues(child.get());
		}
	}
}

int OctreeNode::countNodesWithSignChange(OctreeNode* root, CornerScalarCache& cornerScalarCache) {
	if (!root) return 0;

	int count = 0;

	if (root->isLeaf()) {
		bool hasPositive = false;
		bool hasNegative = false;

		for (int i = 0; i < 8; i++) {
			Corner corner = static_cast<Corner>(i);
			auto scalarOpt = cornerScalarCache.getScalarValueForCorner(root, corner);
			if (scalarOpt > 0) hasPositive = true;
			if (scalarOpt < 0) hasNegative = true;

			if (hasPositive && hasNegative) {
				count++;
				break;
			}

		}
	}
	else {

		for (auto& child : root->children) {
			count += countNodesWithSignChange(child.get(), cornerScalarCache);
		}
	}

	return count;
}

Vector3 OctreeNode::getCornerPosition(Corner corner) const {
	Vector3 cornerPos;

	switch (corner) {
	case Corner::BOTTOM_FRONT_LEFT:
		cornerPos = Vector3(boundingBox.min.x, boundingBox.min.y, boundingBox.min.z);
		break;
	case Corner::BOTTOM_FRONT_RIGHT:
		cornerPos = Vector3(boundingBox.max.x, boundingBox.min.y, boundingBox.min.z);
		break;
	case Corner::BOTTOM_BACK_RIGHT:
		cornerPos = Vector3(boundingBox.max.x, boundingBox.min.y, boundingBox.max.z);
		break;
	case Corner::BOTTOM_BACK_LEFT:
		cornerPos = Vector3(boundingBox.min.x, boundingBox.min.y, boundingBox.max.z);
		break;
	case Corner::TOP_FRONT_LEFT:
		cornerPos = Vector3(boundingBox.min.x, boundingBox.max.y, boundingBox.min.z);
		break;
	case Corner::TOP_FRONT_RIGHT:
		cornerPos = Vector3(boundingBox.max.x, boundingBox.max.y, boundingBox.min.z);
		break;
	case Corner::TOP_BACK_RIGHT:
		cornerPos = Vector3(boundingBox.max.x, boundingBox.max.y, boundingBox.max.z);
		break;
	case Corner::TOP_BACK_LEFT:
		cornerPos = Vector3(boundingBox.min.x, boundingBox.max.y, boundingBox.max.z);
		break;
	default:

		break;
	}

	return cornerPos;
}