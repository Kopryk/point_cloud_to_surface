#include "octree.h"
#include <iostream>
#include "index_octree.h"
#include <chrono>
#include <thread>


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

void OctreeNode::solvePoissonProblem(OctreeNode* root) {


	computeNormalForAllPoints(*root, root);
	IndexOctreeHelper indexer{};
	indexer.indexOctree(root);

	Eigen::SparseMatrix<double> A;
	Eigen::VectorXd b, x;
	setupPoissonSystem(root, A, b, indexer);

	Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower | Eigen::Upper> solver;
	solver.setMaxIterations(1000);
	solver.setTolerance(0.1);

	solver.compute(A);

	if (solver.info() != Eigen::Success) {
		std::cout << __LINE__ << "compute failed\n";
		return;
	}


	x = solver.solve(b);

	std::cout << "Number of iterations : " << solver.iterations() << std::endl;
	std::cout << "Estimated error: " << solver.error() << std::endl;


	if (solver.info() != Eigen::Success) {
		std::cout << __LINE__ << "solve failed error: " << solver.info() << std::endl;
		return;
	}

	auto allNodes = indexer.getAllNodesMap();

	for (auto& [node, index] : allNodes) {
		node->scalarValue = x(index);
		std::cout << "index= " << index << "scalar value = " << node->scalarValue << "\n";
	}

	std::cout << "SolvePoissonProblem passed!\n";
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

inline double OctreeNode::computeDivergence(OctreeNode* node, const Point& point) {
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
		bool is2BitSet = (i & 2) >> 1;  // You need to right-shift to consider the bit correctly
		bool is4BitSet = (i & 4) >> 2;  // You need to right-shift to consider the bit correctly

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