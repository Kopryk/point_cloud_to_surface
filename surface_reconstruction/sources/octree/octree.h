#pragma once

#include <vector>
#include <memory>
#include <cmath>
#include <optional>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Sparse>

#include "corner.h"
#include "../marching_cubes_from_octree/triangle.h"

struct Point {
	double x, y, z;
	double nx, ny, nz;
};

struct BoundingBox {
	Point min{};
	Point max{};
};

class CornerScalarCache;
class IndexOctreeHelper;

class OctreeNode {
public:
	inline static double radius = 1.0f;
	static constexpr uint32_t maxPoints = 6;
	static constexpr uint32_t maxDepth = 9;
	static constexpr uint32_t maxChildren = 8;
	static constexpr uint32_t minNeighbors = 3;
	static constexpr uint32_t maxNeighbors = 15;
	inline static uint32_t nAllPoints = 0u;

	std::unique_ptr<OctreeNode> children[maxChildren] = { nullptr };
	std::vector<Point> points;
	OctreeNode* parent = nullptr;
	BoundingBox boundingBox{};
	double scalarValue = 0.0;

	bool containsScalarValue = false;

	OctreeNode(BoundingBox boundingBox) : boundingBox(boundingBox) {}

	bool intersects(const BoundingBox& a, const BoundingBox& b);
	bool isLeaf();
	bool isPointInsideBoundingBox(Point& point);
	BoundingBox shiftedBoundingBox(const BoundingBox& box, int dx, int dy, int dz);
	double computeDivergenceForNode(OctreeNode* node, OctreeNode* root);
	double getCornerValue(OctreeNode* node, Corner corner);
	Eigen::Vector3f computeNormalForPoint(OctreeNode& octreeNode, Point p);
	Eigen::VectorXd computeDivergenceForAllPoints(OctreeNode* rootNode);;
	OctreeNode* getNeighborInDirection(int dx, int dy, int dz);
	std::vector<OctreeNode*> getNeighbors(OctreeNode* node, std::unordered_map<OctreeNode*, int>& nodeToIndex);
	std::vector<Point> findNeigborsInRadius(Point p);
	std::vector<Point> getAllPoints();
	std::vector<Point> queryRadius(Point p, double radius);
	uint32_t getNumberOfNodesInOctree();
	void computeNormalForAllPoints(OctreeNode& octreeNode, OctreeNode* root);
	void insert(Point& point, uint32_t depth = 0);
	void setDirectionForCorner(Corner corner, int& dx, int& dy, int& dz);
	void setupPoissonSystem(OctreeNode* root, Eigen::SparseMatrix<double>& A, Eigen::VectorXd& b, IndexOctreeHelper& indexer);
	std::vector<Triangle> solvePoissonProblem(OctreeNode* root);
	void averageMissingLeafScalarValues(OctreeNode* node);
	std::optional<double> interpolateFromNeighbors(OctreeNode* node);;
	bool fillLeafScalars(OctreeNode* node);
	void propagateScalarValues(OctreeNode* rootNode, int maxIterations = 100000);
	int countLeafsWithScalar(OctreeNode* node);
	int countNodesWithSignChange(OctreeNode* root, CornerScalarCache& cornerScalarCache);
	Vector3 getCornerPosition(Corner corner) const;

private:

	void subdivide();
	double distance(const Point& a, const Point& b);
	bool intersectsSphere(const Point& sphereCenter, double radius);
	double computeDivergence(OctreeNode* node, const Point& point);
};

