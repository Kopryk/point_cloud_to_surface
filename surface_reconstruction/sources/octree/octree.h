#pragma once

#include <vector>
#include <memory>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Sparse>


struct Point {
	double x, y, z;
	double nx, ny, nz;
};

struct BoundingBox {
	Point min{};
	Point max{};
};

class IndexOctreeHelper;

class OctreeNode {
public:
	double scalarValue = 0.0;
	static constexpr uint32_t maxPoints = 10;
	static constexpr uint32_t maxDepth = 6;
	static constexpr uint32_t maxChildren = 8;

	static constexpr uint32_t minNeighbors = 6;
	static constexpr uint32_t maxNeighbors = 10;

	inline static double radius = 1.0f;
	inline static uint32_t nAllPoints = 0u;
	BoundingBox boundingBox{};
	std::vector<Point> points;
	std::unique_ptr<OctreeNode> children[maxChildren] = { nullptr };
	OctreeNode(BoundingBox boundingBox) : boundingBox(boundingBox) {}

	void insert(Point& point, uint32_t depth = 0);
	bool isPointInsideBoundingBox(Point& point);
	std::vector<Point> queryRadius(Point p, double radius);
	std::vector<Point> findNeigborsInRadius(Point p);
	Eigen::Vector3f computeNormalForPoint(OctreeNode& octreeNode, Point p);
	void computeNormalForAllPoints(OctreeNode& octreeNode, OctreeNode* root);
	std::vector<Point> getAllPoints();
	Eigen::VectorXd computeDivergenceForAllPoints(OctreeNode* rootNode);;
	uint32_t getNumberOfNodesInOctree();

	void setupPoissonSystem(OctreeNode* root, Eigen::SparseMatrix<double>& A, Eigen::VectorXd& b, IndexOctreeHelper& indexer);
	std::vector<OctreeNode*> getNeighbors(OctreeNode* node, std::unordered_map<OctreeNode*, int>& nodeToIndex);
	double computeDivergenceForNode(OctreeNode* node, OctreeNode* root);

	void solvePoissonProblem(OctreeNode* root);

	bool isLeaf();
private:

	void subdivide();
	double distance(const Point& a, const Point& b);
	bool intersectsSphere(const Point& sphereCenter, double radius);
	double computeDivergence(OctreeNode* node, const Point& point);
};

