#include <vector>
#include <memory>
#include <cmath>

struct Point {
	float x, y, z;
	float nx, ny, nz;
};

struct BoundingBox {
	Point min{};
	Point max{};
};

class OctreeNode {
public:
	static constexpr uint32_t maxPoints = 10;
	static constexpr uint32_t maxDepth = 6;
	static constexpr uint32_t maxChildren = 8;
	static constexpr uint32_t minNeighbors = 6;
	static constexpr uint32_t maxNeighbors = 10;

	BoundingBox boundingBox{};
	std::vector<Point> points;
	std::unique_ptr<OctreeNode> children[maxChildren] = { nullptr };
	OctreeNode(BoundingBox boundingBox) : boundingBox(boundingBox) {}


	void insert(Point& point, uint32_t depth = 0);
	bool isPointInsideBoundingBox(Point& point);
	std::vector<Point> queryRadius(Point p, float radius);
	std::vector<Point> findNeigborsInRadius(Point p);

	


	float distance(const Point& a, const Point& b);
	bool intersectsSphere(const Point& sphereCenter, float radius);
	void subdivide();
};
