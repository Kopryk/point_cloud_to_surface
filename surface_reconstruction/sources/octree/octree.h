#include <vector>
#include <memory>

struct Point {
	float x, y, z;
};

struct BoundingBox {
	Point min{};
	Point max{};
};

class Octree {
public:
	static constexpr uint32_t maxPoints = 10;
	static constexpr uint32_t maxDepth = 6;
	static constexpr uint32_t maxChildren = 8;
	BoundingBox boundingBox{};

	std::vector<Point> points;
	std::unique_ptr<Octree> children[maxChildren] = { nullptr };

	Octree(BoundingBox boundingBox) : boundingBox(boundingBox) {}


	void insert(Point& point, uint32_t depth = 0);

	bool isPointInsideBoundingBox(Point& point);

	void subdivide();

};
