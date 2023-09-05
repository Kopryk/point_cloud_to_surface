#include "marching_cubes_from_octree.h"

#include "../octree/corner_scalar_cache.h"
#include "../octree/octree.h"
#include "../octree/corner.h"

int MarchingCubesFromOctree::computeLookupIndex(OctreeNode* node, CornerScalarCache& cornerScalarCache) {
	auto index = 0;
	for (auto i = 0; i < 8; i++) {
		Corner corner = static_cast<Corner>(i);
		if (cornerScalarCache.getScalarValueForCorner(node, corner) < 0.0)
			index |= (1 << i);
	}
	return index;
}


Vector3 computeIntersection(OctreeNode* node, int edgeNum, CornerScalarCache& cornerScalarCache) {
    // Define the two vertices of each edge
    static int edgeCorners[12][2] = {
        {0, 1}, {1, 2}, {2, 3}, {3, 0},
        {4, 5}, {5, 6}, {6, 7}, {7, 4},
        {0, 4}, {1, 5}, {2, 6}, {3, 7}
    };

    Corner corner1 = static_cast<Corner>(edgeCorners[edgeNum][0]);
    Corner corner2 = static_cast<Corner>(edgeCorners[edgeNum][1]);

    double val1 = cornerScalarCache.getScalarValueForCorner(node, corner1);
    double val2 = cornerScalarCache.getScalarValueForCorner(node, corner2);

    // if both corners are on the same side of the iso-value return default position
    // shouldn't happen because we already checked with edgeTable
    if (std::signbit(val1) == std::signbit(val2)) {
        return Vector3(0, 0, 0);
    }

    // Get the positions of the two corners
    Vector3 pos1 = node->getCornerPosition(corner1);
    Vector3 pos2 = node->getCornerPosition(corner2);

    // Linear interpolation formula
    double t = (0.0 - val1) / (val2 - val1);
    Vector3 intersection = pos1 + t * (pos2 - pos1);

    return intersection;
}


std::vector<Triangle> MarchingCubesFromOctree::extractTrianglesForNode(OctreeNode* node, CornerScalarCache& cornerScalarCache) {
    std::vector<Triangle> triangles;
    int index = computeLookupIndex(node, cornerScalarCache);

    if (edgeTable[index] == 0)
        return triangles;

    Vector3 intersectionPoints[12];
    for (int i = 0; i < 12; i++) {
        if (edgeTable[index] & (1 << i))
            intersectionPoints[i] = computeIntersection(node, i, cornerScalarCache);
    }

    for (int i = 0; triTable[index][i] != -1; i += 3) {
        Triangle tri;
        // TODO CHECK IF CORRECT
        tri.vertices[0] = intersectionPoints[triTable[index][i]];
        tri.vertices[1] = intersectionPoints[triTable[index][i + 1]];
        tri.vertices[2] = intersectionPoints[triTable[index][i + 2]];
        triangles.push_back(tri);
    }

    return triangles;
}
