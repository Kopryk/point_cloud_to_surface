#pragma once

#include <map>

class OctreeNode;
enum class Corner;

class CornerScalarCache {
public:
	std::map<std::pair<OctreeNode*, Corner>, double> cornerScalarCache;
    double getScalarValueForCorner(OctreeNode* node, Corner corner);
    double computeCornerScalarValue(OctreeNode* node, Corner corner);

};