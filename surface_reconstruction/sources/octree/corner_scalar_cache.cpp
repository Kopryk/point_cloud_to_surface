#include "corner_scalar_cache.h"
#include "octree.h"

double CornerScalarCache::getScalarValueForCorner(OctreeNode* node, Corner corner) {

    auto key = std::make_pair(node, corner);

    auto it = cornerScalarCache.find(key);
    if (it != cornerScalarCache.end()) {
        return it->second;
    }
    else {

        double scalarValue = computeCornerScalarValue(node, corner);
        cornerScalarCache[key] = scalarValue;
        return scalarValue;
    }
}

double CornerScalarCache::computeCornerScalarValue(OctreeNode* node, Corner corner) {
    double computedValue = node->getCornerValue(node, corner);
    return computedValue;
}
