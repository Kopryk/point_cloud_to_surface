#pragma once

#include <unordered_map>


class OctreeNode;

class IndexOctreeHelper {
public:
    IndexOctreeHelper() = default;
    void indexOctree(OctreeNode* node);
    int getIndexOfNode(OctreeNode* node);
    std::unordered_map<OctreeNode*, int>& getAllNodesMap();

private:

    std::unordered_map<OctreeNode*, int> nodeToIndex;
    int currentIndex = 0;
};
