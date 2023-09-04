#include "index_octree.h"
#include <iostream>
#include <thread>
#include <chrono>
#include "octree.h"

void IndexOctreeHelper::indexOctree(OctreeNode* node) {
	if (node == nullptr) {
		return;
	}


	if (node->isLeaf() && node->points.size() > 0 ) {
		nodeToIndex[node] = currentIndex;
		++currentIndex;
	}
	else {
		for (auto i = 0u; i < OctreeNode::maxChildren; i++) {
			indexOctree(node->children[i].get());
		}
	}
}

int IndexOctreeHelper::getIndexOfNode(OctreeNode* node) {
	using namespace std::chrono_literals;
	if (auto it = nodeToIndex.find(node);  it != nodeToIndex.end()) {
		return it->second;
	}
	else {

		std::cout << __LINE__ << " ERROR: couldnt find index of a node!\n";
		std::this_thread::sleep_for(2000ms);

		return -1;
	}
}

std::unordered_map<OctreeNode*, int>& IndexOctreeHelper::getAllNodesMap() {
	return nodeToIndex;
}
