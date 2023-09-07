#pragma once

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <random>
#include "../octree/point.h"

#include <vector>
#include "../../utils/sources/utils.h"

class PointCloudLibrary {
public:
	PointCloudLibrary() = default;

	std::vector < Vertex4<float>> calculateSurface(std::vector<Point>& points);
    void generateFlatSurfacePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        float width, float height,
        size_t numPoints,
        const pcl::PointXYZ& origin = pcl::PointXYZ(0, 0, 0)) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> disWidth(origin.x, origin.x + width);
        std::uniform_real_distribution<> disHeight(origin.y, origin.y + height);

        for (size_t i = 0; i < numPoints; ++i) {
            pcl::PointXYZ point;
            point.x = disWidth(gen);
            point.y = disHeight(gen);
            point.z = origin.z; // As it's a flat surface, z remains constant
            cloud->points.push_back(point);
        }
    }

private:



};