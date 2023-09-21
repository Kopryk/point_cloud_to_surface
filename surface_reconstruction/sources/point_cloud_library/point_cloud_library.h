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


struct Float3 {
	float x, y, z;
};

class PointCloudLibrary {
public:

	PointCloudLibrary() = default;
	std::vector < Vertex4<float>> normalizePoints(std::vector < Vertex4<float>>& points);
	std::unique_ptr<PointCloudData> loadPoints();
	std::unique_ptr<PointCloudData>  calculateSurface(std::vector <Vertex4<float>>& pointCloud, bool useGridFilter, double gridSizeInPercent = 0.001, double neighbourRangeInPercent = 0.001);


private:
	uint32_t countLines(const std::string& filename);
};