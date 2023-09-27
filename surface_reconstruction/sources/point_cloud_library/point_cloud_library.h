#pragma once

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <random>
#include "../octree/point.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

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
	std::unique_ptr<PointCloudData>  calculateSurface(std::vector <Vertex4<float>>& pointCloud, PointCloudOptimizationMode optimizationMode, SurfaceReconstructionMode reconstructionMode, uint32_t depthOctree = 8, uint32_t gridResolution = 100, double gridSizeInPercent = 0.001, double neighbourRangeInPercent = 0.001, float dilationVoxelSizeInPercent = 1.0f, uint32_t dilationIteration = 2);
	float computeMeanSquaredError( const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& reconstructedSurface);

private:
	uint32_t countLines(const std::string& filename);
};