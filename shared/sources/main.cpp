

#include "marching_cubes/marching_cubes.h"
#include "normal_estimation/normal_estimation.h"
#include "utils.h"
#include "position.h"
#include "graphics/GraphicsApplication.h"

#include <thread>
#include <vector>
#include <memory>
#include <iostream>
#include <mutex>
#include <execution>
#include <random>
#include "scale/scale.h"
#include <vector>
#include <algorithm>
#include <limits>

#include "voxelization/voxelization.h"
#include "voxelization_with_average/voxelization_with_average.h"

#include "scalar_field/scalar_field.h"
#include "divergence/divergence.h"
#include "weighted_poisson_equation/weighted_poisson_equation.h"
#include "octree/octree.h"
#include "octree/corner_scalar_cache.h"
#include "marching_cubes_from_octree/triangle.h"


bool isAlmostEqual(const Point& a, const Point& b, double epsilon) {
	return std::fabs(a.x - b.x) < epsilon &&
		std::fabs(a.y - b.y) < epsilon &&
		std::fabs(a.z - b.z) < epsilon;
}

void removeDuplicates(std::vector<Point>& points, double epsilon) {
	points.erase(std::unique(points.begin(), points.end(),
		[epsilon](const Point& a, const Point& b) {
			return isAlmostEqual(a, b, epsilon);
		}),
		points.end());
}

void normalizeVertices(std::vector<Vertex4<float>>& vertices) {
	float minX = std::numeric_limits<float>::max();
	float minY = minX;
	float minZ = minX;

	float maxX = std::numeric_limits<float>::lowest();
	float maxY = maxX;
	float maxZ = maxX;

	// Step 1: Determine min and max values for each component
	for (const auto& vertex : vertices) {
		minX = std::min(minX, vertex.x);
		minY = std::min(minY, vertex.y);
		minZ = std::min(minZ, vertex.z);

		maxX = std::max(maxX, vertex.x);
		maxY = std::max(maxY, vertex.y);
		maxZ = std::max(maxZ, vertex.z);
	}

	// Step 2: Normalize the data using the min and max values
	for (auto& vertex : vertices) {
		vertex.x = (vertex.x - minX) / (maxX - minX);
		vertex.y = (vertex.y - minY) / (maxY - minY);
		vertex.z = (vertex.z - minZ) / (maxZ - minZ);
	}
}


void demo_sphere() {

	std::vector<Vertex4<float> >points;
	auto nPoints = 50;

	for (int i = 0; i < nPoints; ++i) {
		float theta = 2.0f * 3.1415f * float(i) / float(nPoints);
		for (int j = 0; j < nPoints; ++j) {
			float phi = 3.1415f * float(j) / float(nPoints);
			Vertex4<float> point(0, 0, 0, 0);
			point.x = sin(phi) * cos(theta);
			point.y = sin(phi) * sin(theta);
			point.z = cos(phi);
			points.push_back(point);
		}
	}

	BoundingBox box{};
	box.min.x = 0;
	box.min.y = 0;
	box.min.z = 0;
	box.max.x = 1000;
	box.max.y = 1000;
	box.max.z = 1000;

	std::cout << "Points size = " << points.size() << std::endl;

	auto pointsClFloat3 = std::vector<cl_float3>(points.size());
	for (auto i = 0u; i < pointsClFloat3.size(); i++) {
		pointsClFloat3[i].x = points[i].x;
		pointsClFloat3[i].y = points[i].y;
		pointsClFloat3[i].z = points[i].z;
		pointsClFloat3[i].w = points[i].w;
	}

	auto scaleHelper = SR::Scale(pointsClFloat3);
	auto scalledPoints = scaleHelper.processOnGpu();


	OctreeNode root{ box };
	std::vector<Point> pointsToCheck{};

	for (auto& point : scalledPoints.value()) {
		Point p;
		p.x = point.x;
		p.y = point.y;
		p.z = point.z;

		pointsToCheck.push_back(p);

	}
	std::sort(pointsToCheck.begin(), pointsToCheck.end(), [](const Point& a, const Point& b) {
		if (a.x != b.x) return a.x < b.x;
		if (a.y != b.y) return a.y < b.y;
		return a.z < b.z;
		});


	removeDuplicates(pointsToCheck, 0.0001);


	for (auto& point : pointsToCheck) {
		root.insert(point, 0);
	}


	auto result = root.solvePoissonProblem(&root);


	std::vector<Vertex4<float> > triangleVerticles;
	for (auto i = 0u; i < result.size(); i++) {

		for (int j = 0; j < 3; j++) {
			Vertex4<float> point(0, 0, 0, 0);
			point.x = result[i].getVertex(j).x;
			point.y = result[i].getVertex(j).y;
			point.z = result[i].getVertex(j).z;
			point.w = 1;
			triangleVerticles.push_back(point);
		}
	}


	normalizeVertices(triangleVerticles);
	normalizeVertices(points);

	auto& ga = GraphicsApplication::get();

	ga.initWithTrianglesWithPoints(triangleVerticles, points);


	ga.mainLoop();

}

int main() {

	demo_sphere();

	return 0;
	//std::vector<std::jthread> threads;
	//std::mutex m{};
	//for (int i = 0; i < 10; i++) {

	//	threads.push_back(std::jthread([&m](int a, int b) {
	//		auto c = a;
	//		std::cout << c << std::endl;
	//		}, i, i));
	//}

	//auto randomPoints = []() {
	//	using namespace UTILS;
	//	constexpr size_t size = 1ull * 1024;
	//	auto points = std::vector<cl_float3>(size);

	//	std::random_device random_device;
	//	std::mt19937 engine{ random_device() };
	//	std::uniform_real_distribution<PositionType> distribution(0.0f, 1000.0f);

	//	for (auto i = 0ull; i < size; i++) {
	//		Position < PositionType> position;
	//		position.x = distribution(engine);
	//		position.y = distribution(engine);
	//		position.z = distribution(engine);

	//		points.at(i).x = position.x;
	//		points.at(i).y = position.y;
	//		points.at(i).z = position.z;

	//	}

	//	return points;
	//	};

	//auto points = randomPoints();


	auto& ga = GraphicsApplication::get();



	//for (int i = 0; i < 1000; i++) {

	//	float x = i / 1.0;
	//	float y = i / 1.0  - 0.5f;
	//	float z = i / 1.0 + 0.5f;


	//	points.push_back(Vertex4<float>(x, y, z, 0));
	//	points.push_back(Vertex4<float>(0, y, 0, 0));
	//	points.push_back(Vertex4<float>(0, 0, z, 0));
	//	points.push_back(Vertex4<float>(x, 0, z, 0));


	//}


	// Sample data: std::vector<cl_float3> for sphere coordinates
	std::vector<Vertex4<float> >points;
	auto nPoints = 25;

	for (int i = 0; i < nPoints; ++i) {
		float theta = 2.0f * 3.1415f * float(i) / float(nPoints);
		for (int j = 0; j < nPoints; ++j) {
			float phi = 3.1415f * float(j) / float(nPoints);

			Vertex4<float> point(0, 0, 0, 0);
			point.x = sin(phi) * cos(theta);
			point.y = sin(phi) * sin(theta);
			point.z = cos(phi);

			points.push_back(point);


		}
	}


	std::cout << "Points size = " << points.size() << std::endl;

	auto pointsToNormalize = std::vector<cl_float3>(points.size());
	for (auto i = 0u; i < pointsToNormalize.size(); i++) {
		pointsToNormalize[i].x = points[i].x;
		pointsToNormalize[i].y = points[i].y;
		pointsToNormalize[i].z = points[i].z;
		pointsToNormalize[i].w = points[i].w;
	}

	auto scaleHelper = SR::Scale(pointsToNormalize);

	auto scalledPoints = scaleHelper.processOnGpu();


	SR::NormalEstimation normalEstimation{ scalledPoints.value() };
	auto normals = normalEstimation.processOnGpu();


	//auto voxelHelper = SR::Voxelization(scalledPoints.value());
	//auto voxels = voxelHelper.processOnGpu();
	//if (voxels.has_value()) {
	//	std::cout << "correct voxels!" << std::endl;
	//}


	uint32_t gridSizeX = 16;
	uint32_t gridSizeY = 16;
	uint32_t gridSizeZ = 16;

	auto nVoxels = gridSizeX * gridSizeY * gridSizeZ;
	std::vector<cl_float3> normalsInVoxels(nVoxels);

	auto voxelHelper = SR::VoxelizationWithAverage(scalledPoints.value(), normals.value(), normalsInVoxels, gridSizeX, gridSizeY, gridSizeZ);
	auto voxelsAverage = voxelHelper.processOnGpu();
	if (voxelsAverage.has_value()) {
		std::cout << "correct voxels!" << std::endl;
	}




	uint32_t xCenter = voxelHelper.xCenter;
	uint32_t yCenter = voxelHelper.yCenter;
	uint32_t zCenter = voxelHelper.zCenter;/*
	uint32_t gridSizeX = voxelHelper.gridSizeX;
	uint32_t gridSizeY = voxelHelper.gridSizeY;
	uint32_t gridSizeZ = voxelHelper.gridSizeZ;*/



	auto scalarFieldDistancesHelper = SR::ScalarField(xCenter, yCenter, zCenter, gridSizeX, gridSizeY, gridSizeZ);
	auto scalarFieldsDistances = scalarFieldDistancesHelper.processOnGpu();

	if (scalarFieldsDistances.has_value()) {
		std::cout << "correct scalarFieldsDistances!" << std::endl;
	}




	auto divergenceHelper = SR::Divergence(normals.value(), normalsInVoxels, voxelHelper.voxelsWtinIndices, 0, gridSizeX, gridSizeY, gridSizeZ);

	auto divergences = divergenceHelper.processOnGpu();


	if (divergences.has_value()) {


		int correctDivergences = 0;
		for (auto value : divergences.value()) {
			if (value != 0.0f) {
				correctDivergences++;

			}
		}
		std::cout << "correct divergences! n: " << correctDivergences << std::endl;
	}


	auto weightedPoissonEquationHelper = SR::WeightedPoissonEquation::WeightedPoissonEquation(divergences.value(), scalarFieldsDistances.value(), voxelHelper.voxelsWtinIndices, 0, gridSizeX, gridSizeY, gridSizeZ);

	auto scalarFieldsUpdated = weightedPoissonEquationHelper.processOnGpu();

	int countDifferences = 0;

	if (scalarFieldsUpdated.has_value()) {
		for (int i = 0; i < scalarFieldsUpdated.value().size(); i++) {
			auto before = scalarFieldsDistances.value()[i];
			auto after = scalarFieldsUpdated.value()[i];
			if (before != after) {
				std::cout << " diff i = " << i << std::endl;
				std::cout << "before = " << before << std::endl;
				std::cout << "after = " << after << std::endl;
				std::cout << std::endl;
				countDifferences++;
			}
		}
	}
	std::cout << " scalar Fields updated n= " << countDifferences << std::endl;


	if (normals.has_value()) {



		std::vector<Vertex4<float> > normals4f;
		for (auto i = 0u; i < normals->size(); i++) {
			Vertex4<float> point(0, 0, 0, 0);
			point.x = normals->at(i).x;
			point.y = normals->at(i).y;
			point.z = normals->at(i).z;
			point.w = normals->at(i).w;
			normals4f.push_back(point);
		}

		ga.init(points, normals4f);


		ga.mainLoop();

	}
	else {

		ga.init(points);

		ga.mainLoop();
	}



	return 0;
}