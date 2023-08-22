

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
#include "voxelization/voxelization.h"



int main() {


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
	auto result = normalEstimation.processOnGpu();


	auto voxelHelper = SR::Voxelization(scalledPoints.value());
	auto voxels = voxelHelper.processOnGpu();
	if (voxels.has_value()) {
		std::cout << "correct voxels!" << std::endl;
	}



	if (result.has_value()) {



		std::vector<Vertex4<float> > normals;
		for (auto i = 0u; i < result->size(); i++) {
			Vertex4<float> point(0, 0, 0, 0);
			point.x = result->at(i).x;
			point.y = result->at(i).y;
			point.z = result->at(i).z;
			point.w = result->at(i).w;
			normals.push_back(point);
		}

		ga.init(points, normals);


		ga.mainLoop();

	}
	else {

		ga.init(points);

		ga.mainLoop();
	}



	return 0;
}