

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
	//SR::NormalEstimation normalEstimation{ points };
	//normalEstimation.processOnGpu();

	auto& ga = GraphicsApplication::get();
	std::vector<Vertex4<float> >points;

	for (int i = 0; i < 1000'000; i++) {

		float x = i / 1000.0;
		float y = i / 1000.0  - 0.5f;
		float z = i / 1000.0 + 0.5f;


		points.push_back(Vertex4<float>(x, 0, 0, 0));
		points.push_back(Vertex4<float>(0, y, 0, 0));
		points.push_back(Vertex4<float>(0, 0, z, 0));
		points.push_back(Vertex4<float>(x, 0, z, 0));


	}
	ga.init(points);

	ga.mainLoop();

	return 0;
}