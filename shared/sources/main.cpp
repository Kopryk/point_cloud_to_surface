#include "renderer/renderer.h"
#include "marching_cubes/marching_cubes.h"
#include "normal_estimation/normal_estimation.h"
#include "utils.h"
#include "position.h"


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

	auto randomPoints = []() {
		using namespace UTILS;
		constexpr size_t size = 1ull * 1024;
		auto points = std::vector<cl_float3>(size);

		std::random_device random_device;
		std::mt19937 engine{ random_device() };
		std::uniform_real_distribution<PositionType> distribution(0.0f, 1000.0f);

		for (auto i = 0ull; i < size; i++) {
			Position < PositionType> position;
			position.x = distribution(engine);
			position.y = distribution(engine);
			position.z = distribution(engine);

			points.at(i).x = position.x;
			points.at(i).y = position.y;
			points.at(i).z = position.z;

		}

		return points;
		};

	auto points = randomPoints();
	SR::NormalEstimation normalEstimation{ points };
	normalEstimation.processOnGpu();

	return 0;
}