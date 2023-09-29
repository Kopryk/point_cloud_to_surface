#pragma once 
#include <vector>
#include <chrono>
#include <iostream>

namespace UTILS {


	// all classes should be stateless ( have only static helper functions )
	class Helper {
	public:

	};

}


template <class T>
class Vertex4 {
public:
	union {
		struct {
			T x, y, z, w;
		};
		struct {
			T r, g, b, a;
		};
		T data[4];
	};
	T& operator[](size_t index) {
		return data[index];
	}

	const T& operator[](size_t index) const {
		return data[index];
	}

	Vertex4(T a, T b, T c, T d) : x(a), y(b), z(c), w(d) {}

};


struct PointCloudData {
	struct Points {
		std::vector <Vertex4<float>> points;
		std::vector <Vertex4<float>> colors;
	} environment;
	struct Buildings {
		std::vector <Vertex4<float>> points;
		std::vector <Vertex4<float>> colors;
	} buildings;

	std::vector <Vertex4<float>> surface;

	bool containsPoints = false;
	bool containsSurface = false;
	bool containsPointsWithColors = false;
};


enum SurfaceReconstructionMode : uint32_t {
	Poisson,
	GreedyProjectionTriangulation,
	MarchingCubesHoppe
};

enum PointCloudOptimizationMode : uint32_t {
	None,
	GridFilter,
	MLS,
	MLSUpsampling
};



class Clock {
public:
	Clock() : startTime{ std::chrono::high_resolution_clock::now() } {}

	uint64_t getTimeInMilliseconds() const {
		auto endTime = std::chrono::high_resolution_clock::now();
		return std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
	}

private:
	std::chrono::high_resolution_clock::time_point startTime;
};

class LogHelper {

public:

	static void logSurfaceReconsructionMode(SurfaceReconstructionMode mode) {
		std::cout << "SurfaceReconstructionMode = " << surfaceReconstructionModeToString(mode) << std::endl;
	}

	static void logPointCloudOptimizationMode(PointCloudOptimizationMode mode) {
		std::cout << "PointCloudOptimizationMode = " << pointCloudOptimizationModeToString(mode) << std::endl;
	}

	static void logTimeInMS(uint64_t time) {
		std::cout << "time = " << time << "ms" << std::endl;
	}



private:
	static std::string surfaceReconstructionModeToString(SurfaceReconstructionMode mode) {
		switch (mode) {
		case Poisson: return "Poisson";
		case GreedyProjectionTriangulation: return "GreedyProjectionTriangulation";
		case MarchingCubesHoppe: return "MarchingCubesHoppe";
		}

		return "";
	}

	static std::string pointCloudOptimizationModeToString(PointCloudOptimizationMode mode) {
		switch (mode) {
		case None: return "None";
		case GridFilter: return "GridFilter";
		case MLS: return "MLS";
		case MLSUpsampling: return "MLSUpsampling";
		}

		return "";
	}




};