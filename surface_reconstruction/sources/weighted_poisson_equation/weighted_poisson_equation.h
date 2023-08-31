#pragma once
#include "../../utils/sources/utils.h"
#include "../../utils/sources/position.h"
#include <CL/opencl.hpp>

#include <vector>
#include <optional>
#include <memory>

namespace SR {
	using namespace UTILS;;

	// compute WeightedPoissonEquation for each occupied voxel
	// using derivate of normal in each occupied voxel
	// WeightedPoissonEquation is a sum of x,y,z component of derivate of normal in voxel

	class WeightedPoissonEquation
	{
	public:
		WeightedPoissonEquation(std::vector<float>& divergencesVector, std::vector<float> &scalarFieldsDistancesVector, std::vector<uint32_t>& voxels, uint32_t invalidVoxelValue, uint32_t gridSizeX, uint32_t gridSizeY, uint32_t gridSizeZ);


		std::optional<std::vector<float>> processOnGpu();
		bool processOnCpu();
		void checkIfGpuIsAvailable();

	private:

		uint32_t indexOfCentroidVoxel = 0u;
		static constexpr uint32_t invalidIndex = std::numeric_limits<uint32_t>::max();
		uint32_t invalidVoxelValue;

		const uint32_t gridSizeX = 16;
		const uint32_t gridSizeY = 16;
		const uint32_t gridSizeZ = 16;

		bool isGpuAvailable = false;
		// this function should be write somewhere else

		void printGpuDetails();

		std::vector<uint32_t>& voxels;
		std::vector<float> &divergencesVector;
		std::vector<float>& scalarFieldsDistancesVector;
		std::vector<cl::Platform> platforms;
		cl::Context context;
		cl::Device device;
		cl::Program::Sources sources;
		std::string kernelCode;
		std::unique_ptr<cl::Program >program;
		std::unique_ptr<cl::CommandQueue> queue;
		float radius = 0.6f; // TODO remove hardcoded default


	};


}