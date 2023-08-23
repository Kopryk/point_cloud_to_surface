#pragma once
#include "../../utils/sources/utils.h"
#include "../../utils/sources/position.h"
#include <CL/opencl.hpp>

#include <vector>
#include <optional>
#include <memory>

namespace SR {
	using namespace UTILS;;

	class ScalarField
	{
	public:
		ScalarField( float refX, float refY, float refZ, uint32_t gridSizeX, uint32_t gridSizeY, uint32_t gridSizeZ);

		std::optional<std::vector<float>> processOnGpu();
		bool processOnCpu();
		void checkIfGpuIsAvailable();

	private:

		uint32_t indexOfCentroidVoxel = 0u;
		static constexpr uint32_t invalidIndex = std::numeric_limits<uint32_t>::max();

		const float refX = 0;
		const float refY = 0;
		const float refZ = 0;
		const uint32_t gridSizeX = 200;
		const uint32_t gridSizeY = 200;
		const uint32_t gridSizeZ = 200;

		bool isGpuAvailable = false;
		// this function should be write somewhere else

		void printGpuDetails();

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