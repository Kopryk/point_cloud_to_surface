#pragma once
#include "../../utils/sources/utils.h"
#include "../../utils/sources/position.h"
#include <CL/opencl.hpp>

#include <vector>
#include <optional>
#include <memory>

namespace SR {
	using namespace UTILS;;

	class NormalEstimation
	{
	public:
		NormalEstimation(std::vector<cl_float3>& points);

		std::optional<std::vector<cl_float3>> processOnGpu();
		bool processOnCpu();
		void checkIfGpuIsAvailable();

	private:
		std::vector< cl_float3>& points;

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
		float radius = 0.2f; // TODO remove hardcoded default


	};


}