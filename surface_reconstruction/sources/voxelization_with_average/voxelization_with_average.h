#pragma once
#include "../../utils/sources/utils.h"
#include "../../utils/sources/position.h"
#include <CL/opencl.hpp>

#include <vector>
#include <optional>
#include <memory>

namespace SR {
	using namespace UTILS;;

	class VoxelizationWithAverage
	{
	public:
		VoxelizationWithAverage(std::vector<cl_float3>& points, std::vector<cl_float3>& normals, std::vector<cl_float3>& voxelsWithNormals, uint32_t gridSizeX, uint32_t gridSizeY, uint32_t gridSizeZ);

		std::optional<std::vector<cl_float3>> processOnGpu();
		bool processOnCpu();
		void checkIfGpuIsAvailable();

		uint32_t xCenter = 0u;
		uint32_t yCenter = 0u;
		uint32_t zCenter = 0u;
		static constexpr uint32_t invalidIndex = std::numeric_limits<uint32_t>::max();
		uint32_t gridSizeX = 16;
		uint32_t gridSizeY = 16;
		uint32_t gridSizeZ = 16;

		std::vector<uint32_t> voxelsWtinIndices;

	private:

		uint32_t indexOfCentroidVoxel = 0u;



		uint32_t findIndexOfCentroidVoxel(std::vector<uint32_t>& voxelsVec, size_t nPoints, uint32_t& xCenter, uint32_t& yCenter, uint32_t& zCenter);

		std::vector< cl_float3>& points;
		std::vector<cl_float3>& normals;
		std::vector<cl_float3>& voxelsWithNormals;

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