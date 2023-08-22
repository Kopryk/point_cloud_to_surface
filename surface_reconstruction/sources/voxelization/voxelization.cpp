#include "voxelization.h"

#include <iostream>




SR::Voxelization::Voxelization(std::vector<cl_float3>& points) : points(points)
{

	checkIfGpuIsAvailable();
}

std::optional<std::vector<uint8_t>> SR::Voxelization::processOnGpu()
{
	try {
			auto numPoints = points.size();
			auto sizePointsInBytes = numPoints * sizeof(cl_float3);

			constexpr size_t voxelDim = 1000;
			constexpr size_t nVoxels = voxelDim * voxelDim * voxelDim;
			auto voxelsNumber = nVoxels;
			auto voxelsInBytes = nVoxels * sizeof(uint8_t);
			// 2.0f is hardcoded for now but its abs(-1) + 1 = range  = 2.0

			auto voxelSize = 2.0f / voxelDim;

			cl::Buffer inputBuffer(context, CL_MEM_READ_WRITE, sizePointsInBytes);
			cl::Buffer voxels(context, CL_MEM_READ_WRITE, voxelsInBytes);

			// Fill the buffer with zeros
			cl_char pattern = 0;
			size_t offset = 0;

			queue->enqueueFillBuffer(voxels, pattern, offset, voxelsInBytes);

			auto error = queue->enqueueWriteBuffer(inputBuffer, CL_TRUE, 0, sizePointsInBytes, points.data());

			if (error) {
				std::cout << "queue->enqueueWriteBuffer(inputBuffer, CL_TRUE, 0, sizePointsInBytes, points_.data()); error: " << error << std::endl;
			}

			// float* points, __global uint8_t* voxels, int numPoints, float voxelSize

			cl::Kernel kernel(*program, "voxelizationKernel");
			error = kernel.setArg(0, inputBuffer);
			if (error) {
				std::cout << "  kernel.setArg(0, inputBuffer) error: " << error << std::endl;
			}
			error = kernel.setArg(1, voxels);
			if (error) {
				std::cout << "  kernel.setArg(1, inputBuffer) error: " << error << std::endl;
			}
			error = kernel.setArg(2, static_cast<int>(numPoints));
			if (error) {
				std::cout << "  kernel.setArg(2, inputBuffer) error: " << error << std::endl;
			}
			error = kernel.setArg(3, voxelSize);
			if (error) {
				std::cout << "  kernel.setArg(3, inputBuffer) error: " << error << std::endl;
			}


			error = queue->enqueueNDRangeKernel(kernel, cl::NullRange, cl::NDRange(numPoints), cl::NullRange);
			if (error) {
				std::cout << " queue->enqueueNDRangeKernel(kernel, cl::NullRange, cl::NDRange(numPoints), cl::NullRange); error: " << error << std::endl;
			}
			error = queue->finish();
			if (error) {
				std::cout << " queue->finish(); error: " << error << std::endl;
			}

			std::vector<uint8_t> voxelsVec(nVoxels);

			error = queue->enqueueReadBuffer(voxels, CL_TRUE, 0, voxelsInBytes, voxelsVec.data());
			if (error) {
				std::cout << "queue->enqueueReadBuffer(outputBuffer, CL_TRUE, 0, numPoints * sizeof(cl_float3), normalsVector.data()); error: " << error << std::endl;
			}


			auto activeVoxels = 0u;
			auto countPoints = 0u;
			for (auto value : voxelsVec) {
				if (value > 0) {
					activeVoxels++;
					countPoints += value;
				}
			}
			std::cout << "Voxels active : " << activeVoxels << std::endl;
			std::cout << "Counted points in all active voxels: " << countPoints << std::endl;


			return voxelsVec;
	}
	catch (std::exception e) {
		std::cout << "Exception error: " << e.what() << std::endl;
		return std::nullopt;
	}

	return std::nullopt;
}

bool SR::Voxelization::processOnCpu()
{
	return false;
}


void SR::Voxelization::checkIfGpuIsAvailable()
{
	try {
		this->isGpuAvailable = false;
		cl::Platform::get(&platforms);

		for (const auto& platform : platforms) {
			std::vector<cl::Device> devices;
			platform.getDevices(CL_DEVICE_TYPE_GPU, &devices);

			if (!devices.empty()) {
				device = devices[0];
				context = cl::Context(device);
				this->isGpuAvailable = true;
				break;
			}

		}

		if (this->isGpuAvailable == false) {
			std::cout << "Couldn't find gpu!" << std::endl;
			return;
		}

		// 100x 100 x 100
		kernelCode =
			R"(
		void kernel voxelizationKernel(global float3* points, global uchar* voxels, int numPoints, float voxelSize) {
			size_t id = get_global_id(0);
			float3 point = points[id];
			size_t voxelXIndex = (size_t)((point.x + 1.0f) / voxelSize);

			if(voxelXIndex > 999){
				voxelXIndex = 999;			
			}
			size_t voxelYIndex = (size_t)((point.y + 1.0f) / voxelSize);
			if(voxelYIndex > 999){
				voxelYIndex = 999;			
			}
			size_t voxelZIndex = (size_t)((point.z + 1.0f) / voxelSize);
			if(voxelZIndex > 999){
				voxelZIndex = 999;			
			}
			size_t voxelIndex = voxelXIndex + voxelYIndex * 1000 + voxelZIndex * 1000000;
			voxels[voxelIndex]=1;
		})";

		sources.push_back({ kernelCode.c_str(), kernelCode.length() });
		program = std::make_unique<cl::Program>(context, sources);
		auto error = program->build("-cl-std=CL3.0");
		if (error) {
			std::cout << "Program build failed ! " << error << std::endl;
		}
		queue = std::make_unique < cl::CommandQueue>(context, device);

	}
	catch (std::exception exception) {
		std::cout << "Exception error: " << exception.what() << std::endl;

		this->isGpuAvailable = false;
	}
}

void SR::Voxelization::printGpuDetails()
{
	try {
		std::vector<cl::Platform> platforms;
		cl::Platform::get(&platforms);
		cl::Context context;
		cl::Device device;

		size_t j = 0u;
		for (const auto& platform : platforms) {
			std::vector<cl::Device> devices;
			platform.getDevices(CL_DEVICE_TYPE_GPU, &devices);

			std::cout << "	Platform " << j << ":" << std::endl;
			std::cout << "  Name: " << platforms[j].getInfo<CL_PLATFORM_NAME>() << std::endl;
			std::cout << "  Vendor: " << platforms[j].getInfo<CL_PLATFORM_VENDOR>() << std::endl;
			std::cout << "  Version: " << platforms[j].getInfo<CL_PLATFORM_VERSION>() << std::endl;
			std::cout << "  Profile: " << platforms[j].getInfo<CL_PLATFORM_PROFILE>() << std::endl;
			std::cout << "  Extensions: " << platforms[j].getInfo<CL_PLATFORM_EXTENSIONS>() << std::endl;
			j++;

			for (size_t i = 0; i < devices.size(); ++i) {
				std::cout << "	Device " << i << ":" << std::endl;
				std::cout << "  Name: " << devices[i].getInfo<CL_DEVICE_NAME>() << std::endl;
				std::cout << "  Vendor: " << devices[i].getInfo<CL_DEVICE_VENDOR>() << std::endl;
				std::cout << "  Version: " << devices[i].getInfo<CL_DEVICE_VERSION>() << std::endl;
				std::cout << "  Driver Version: " << devices[i].getInfo<CL_DRIVER_VERSION>() << std::endl;
				std::cout << "  OpenCL C Version: " << devices[i].getInfo<CL_DEVICE_OPENCL_C_VERSION>() << std::endl;
				std::cout << "  Global Memory Size: " << devices[i].getInfo<CL_DEVICE_GLOBAL_MEM_SIZE>() << " bytes" << std::endl;
				std::cout << "  Max Compute Units: " << devices[i].getInfo<CL_DEVICE_MAX_COMPUTE_UNITS>() << std::endl;
				std::cout << "  Max Work Group Size: " << devices[i].getInfo<CL_DEVICE_MAX_WORK_GROUP_SIZE>() << std::endl;
				std::cout << "  Extensions: " << devices[i].getInfo<CL_DEVICE_EXTENSIONS>() << std::endl;
			}
		}
	}
	catch (std::exception exception) {
		std::cout << "Exception error: " << exception.what() << std::endl;
	}

}
