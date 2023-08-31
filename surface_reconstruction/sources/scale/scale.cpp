#include "scale.h"

#include <algorithm>
#include <iostream>
#include <execution>


SR::Scale::Scale(std::vector<cl_float3>& points) : points(points)
{
	checkIfGpuIsAvailable();
}

std::optional<std::vector<cl_float3>> SR::Scale::processOnGpu()
{
	try {
		auto validNormals = 0u;

		auto numPoints = points.size();
		auto sizePointsInBytes = numPoints * sizeof(cl_float3);

		cl::Buffer inputOutputBuffer(context, CL_MEM_READ_WRITE, sizePointsInBytes); // todo use correct cl mem flag
		auto error = queue->enqueueWriteBuffer(inputOutputBuffer, CL_TRUE, 0, sizePointsInBytes, points.data());

		if (error) {
			std::cout << "queue->enqueueWriteBuffer(inputOutputBuffer, CL_TRUE, 0, sizePointsInBytes, points_.data()); error: " << error << std::endl;
		}

		cl::Kernel kernel(*program, "scaleKernel");


	



		auto [positionMinX, positionMaxX] = std::minmax_element(std::execution::par, points.cbegin(), points.cend(), [](const auto& a, const auto& b)
			{
				if (b.x > a.x) {
					return true;
				}
				return false;
			});


		auto [positionMinY, positionMaxY] = std::minmax_element(std::execution::par, points.cbegin(), points.cend(), [](const auto& a, const auto& b)
			{
				if (b.y > a.y) {
					return true;
				}
				return false;
			});


		auto [positionMinZ, positionMaxZ] = std::minmax_element(std::execution::par, points.cbegin(), points.cend(), [](const auto& a, const auto& b)
			{
				if (b.z > a.z) {
					return true;
				}
				return false;
			});


		const float minX = positionMinX->x;
		const float maxX = positionMaxX->x;
		const float minY = positionMinY->y;
		const float maxY = positionMaxY->y;
		const float minZ = positionMinZ->z;
		const float maxZ = positionMaxZ->z;
		const float minRange = 0.0f;
		const float maxRange = 999.0f;

		std::cout << "Before scalling!\n";
		std::cout << " minX : " << minX << std::endl;
		std::cout << " maxX : " << maxX << std::endl;
		std::cout << " minY : " << minY << std::endl;
		std::cout << " maxY : " << maxY << std::endl;
		std::cout << " minZ : " << minZ << std::endl;
		std::cout << " maxZ : " << maxZ << std::endl;


		error = kernel.setArg(0, inputOutputBuffer);
		if (error) {
			std::cout << " kernel.setArg error: " << error << std::endl;
		}
		error = kernel.setArg(1, minX);
		if (error) {
			std::cout << " kernel.setArg error: " << error << std::endl;
		}
		error = kernel.setArg(2, maxX);
		if (error) {
			std::cout << " kernel.setArg error: " << error << std::endl;
		}
		error = kernel.setArg(3, minY);
		if (error) {
			std::cout << " kernel.setArg error: " << error << std::endl;
		}
		error = kernel.setArg(4, maxY);
		if (error) {
			std::cout << " kernel.setArg error: " << error << std::endl;
		}
		error = kernel.setArg(5, minZ);
		if (error) {
			std::cout << " kernel.setArg error: " << error << std::endl;
		}
		error = kernel.setArg(6, maxZ);
		if (error) {
			std::cout << " kernel.setArg error: " << error << std::endl;
		}
		error = kernel.setArg(7, minRange);
		if (error) {
			std::cout << " kernel.setArg error: " << error << std::endl;
		}
		error = kernel.setArg(8, maxRange);
		if (error) {
			std::cout << " kernel.setArg error: " << error << std::endl;
		}

		error = queue->enqueueNDRangeKernel(kernel, cl::NullRange, cl::NDRange(numPoints), cl::NullRange);
		if (error) {
			std::cout << " queue->enqueueNDRangeKernel(kernel, cl::NullRange, cl::NDRange(numPoints), cl::NullRange); error: " << error << std::endl;
		}
		error = queue->finish();
		if (error) {
			std::cout << " queue->finish(); error: " << error << std::endl;
		}

		std::vector<cl_float3> scalledPoints(numPoints);
		error = queue->enqueueReadBuffer(inputOutputBuffer, CL_TRUE, 0, numPoints * sizeof(cl_float3), scalledPoints.data());
		if (error) {
			std::cout << "queue->enqueueReadBuffer(inputOutputBuffer, CL_TRUE, 0, numPoints * sizeof(cl_float3), normalsVector.data()); error: " << error << std::endl;
		}

		std::cout << "scalled points to range :" << std::endl;


		std::cout << " minRange : " << minRange << std::endl;
		std::cout << " maxRange : " << maxRange << std::endl;

		return scalledPoints;

	}
	catch (std::exception e) {
		std::cout << "Exception error: " << e.what() << std::endl;
		return std::nullopt;
	}

	return std::nullopt;
}

bool SR::Scale::processOnCpu()
{
	return false;
}


void SR::Scale::checkIfGpuIsAvailable()
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

		kernelCode =
			R"(
    void kernel scaleKernel(global float3 *points,
               const float minX, const float maxX,
               const float minY, const float maxY,
               const float minZ, const float maxZ,
               const float newMin, const float newMax) {
        size_t id = get_global_id(0);
        
        points[id].x = ((points[id].x - minX) / (maxX - minX)) * (newMax - newMin) + newMin;
        points[id].y = ((points[id].y - minY) / (maxY - minY)) * (newMax - newMin) + newMin;
        points[id].z = ((points[id].z - minZ) / (maxZ - minZ)) * (newMax - newMin) + newMin;
    }
    )";

		sources.push_back({ kernelCode.c_str(), kernelCode.length() });
		program = std::make_unique<cl::Program>(context, sources);
		auto error = program->build("-cl-std=CL3.0");
		if (error) {
			std::cout << "program->build error: " << error << std::endl;
		}

		queue = std::make_unique < cl::CommandQueue>(context, device);

	}
	catch (std::exception exception) {
		std::cout << "Exception error: " << exception.what() << std::endl;

		this->isGpuAvailable = false;
	}
}

void SR::Scale::printGpuDetails()
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
