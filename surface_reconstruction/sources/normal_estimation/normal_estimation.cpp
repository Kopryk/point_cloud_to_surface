#include "normal_estimation.h"

#include <iostream>




SR::NormalEstimation::NormalEstimation(std::vector<cl_float3>& points) : points(points)
{
	printGpuDetails();
	checkIfGpuIsAvailable();
}

std::optional<std::vector<cl_float3>> SR::NormalEstimation::processOnGpu()
{
	try {
		auto validNormals = 0u;

		while (validNormals == 0) {


			auto numPoints = points.size();
			auto sizePointsInBytes = numPoints * sizeof(cl_float3);

			cl::Buffer inputBuffer(context, CL_MEM_READ_WRITE, sizePointsInBytes);
			cl::Buffer outputBuffer(context, CL_MEM_WRITE_ONLY, numPoints * sizeof(cl_float3));
			auto error = queue->enqueueWriteBuffer(inputBuffer, CL_TRUE, 0, sizePointsInBytes, points.data());

			if (error) {
				std::cout << "queue->enqueueWriteBuffer(inputBuffer, CL_TRUE, 0, sizePointsInBytes, points_.data()); error: " << error << std::endl;
			}

			cl::Kernel kernel(*program, "calculateNormalVectors");
			kernel.setArg(0, static_cast<cl_uint>(numPoints));
			kernel.setArg(1, radius);
			kernel.setArg(2, inputBuffer);
			kernel.setArg(3, outputBuffer);

			error = queue->enqueueNDRangeKernel(kernel, cl::NullRange, cl::NDRange(numPoints), cl::NullRange);
			if (error) {
				std::cout << " queue->enqueueNDRangeKernel(kernel, cl::NullRange, cl::NDRange(numPoints), cl::NullRange); error: " << error << std::endl;
			}
			error = queue->finish();
			if (error) {
				std::cout << " queue->finish(); error: " << error << std::endl;
			}

			std::vector<cl_float3> normalsVector(numPoints);
			error = queue->enqueueReadBuffer(outputBuffer, CL_TRUE, 0, numPoints * sizeof(cl_float3), normalsVector.data());
			if (error) {
				std::cout << "queue->enqueueReadBuffer(outputBuffer, CL_TRUE, 0, numPoints * sizeof(cl_float3), normalsVector.data()); error: " << error << std::endl;
			}

			std::cout << "Normals computed using fixed radius:" << std::endl;

			for (const auto& normal : normalsVector) {
				std::cout << normal.s[0] << " " << normal.s[1] << " " << normal.s[2] << std::endl;

				if (normal.s[0] != 0.0f || normal.s[1] != 0.0f || normal.s[2] != 0.0f) {
					validNormals++;
				}

			}


			if (validNormals > numPoints / 2) {
				return normalsVector;
			}

			// auto balance radius
			// TODO - find a way to chose a good radius based on point cloud
			radius *= 10;

		}



	}
	catch (std::exception e) {
		std::cout << "Exception error: " << e.what() << std::endl;
		return std::nullopt;
	}

	return std::nullopt;
}

bool SR::NormalEstimation::processOnCpu()
{
	return false;
}


void SR::NormalEstimation::checkIfGpuIsAvailable()
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
			R"(void kernel calculateNormalVectors( const unsigned int numPoints, const float radius, const global float3 *inputPoints, global float3 *outputNormalVectors) {
					size_t globalId = get_global_id(0);
					//printf("%zu \n",globalId);
					float3 normalVector = (float3)(0.0f, 0.0f, 0.0f);
					unsigned int nCorrectNeighbors = 0;
					for (size_t i = 0; i < numPoints; ++i) {
						if (i != globalId) {
							float3 difference = inputPoints[i] - inputPoints[globalId];
							float distanceDotSquared = dot(difference, difference);
							float radiusSquared = radius * radius;

							#printf("distanceDotSquared=%.7f \n", distanceDotSquared);
							#printf("radiusSquared=%.7f \n", radiusSquared);

							if (distanceDotSquared <= radiusSquared) {
								normalVector += difference;
								nCorrectNeighbors++;
							}
						}
				    }

				    if (nCorrectNeighbors >= 2) {
				        outputNormalVectors[globalId] = normalize(normalVector);
				    } else {
				        outputNormalVectors[globalId] = (float3)(0.0f, 0.0f, 0.0f);
				    }
				 })";

		sources.push_back({ kernelCode.c_str(), kernelCode.length() });
		program = std::make_unique<cl::Program>(context, sources);
		program->build("-cl-std=CL3.0");
		queue = std::make_unique < cl::CommandQueue>(context, device);

	}
	catch (std::exception exception) {
		std::cout << "Exception error: " << exception.what() << std::endl;

		this->isGpuAvailable = false;
	}
}

void SR::NormalEstimation::printGpuDetails()
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
