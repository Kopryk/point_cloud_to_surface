#include "scalar_field.h"

#include <iostream>


SR::ScalarField::ScalarField( float refX, float refY, float refZ, uint32_t gridSizeX, uint32_t gridSizeY, uint32_t gridSizeZ) :
	refX(refX),
	refY(refY),
	refZ(refZ),
	gridSizeX(gridSizeX),
	gridSizeY(gridSizeY),
	gridSizeZ(gridSizeZ)
{

	checkIfGpuIsAvailable();
}

std::optional<std::vector<float>> SR::ScalarField::processOnGpu()
{
	try {
		size_t nVoxels = gridSizeX * gridSizeY * gridSizeZ;
		auto scalarFieldInBytes = nVoxels * sizeof(float);
		cl::Buffer scalarFieldDistances(context, CL_MEM_READ_WRITE, scalarFieldInBytes);

		cl::Kernel kernel(*program, "calculateScalarField");
		auto error = kernel.setArg(0, scalarFieldDistances);
		if (error) {
			std::cout << "  kernel.setArg(0, scalarFieldDistances) error: " << error << std::endl;
		}
		error = kernel.setArg(1, refX);
		if (error) {
			std::cout << "  kernel.setArg(1, refX) error: " << error << std::endl;
		}
		error = kernel.setArg(2, refY);
		if (error) {
			std::cout << "  kernel.setArg(2, refY) error: " << error << std::endl;
		}
		error = kernel.setArg(3, refZ);
		if (error) {
			std::cout << "  kernel.setArg(3, refZ) error: " << error << std::endl;
		}
		error = kernel.setArg(4, gridSizeX);
		if (error) {
			std::cout << "  kernel.setArg(4, GRID_SIZE_X) error: " << error << std::endl;
		}
		error = kernel.setArg(5, gridSizeY);
		if (error) {
			std::cout << "  kernel.setArg(5, gridSizeY) error: " << error << std::endl;
		}
		error = kernel.setArg(6, gridSizeZ);
		if (error) {
			std::cout << "  kernel.setArg(6, gridSizeZ) error: " << error << std::endl;
		}

		cl::NDRange globalSize(gridSizeX, gridSizeY, gridSizeZ);
		error = queue->enqueueNDRangeKernel(kernel, cl::NullRange, globalSize);
		if (error) {
			std::cout << " queue->enqueueNDRangeKernel(kernel, cl::NullRange, globalSize); error: " << error << std::endl;
		}
		error = queue->finish();
		if (error) {
			std::cout << " queue->finish(); error: " << error << std::endl;
		}

		std::vector<float> scalarFieldsDistancesVector(nVoxels);

		error = queue->enqueueReadBuffer(scalarFieldDistances, CL_TRUE, 0, scalarFieldInBytes, scalarFieldsDistancesVector.data());
		if (error) {
			std::cout << "enqueueReadBuffer(scalarFieldDistances, CL_TRUE, 0, scalarFieldInBytes, scalarFieldsDistancesVector.data()); error: " << error << std::endl;
		}

		return scalarFieldsDistancesVector;
	}
	catch (std::exception e) {
		std::cout << "Exception error: " << e.what() << std::endl;
		return std::nullopt;
	}

	return std::nullopt;
}

bool SR::ScalarField::processOnCpu()
{
	return false;
}

void SR::ScalarField::checkIfGpuIsAvailable()
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

		// 200x200x200 grip for now

		kernelCode = R"( 
			void kernel calculateScalarField(
			global float* scalarField,
			const float refX, const float refY, const float refZ, const uint GRID_SIZE_X, const uint GRID_SIZE_Y, const uint GRID_SIZE_Z)
		{
			uint x = get_global_id(0);
			uint y = get_global_id(1);
			uint z = get_global_id(2);
		
			uint index = x + y * GRID_SIZE_X + z * GRID_SIZE_X * GRID_SIZE_Y;

			float dx = (float)x - refX;
			float dy = (float)y - refY;
			float dz = (float)z - refZ;

			float squaredDistance = dx * dx + dy * dy + dz * dz;
			float distance = sqrt(squaredDistance);

			scalarField[index] = distance;
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


void SR::ScalarField::printGpuDetails()
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
