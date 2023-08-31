#include "weighted_poisson_equation.h"

#include <iostream>



SR::WeightedPoissonEquation::WeightedPoissonEquation(std::vector<float>& divergencesVector, std::vector<float>& scalarFieldsDistancesVector, std::vector<uint32_t>& voxels, uint32_t invalidVoxelValue, uint32_t gridSizeX, uint32_t gridSizeY, uint32_t gridSizeZ) :
	divergencesVector(divergencesVector),
	scalarFieldsDistancesVector(scalarFieldsDistancesVector),
	voxels(voxels),
	invalidVoxelValue(std::numeric_limits<uint32_t>::max()),
	gridSizeX(gridSizeX),
	gridSizeY(gridSizeY),
	gridSizeZ(gridSizeZ)
{

	checkIfGpuIsAvailable();
}

std::optional<std::vector<float>> SR::WeightedPoissonEquation::processOnGpu()
{
	try {


		int radiusForNeigherhood = 3;
		size_t nVoxels = gridSizeX * gridSizeY * gridSizeZ;

		// divergence

		auto divergenceInBytes = nVoxels * sizeof(float);
		cl::Buffer divergencesCl(context, CL_MEM_READ_WRITE, divergenceInBytes);
		auto error = queue->enqueueWriteBuffer(divergencesCl, CL_TRUE, 0, divergenceInBytes, divergencesVector.data());
		if (error) {
			std::cout << "enqueueWriteBuffer(divergencesCl, CL_TRUE, 0, divergenceInBytes, divergencesVector.data()); error: " << error << std::endl;
		}

		// scalar fields to update
		auto scalarFieldInBytes = nVoxels * sizeof(float);
		cl::Buffer scalarFieldCl(context, CL_MEM_READ_WRITE, scalarFieldInBytes);
		error = queue->enqueueWriteBuffer(scalarFieldCl, CL_TRUE, 0, scalarFieldInBytes, scalarFieldsDistancesVector.data());
		if (error) {
			std::cout << "enqueueWriteBuffer(scalarFieldCl, CL_TRUE, 0, scalarFieldInBytes, scalarFieldsDistancesVector.data()); error: " << error << std::endl;
		}

		// scalar fields input ( not updated during iterations)

		cl::Buffer scalarFieldInputCl(context, CL_MEM_READ_WRITE, scalarFieldInBytes);
		error = queue->enqueueWriteBuffer(scalarFieldInputCl, CL_TRUE, 0, scalarFieldInBytes, scalarFieldsDistancesVector.data());
		if (error) {
			std::cout << "enqueueWriteBuffer(scalarFieldInputCl, CL_TRUE, 0, scalarFieldInBytes, scalarFieldsDistancesVector.data()); error: " << error << std::endl;
		}

		// voxels
		auto voxelsInBytes = nVoxels * sizeof(uint32_t);
		cl::Buffer voxelsCl(context, CL_MEM_READ_WRITE, voxelsInBytes);
		error = queue->enqueueWriteBuffer(voxelsCl, CL_TRUE, 0, voxelsInBytes, voxels.data());
		if (error) {
			std::cout << "queue->enqueueWriteBuffer(voxelsCl, CL_TRUE, 0, voxelsInBytes, voxels.data()); error: " << error << std::endl;
		}

	



		cl::Kernel kernel(*program, "WeightedPoissonEquation");
		error = kernel.setArg(0, divergencesCl);
		if (error) {
			std::cout << "  kernel.setArg(0, divergencesCl) error: " << error << std::endl;
		}
		error = kernel.setArg(1, voxelsCl);
		if (error) {
			std::cout << "  kernel.setArg(1, voxelsCl) error: " << error << std::endl;
		}
		error = kernel.setArg(2, scalarFieldCl);
		if (error) {
			std::cout << "  kernel.setArg(2, scalarFieldCl) error: " << error << std::endl;
		}
		error = kernel.setArg(3, scalarFieldInputCl);
		if (error) {
			std::cout << "  kernel.setArg(3, scalarFieldInputCl) error: " << error << std::endl;
		}
		error = kernel.setArg(4, gridSizeX);
		if (error) {
			std::cout << "  kernel.setArg(4, gridSizeX) error: " << error << std::endl;
		}
		error = kernel.setArg(5, gridSizeY);
		if (error) {
			std::cout << "  kernel.setArg(5, gridSizeY) error: " << error << std::endl;
		}
		error = kernel.setArg(6, gridSizeZ);
		if (error) {
			std::cout << "  kernel.setArg(6, gridSizeZ) error: " << error << std::endl;
		}
		error = kernel.setArg(7, radiusForNeigherhood);
		if (error) {
			std::cout << "  kernel.setArg(6, radiusForNeigherhood) error: " << error << std::endl;
		}
		error = kernel.setArg(8, invalidVoxelValue);
		if (error) {
			std::cout << "  kernel.setArg(8, invalidVoxelValue) error: " << error << std::endl;
		}


		cl::NDRange globalSize(gridSizeX, gridSizeY, gridSizeZ);
		cl::NDRange localSize(1,1,1);
		error = queue->enqueueNDRangeKernel(kernel, cl::NullRange, globalSize, localSize);
		if (error) {
			std::cout << " queue->enqueueNDRangeKernel(kernel, cl::NullRange, globalSize); error: " << error << std::endl;
		}
		error = queue->finish();
		if (error) {
			std::cout << " queue->finish(); error: " << error << std::endl;
		}

		std::vector<float> scalarFieldsUpdated(nVoxels);

		error = queue->enqueueReadBuffer(scalarFieldCl, CL_TRUE, 0, scalarFieldInBytes, scalarFieldsUpdated.data());
		if (error) {
			std::cout << "enqueueReadBuffer(scalarFieldCl, CL_TRUE, 0, scalarFieldInBytes, scalarFieldsUpdated.data()); error: " << error << std::endl;
		}

		error = queue->finish();
		if (error) {
			std::cout << "error = queue->finish(); " << error << std::endl;
		}
		return scalarFieldsUpdated;
	}
	catch (std::exception e) {
		std::cout << "Exception error: " << e.what() << std::endl;
		return std::nullopt;
	}

	return std::nullopt;
}

bool SR::WeightedPoissonEquation::processOnCpu()
{
	return false;
}

void SR::WeightedPoissonEquation::checkIfGpuIsAvailable()
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

		// 16x16x16 grip for now


		kernelCode = R"( 
	 void kernel WeightedPoissonEquation( global float* divergence,
											   global uint* voxelMask,
											   global float* scalarField,
											   global float* scalarFieldInput,
											   int gridSizeX, int gridSizeY, int gridSizeZ,
											   int r, uint invalidVoxelValue) {

			uint x = get_global_id(0);
			uint y = get_global_id(1);
			uint z = get_global_id(2);
		
			uint index = x + (y * gridSizeX) + (z * gridSizeX * gridSizeY);

			// skip empty voxels
			if(voxelMask[index] == invalidVoxelValue)
			{
				return;
			}
    
			printf("not skipped");
			// Gauss-Seidel iteration for solving the Weighted Poisson Equation
			int maxIterations = 100;
			for (int iter = 0; iter < maxIterations; ++iter) {

				float laplacian=0.0f;
							
				uint nNonEmptyNeighbors=0;

				if(x > 0 && voxelMask[index - 1] != invalidVoxelValue)
				{
					uint i = index - 1;
					laplacian = scalarField[i];
					nNonEmptyNeighbors++;
				}
				if(x < gridSizeX - 1 && voxelMask[index+1] != invalidVoxelValue)
				{
					uint i = index +1;
					laplacian += scalarField[i];
					nNonEmptyNeighbors++;
				}

				if (y > 0 && voxelMask[index - gridSizeX] != invalidVoxelValue)
				{
					uint i = index - gridSizeX;
					laplacian += scalarField[i];
					nNonEmptyNeighbors++;
				}
				if (y < gridSizeY - 1 && voxelMask[index + gridSizeX] != invalidVoxelValue)
				{
					uint i = index + gridSizeX;
					laplacian += scalarField[i];
					nNonEmptyNeighbors++;
				}

				if (z > 0 && voxelMask[index - (gridSizeX * gridSizeY)] != invalidVoxelValue)
				{
					uint i = index - (gridSizeX * gridSizeY);
					laplacian += scalarField[i];
					nNonEmptyNeighbors++;
				}
				if (z < gridSizeZ - 1 && voxelMask[index + (gridSizeX * gridSizeY)] != invalidVoxelValue)
				{
					uint i = index + (gridSizeX * gridSizeY);
					laplacian += scalarField[i];
					nNonEmptyNeighbors++;
				}
							
				if(nNonEmptyNeighbors == 0){
					printf("Empty index = %u", (unsigned int)index);
					return;
				}
				printf("index = %u laplacian before  = %0.6f\n", (unsigned int)index, laplacian);
				printf("index = %u scalarField[index]  before  = %0.6f\n", (unsigned int)index, scalarField[index] );
				printf("index = %u nNonEmptyNeighbors before  = %u\n", (unsigned int)index, nNonEmptyNeighbors );



				laplacian+= -1 * scalarField[index] * nNonEmptyNeighbors;
            
				barrier(CLK_LOCAL_MEM_FENCE);
				float weight = 50;
				scalarField[index] += (divergence[index] - laplacian)/ weight;
				barrier(CLK_LOCAL_MEM_FENCE);

			}
		}
		
)";

		sources.push_back({ kernelCode.c_str(), kernelCode.length() });
		program = std::make_unique<cl::Program>(context, sources);
		auto error = program->build("-cl-std=CL3.0");
		if (error){
			std::string buildLog = program->getBuildInfo<CL_PROGRAM_BUILD_LOG>(device);
			std::cout << "Build error log:\n" << buildLog << std::endl;
			std::cout << "Program build failed ! " << error << std::endl;
		}
		queue = std::make_unique < cl::CommandQueue>(context, device);

	}
	catch (std::exception exception) {
		std::cout << "Exception error: " << exception.what() << std::endl;

		this->isGpuAvailable = false;
	}
}


void SR::WeightedPoissonEquation::printGpuDetails()
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
