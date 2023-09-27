#include "divergence.h"

#include <iostream>



SR::Divergence::Divergence(std::vector<cl_float3>& normalVectors, std::vector<cl_float3>& normalsInVoxels, std::vector<uint32_t>& voxels, uint32_t invalidVoxelValue, uint32_t gridSizeX, uint32_t gridSizeY, uint32_t gridSizeZ) :
	normalVectors(normalVectors),
	normalsInVoxels(normalsInVoxels),
	voxels(voxels),
	invalidVoxelValue(std::numeric_limits<uint32_t>::max()),
	gridSizeX(gridSizeX),
	gridSizeY(gridSizeY),
	gridSizeZ(gridSizeZ)
{

	checkIfGpuIsAvailable();
}

std::optional<std::vector<float>> SR::Divergence::processOnGpu()
{
	try {

		//void kernel calculateDivergence(global float* divergence,
		//	global float3 * normalVectors,
		//	global uint * voxels,
		//	int GRID_SIZE_X, int GRID_SIZE_Y, int GRID_SIZE_Z,
		//	
		//  int r, uint invalidVoxelValue)



		size_t nVoxels = gridSizeX * gridSizeY * gridSizeZ;
		auto divergenceInBytes = nVoxels * sizeof(float);
		cl::Buffer divergencesCl(context, CL_MEM_READ_WRITE, divergenceInBytes);

		auto normalsInBytes = normalVectors.size() * sizeof(cl_float3);

		cl::Buffer normalsPerVoxelCl(context, CL_MEM_READ_WRITE, normalsInBytes);
		auto error = queue->enqueueWriteBuffer(normalsPerVoxelCl, CL_TRUE, 0, normalsInBytes, normalVectors.data());
		if (error) {
			std::cout << " queue->enqueueWriteBuffer(normalsPerVoxelCl, CL_TRUE, 0, normalsInBytes, normalVectors.data()); Error: " << error << std::endl;
		}

		auto voxelsInBytes = nVoxels * sizeof(uint32_t);
		cl::Buffer voxelsCl(context, CL_MEM_READ_WRITE, voxelsInBytes);
		 error = queue->enqueueWriteBuffer(voxelsCl, CL_TRUE, 0, voxelsInBytes, voxels.data());
		if (error) {
			std::cout << " queue->enqueueWriteBuffer(voxelsCl, CL_TRUE, 0, voxelsInBytes, voxels.data()); Error: " << error << std::endl;
		}


		//
		//
		//
		//
		/*void kernel calculateDivergence(global float* divergences,
			global float3 * normalsPerVoxel,
			global uint * voxels,
			int gridSizeX, int gridSizeY, int gridSizeZ, uint invalidVoxelValue)*/


		cl::Kernel kernel(*program, "calculateDivergence");
		 error = kernel.setArg(0, divergencesCl);
		if (error) {
			std::cout << "  kernel.setArg(0, DivergenceDistances) error: " << error << std::endl;
		}
		error = kernel.setArg(1, normalsPerVoxelCl);
		if (error) {
			std::cout << "  kernel.setArg(1, normalsPerVoxelCl) error: " << error << std::endl;
		}
		error = kernel.setArg(2, voxelsCl);
		if (error) {
			std::cout << "  kernel.setArg(2, refY) error: " << error << std::endl;
		}
		error = kernel.setArg(3, gridSizeX);
		if (error) {
			std::cout << "  kernel.setArg(3, gridSizeX) error: " << error << std::endl;
		}
		error = kernel.setArg(4, gridSizeY);
		if (error) {
			std::cout << "  kernel.setArg(4, gridSizeY) error: " << error << std::endl;
		}
		error = kernel.setArg(5, gridSizeZ);
		if (error) {
			std::cout << "  kernel.setArg(5, gridSizeZ) error: " << error << std::endl;
		}
		error = kernel.setArg(6, invalidVoxelValue);
		if (error) {
			std::cout << "  kernel.setArg(6, invalidVoxelValue) error: " << error << std::endl;
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

		std::vector<float> divergencesVector(nVoxels);
		error = queue->enqueueReadBuffer(divergencesCl, CL_TRUE, 0, divergenceInBytes, divergencesVector.data());
		if (error) {
			std::cout << "enqueueReadBuffer(divergencesCl, CL_TRUE, 0, divergenceInBytes, divergencesVector.data()); error: " << error << std::endl;
		}
		error = queue->finish();
		if (error) {
			std::cout << " queue->finish(); error: " << error << std::endl;
		}
		

		return divergencesVector;
	}
	catch (std::exception e) {
		std::cout << "Exception error: " << e.what() << std::endl;
		return std::nullopt;
	}

	return std::nullopt;
}

bool SR::Divergence::processOnCpu()
{
	return false;
}

void SR::Divergence::checkIfGpuIsAvailable()
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

		// 32x32x32 grip for now

		// normalVectors are indexed with indexes of points in point cloud
		// voxels contains index of a point inside voxel or invalidIndex
		// to get good normalVector we can check index inside voxel and use it as index for normal vector

		kernelCode = R"( 

		void kernel calculateDivergence(	global float* divergences,
										    global float3 * normalsPerVoxel,
										    global uint* voxels,
										    int gridSizeX, int gridSizeY, int gridSizeZ, uint invalidVoxelValue) 
{
		
			uint x = get_global_id(0);
			uint y = get_global_id(1);
			uint z = get_global_id(2);
		
			uint index = x + (y * gridSizeX) + (z * gridSizeX * gridSizeY);

			// skip empty voxels
			if(voxels[index] == invalidVoxelValue)
			{
				return;
			}

			float divergenceX = 0.0f;
			float divergenceY = 0.0f;
			float divergenceZ = 0.0f;
			float3 normal = normalsPerVoxel[index];


			if (x > 0 && voxels[index - 1] != invalidVoxelValue ) {
				divergenceX += normal.x - normalsPerVoxel[index - 1].x;
			}
			if (x < gridSizeX - 1 && voxels[index + 1] != invalidVoxelValue) {
				divergenceX += normalsPerVoxel[index + 1].x - normal.x;
			}


			uint yIndexA = index - gridSizeX;
			if (y > 0 && voxels[yIndexA] != invalidVoxelValue) {
				divergenceY += normal.y - normalsPerVoxel[yIndexA].y;
			}
			uint yIndexB = index + gridSizeX;
			if (y < gridSizeY - 1 && voxels[yIndexB] != invalidVoxelValue) {
				divergenceY += normalsPerVoxel[yIndexB].y - normal.y;
			}


			uint zIndexA = index - (gridSizeX * gridSizeY);
			if (z > 0 && voxels[zIndexA] != invalidVoxelValue) {
				divergenceZ += normal.z - normalsPerVoxel[zIndexA].z;
			}
			uint zIndexB = index + (gridSizeX * gridSizeY);
			if (z < gridSizeZ - 1 && voxels[zIndexB] != invalidVoxelValue) {
				divergenceZ += normalsPerVoxel[zIndexB].z - normal.z;
			}


			divergences[index] = divergenceX + divergenceY + divergenceZ;
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


void SR::Divergence::printGpuDetails()
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
