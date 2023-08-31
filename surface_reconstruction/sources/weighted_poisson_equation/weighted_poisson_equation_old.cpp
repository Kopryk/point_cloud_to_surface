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
		error = queue->enqueueNDRangeKernel(kernel, cl::NullRange, globalSize);
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

		// 200x200x200 grip for now

		// normalVectors are indexed with indexes of points in point cloud
		// voxels contains index of a point inside voxel or invalidIndex
		// to get good normalVector we can check index inside voxel and use it as index for normal vector

		kernelCode = R"( 
	 void kernel WeightedPoissonEquation( global float* divergence,
											   global uint* voxels,
											   global float* scalarField,
											   global float* scalarFieldInput,
											   int GRID_SIZE_X, int GRID_SIZE_Y, int GRID_SIZE_Z,
											   int r, uint invalidVoxelValue) {
		int x = get_global_id(0);
		int y = get_global_id(1);
		int z = get_global_id(2);
		int index = x * GRID_SIZE_Y * GRID_SIZE_Z + y * GRID_SIZE_Z + z;
    
		// if voxel is empty
		if(voxels[index] == invalidVoxelValue)
		{
			return;
		}

		// if divergence is zero skip 
		
		const float epsilon = 1e-5;

		if (fabs(divergence[index]) < epsilon) {
				return;
		}
		
		int leftX = x-1;
		int leftIndex = leftX + y * GRID_SIZE_X + z * GRID_SIZE_X * GRID_SIZE_Y;

		bool isEmptyVoxelLeft = true;
		// boundary condition
		if(x <= 0){
				isEmptyVoxelLeft=true;
		}else{
			for(int i=0; i<r; i++){
				if(voxels[leftIndex] == invalidVoxelValue){
					leftX--;
					leftIndex = leftX + y * GRID_SIZE_X + z * GRID_SIZE_X * GRID_SIZE_Y;
					isEmptyVoxelLeft=true;
				}
				else{
					isEmptyVoxelLeft=false;
					break;
				}
				// check if we didn't cross range
				if(leftX<0){
					isEmptyVoxelLeft=true;
					break;	
				}
			}
		}

		int rightX = x+1;
		int rightIndex = rightX + y * GRID_SIZE_X + z * GRID_SIZE_X * GRID_SIZE_Y;
		// check if right voxel is not empty if not find left voxel in "r" range
		bool isEmptyVoxelRight = true;

		// boundary condition
		if(x >= GRID_SIZE_X-1){
				isEmptyVoxelRight=true;
		}else{
			for(int i=0; i<r; i++){
				if(voxels[rightIndex] == invalidVoxelValue){
					rightX++;
					rightIndex = rightX + y * GRID_SIZE_X + z * GRID_SIZE_X * GRID_SIZE_Y;
					isEmptyVoxelRight=true;
				}
				else{
					isEmptyVoxelRight=false;
					break;
				}
				// check if we didn't cross range
				if(rightX > GRID_SIZE_X-1){
					isEmptyVoxelRight=true;
					break;	
				}
			}
		}

		int topY = y-1;
		int topIndex = x + topY * GRID_SIZE_X + z * GRID_SIZE_X * GRID_SIZE_Y;

		bool isEmptyVoxelTop = true;

		// boundary condition
		if(y <= 0){
				isEmptyVoxelTop=true;
		}else{
			for(int i=0; i<r; i++){
				if(voxels[topIndex] == invalidVoxelValue){
					topY--;
					topIndex = x + topY * GRID_SIZE_X + z * GRID_SIZE_X * GRID_SIZE_Y;
					isEmptyVoxelTop=true;
				}
				else{
					isEmptyVoxelTop=false;
					break;
				}
				// check if we didn't cross range
				if(topY<0){
					isEmptyVoxelTop=true;
					break;	
				}
			}
		}
			
		int bottomY = y+1;
		int bottomIndex = x + bottomY * GRID_SIZE_X + z * GRID_SIZE_X * GRID_SIZE_Y;
		// check if right voxel is not empty if not find left voxel in "r" range
		bool isEmptyVoxelBottom = true;

		// boundary condition
		if(y >= GRID_SIZE_Y-1){
				isEmptyVoxelBottom=true;
		}else{
			for(int i=0; i<r; i++){
				if(voxels[bottomIndex] == invalidVoxelValue){
					bottomY++;
					bottomIndex = x + bottomY * GRID_SIZE_X + z * GRID_SIZE_X * GRID_SIZE_Y;
					isEmptyVoxelBottom=true;
				}
				else{
					isEmptyVoxelBottom=false;
					break;
				}
				// check if we didn't cross range
				if(bottomY > GRID_SIZE_Y-1){
					isEmptyVoxelBottom=true;
					break;	
				}
			}
		}

		int forwardZ = z-1;
		int forwardIndex = x + y * GRID_SIZE_X + forwardZ * GRID_SIZE_X * GRID_SIZE_Y;

		bool isEmptyVoxelForward = true;

		// boundary condition
		if(z <= 0){
				isEmptyVoxelForward=true;
		}else{
			for(int i=0; i<r; i++){
				if(voxels[forwardIndex] == invalidVoxelValue){
					forwardZ--;
					forwardIndex = x + y * GRID_SIZE_X + forwardZ * GRID_SIZE_X * GRID_SIZE_Y;
					isEmptyVoxelForward=true;
				}
				else{
					isEmptyVoxelForward=false;
					break;
				}
				// check if we didn't cross range
				if(forwardZ<0){
					isEmptyVoxelForward=true;
					break;	
				}
			}
		}
			
		int backwardZ = z+1;
		int backwardIndex = x + y * GRID_SIZE_X + backwardZ * GRID_SIZE_X * GRID_SIZE_Y;
		// check if right voxel is not empty if not find left voxel in "r" range
		bool isEmptyVoxelBackward = true;

		// boundary condition
		if(z >= GRID_SIZE_Z-1){
				isEmptyVoxelBackward=true;
		}else{
			for(int i=0; i<r; i++){
				if(voxels[backwardIndex] == invalidVoxelValue){
					backwardZ++;
					backwardIndex = x + y * GRID_SIZE_X + backwardZ * GRID_SIZE_X * GRID_SIZE_Y;
					isEmptyVoxelBackward=true;
				}
				else{
					isEmptyVoxelBackward=false;
					break;
				}
				// check if we didn't cross range
				if(backwardZ > GRID_SIZE_Z-1){
					isEmptyVoxelBackward=true;
					break;	
				}
			}
		}

		// gauss-Seidel iteration for solving the Weighted Poisson Equation

		int maxIterations = 100;

		for (int iter = 0; iter < maxIterations; ++iter) {
			// Check occupancy of neighboring voxels
			
			float laplacian =0.0f;
			int validScalarFields=0;     
			// compute Laplacian using neighboring values

			if(isEmptyVoxelLeft==false){
				laplacian+=scalarFieldInput[leftIndex];
				printf("index= %d, scalarFieldInput[leftIndex]= %.6f\n", index, scalarFieldInput[leftIndex]);


				validScalarFields++;
			}

			if(isEmptyVoxelRight==false){
				laplacian+=scalarFieldInput[rightIndex];
printf("index= %d, scalarFieldInput[rightIndex]= %.6f\n", index, scalarFieldInput[rightIndex]);
				validScalarFields++;
			}
			if(isEmptyVoxelForward==false){
				laplacian+=scalarFieldInput[forwardIndex];
printf("index= %d, scalarFieldInput[forwardIndex]= %.6f\n", index, scalarFieldInput[forwardIndex]);
				validScalarFields++;
			}	
			if(isEmptyVoxelBackward==false){
				laplacian+=scalarFieldInput[backwardIndex];
printf("index= %d, scalarFieldInput[backwardIndex]= %.6f\n", index, scalarFieldInput[backwardIndex]);
				validScalarFields++;
			}
			if(isEmptyVoxelBottom==false){
				laplacian+=scalarFieldInput[bottomIndex];
printf("index= %d, scalarFieldInput[bottomIndex]= %.6f\n", index, scalarFieldInput[bottomIndex]);
				validScalarFields++;
			}
			if(isEmptyVoxelTop==false){
				laplacian+=scalarFieldInput[topIndex];
printf("index= %d, scalarFieldInput[topIndex]= %.6f\n", index, scalarFieldInput[topIndex]);
				validScalarFields++;
			}
			

			if(validScalarFields <2)
			{
				return;
			}
			else{
				printf("Valid scalar fields n = %d", validScalarFields);
			}
			
			// comment
			printf("index= %d, before laplacian = %.6f\n", index, laplacian);

			laplacian =  laplacian + (-1 * validScalarFields* scalarField[index]);

			// comment
			printf("index= %d, after laplacian = %.6f\n", index, laplacian);
			printf("index= %d, scalarField[index] = %.6f\n", index, scalarField[index]);

			if(fabs(laplacian - divergence[index]) < epsilon){
				printf("Match! ");
				return;
			}

			// comment
			printf("index = %d, divergence[index] = %.6f\n", index, divergence[index]);
			printf("index = %d, before scalarField[index] = %.6f\n", index, scalarField[index]);

			scalarField[index] += divergence[index] - laplacian;

			// comment
			printf("index = %d, after scalarField[index] = %.6f\n", index, scalarField[index]);

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
