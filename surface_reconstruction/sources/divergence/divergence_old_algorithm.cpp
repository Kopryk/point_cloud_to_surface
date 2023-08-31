#include "divergence.h"

#include <iostream>



SR::Divergence::Divergence(std::vector<cl_float3>& normalVectors, std::vector<uint32_t>& voxels, uint32_t invalidVoxelValue, uint32_t gridSizeX, uint32_t gridSizeY, uint32_t gridSizeZ) :
	normalVectors(normalVectors),
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

		cl::Buffer normals(context, CL_MEM_READ_WRITE, normalsInBytes);
		auto error = queue->enqueueWriteBuffer(normals, CL_TRUE, 0, normalsInBytes, normalVectors.data());
		if (error) {
			std::cout << " queue->enqueueWriteBuffer(normals, CL_TRUE, 0, normalsInBytes, normalVectors.data()); Error: " << error << std::endl;
		}

		auto voxelsInBytes = nVoxels * sizeof(uint32_t);
		cl::Buffer voxelsCl(context, CL_MEM_READ_WRITE, voxelsInBytes);
		 error = queue->enqueueWriteBuffer(voxelsCl, CL_TRUE, 0, voxelsInBytes, voxels.data());
		if (error) {
			std::cout << " queue->enqueueWriteBuffer(voxelsCl, CL_TRUE, 0, voxelsInBytes, voxels.data()); Error: " << error << std::endl;
		}

		int radiusForNeigherhood = 3;

		cl::Kernel kernel(*program, "calculateDivergence");
		 error = kernel.setArg(0, divergencesCl);
		if (error) {
			std::cout << "  kernel.setArg(0, DivergenceDistances) error: " << error << std::endl;
		}
		error = kernel.setArg(1, normals);
		if (error) {
			std::cout << "  kernel.setArg(1, normals) error: " << error << std::endl;
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
		error = kernel.setArg(6, radiusForNeigherhood);
		if (error) {
			std::cout << "  kernel.setArg(6, radiusForNeigherhood) error: " << error << std::endl;
		}
		error = kernel.setArg(7, invalidVoxelValue);
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

		// 200x200x200 grip for now

		// normalVectors are indexed with indexes of points in point cloud
		// voxels contains index of a point inside voxel or invalidIndex
		// to get good normalVector we can check index inside voxel and use it as index for normal vector

		kernelCode = R"( 

		void kernel calculateDivergence(global float* divergence,
										   global float3 * normalVectors,
										   global uint* voxels,
										   int GRID_SIZE_X, int GRID_SIZE_Y, int GRID_SIZE_Z,
										   int r, uint invalidVoxelValue) 
{

			
			int x = get_global_id(0);
			int y = get_global_id(1);
			int z = get_global_id(2);
		
			int index = x + y * GRID_SIZE_X + z * GRID_SIZE_X * GRID_SIZE_Y;

			// skip empty voxels
			if(voxels[index] == invalidVoxelValue)
			{
				return;
			}
    
			int indexInNormalVector = voxels[index];

			float3 normal = normalVectors[indexInNormalVector];
			float dNx_dx = 0.0f;
			float dNy_dy = 0.0f;
			float dNz_dz = 0.0f;
    
			// find left and right voxels which are not empty
			// check if left voxel is not empty if not find left voxel in "r" range
			
			//
			//
			//	left, right directions
			//
			//

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

			uint indexNormalVectorLeft = voxels[leftIndex];
			uint indexNormalVectorRight = voxels[rightIndex];


			if((isEmptyVoxelLeft==false) && (isEmptyVoxelRight==false)){
				dNx_dx = (normalVectors[indexNormalVectorRight].x - normalVectors[indexNormalVectorLeft].x) * 0.5f; // Central finite differences
			}
			else if(isEmptyVoxelLeft == false){	// if left voxel is not empty 
				dNx_dx = (normal.x - normalVectors[indexNormalVectorLeft].x); // Backward finite differences
			}
			else if(isEmptyVoxelRight == false){// if right voxel is not empty 
				dNx_dx = (normalVectors[indexNormalVectorRight].x - normal.x); // Forward finite differences
			}
			else{
				dNx_dx=0.0f;

			}


			//
			//
			//	top, bottom directions
			//
			//

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


			uint indexNormalVectorBottom = voxels[bottomIndex];
			uint indexNormalVectorTop = voxels[topIndex];
			
			if((isEmptyVoxelBottom==false) && (isEmptyVoxelTop==false)){
				dNy_dy = (normalVectors[indexNormalVectorBottom].y - normalVectors[indexNormalVectorTop].y) * 0.5f; // Central finite differences
			}
			else if(isEmptyVoxelTop == false){
				dNy_dy = (normal.y - normalVectors[indexNormalVectorTop].y); // Backward finite differences
			}
			else if(isEmptyVoxelBottom == false){// if right voxel is not empty 
				dNy_dy = (normalVectors[indexNormalVectorBottom].y - normal.y); // Forward finite differences
			}
			else{
				dNy_dy=0.0f;
			}

			// END TOP/BOTTOM

			
			//
			//
			//	forward, backward directions
			//
			//

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

			
			uint indexNormalVectorForward = voxels[forwardIndex];
			uint indexNormalVectorBackward = voxels[backwardIndex];
			

			if((isEmptyVoxelForward==false) && (isEmptyVoxelBackward==false)){
				dNz_dz = (normalVectors[indexNormalVectorBackward].z - normalVectors[indexNormalVectorForward].z) * 0.5f; // Central finite differences
			}
			else if(isEmptyVoxelBackward == false){
				dNz_dz = (normal.z - normalVectors[indexNormalVectorBackward].z); // Backward finite differences
			}
			else if(isEmptyVoxelForward == false){//
				dNz_dz = (normalVectors[indexNormalVectorForward].z - normal.z); // Forward finite differences
			}
			else{
				dNz_dz =0.0f;
			}

			// END backward/forward

			printf("%.6f\n", dNx_dx);
			printf("%.6f\n", dNy_dy);
			printf("%.6f\n", dNz_dz);
			divergence[index] = dNx_dx + dNy_dy + dNz_dz;
			
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
