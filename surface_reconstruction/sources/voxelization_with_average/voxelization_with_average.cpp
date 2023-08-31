#include "voxelization_with_average.h"

#include <iostream>




SR::VoxelizationWithAverage::VoxelizationWithAverage(std::vector<cl_float3>& points, std::vector<cl_float3>& normals, std::vector<cl_float3>& voxelsWithNormals, uint32_t gridSizeX, uint32_t gridSizeY, uint32_t gridSizeZ) : points(points), normals(normals), voxelsWithNormals(voxelsWithNormals), gridSizeX(gridSizeX), gridSizeZ(gridSizeZ), gridSizeY(gridSizeY)
{

	checkIfGpuIsAvailable();
}

std::optional<std::vector<cl_float3>> SR::VoxelizationWithAverage::processOnGpu()
{
	try {
		auto numPoints = points.size();
		auto sizePointsInBytes = numPoints * sizeof(cl_float3);

		auto voxelDim = gridSizeX;
		size_t nVoxels = gridSizeX * gridSizeY * gridSizeZ;
		auto voxelsNumber = nVoxels;
		auto voxelsInBytes = nVoxels * sizeof(uint32_t);


		uint32_t maxVoxelIndexX = gridSizeX-1;
		uint32_t maxVoxelIndexY = gridSizeY-1;
		uint32_t maxVoxelIndexZ = gridSizeZ-1;

		auto voxelSize = 1000.0f / voxelDim;		// 2.0f is hardcoded for now but its abs(-1) + 1 = range  = 2.0

		cl::Buffer pointCloudCl(context, CL_MEM_READ_WRITE, sizePointsInBytes);
		auto error = queue->enqueueWriteBuffer(pointCloudCl, CL_TRUE, 0, sizePointsInBytes, points.data());
		if (error) {
			std::cout << "queue->enqueueWriteBuffer(inputBuffer, CL_TRUE, 0, sizePointsInBytes, points_.data()); error: " << error << std::endl;
		}


		cl::Buffer voxelIndicesToPointCloudCl(context, CL_MEM_READ_WRITE, voxelsInBytes);
		error = queue->enqueueFillBuffer(voxelIndicesToPointCloudCl, invalidIndex, 0, voxelsInBytes);
		if (error) {
			std::cout << "queue->enqueueFillBuffer(voxelIndicesToPointCloudCl, CL_TRUE, 0, sizePointsInBytes, points_.data()); error: " << error << std::endl;
		}


		// createVoxels(global float3* points, global uint* voxels, int numPoints, float voxelSize, uint maxVoxelIndexX, uint maxVoxelIndexY, uint maxVoxelIndexZ)

		cl::Kernel kernel(*program, "createVoxels");
		error = kernel.setArg(0, pointCloudCl);
		if (error) {
			std::cout << "  kernel.setArg(0, pointCloudCl) error: " << error << std::endl;
		}
		error = kernel.setArg(1, voxelIndicesToPointCloudCl);
		if (error) {
			std::cout << "  kernel.setArg(1, voxelIndicesToPointCloudCl) error: " << error << std::endl;
		}
		error = kernel.setArg(2, static_cast<int>(numPoints));
		if (error) {
			std::cout << "  kernel.setArg(2, numPoints) error: " << error << std::endl;
		}
		error = kernel.setArg(3, voxelSize);
		if (error) {
			std::cout << "  kernel.setArg(3, voxelSize) error: " << error << std::endl;
		}
		error = kernel.setArg(4, maxVoxelIndexX);
		if (error) {
			std::cout << "  kernel.setArg(4, maxVoxelIndexX) error: " << error << std::endl;
		}
		error = kernel.setArg(5, maxVoxelIndexY);
		if (error) {
			std::cout << "  kernel.setArg(5, maxVoxelIndexY) error: " << error << std::endl;
		}		
		error = kernel.setArg(6, maxVoxelIndexZ);
		if (error) {
			std::cout << "  kernel.setArg(6, maxVoxelIndexZ) error: " << error << std::endl;
		}


		error = queue->enqueueNDRangeKernel(kernel, cl::NullRange, cl::NDRange(numPoints), cl::NullRange);
		if (error) {
			std::cout << " queue->enqueueNDRangeKernel(kernel, cl::NullRange, cl::NDRange(numPoints), cl::NullRange); error: " << error << std::endl;
		}
		error = queue->finish();
		if (error) {
			std::cout << " queue->finish(); error: " << error << std::endl;
		}

		
		voxelsWtinIndices.resize(nVoxels);
		error = queue->enqueueReadBuffer(voxelIndicesToPointCloudCl, CL_TRUE, 0, voxelsInBytes, voxelsWtinIndices.data());
		if (error) {
			std::cout << "queue->enqueueReadBuffer(outputBuffer, CL_TRUE, 0, numPoints * sizeof(cl_float3), normalsVector.data()); error: " << error << std::endl;
		}


		auto activeVoxels = 0u;
		auto countPoints = 0u;
		for (auto value : voxelsWtinIndices) {
			if (value != invalidIndex) {
				activeVoxels++;
				countPoints += value;
			}
		}
		std::cout << "Voxels active : " << activeVoxels << std::endl;


		auto index = findIndexOfCentroidVoxel(voxelsWtinIndices, numPoints, xCenter, yCenter, zCenter);
		std::cout << " IndexOfCentroidVoxel: " << index << std::endl;




		// count average point in each voxel in case if there are multiple points in a voxel
		auto averagePointPerVoxelAllVoxelsSize = nVoxels * sizeof(cl_float3);
		cl::Buffer averagePointPerVoxelAllVoxelsCl(context, CL_MEM_READ_WRITE, averagePointPerVoxelAllVoxelsSize);



		// count average point in each voxel in case if there are multiple points in a voxel
		auto normalsSize= numPoints * sizeof(cl_float3);
		cl::Buffer normalsCl(context, CL_MEM_READ_WRITE, normalsSize);

		error = queue->enqueueWriteBuffer(normalsCl, CL_TRUE, 0, normalsSize, normals.data());
		if (error) {
			std::cout << "queue->enqueueWriteBuffer(normalsCl, CL_TRUE, 0, normalsSize, normals.data()); error: " << error << std::endl;
		}


		// count average point in each voxel in case if there are multiple points in a voxel
		auto normalsPerVoxelSize = nVoxels * sizeof(cl_float3);
		cl::Buffer normalsPerVoxelCl(context, CL_MEM_READ_WRITE, normalsPerVoxelSize);

		uint32_t pattern = invalidIndex;
		size_t offset = 0;



		cl::Kernel kernelAverageVoxels(*program, "createVoxelsWithAveragePoints");
		error = kernelAverageVoxels.setArg(0, pointCloudCl);
		if (error) {
			std::cout << "  kernel.setArg(0, pointCloudCl) error: " << error << std::endl;
		}
		error = kernelAverageVoxels.setArg(1, voxelIndicesToPointCloudCl);
		if (error) {
			std::cout << "  kernel.setArg(1, voxelIndicesToPointCloudCl) error: " << error << std::endl;
		}
		error = kernelAverageVoxels.setArg(2, static_cast<int>(numPoints));
		if (error) {
			std::cout << "  kernel.setArg(2, numPoints) error: " << error << std::endl;
		}
		error = kernelAverageVoxels.setArg(3, voxelSize);
		if (error) {
			std::cout << "  kernel.setArg(3, voxelSize) error: " << error << std::endl;
		}
		error = kernelAverageVoxels.setArg(4, gridSizeX);
		if (error) {
			std::cout << "  kernel.setArg(4, gridSizeX) error: " << error << std::endl;
		}
		error = kernelAverageVoxels.setArg(5, gridSizeY);
		if (error) {
			std::cout << "  kernel.setArg(5, gridSizeY) error: " << error << std::endl;
		}
		error = kernelAverageVoxels.setArg(6, gridSizeZ);
		if (error) {
			std::cout << "  kernel.setArg(6, gridSizeZ) error: " << error << std::endl;
		}
		error = kernelAverageVoxels.setArg(7, invalidIndex);
		if (error) {
			std::cout << "  kernel.setArg(7, invalidIndex) error: " << error << std::endl;
		}
		error = kernelAverageVoxels.setArg(8, averagePointPerVoxelAllVoxelsCl);
		if (error) {
			std::cout << "  kernel.setArg(8,  averagePointPerVoxelAllVoxelsCl) error: " << error << std::endl;
		}
		error = kernelAverageVoxels.setArg(9, normalsCl);
		if (error) {
			std::cout << "  kernel.setArg(9,  normalsCl) error: " << error << std::endl;
		}
		error = kernelAverageVoxels.setArg(10, normalsPerVoxelCl);
		if (error) {
			std::cout << "  kernel.setArg(8,  averagePointPerVoxelAllVoxelsCl) error: " << error << std::endl;
		}

		

		cl::NDRange globalSize(gridSizeX, gridSizeY, gridSizeZ);
		error = queue->enqueueNDRangeKernel(kernelAverageVoxels, cl::NullRange, globalSize);
		if (error) {
			std::cout << "queue->enqueueNDRangeKernel(kernelAverageVoxels, cl::NullRange, globalSize); error: " << error << std::endl;
		}
		error = queue->finish();
		if (error) {
			std::cout << " queue->finish(); error: " << error << std::endl;
		}

		std::vector<cl_float3> voxelsWithAveragePoints(nVoxels);
		auto voxelsWithAveragePointsSize = sizeof(cl_float3) * nVoxels;
		error = queue->enqueueReadBuffer(voxelIndicesToPointCloudCl, CL_TRUE, 0, voxelsWithAveragePointsSize, voxelsWithAveragePoints.data());
		if (error) {
			std::cout << "queue->enqueueReadBuffer(voxelIndicesToPointCloudCl, CL_TRUE, 0, voxelsWithAveragePointsSize, voxelsWithAveragePoints.data()); error: " << error << std::endl;
		}


		auto voxelsWithNormalsSize = sizeof(cl_float3) * nVoxels;
		error = queue->enqueueReadBuffer(normalsPerVoxelCl, CL_TRUE, 0, voxelsWithNormalsSize, voxelsWithNormals.data());
		if (error) {
			std::cout << "queue->enqueueReadBuffer(normalsPerVoxelCl, CL_TRUE, 0, voxelsWithNormalsSize, voxelsWithNormals.data()); error: " << error << std::endl;
		}


		error = queue->finish();
		if (error) {
			std::cout << " queue->finish(); error: " << error << std::endl;
		}
		return voxelsWithAveragePoints;

	}
	catch (std::exception e) {
		std::cout << "Exception error: " << e.what() << std::endl;
	
		return std::nullopt;
	}
}

bool SR::VoxelizationWithAverage::processOnCpu()
{
	return false;
}

void SR::VoxelizationWithAverage::checkIfGpuIsAvailable()
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

		// 32x32x32 grid for now
		kernelCode =
			R"(
		void kernel createVoxels(global float3* points, global uint* voxels, int numPoints, float voxelSize, uint maxVoxelIndexX, uint maxVoxelIndexY, uint maxVoxelIndexZ) {
			size_t id = get_global_id(0);
			float3 point = points[id];
			size_t voxelXIndex =point.x / voxelSize;

			if(voxelXIndex > maxVoxelIndexX){
				printf("id = %u, point.x = %.6f\n", (unsigned int)id, point.x);
				voxelXIndex = 	maxVoxelIndexX;	
			}
			size_t voxelYIndex = point.y / voxelSize;
			if(voxelYIndex > maxVoxelIndexY){
				printf("id = %u, point.y = %.6f\n",  (unsigned int)id, point.y);
				voxelYIndex = 	maxVoxelIndexY;			
			}
			size_t voxelZIndex = point.z / voxelSize;
			if(voxelZIndex > maxVoxelIndexZ){
				printf("id = %u, point.z = %.6f\n",  (unsigned int)id, point.z);
				voxelZIndex = 	maxVoxelIndexZ;			
			}
			size_t voxelIndex = voxelXIndex + voxelYIndex * (maxVoxelIndexX+1) + voxelZIndex * (maxVoxelIndexX+1) * (maxVoxelIndexY +1);
			voxels[voxelIndex]=(uint)(id);
		}

		void kernel createVoxelsWithAveragePoints(global float3* points, global uint* voxels, uint numPoints, float voxelSize,
								 int GRID_SIZE_X, int GRID_SIZE_Y, int GRID_SIZE_Z,
								 uint invalidVoxelValue, global float3* voxelsAveragePoints, global float3 *normalVectors, global float3 *normalVectorsPerVoxel) {
			size_t x = get_global_id(0);
			size_t y = get_global_id(1);
			size_t z = get_global_id(2);
			

			int index = x + y * GRID_SIZE_X + z * GRID_SIZE_X * GRID_SIZE_Y;
			
			voxelsAveragePoints[index].x=0.0f;
			voxelsAveragePoints[index].y=0.0f;
			voxelsAveragePoints[index].z=0.0f;


			float3 normal;
			normal.x = 0.0f;
			normal.y = 0.0f;
			normal.z = 0.0f;


			normalVectorsPerVoxel[index]=normal;

			if (voxels[index] == invalidVoxelValue){
				return;
			}

			float voxelStartX = x*voxelSize;
			float voxelStartY = y*voxelSize;
			float voxelStartZ = z*voxelSize;

			float voxelEndX = voxelStartX + voxelSize;
			float voxelEndY = voxelStartY + voxelSize;
			float voxelEndZ = voxelStartZ + voxelSize;
	

			int nPointsInsideVoxel=0;

			for(int i=0; i< numPoints; i++)
			{
				float3 position = points[i];
				
				// check if inside voxel
				if( (position.x >= voxelStartX && position.x <= voxelEndX) &&
					(position.y >= voxelStartY && position.y <= voxelEndY) &&
					(position.z >= voxelStartZ && position.z <= voxelEndZ))
				{
					voxelsAveragePoints[index]+=position;	// check if it works
					normal+=normalVectors[i];
					nPointsInsideVoxel++;
				}
			}

			if(nPointsInsideVoxel == 0)
			{
				printf("Shouldn't be 0 points inside this voxel!");
				return;
			}
	
			voxelsAveragePoints[index].x = voxelsAveragePoints[index].x / nPointsInsideVoxel;
			voxelsAveragePoints[index].y = voxelsAveragePoints[index].y / nPointsInsideVoxel;
			voxelsAveragePoints[index].z = voxelsAveragePoints[index].z / nPointsInsideVoxel;

			
			normal.x = normal.x / nPointsInsideVoxel;
			normal.y = normal.y / nPointsInsideVoxel;
			normal.z = normal.z / nPointsInsideVoxel;

			normalVectorsPerVoxel[index] = normal;

		}

		)";

		sources.push_back({ kernelCode.c_str(), kernelCode.length() });
		program = std::make_unique<cl::Program>(context, sources);
		auto error = program->build("-cl-std=CL3.0");
		if (error) {
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

uint32_t SR::VoxelizationWithAverage::findIndexOfCentroidVoxel(std::vector<uint32_t>& voxelsVec, size_t nPoints, uint32_t& xCenter, uint32_t& yCenter, uint32_t& zCenter)
{
	uint64_t xSum = 0;
	uint64_t ySum = 0;
	uint64_t zSum = 0;

	auto iVoxels = 0u;

	for (auto x = 0u; x < gridSizeX; x++) {
		for (auto y = 0u; y < gridSizeY; y++) {
			for (auto z = 0u; z < gridSizeZ; z++) {

				auto indexInVoxel = x + (y * gridSizeX) + (z * gridSizeX * gridSizeY);
				auto pointIndex = voxelsVec[indexInVoxel];

				bool isOccupied = pointIndex != invalidIndex;
				if (isOccupied) {

					iVoxels++;

					xSum += x;
					ySum += y;
					zSum += z;
				}
			}
		}
	}

	uint32_t xCentroidVoxel = static_cast<uint32_t>(xSum / iVoxels);
	uint32_t yCentroidVoxel = static_cast<uint32_t>(ySum / iVoxels);
	uint32_t zCentroidVoxel = static_cast<uint32_t>(zSum / iVoxels);

	xCenter = xCentroidVoxel;
	yCenter = yCentroidVoxel;
	zCenter = zCentroidVoxel;


	std::cout << "xCentroidVoxel: " << xCentroidVoxel << std::endl;
	std::cout << "yCentroidVoxel: " << yCentroidVoxel << std::endl;
	std::cout << "zCentroidVoxel: " << zCentroidVoxel << std::endl;


	auto indexCentroidVoxel = xCentroidVoxel + (yCentroidVoxel * gridSizeX) + (zCentroidVoxel * gridSizeX * gridSizeY);

	this->indexOfCentroidVoxel = indexCentroidVoxel;

	std::cout << "indexOfCentroidVoxel: " << indexOfCentroidVoxel << std::endl;

	return indexCentroidVoxel;
}

void SR::VoxelizationWithAverage::printGpuDetails()
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
