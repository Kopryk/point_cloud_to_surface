#pragma once

#include <thread>
#include <mutex>
#include <vector>

#include "../../../utils/sources/utils.h"
#include "../../../surface_reconstruction/sources/point_cloud_library/point_cloud_library.h"

class TaskManager {
public:
	TaskManager(PointCloudLibrary* pcl) : pcl(pcl) {}

	void startLoadPoints();
	void startSurfaceReconstruction(std::vector< Vertex4<float>>* data, bool useGridFilter, double gridSizeInPercent, double neighbourRangeInPercent);
	bool isJobDone();
	std::unique_ptr<PointCloudData> getResults();

	// for consumer use
	bool isLoadPointsResultReceived = true;
	bool isSurfaceReconstructionResultReceived = true;

private:

	std::unique_ptr<PointCloudData> result = nullptr;

	std::thread worker;
	PointCloudLibrary* pcl;
	std::mutex mtx;

	bool jobDone = false;
	bool jobRunning = false;

	static void taskLoadPoints(TaskManager* taskManager);
	static void taskSurfaceReconstruction(TaskManager* taskManager, std::vector< Vertex4<float>>* data, bool useGridFilter, double gridSizeInPercent, double neighbourRangeInPercent);

};
