#include "TaskManager.h"

void TaskManager::startLoadPoints() {
	std::lock_guard<std::mutex> lock(mtx);
	if (jobRunning == false) {
		jobDone = false;
		jobRunning = true;
		worker = std::thread(&TaskManager::taskLoadPoints, this);
		worker.detach();
	}
}


std::unique_ptr<PointCloudData> TaskManager::getResults() {
	return std::move(result);
}

void TaskManager::taskLoadPoints(TaskManager* taskManager) {
	taskManager->result = taskManager->pcl->loadPoints();
	std::lock_guard<std::mutex> lock(taskManager->mtx);
	taskManager->jobDone = true;
	taskManager->jobRunning = false;
}


bool TaskManager::isJobDone() {
	std::lock_guard<std::mutex> lock(mtx);
	return jobDone;
}

void TaskManager::startSurfaceReconstruction(std::vector<Vertex4<float>>* data, PointCloudOptimizationMode optimizationMode, SurfaceReconstructionMode reconstructionMode, uint32_t depthOctree, uint32_t gridResolution, double gridSizeInPercent, double neighbourRangeInPercent, float dilationVoxelSizeInPercent, uint32_t dilationIteration) {
	std::lock_guard<std::mutex> lock(mtx);
	if (jobRunning == false) {
		jobDone = false;
		jobRunning = true;
		worker = std::thread(&TaskManager::taskSurfaceReconstruction, this, data, optimizationMode, reconstructionMode, depthOctree, gridResolution, gridSizeInPercent, neighbourRangeInPercent, dilationVoxelSizeInPercent, dilationIteration);
		worker.detach();
	}
}

void TaskManager::taskSurfaceReconstruction(TaskManager* taskManager, std::vector<Vertex4<float>>* data, PointCloudOptimizationMode optimizationMode, SurfaceReconstructionMode reconstructionMode, uint32_t depthOctree, uint32_t gridResolution, double gridSizeInPercent, double neighbourRangeInPercent, float dilationVoxelSizeInPercent, uint32_t dilationIteration) {
	taskManager->result = taskManager->pcl->calculateSurface(
		*data,
		optimizationMode,
		reconstructionMode,
		depthOctree,
		gridResolution,
		gridSizeInPercent,
		neighbourRangeInPercent,
		dilationVoxelSizeInPercent,
		dilationIteration);
	std::lock_guard<std::mutex> lock(taskManager->mtx);
	taskManager->jobDone = true;
	taskManager->jobRunning = false;
}
