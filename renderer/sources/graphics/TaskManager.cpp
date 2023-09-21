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

void TaskManager::startSurfaceReconstruction(std::vector<Vertex4<float>>* data, bool useGridFilter, double gridSizeInPercent, double neighbourRangeInPercent) {
	std::lock_guard<std::mutex> lock(mtx);

	if (jobRunning == false) {
		jobDone = false;
		jobRunning = true;
		worker = std::thread(&TaskManager::taskSurfaceReconstruction, this, data, useGridFilter, gridSizeInPercent, neighbourRangeInPercent);
		worker.detach();
	}
}

bool TaskManager::isJobDone() {
	std::lock_guard<std::mutex> lock(mtx);
	return jobDone;
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

void TaskManager::taskSurfaceReconstruction(TaskManager* taskManager, std::vector<Vertex4<float>>* data, bool useGridFilter, double gridSizeInPercent, double neighbourRangeInPercent) {

	taskManager->result = taskManager->pcl->calculateSurface(
		*data,
		useGridFilter,
		gridSizeInPercent,
		neighbourRangeInPercent);

	std::lock_guard<std::mutex> lock(taskManager->mtx);
	taskManager->jobDone = true;
	taskManager->jobRunning = false;
}
