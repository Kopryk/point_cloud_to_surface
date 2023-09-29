#include "point_cloud_library.h"
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h> 
#include <pcl/surface/poisson.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/surface/mls.h>


#include "../../external/nativefiledialog/src/include/nfd.h"


uint32_t PointCloudLibrary::countLines(const std::string& filename) {
	std::ifstream inFile(filename);
	if (!inFile.is_open()) {
		std::cerr << "Failed to open the file: " << filename << std::endl;
		return 0;
	}

	uint32_t count = 0;
	std::string line;
	while (std::getline(inFile, line)) {
		++count;
	}

	return count;
}


std::unique_ptr<PointCloudData> PointCloudLibrary::loadPoints()
{
	nfdchar_t* outPath = NULL;
	nfdresult_t result = NFD_OpenDialog(NULL, NULL, &outPath);
	std::string pathPoints;

	if (result == NFD_OKAY) {
		printf("Success!\nPath: %s\n", outPath);
		pathPoints = outPath;
		free(outPath);
	}
	else if (result == NFD_CANCEL) {
		printf("User pressed cancel.\n");
		return {};
	}
	else {
		printf("Error: %s\n", NFD_GetError());
		return {};
	}

	auto nLines = countLines(pathPoints);

	std::ifstream file(pathPoints);
	if (!file.is_open()) {
		std::cerr << "Error opening file " << pathPoints << std::endl;
		return {};
	}

	auto data = std::make_unique< PointCloudData>();

	data->containsPointsWithColors = false;
	data->containsPoints = true;
	if (pathPoints.find("_klasyfikacja_kolory") != std::string::npos) {
		data->containsPointsWithColors = true;
	}


	auto numberOfPoints = nLines;
	//file >> numberOfPoints;

	std::string line;
	std::getline(file, line);  // consume newline after reading numberOfPoints

	int i = 0;
	if (data->containsPointsWithColors == false) {

		data->buildings.points.reserve(numberOfPoints);

		while (std::getline(file, line)) {
			std::istringstream iss(line);
			Vertex4<float> point(0, 0, 0, 0);
			if (iss >> point.x >> point.y >> point.z) {
				data->buildings.points.push_back(point);
			}
		}
	}
	else {

		data->environment.points.reserve(numberOfPoints);
		data->environment.colors.reserve(numberOfPoints);
		data->buildings.points.reserve(numberOfPoints / 100);
		data->buildings.colors.reserve(numberOfPoints / 100);

		while (std::getline(file, line)) {
			std::istringstream iss(line);
			Vertex4<float> point(0, 0, 0, 0);
			int className = 0;
			float r, g, b;
			if (iss >> point.x >> point.y >> point.z >> className >> r >> g >> b) {

				constexpr int buildingClass = 6;

				// colors read from furgeviewer are in uint16format, need to conver it to float

				r = r / std::numeric_limits<uint16_t>::max();
				g = g / std::numeric_limits<uint16_t>::max();
				b = b / std::numeric_limits<uint16_t>::max();

				if (className == buildingClass) {
					data->buildings.points.push_back(point);
					data->buildings.colors.push_back({ r,g,b,1.0f });
				}
				else {
					data->environment.points.push_back(point);
					data->environment.colors.push_back({ r,g,b,1.0f });
				}

			}
		}
	}

	if (data->buildings.points.size() == 0) {
		return {};
	}

	static float globalX = data->buildings.points[0].x;
	static float globalY = data->buildings.points[0].y;
	static float globalZ = data->buildings.points[0].z;

	if (data->environment.points.size() != 0) {
		for (auto& point : data->environment.points) {
			point.x -= globalX;
			point.y -= globalY;
			point.z -= globalZ;
		}
	}


	for (auto& point : data->buildings.points) {
		point.x -= globalX;
		point.y -= globalY;
		point.z -= globalZ;
	}

	return data;

}

std::unique_ptr<PointCloudData>  PointCloudLibrary::calculateSurface(std::vector <Vertex4<float>>& pointCloud, PointCloudOptimizationMode optimizationMode, SurfaceReconstructionMode reconstructionMode, uint32_t depthOctree, uint32_t gridResolution, uint32_t kNeigborsForNormals, double isoValue, double percentageExtendGrid, double gridSizeInPercent, double neighbourRangeInPercent, double neighbourMLSRangeInPercent, float dilationVoxelSizeInPercent, uint32_t dilationIteration)
{
	// gridSizeInPercent = 0.0 - 1.0
	// neighbourRangeInPercent = 0.0 - 1.0


	std::cout << std::endl;
	std::cout << "Input point cloud size = " << pointCloud.size() << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());


	float minX = std::numeric_limits<float>::max();
	float maxX = std::numeric_limits<float>::lowest();
	float minY = minX;
	float maxY = maxX;
	float minZ = minX;
	float maxZ = maxX;

	auto surface = std::make_unique< PointCloudData>();

	for (const auto& point : pointCloud) {

		minX = std::min(minX, point.x);
		maxX = std::max(maxX, point.x);
		minY = std::min(minY, point.y);
		maxY = std::max(maxY, point.y);
		minZ = std::min(minZ, point.z);
		maxZ = std::max(maxZ, point.z);
	}

	double gridSizeX = 0;
	double gridSizeY = 0;
	double gridSizeZ = 0;

	if (optimizationMode == PointCloudOptimizationMode::GridFilter) {
		gridSizeX = (maxX - minX) * gridSizeInPercent;
		gridSizeY = (maxY - minY) * gridSizeInPercent;
		gridSizeZ = (maxZ - minZ) * gridSizeInPercent;
	}

	double neighborSearchRadius = (maxX - minX) * neighbourRangeInPercent;
	double dilatationVoxelSize = (maxX - minX) * dilationVoxelSizeInPercent;
	double mlsRangeRadius = (maxX - minX) * neighbourMLSRangeInPercent;



	std::cout << "depthOctree = " << depthOctree << std::endl;
	std::cout << "gridResolution = " << gridResolution << std::endl;
	std::cout << "kNeigborsForNormals = " << kNeigborsForNormals << std::endl;
	std::cout << "isoValue = " << isoValue << std::endl;
	std::cout << "percentageExtendGrid = " << percentageExtendGrid << std::endl;
	std::cout << "gridSizeInPercent = " << gridSizeInPercent << std::endl;
	std::cout << "neighbourRangeInPercent = " << neighbourRangeInPercent << std::endl;
	std::cout << "neighbourMLSRangeInPercent = " << neighbourMLSRangeInPercent << std::endl;
	std::cout << "dilationVoxelSizeInPercent = " << dilationVoxelSizeInPercent << std::endl;
	std::cout << "dilationIteration = " << dilationIteration << std::endl;


	cloud->points.resize(pointCloud.size());

	for (int i = 0; i < pointCloud.size(); i++) {
		cloud->points[i].x = pointCloud[i].x;
		cloud->points[i].y = pointCloud[i].y;
		cloud->points[i].z = pointCloud[i].z;
	}
	cloud->width = cloud->points.size();
	cloud->height = 1;
	cloud->is_dense = false;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);

	if (optimizationMode == PointCloudOptimizationMode::GridFilter) {

		pcl::VoxelGrid<pcl::PointXYZ> sor;
		sor.setInputCloud(cloud);
		sor.setLeafSize(gridSizeX, gridSizeX, gridSizeX);  // for now use only gridSizeX -> should be correct

		Clock clock;
		sor.filter(*cloudFiltered);
		std::cout << "PointCloudOptimizationMode::GridFilter \n";
		LogHelper::logTimeInMS(clock.getTimeInMilliseconds());

		std::cout << "AFTER: cloudFiltered size=" << cloudFiltered->size() << std::endl;

		tree->setInputCloud(cloudFiltered);
		n.setInputCloud(cloudFiltered);
		n.setSearchMethod(tree);
		n.setKSearch(kNeigborsForNormals);
		n.compute(*normals);
	}
	else if (optimizationMode == PointCloudOptimizationMode::None) {
		tree->setInputCloud(cloud);
		n.setInputCloud(cloud);
		n.setSearchMethod(tree);
		n.setKSearch(kNeigborsForNormals);
		n.compute(*normals);
	}

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);

	if (optimizationMode == PointCloudOptimizationMode::GridFilter) {
		pcl::concatenateFields(*cloudFiltered, *normals, *cloud_with_normals);
	}
	else if (optimizationMode == PointCloudOptimizationMode::MLS) {


		pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
		mls.setInputCloud(cloud);
		//mls.setPolynomialOrder(2);
		mls.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
		mls.setPolynomialOrder(2);
		mls.setSearchRadius(mlsRangeRadius);
		mls.setComputeNormals(true);
		mls.setNumberOfThreads(8);

		Clock clock;
		mls.process(*cloud_with_normals);


		std::cout << "PointCloudOptimizationMode::MLS \n";
		LogHelper::logTimeInMS(clock.getTimeInMilliseconds());
		std::cout << "AFTER: cloud_with_normals size = " << cloud_with_normals->points.size() << std::endl;
	}
	else if (optimizationMode == PointCloudOptimizationMode::MLSUpsampling) {

		pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
		mls.setInputCloud(cloud);
		//mls.setPolynomialOrder(2);
		mls.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
		mls.setSearchRadius(mlsRangeRadius);
		mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::VOXEL_GRID_DILATION);
		mls.setDilationVoxelSize(dilatationVoxelSize);
		mls.setDilationIterations(dilationIteration);
		mls.setComputeNormals(true);
		mls.setNumberOfThreads(8);

		Clock clock;
		mls.process(*cloud_with_normals);
		std::cout << "PointCloudOptimizationMode::MLSUpsampling \n";
		LogHelper::logTimeInMS(clock.getTimeInMilliseconds());
		std::cout << "AFTER: cloud_with_normals size = " << cloud_with_normals->size() << std::endl;

	}
	else if (optimizationMode == PointCloudOptimizationMode::None) {
		pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	}
	else {
		std::cerr << "Wrong optimizationMode\n";
		std::abort();
	}

	pcl::PolygonMesh triangles;

	if (reconstructionMode == SurfaceReconstructionMode::Poisson) {
		pcl::Poisson<pcl::PointNormal> poisson;
		poisson.setDepth(depthOctree);
		poisson.setInputCloud(cloud_with_normals);

		Clock clock;
		poisson.reconstruct(triangles);
		std::cout << "SurfaceReconstructionMode::Poisson" << std::endl;
		LogHelper::logTimeInMS(clock.getTimeInMilliseconds());

	}
	else if (reconstructionMode == SurfaceReconstructionMode::GreedyProjectionTriangulation) {
		pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
		tree2->setInputCloud(cloud_with_normals);

		pcl::GreedyProjectionTriangulation<pcl::PointNormal> greedProjectionTriangulation;

		greedProjectionTriangulation.setSearchRadius(neighborSearchRadius);

		greedProjectionTriangulation.setMu(2.5);
		greedProjectionTriangulation.setMaximumNearestNeighbors(100);
		greedProjectionTriangulation.setMaximumSurfaceAngle(M_PI / 4);
		greedProjectionTriangulation.setMinimumAngle(M_PI / 18);
		greedProjectionTriangulation.setMaximumAngle(2 * M_PI / 3);
		greedProjectionTriangulation.setNormalConsistency(false);

		greedProjectionTriangulation.setInputCloud(cloud_with_normals);
		greedProjectionTriangulation.setSearchMethod(tree2);

		Clock clock;
		greedProjectionTriangulation.reconstruct(triangles);
		std::cout << "SurfaceReconstructionMode::GreedyProjectionTriangulation" << std::endl;
		LogHelper::logTimeInMS(clock.getTimeInMilliseconds());


	}
	else if (reconstructionMode == SurfaceReconstructionMode::MarchingCubesHoppe) {
		pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
		tree2->setInputCloud(cloud_with_normals);

		pcl::MarchingCubesHoppe<pcl::PointNormal> marchingCubesHoppe;
		marchingCubesHoppe.setIsoLevel(isoValue);
		marchingCubesHoppe.setGridResolution(gridResolution, gridResolution, gridResolution);
		marchingCubesHoppe.setPercentageExtendGrid(percentageExtendGrid);
		marchingCubesHoppe.setInputCloud(cloud_with_normals);
		marchingCubesHoppe.setSearchMethod(tree2);

		Clock clock;
		marchingCubesHoppe.reconstruct(triangles);
		std::cout << "SurfaceReconstructionMode::MarchingCubesHoppe" << std::endl;
		LogHelper::logTimeInMS(clock.getTimeInMilliseconds());
	}

	pcl::PointCloud<pcl::PointXYZ> cloud2;
	pcl::fromPCLPointCloud2(triangles.cloud, cloud2);

	surface->surface.clear();
	surface->surface.reserve(triangles.polygons.size() * 3);

	std::cout << "Model after reconstruction have =" << triangles.polygons.size() * 3 << " points" << std::endl;


	for (const auto& triangle : triangles.polygons) {
		pcl::PointXYZ p1 = cloud2.points[triangle.vertices[0]];
		pcl::PointXYZ p2 = cloud2.points[triangle.vertices[1]];
		pcl::PointXYZ p3 = cloud2.points[triangle.vertices[2]];

		Vertex4<float> vertex(p1.x, p1.y, p1.z, 1.0f);
		surface->surface.push_back(vertex);

		Vertex4<float> vertex2(p2.x, p2.y, p2.z, 1.0f);
		surface->surface.push_back(vertex2);

		Vertex4<float> vertex3(p3.x, p3.y, p3.z, 1.0f);
		surface->surface.push_back(vertex3);

		//std::cout << "Triangle vertices:" << std::endl;
		//std::cout << "P1: (" << p1.x << ", " << p1.y << ", " << p1.z << ")" << std::endl;
		//std::cout << "P2: (" << p2.x << ", " << p2.y << ", " << p2.z << ")" << std::endl;
		//std::cout << "P3: (" << p3.x << ", " << p3.y << ", " << p3.z << ")" << std::endl;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr surfaceReconstructed(new pcl::PointCloud<pcl::PointXYZ>);

	for (const auto& polygon : triangles.polygons) {
		for (const auto& index : polygon.vertices) {
			surfaceReconstructed->points.push_back(cloud2.points[index]);
		}
	}

	surfaceReconstructed->width = static_cast<uint32_t>(surfaceReconstructed->points.size());
	surfaceReconstructed->height = 1;

	auto meanSquaredError1 = computeMeanSquaredError(cloud, surfaceReconstructed);
	std::cout << "meanSquaredError1 = " << meanSquaredError1 << std::endl;


	auto meanSquaredError2 = computeMeanSquaredError(surfaceReconstructed, cloud);
	std::cout << "meanSquaredError2 = " << meanSquaredError2 << std::endl;

	std::cout << "****************************************************************************" << std::endl;

	return surface;
}

float PointCloudLibrary::computeMeanSquaredError(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& reconstructedSurface)
{


	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(reconstructedSurface);

	auto sumSquaredDistances = 0.0f;
	auto nPoints = 0u;

	for (const auto& point : cloud->points)
	{
		std::vector<int> pointIdxNKNSearch(1);
		std::vector<float> pointNKNSquaredDistance(1);


		if (kdtree.nearestKSearch(point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			sumSquaredDistances += pointNKNSquaredDistance[0];
			nPoints++;
		}
	}

	if (nPoints == 0) {
		return -1;
	}

	float meanSquaredError = sumSquaredDistances / nPoints;
	return meanSquaredError;
}
