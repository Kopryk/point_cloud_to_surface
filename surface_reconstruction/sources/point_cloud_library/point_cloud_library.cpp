#include "point_cloud_library.h"
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h> 

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

				// colors read from furgeviewer are in uint16format, we need to conver it to float

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

std::unique_ptr<PointCloudData>  PointCloudLibrary::calculateSurface(std::vector <Vertex4<float>>& pointCloud, bool useGridFilter, double gridSizeInPercent, double neighbourRangeInPercent)
{
	// gridSizeInPercent = 0.0 - 1.0
	// neighbourRangeInPercent = 0.0 - 1.0
	// Create a point cloud object
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

	// Normal estimation
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

	for (auto& point : pointCloud) {

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

	if (useGridFilter) {
		gridSizeX = (maxX - minX) * gridSizeInPercent;
		gridSizeY = (maxY - minY) * gridSizeInPercent;
		gridSizeZ = (maxZ - minZ) * gridSizeInPercent;
	}

	double neighborSearchRadius = (maxX - minX) * neighbourRangeInPercent;
	std::cout << " neighborSearchRadius = " << neighborSearchRadius << std::endl;
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

	if (useGridFilter) {


		// Create the filtering object
		pcl::VoxelGrid<pcl::PointXYZ> sor;
		sor.setInputCloud(cloud);
		sor.setLeafSize(gridSizeX, gridSizeX, gridSizeX);  // for now use only gridSizeX -> should be correct
		sor.filter(*cloudFiltered);


		tree->setInputCloud(cloudFiltered);
		n.setInputCloud(cloudFiltered);
	}
	else {
		tree->setInputCloud(cloud);
		n.setInputCloud(cloud);
	}


	n.setSearchMethod(tree);
	n.setKSearch(21);
	//n.setRadiusSearch(0.01f);
	n.compute(*normals);

	// Concatenate the point cloud with the normals
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);

	if (useGridFilter) {
		pcl::concatenateFields(*cloudFiltered, *normals, *cloud_with_normals);
	}
	else {
		pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	}


	// Create search tree
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set typical values for the parameters
	gp3.setSearchRadius(neighborSearchRadius);

	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);

	pcl::PointCloud<pcl::PointXYZ> cloud2;
	pcl::fromPCLPointCloud2(triangles.cloud, cloud2);

	surface->surface.clear();
	surface->surface.reserve(triangles.polygons.size() * 3);

	for (const auto& triangle : triangles.polygons) {

		// For each triangle, retrieve vertices by indexing into cloud2.
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


	return surface;

}
