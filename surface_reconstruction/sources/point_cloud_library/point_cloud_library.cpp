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

std::vector <Vertex4<float>> PointCloudLibrary::calculateSurface(std::vector < Vertex4<float>>& points)
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
	}
	else {
		printf("Error: %s\n", NFD_GetError());
	}

	// Create a point cloud object
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

	// Normal estimation
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());


	std::ifstream file(pathPoints);
	if (!file.is_open()) {
		std::cerr << "Error opening file " << pathPoints << std::endl;
	}

	int numberOfPoints;
	file >> numberOfPoints;

	std::vector<Float3> points2;
	points.reserve(numberOfPoints);
	points2.reserve(numberOfPoints);

	std::string line;
	std::getline(file, line);  // consume newline after reading numberOfPoints


	float minX = std::numeric_limits<float>::max();
	float maxX = std::numeric_limits<float>::lowest();
	float minY = minX;
	float maxY = maxX;
	float minZ = minX;
	float maxZ = maxX;


	while (std::getline(file, line) && points2.size() < numberOfPoints) {
		std::istringstream iss(line);
		Float3 point;
		if (iss >> point.x >> point.y >> point.z) {

			minX = std::min(minX, point.x);
			maxX = std::max(maxX, point.x);
			minY = std::min(minY, point.y);
			maxY = std::max(maxY, point.y);
			minZ = std::min(minZ, point.z);
			maxZ = std::max(maxZ, point.z);

			points2.push_back(point);
		}
	}

	file.close();

	for (auto& point : points2) {
		point.x = 10 * (point.x - minX) / (maxX - minX);
		point.y = 10 * (point.y - minY) / (maxY - minY);
		point.z = 10 * (point.z - minZ) / (maxZ - minZ);
	}


	cloud->points.resize(points2.size());
	for (int i = 0; i < points2.size(); i++) {
		cloud->points[i].x = points2[i].x;
		cloud->points[i].y = points2[i].y;
		cloud->points[i].z = points2[i].z;
		

		Vertex4<float> p(points2[i].x, points2[i].y, points2[i].z, 1.0f);
		points.push_back(p);
	}

	cloud->width = cloud->points.size();
	cloud->height = 1;
	cloud->is_dense = false;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);


	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(0.5, 0.5, 0.5);  // resolution of the voxel grid
	sor.filter(*cloudFiltered);


	tree->setInputCloud(cloudFiltered);
	n.setInputCloud(cloudFiltered);
	n.setSearchMethod(tree);
	n.setKSearch(21);
	//n.setRadiusSearch(0.01f);
	n.compute(*normals);

	// Concatenate the point cloud with the normals
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloudFiltered, *normals, *cloud_with_normals);

	// Create search tree
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set typical values for the parameters
	gp3.setSearchRadius(2);

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



	std::vector<Vertex4<float>> vertices;

	for (const auto& triangle : triangles.polygons) {

		// For each triangle, retrieve vertices by indexing into cloud2.
		pcl::PointXYZ p1 = cloud2.points[triangle.vertices[0]];
		pcl::PointXYZ p2 = cloud2.points[triangle.vertices[1]];
		pcl::PointXYZ p3 = cloud2.points[triangle.vertices[2]];

		Vertex4<float> vertex(p1.x, p1.y, p1.z, 1.0f);
		vertices.push_back(vertex);

		Vertex4<float> vertex2(p2.x, p2.y, p2.z, 1.0f);
		vertices.push_back(vertex2);
		Vertex4<float> vertex3(p3.x, p3.y, p3.z, 1.0f);
		vertices.push_back(vertex3);
		std::cout << "Triangle vertices:" << std::endl;
		std::cout << "P1: (" << p1.x << ", " << p1.y << ", " << p1.z << ")" << std::endl;
		std::cout << "P2: (" << p2.x << ", " << p2.y << ", " << p2.z << ")" << std::endl;
		std::cout << "P3: (" << p3.x << ", " << p3.y << ", " << p3.z << ")" << std::endl;

	}

	return vertices;
}
