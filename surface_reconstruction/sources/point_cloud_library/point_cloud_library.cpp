#include "point_cloud_library.h"
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>

std::vector < Vertex4<float>> PointCloudLibrary::calculateSurface(std::vector<Point>& points)
{




	// Normal estimation
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());


	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	generateFlatSurfacePointCloud(cloud, 10, 5, 1000);

	/*cloud->points.resize(points.size());
	for (int i = 0; i < points.size(); i++) {
		cloud->points[i].x = points[i].x;
		cloud->points[i].y = points[i].y;
		cloud->points[i].z = points[i].z;

	}*/
	//for (const auto& vertex : points) {
	//	pcl::PointXYZ point;
	//	point.x = vertex.x;
	//	point.y = vertex.y;
	//	point.z = vertex.z;
	//	cloud->points.push_back(point);
	//}/*

	/*cloud->width = cloud->points.size();
	cloud->height = 1;
	cloud->is_dense = false;*/

	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);

	// Concatenate the point cloud with the normals
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

	// Create search tree
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set typical values for the parameters
	gp3.setSearchRadius(0.025);
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4);
	gp3.setMinimumAngle(M_PI / 18);
	gp3.setMaximumAngle(2 * M_PI / 3);
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);


	pcl::PointCloud<pcl::PointXYZ> cloud2;
	pcl::fromPCLPointCloud2(triangles.cloud, cloud2);

	std::cout << " triangles.cloud.width = " << triangles.cloud.width << std::endl;

	std::vector<Vertex4<float>> vertices;
	for (const auto& point : cloud2) {
		Vertex4<float> vertex(point.x, point.y, point.z, 1.0f);
		vertices.push_back(vertex);
	}
	std::cout << " vertices size=  " << vertices.size() << std::endl;


	for (int i = 0; i < 10; i++) {
		auto vertex = vertices[i];
		std::cout << " x = " << vertex.x << "  ,";
		std::cout << " y = " << vertex.y << "  ,";
		std::cout << " z = " << vertex.z << "\n\n";
	}


	return vertices;
}
