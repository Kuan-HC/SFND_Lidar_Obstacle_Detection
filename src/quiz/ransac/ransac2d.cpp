/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <iostream>

using std::unordered_set;

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// Add inliers
	float scatter = 0.6;
	for (int i = -5; i < 5; i++)
	{
		double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		pcl::PointXYZ point;
		point.x = i + scatter * rx;
		point.y = i + scatter * ry;
		point.z = 0;

		cloud->points.push_back(point);
	}
	// Add outliers
	int numOutliers = 10;
	while (numOutliers--)
	{
		double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		pcl::PointXYZ point;
		point.x = 5 * rx;
		point.y = 5 * ry;
		point.z = 0;

		cloud->points.push_back(point);
	}
	cloud->width = cloud->points.size();
	cloud->height = 1;

	return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->initCameraParameters();
	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
	viewer->addCoordinateSystem(1.0);
	return viewer;
}

std::unordered_set<int> RansacLine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	int &&len = cloud->points.size();
	std::unordered_set<int> inliersResult;
	std::unordered_set<int> tempInliers;
	srand(time(NULL));
	// TODO: Fill in this function

	while (maxIterations-- > 0)
	{
		// 隨機兩個點決定方式
		tempInliers.clear();
		int &&ptr1 = rand() % len;
		int &&ptr2 = rand() % len;
		tempInliers.insert(ptr1);
		tempInliers.insert(ptr2);
		// 直線方程式 ax + by + c = 0
		float x1 = cloud->points[ptr1].x;
		float x2 = cloud->points[ptr2].x;
		float y1 = cloud->points[ptr1].y;
		float y2 = cloud->points[ptr2].y;

		float a = y1 - y2;
		float b = x2 - x1;
		float c = x1 * y2 - x2 * y1;
		float &&dem = sqrt(a * a + b * b);

		for (int i = 0; i < len; ++i)
		{
			if (i == ptr1 || i == ptr2)
				continue;

			pcl::PointXYZ point = cloud->points[i];
			float dist = fabs(a * point.x + b * point.y + c) / dem;
			if (dist <= distanceTol)
				tempInliers.insert(i);
		}
		if (tempInliers.size() > inliersResult.size())
			inliersResult = move(tempInliers);
	}
	return inliersResult;
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	int &&len = cloud->points.size();
	// 初始化
	unordered_set<int> inliersResult;
	unordered_set<int> tempInliers;
	srand((unsigned)NULL);

	while (maxIterations-- > 0)
	{
		// 隨機挑三個點出來
		tempInliers.clear();
		int &&ptr1 = rand() % len;
		int &&ptr2 = rand() % len;
		int &&ptr3 = rand() % len;
		tempInliers.insert(ptr1);
		tempInliers.insert(ptr2);
		tempInliers.insert(ptr3);
		pcl::PointXYZ p1 = cloud->points[ptr1];
		pcl::PointXYZ p2 = cloud->points[ptr2];
		pcl::PointXYZ p3 = cloud->points[ptr3];

		// 計算平面方程式
		double a = ((p2.y - p1.y) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.y - p1.y));
		double b = ((p2.z - p1.z) * (p3.x - p1.x) - (p2.x - p1.x) * (p3.z - p1.z));
		double c = ((p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x));
		double d = (0 - (a * p1.x + b * p1.y + c * p1.z));

		float &&dem = sqrt(a * a + b * b + c * c);

		for (int i = 0; i < len; ++i)
		{
			if (i == ptr1 || i == ptr2 || i == ptr3)
				continue;

			pcl::PointXYZ point = cloud->points[i];
			float dist = fabs(a * point.x + b * point.y + c * point.z + d) / dem;
			if (dist <= distanceTol)
				tempInliers.insert(i);
		}

		if (tempInliers.size() > inliersResult.size())
			inliersResult = move(tempInliers);
	}
	return inliersResult;
}

int main()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 10, 1.0);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for (int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if (inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	// Render 2D point cloud with inliers and outliers
	if (inliers.size())
	{
		renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
		renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
	}
	else
	{
		renderPointCloud(viewer, cloud, "data");
	}

	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}
}
