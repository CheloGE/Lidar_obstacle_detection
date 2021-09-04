/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
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
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
	while(maxIterations--)
	{	
	// Randomly sample subset and fit line
		std::unordered_set<int> inliers_indx; // set structure helps us to get only unique points
		while (inliers_indx.size()<2)
		{
			inliers_indx.insert(rand()%(cloud->points.size()));
		}
	// Measure distance between every point and fitted line
	float x1, y1, x2, y2;

	auto itr = inliers_indx.begin();
	x1 = cloud -> points[*itr].x;
	y1 = cloud -> points[*itr].y;
	itr++;
	x2 = cloud -> points[*itr].x;
	y2 = cloud -> points[*itr].y;

	float dy = (y2-y1);
	float dx = (x2-x1);
	// line equation y = m*x+b
	float m = dy/dx;
	float b = y2 - m*x2;
	
	
	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		if(inliers_indx.count(i)==1){ //if cloud index i presented in inliers_indx not process point
			continue;
		}
		
		float x3, y3, x4, y4;

		x3 = cloud -> points[i].x;
		y3 = cloud -> points[i].y;
		// perpendicular line to previous line y = -1/m*x+b2
		float b2 = y3 + x3/m;
		x4 = m*(b2-b)/(m*m+1);
		y4 = m * x4 + b;
		float dist = sqrt((y3-y4)*(y3-y4)+(x3-x4)*(x3-x4));
		// If distance is smaller than threshold count it as inlier
		if (dist<distanceTol)
		{
			inliers_indx.insert(i);
		}
	
		
	}

	if (inliers_indx.size()>inliersResult.size())
	{
		inliersResult = inliers_indx;
	}
	// Return indicies of inliers from fitted line with most inliers
	
	}
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 10, 1.0);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
