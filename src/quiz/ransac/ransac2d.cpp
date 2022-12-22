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

// Time segmenation process
auto startTime = std::chrono::steady_clock::now(); // holds best inliers

	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers Anthony was here 12/16/2022
	
	while(maxIterations--) {

	// Randomly pick two points

		std::unordered_set<int> inliers;
		while (inliers.size()<2)
			inliers.insert(rand()%(cloud->points.size()));

		float x1, y1, x2, y2;

		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;

		float a = (y1-y2);
		float b = (x2-x1);
		float c = (x1*y2-x2*y1);

		for(int index =0; index< cloud->points.size(); index++)
		{
			if(inliers.count(index)>0)
				continue;

			pcl::PointXYZ point = cloud->points[index];
			float x3 = point.x;
			float y3 = point.y;

			float d = fabs(a*x3+b*y3+c)/sqrt(a*a+b*b);

			if(d<= distanceTol)
			{
				inliers.insert(index);
			}		

		}

		if(inliers.size()>inliersResult.size()){
		inliersResult = inliers;

		}

	}

		
		
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout<<"Ransac took" <<elapsedTime.count()<<"milliseconds"<<std::endl;
	return inliersResult;

}



std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{


	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// For max iterations
	while (maxIterations-- >0) {
		// Randomly pick data points
		pcl::PointXYZ point1 = cloud->points.at(rand() % (cloud->size()));
		pcl::PointXYZ point2 = cloud->points.at(rand() % (cloud->size()));	
		pcl::PointXYZ point3 = cloud->points.at(rand() % (cloud->size()));
		// Fit data to a plane, Ax+Bx+Cz+D=0
		float A, B, C, D;
		A = (point2.y -point1.y)*(point3.z - point1.z) - (point2.z - point1.z)*(point3.y-point1.y);
		B = (point2.z -point1.z)*(point3.x - point1.x) - (point2.x - point1.x)*(point3.z-point1.z);
		C = (point2.x -point1.x)*(point3.y - point1.y) - (point2.y - point1.y)*(point3.x-point1.x);
		D = -1*(A*point1.x + B*point1.y + C*point1.z); 

		// Calculate Distance
		std::unordered_set<int> inliersTemp;
		for (auto itr = cloud->points.begin(); itr != cloud->points.end(); ++itr) 
		{
			float d = fabs(A * (*itr).x+ B * (*itr).y + C 	* (*itr).z +D) / sqrt(A*A + B*B + C*C);
			//
			if (d<distanceTol)
			{
				inliersTemp.insert(itr - cloud->begin());
			}
			}

			if (inliersTemp.size() > inliersResult.size()){
			inliersResult = inliersTemp;

			}
	// Return indicies of the linlers from the fitted data points 
	return inliersResult;

}			

int main () 
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3d();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	//std::unordered_set<int> inliers = Ransac(cloud,10, 1.0); // For Ransac Line 

	std::unordered_set<int> inliers = RansacPlane(cloud,50, 0.2); // For RansacPlane

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
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
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,255,255));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(0,250,0));
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
