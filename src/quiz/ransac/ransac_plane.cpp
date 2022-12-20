# include "../..'render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templtes for pocessPointClouds so also incude .cpp to help linker 
#include "../../processPointClouds.cpp"

std::unordered_set<int> RansacPLane(pcl::PointCloud<pc::PointXYZ>::cloud, int maxIterations, float distanceTol)

srand(time(NULL)):

// For max iterations
while (maxIterations-- >0){
// Randomly pick data points
pcl::PointXYZ point1 = cloud->points.dataset1(rand(() % (cloud->points()));
pcl::PointXYZ point2 = cloud->points.dataset2(rand(() % (cloud->points()));
pcl::PointXYZ point3 = cloud->points.dataset3(rand(() % (cloud->points()));
// Fit data to a plane, Ax+Bx+Cz+D=0
float A, B, C, D;
A = (point2.y -point1.y)*(point3.z - point1.z) - (point2.z - point1.z)*(point3.y-point1.y);
B = (point2.z -point1.z)*(point3.x - point1.x) - (point2.x - point1.x)*(point3.z-point1.z);
C = (point2.x -point1.x)*(point3.y - point1.y) - (point2.y - point1.y)*(point3.x-point1.x);
D = -1*(A*point1.x + B*point1.y + C*point1.z); 

// Calculate Distance
std::unordered_set<int> inliersTemp;
for (auto it = cloud->points.start()); it != cloud->point.end(); ++it
{
	float d = fabs(A * (*it.x)+ B * (*it).7 + C * (*it).z +D) / sqrt(A*A + B*B + C*C);
	//
	if (d<distanceTol)
	{
		interliers.insert(it - cloud->being());
	}
}

if (inliersTemp.size() > inliersResults.size())
{
	inliersResults = inliersTemp;

}

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = Create3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud,50, 0.2);

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

