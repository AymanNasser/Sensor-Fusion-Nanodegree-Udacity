/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <math.h>

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

	int cloudSize = cloud->points.size();
	while(maxIterations--){
		
		std::unordered_set<int> inliers;

		int randIndex_1 = rand() %  cloudSize;
		int randIndex_2 = rand() % cloudSize;

		pcl::PointXYZ point1 = cloud->points[randIndex_1];
		pcl::PointXYZ point2 = cloud->points[randIndex_2];

		for(int pointIndex=0; pointIndex< cloudSize; pointIndex++){
			
			float point_x = cloud->points[pointIndex].x;
			float point_y = cloud->points[pointIndex].y;

			float lineFormula = (point1.y - point2.y) * point_x +
								(point2.x - point1.x) * point_y +
								(point1.x * point2.y - point2.x * point1.y);
			float a_2 = pow(point1.y - point2.y, 2);
			float b_2 = pow(point2.x - point1.x, 2);
			float distance = fabs(lineFormula) / sqrt(a_2 + b_2);

			if( distance <= distanceTol)
				inliers.insert(pointIndex);
		}
		/* Checking if the no. of inliers are bigger the current best result & if so
		*  we update the best result ==> best fitted model
		*/
		if(inliers.size() > inliersResult.size())
			inliersResult = inliers;
	}
	return inliersResult;

}


std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{

	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	 // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

	int cloudSize = cloud->points.size();
	while(maxIterations--){
		
		std::unordered_set<int> inliers;

		int randIndex_1 = rand() %  cloudSize;
		int randIndex_2 = rand() % cloudSize;
		int randIndex_3 = rand() & cloudSize;

		pcl::PointXYZ point1 = cloud->points[randIndex_1];
		pcl::PointXYZ point2 = cloud->points[randIndex_2];
		pcl::PointXYZ point3 = cloud->points[randIndex_3];

		float i = (point2.y -point1.y)*(point3.z - point1.z) - 
				  (point2.z - point1.z)*(point3.y - point1.y);

		float j = (point2.z - point1.z)*(point3.x - point1.x) - 
				  (point2.x - point1.x)*(point3.z - point1.z);

		float k = (point2.x - point1.x)*(point3.y - point1.y) - 
				  (point2.y - point1.y)*(point3.x - point1.x);


		for(int pointIndex=0; pointIndex< cloudSize; pointIndex++){
			
			float point_x = cloud->points[pointIndex].x;
			float point_y = cloud->points[pointIndex].y;
			float point_z = cloud->points[pointIndex].z;

			float planeFormula = i*point_x + j*point_y + k*point_z - 
								 (i*point1.x + j*point1.y + k*point1.z);

			float distance = fabs(i*point_x + j*point_y + k*point_z + (i*point1.x + j*point1.y + k*point1.z)) \ 
							/ sqrt(i*i + j*j + k*k);


			if( distance <= distanceTol)
				inliers.insert(pointIndex);
		}
		/* Checking if the no. of inliers are bigger the current best result & if so
		*  we update the best result ==> best fitted model
		*/
		if(inliers.size() > inliersResult.size())
			inliersResult = inliers;
	}

	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Ransac self-implemented segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

	return inliersResult;

}



int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();

	// Create 3d-data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 50, 0.5);

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