/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <string>

// Segmentation macros
#define SEG_MAX_ITER 30
#define SEG_THRESHOLD 0.35

#define CLUSTER_MIN_SIZE 10
#define CLUSTER_MAX_SIZE 500
#define CLUSTER_TOLERANCE 0.53


std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar * lidar= new Lidar(cars, 0);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();

    // Basic renedering 
    // renderRays(viewer, lidar->position, inputCloud);
    // renderPointCloud(viewer, inputCloud, "Cloud", Color(0,1,0));

    // Segmentation prcoessing 
    
    ProcessPointClouds<pcl::PointXYZ> processPntCld; 
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = processPntCld.SegmentPlane(inputCloud, SEG_MAX_ITER, SEG_THRESHOLD);
    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,0,1));
    

    // Clustring processing 
    // Accesing obstacles cloud in segment cloud
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = \ 
    processPntCld.Clustering(segmentCloud.first,CLUSTER_TOLERANCE, CLUSTER_MIN_SIZE, CLUSTER_MAX_SIZE);

    std::vector<Color> colors = {Color(1,1,1), Color(0,1,0), Color(1,1,0)};
    short colorId = 0;
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster: cloudClusters){

        // std::cout << "Cluster size: " << processPntCld.numPoints(cluster) << std::endl;
        renderPointCloud(viewer, cluster, "Cluster " + std::to_string(colorId), colors[colorId % 3]);
        // Rendering a box 
        Box box = processPntCld.BoundingBox(cluster);
        renderBox(viewer,box,colorId);
        colorId++;
    }

    // renderPointCloud(viewer,segmentCloud.second, "Plane Cloud");

}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI>* processPntCld,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud )
{

    // std::string filePath = "../src/sensors/data/pcd/data_1/0000000000.pcd";
    // pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = processPntCld->loadPcd(filePath);
    //renderPointCloud(viewer, inputCloud, "Input Cloud");

    const float GRID_SIZE = 0.4;
    // Filtering the point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = processPntCld->FilterCloud(inputCloud, GRID_SIZE ,
     Eigen::Vector4f (-10,-5,-2,1),
     Eigen::Vector4f (30,8,1,1) );

    //renderPointCloud(viewer,filterCloud,"filterCloud");

    // Applying segmentation on the point cloud
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> \
    segmentCloud = processPntCld->SegmentPlane(inputCloud, SEG_MAX_ITER, SEG_THRESHOLD);

    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,0,1));
    
    // Applying Clustering on the point cloud
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = \ 
    processPntCld->Clustering(segmentCloud.first,CLUSTER_TOLERANCE, CLUSTER_MIN_SIZE, CLUSTER_MAX_SIZE);

    std::vector<Color> colors = {Color(1,1,1), Color(0,1,0), Color(1,1,0)};
    short colorId = 0;
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster: cloudClusters){

        // std::cout << "Cluster size: " << processPntCld.numPoints(cluster) << std::endl;
        renderPointCloud(viewer, cluster, "Cluster " + std::to_string(colorId), colors[colorId % 3]);
        // Rendering a box 
        Box box = processPntCld->BoundingBox(cluster);
        renderBox(viewer,box,colorId);
        colorId++;
    }


}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);

    std::string filePath = "../src/sensors/data/pcd/data_1";
    ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd(filePath);
    auto streamIter = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIter).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIter++;
        if(streamIter == stream.end())
            streamIter = stream.begin();

        viewer->spinOnce ();
    } 
}