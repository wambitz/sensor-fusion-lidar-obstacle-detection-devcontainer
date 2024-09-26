// Author: Aaron Brown
// Modified by: Julio Castillo
// Create simple 3d highway environment using PCL
// for exploring self-driving car sensors
#include "car.h"
#include "lidar.h"
#include "processPointClouds.h"
#include "render.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

using render::Box;
using render::CameraAngle;
using render::Car;
using render::Color;
using render::renderHighway;
using render::renderPointCloud;
using render::Vect3;
using sensors::lidar::Lidar;

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
    Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
    Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
    Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if (renderScene)
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
    bool renderScene = false; // True will display the highway and the cars
    bool render_clusters = true; // True will display the clusters found with KD-Tree
    bool render_box = true; // True will add bounding box around the clusters
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // Create lidar sensor
    std::unique_ptr<Lidar> lidar = std::make_unique<Lidar>(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();

    // renderRays(viewer, lidar->getPosition(), inputCloud); // This will display the rays cast by the lidar
    // renderPointCloud(viewer, inputCloud, "inputCloud"); // This shows the point cloud without filtering

    // Create point processor
    std::unique_ptr<ProcessPointClouds<pcl::PointXYZ>> pointProcessor =
        std::make_unique<ProcessPointClouds<pcl::PointXYZ>>();

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud =
        pointProcessor->SegmentPlane(inputCloud, 100, 0.2);
    // NOTE: Comment these renders out to remove plane and non-clusterized obstacles
    renderPointCloud(viewer, segmentCloud.first, "obstaclesCloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters =
        pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    // Color(R, G, B)
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        if(render_clusters)
        {
            std::cout << "cluster size ";
            pointProcessor->numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);
        }
        if (render_box)
        {
            Box box = pointProcessor->BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }
        ++clusterId;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    std::unique_ptr<ProcessPointClouds<pcl::PointXYZI>> pointProcessorI = std::make_unique<ProcessPointClouds<pcl::PointXYZI>>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    renderPointCloud(viewer, inputCloud, "inputCloud");

}

// setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();

    // distance away in meters
    int distance = 16;

    switch (setAngle)
    {
    case CameraAngle::XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
    case CameraAngle::TopDown:
        viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;
    case CameraAngle::Side:
        viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
    case CameraAngle::FPS:
        viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != CameraAngle::FPS)
    {
        viewer->addCoordinateSystem(1.0);
    }
}

int main(int argc, char** argv)
{
    std::cout << "starting environment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = CameraAngle::XY;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);
    cityBlock(viewer);

    while (!viewer->wasStopped())
    {
        // NOTE: If using Ubuntu 22.04 use this instead, this will throw an error but it will work
        // viewer->spin();
        // Alternatively, build pcl from source: https://github.com/PointCloudLibrary/pcl.git
        // For  Ubuntu 20.04 the following works fine:
        viewer->spinOnce();
    }
}