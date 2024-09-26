// main.cpp

#include "processPointClouds.h"
#include "render/render.h"
#include "ransac.h"
#include "line_model.h"
#include "plane_model.h"
#include <random>
#include <unordered_set>

// Include processPointClouds.cpp to help linker
#include "processPointClouds.cpp"

using render::Color;
using render::renderPointCloud;

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
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
    viewer->addCoordinateSystem(1.0);
    return viewer;
}

int main(int argc, char** argv)
{
    // Parse command-line arguments
    bool run3D = true; // Default is 3D

    // Check if a mode was provided
    if (argc > 1)
    {
        std::string mode = argv[1];
        if (mode == "2d")
        {
            run3D = false;
            std::cout << "Running in 2D mode..." << std::endl;
        }
        else if (mode == "3d")
        {
            run3D = true;
            std::cout << "Running in 3D mode..." << std::endl;
        }
        else
        {
            std::cerr << "Invalid mode specified! Use '2d' or '3d'. Defaulting to 3D." << std::endl;
        }
    }
    else
    {
        std::cout << "No mode specified. Running in default 3D mode..." << std::endl;
    }

    // Create viewer
    pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    std::unordered_set<int> inliers;

    if (run3D)
    {
        cloud = CreateData3D();
        PlaneModel planeModel;
        Ransac<pcl::PointXYZ> ransac(10, 0.5);
        inliers = ransac.run(cloud, planeModel);
    }
    else
    {
        cloud = CreateData();
        LineModel lineModel;
        Ransac<pcl::PointXYZ> ransac(10, 0.5);
        inliers = ransac.run(cloud, lineModel);
    }

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

    // Render point cloud with inliers and outliers
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
