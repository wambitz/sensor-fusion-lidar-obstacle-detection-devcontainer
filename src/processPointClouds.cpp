// PCL lib Functions for processing point clouds

#include "processPointClouds.h"
#include <boost/make_shared.hpp>


template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(PointCloudPtr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(PointCloudPtr cloud,
                                                                              float filterRes,
                                                                              Eigen::Vector4f minPoint,
                                                                              Eigen::Vector4f maxPoint)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered = boost::make_shared<pcl::PointCloud<PointT>>();

    //std::cout << typeid(vg).name() << endl;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion =  boost::make_shared<pcl::PointCloud<PointT>>();

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers = boost::make_shared<pcl::PointIndices>();

    for (const auto& point: indices) 
    {
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<
    PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, PointCloudPtr cloud)
{
    // NOTE: Example code from Point Cloud Library example

    // // Create the filtering object
    // pcl::ExtractIndices<pcl::PointXYZ> extract;

    // int i = 0, nr_points = (int) cloud_filtered->size();
    // // While 30% of the original cloud is still there
    // while (cloud_filtered->size () > 0.3 * nr_points)
    // {
    //     // Segment the largest planar component from the remaining cloud
    //     seg.setInputCloud (cloud_filtered);
    //     seg.segment (*inliers, *coefficients);
    //     if (inliers->indices.size () == 0)
    //     {
    //     std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    //     break;
    //     }

    //     // Extract the inliers
    //     extract.setInputCloud (cloud_filtered);
    //     extract.setIndices (inliers);
    //     extract.setNegative (false);
    //     extract.filter (*cloud_p);
    //     std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data
    //     points." << std::endl;

    //     // Create the filtering object
    //     extract.setNegative (true);
    //     extract.filter (*cloud_f);
    //     cloud_filtered.swap (cloud_f);
    //     i++;
    // }

    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

    using PointCloudPtr = typename pcl::PointCloud<PointT>::Ptr;
    PointCloudPtr obstaclesCloud = boost::make_shared<pcl::PointCloud<PointT>>();
    PointCloudPtr planeCloud = boost::make_shared<pcl::PointCloud<PointT>>();

    for (const auto& index : inliers->indices)
    {
        planeCloud->points.push_back(cloud->points[index]);
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstaclesCloud);

    // std::pair<PointCloudPtr, PointCloudPtr> segResult(cloud, cloud);
    std::pair<PointCloudPtr, PointCloudPtr> segResult(obstaclesCloud, planeCloud);

    return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<
    PointT>::SegmentPlane(PointCloudPtr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in this function to find inliers for the cloud.
    // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients ());
    // pcl::PointIndices::Ptr inliers(new pcl::PointIndices ());

    typename pcl::ModelCoefficients::Ptr coefficients = boost::make_shared<pcl::ModelCoefficients>();
    typename pcl::PointIndices::Ptr inliers = boost::make_shared<pcl::PointIndices>();

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();

    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<PointCloudPtr, PointCloudPtr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(PointCloudPtr cloud,
                                                                                          float clusterTolerance,
                                                                                          int minSize,
                                                                                          int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree = boost::make_shared<pcl::search::KdTree<PointT>>();
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance); // 1.0f -> 1m
    ec.setMinClusterSize(minSize);            // 3 points
    ec.setMaxClusterSize(maxSize);            // 30 points
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    for (const pcl::PointIndices& cluster : clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster = boost::make_shared<pcl::PointCloud<PointT>>();

        for (const int& idx : cluster.indices)
        {
            cloudCluster->points.push_back(cloud->points[idx]);
        }

        cloudCluster->width = cloudCluster->size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size()
              << " clusters" << std::endl;

    return clusters;
}

template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(PointCloudPtr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(PointCloudPtr cloud, std::string file)
{
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    PointCloudPtr cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath},
                                               boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}