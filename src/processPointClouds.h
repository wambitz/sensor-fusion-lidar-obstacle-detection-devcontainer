// PCL lib Functions for processing point clouds
#pragma once

#include "box.h"

#include <boost/filesystem.hpp>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <chrono>
#include <ctime>
#include <iostream>
#include <string>
#include <vector>

using render::Box;

template <typename PointT>
class ProcessPointClouds
{
public:
    // constructor
    ProcessPointClouds() = default;
    // deconstructor
    ~ProcessPointClouds() = default;

    using PointCloudPtr = typename pcl::PointCloud<PointT>::Ptr;

    void numPoints(PointCloudPtr cloud);

    PointCloudPtr FilterCloud(PointCloudPtr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<PointCloudPtr, PointCloudPtr> SeparateClouds(pcl::PointIndices::Ptr inliers, PointCloudPtr cloud);

    std::pair<PointCloudPtr, PointCloudPtr> SegmentPlane(PointCloudPtr cloud,
                                                         int maxIterations,
                                                         float distanceThreshold);

    std::vector<PointCloudPtr> Clustering(PointCloudPtr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(PointCloudPtr cluster);

    void savePcd(PointCloudPtr cloud, std::string file);

    PointCloudPtr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
};
