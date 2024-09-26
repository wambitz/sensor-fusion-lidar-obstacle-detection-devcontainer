#ifndef RANSAC_H
#define RANSAC_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <unordered_set>

// Abstract base class for models
template <typename PointT>
class Model
{
public:
    virtual void fit(const std::vector<PointT>& points) = 0;
    virtual float distance(const PointT& point) const = 0;
    virtual int requiredSamples() const = 0;
    virtual ~Model() {}
};

// RANSAC class
template <typename PointT>
class Ransac
{
public:
    Ransac(int maxIterations, float distanceTol);

    std::unordered_set<int> run(typename pcl::PointCloud<PointT>::Ptr cloud, Model<PointT>& model);

private:
    int maxIterations_;
    float distanceTol_;
};

#include "Ransac.hpp"

#endif // RANSAC_H
