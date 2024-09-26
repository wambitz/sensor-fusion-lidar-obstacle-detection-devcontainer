#ifndef RANSAC_HPP
#define RANSAC_HPP

#include <chrono>
#include <cmath>
#include <random>

template <typename PointT>
Ransac<PointT>::Ransac(int maxIterations, float distanceTol)
    : maxIterations_(maxIterations), distanceTol_(distanceTol) {}

template <typename PointT>
std::unordered_set<int> Ransac<PointT>::run(typename pcl::PointCloud<PointT>::Ptr cloud, Model<PointT>& model)
{
    auto startTime = std::chrono::steady_clock::now();
    std::unordered_set<int> inliersResult;

    // Use modern C++ random number generation
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dis(0, cloud->points.size() - 1);

    int sampleSize = model.requiredSamples();

    while (maxIterations_--)
    {
        std::unordered_set<int> inliers;
        while (inliers.size() < sampleSize)
        {
            inliers.insert(dis(gen));
        }

        std::vector<PointT> sampledPoints;
        for (auto index : inliers)
        {
            sampledPoints.push_back(cloud->points[index]);
        }

        // Fit model
        model.fit(sampledPoints);

        // Iterate through all points and measure distance
        for (int i = 0; i < cloud->points.size(); ++i)
        {
            if (inliers.count(i))
                continue;

            PointT point = cloud->points[i];
            float distance = model.distance(point);

            if (distance <= distanceTol_)
            {
                inliers.insert(i);
            }
        }

        if (inliers.size() > inliersResult.size())
        {
            inliersResult = inliers;
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "RANSAC Duration: " << duration.count() << " milliseconds" << std::endl;

    return inliersResult;
}

#endif // RANSAC_HPP
