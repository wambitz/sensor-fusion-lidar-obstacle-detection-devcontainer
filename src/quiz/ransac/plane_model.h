#ifndef PLANEMODEL_H
#define PLANEMODEL_H

#include "ransac.h"
#include <pcl/point_types.h>

class PlaneModel : public Model<pcl::PointXYZ>
{
public:
    void fit(const std::vector<pcl::PointXYZ>& points) override;
    float distance(const pcl::PointXYZ& point) const override;
    int requiredSamples() const override;

private:
    float a_, b_, c_, d_;
};

#endif // PLANEMODEL_H
