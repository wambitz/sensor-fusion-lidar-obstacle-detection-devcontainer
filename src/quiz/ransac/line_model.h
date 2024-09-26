#ifndef LINEMODEL_H
#define LINEMODEL_H

#include "Ransac.h"
#include <pcl/point_types.h>

class LineModel : public Model<pcl::PointXYZ>
{
public:
    void fit(const std::vector<pcl::PointXYZ>& points) override;
    float distance(const pcl::PointXYZ& point) const override;
    int requiredSamples() const override;

private:
    float a_, b_, c_;
};

#endif // LINEMODEL_H
