#include "line_model.h"

void LineModel::fit(const std::vector<pcl::PointXYZ>& points)
{
    // Fit line to two points
    float x1 = points[0].x;
    float y1 = points[0].y;
    float x2 = points[1].x;
    float y2 = points[1].y;

    a_ = y1 - y2;
    b_ = x2 - x1;
    c_ = x1 * y2 - x2 * y1;
}

float LineModel::distance(const pcl::PointXYZ& point) const
{
    float x = point.x;
    float y = point.y;
    return std::fabs(a_ * x + b_ * y + c_) / std::sqrt(a_ * a_ + b_ * b_);
}

int LineModel::requiredSamples() const
{
    return 2;
}
