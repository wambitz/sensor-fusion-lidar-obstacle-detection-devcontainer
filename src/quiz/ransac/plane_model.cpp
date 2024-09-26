#include "plane_model.h"

void PlaneModel::fit(const std::vector<pcl::PointXYZ>& points)
{
    // Fit plane to three points
    float x1 = points[0].x;
    float y1 = points[0].y;
    float z1 = points[0].z;

    float x2 = points[1].x;
    float y2 = points[1].y;
    float z2 = points[1].z;

    float x3 = points[2].x;
    float y3 = points[2].y;
    float z3 = points[2].z;

    float i = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
    float j = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
    float k = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);

    a_ = i;
    b_ = j;
    c_ = k;
    d_ = -(i * x1 + j * y1 + k * z1);
}

float PlaneModel::distance(const pcl::PointXYZ& point) const
{
    float x = point.x;
    float y = point.y;
    float z = point.z;
    return std::fabs(a_ * x + b_ * y + c_ * z + d_) / std::sqrt(a_ * a_ + b_ * b_ + c_ * c_);
}

int PlaneModel::requiredSamples() const
{
    return 3;
}
