#pragma once

#include "car.h"
#include "render.h"
#include <chrono>
#include <ctime>

namespace sensors
{

namespace lidar
{

using render::Car;
using render::Vect3;

class Ray
{
public:
    // parameters:
    // setOrigin: the starting position from where the ray is cast
    // horizontalAngle: the angle of direction the ray travels on the xy plane
    // verticalAngle: the angle of direction between xy plane and ray
    // 				  for example 0 radians is along xy plane and pi/2 radians is stright up
    // resolution: the magnitude of the ray's step, used for ray casting, the smaller the more accurate but the more
    // expensive
    Ray(Vect3 setOrigin, double horizontalAngle, double verticalAngle, double setResolution);

    ~Ray() = default;

    void rayCast(const std::vector<Car> &cars,
                 double minDistance,
                 double maxDistance,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                 double slopeAngle,
                 double sderr);

private:
    double resolution;
    double castDistance;

    render::Vect3 origin;
    render::Vect3 direction;
    render::Vect3 castPosition;
};

} // namespace lidar
} // namespace sensors
