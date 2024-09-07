#pragma once

#include "car.h"
#include "ray.h"
#include "render.h"

constexpr double pi = 3.1415;

namespace sensors
{

namespace lidar
{

class Lidar
{
public:
    Lidar(std::vector<render::Car> setCars, double setGroundSlope);

    // pcl uses boost smart pointers for cloud pointer so we don't have to worry about manually freeing the memory
    ~Lidar() = default;

    pcl::PointCloud<pcl::PointXYZ>::Ptr scan();
    Vect3 getPosition() const { return this->position; }

private:
    std::vector<Ray> rays;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    std::vector<Car> cars;
    Vect3 position;
    double groundSlope;
    double minDistance;
    double maxDistance;
    double resolution;
    double sderr;
};

} // namespace lidar
} // namespace sensors
