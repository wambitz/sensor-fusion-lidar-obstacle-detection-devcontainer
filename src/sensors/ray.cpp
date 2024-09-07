#include "ray.h"
#include <cmath>

namespace sensors
{

namespace lidar
{

Ray::Ray(Vect3 setOrigin, double horizontalAngle, double verticalAngle, double setResolution)
    : resolution(setResolution)
    , castDistance(0)
    , origin(setOrigin)
    , direction(resolution * cos(verticalAngle) * cos(horizontalAngle),
                resolution * cos(verticalAngle) * sin(horizontalAngle),
                resolution * sin(verticalAngle))
    , castPosition(origin)
{
}

void Ray::rayCast(const std::vector<Car>& cars,
                  double minDistance,
                  double maxDistance,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                  double slopeAngle,
                  double sderr)
{
    // reset ray
    castPosition = origin;
    castDistance = 0;
    bool collision = false;

    while (!collision && castDistance < maxDistance)
    {
        castPosition = castPosition + direction;
        castDistance += resolution;

        // check if there is any collisions with ground slope
        collision = (castPosition.z <= castPosition.x * tan(slopeAngle));

        // check if there is any collisions with cars
        if (!collision && castDistance < maxDistance)
        {
            for (Car car : cars)
            {
                collision |= car.checkCollision(castPosition);
                if (collision)
                    break;
            }
        }
    }

    if ((castDistance >= minDistance) && (castDistance <= maxDistance))
    {
        // add noise based on standard deviation error
        double rx = ((double)rand() / (RAND_MAX));
        double ry = ((double)rand() / (RAND_MAX));
        double rz = ((double)rand() / (RAND_MAX));
        cloud->points.push_back(
            pcl::PointXYZ(castPosition.x + rx * sderr, castPosition.y + ry * sderr, castPosition.z + rz * sderr));
    }
}

} // namespace lidar
} // namespace sensors
