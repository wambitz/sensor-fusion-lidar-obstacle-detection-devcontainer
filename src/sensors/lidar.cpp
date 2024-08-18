#include "lidar.h"

using render::Car;

namespace sensors
{

namespace lidar
{

Lidar::Lidar(std::vector<Car> setCars, double setGroundSlope)
    : cloud(new pcl::PointCloud<pcl::PointXYZ>())
    , position(0, 0, 2.6)
{
    // Set minDistance to 5 to remove points from roof of ego car
    minDistance = 5;
    maxDistance = 50;
    resolution = 0.2;

    // Set sderr to 0.2 to get more interesting pcd files
    sderr = 0.2;
    cars = setCars;
    groundSlope = setGroundSlope;

    // Increase number of layers to 8 to get higher resolution pcd
    int numLayers = 8;

    // the steepest vertical angle
    double steepestAngle = 30.0 * (-pi / 180);
    double angleRange = 26.0 * (pi / 180);

    // Set to pi/64 to get higher resolution pcd
    double horizontalAngleInc = pi / 64;
    double angleIncrement = angleRange / numLayers;

    for (double angleVertical = steepestAngle; angleVertical < steepestAngle + angleRange;
         angleVertical += angleIncrement)
    {
        for (double angle = 0; angle <= 2 * pi; angle += horizontalAngleInc)
        {
            Ray ray(position, angle, angleVertical, resolution);
            rays.push_back(ray);
        }
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Lidar::scan()
{
    cloud->points.clear();
    auto startTime = std::chrono::steady_clock::now();

    for (Ray ray : rays)
    {
        ray.rayCast(cars, minDistance, maxDistance, cloud, groundSlope, sderr);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    cout << "ray casting took " << elapsedTime.count() << " milliseconds" << endl;

    cloud->width = cloud->points.size();
    cloud->height = 1; // one dimensional unorganized point cloud dataset

    return cloud;
}

} // namespace lidar
} // namespace sensors
