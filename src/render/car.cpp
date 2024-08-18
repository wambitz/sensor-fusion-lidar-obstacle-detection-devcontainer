#include "car.h"

namespace render
{

Car::Car(Vect3 setPosition, Vect3 setDimensions, Color setColor, std::string setName)
    : position(setPosition)
    , dimensions(setDimensions)
    , color(setColor)
    , name(setName)
{
}

void Car::render(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // render bottom of car
    viewer->addCube(position.x - dimensions.x / 2,
                    position.x + dimensions.x / 2,
                    position.y - dimensions.y / 2,
                    position.y + dimensions.y / 2,
                    position.z,
                    position.z + dimensions.z * 2 / 3,
                    color.r,
                    color.g,
                    color.b,
                    name);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
                                        name);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, name);

    // render top of car
    viewer->addCube(position.x - dimensions.x / 4,
                    position.x + dimensions.x / 4,
                    position.y - dimensions.y / 2,
                    position.y + dimensions.y / 2,
                    position.z + dimensions.z * 2 / 3,
                    position.z + dimensions.z,
                    color.r,
                    color.g,
                    color.b,
                    name + "Top");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
                                        name + "Top");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                        color.r,
                                        color.g,
                                        color.b,
                                        name + "Top");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, name + "Top");
}

// collision helper function
bool Car::inbetween(double point, double center, double range)
{
    return (center - range <= point) && (center + range >= point);
}

bool Car::checkCollision(Vect3 point)
{
    return (inbetween(point.x, position.x, dimensions.x / 2) &&
            inbetween(point.y, position.y, dimensions.y / 2) &&
            inbetween(point.z, position.z + dimensions.z / 3, dimensions.z / 3)) ||
           (inbetween(point.x, position.x, dimensions.x / 4) &&
            inbetween(point.y, position.y, dimensions.y / 2) &&
            inbetween(point.z, position.z + dimensions.z * 5 / 6, dimensions.z / 6));
}

} // namespace render 