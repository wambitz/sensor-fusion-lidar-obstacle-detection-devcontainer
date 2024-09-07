#pragma once

#include "render.h"

namespace render
{

class Car
{
public:
    Car(Vect3 setPosition, Vect3 setDimensions, Color setColor, std::string setName);
    void render(pcl::visualization::PCLVisualizer::Ptr& viewer);
    bool inbetween(double point, double center, double range); // collision helper function
    bool checkCollision(Vect3 point);

private:
    // units in meters
    Vect3 position;
    Vect3 dimensions;
    Color color;
    std::string name;
};

} // namespace render
