    #include <iostream>
    #include <pcl/ModelCoefficients.h>
    #include <pcl/point_cloud.h>
    #include <pcl/point_types.h>
    #include <pcl/visualization/pcl_visualizer.h>
    #include <thread>

    struct Box
    {
        float x_min;
        float y_min;
        float z_min;
        float x_max;
        float y_max;
        float z_max;
    };

    int main(int argc, char** argv)
    {

        /////////////////////////////////////////// PLANE EXAMPLE /////////////////////////////////////////////////
        // Initialize a PCL Visualizer
        // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
        // viewer->setBackgroundColor(0, 0, 0);  // Set background to black

        // // Define the coefficients of the plane
        // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        // coefficients->values.resize(4); // 4 values for plane equation ax + by + cz + d = 0
        // coefficients->values[0] = 0.0;  // a (x-coefficient)
        // coefficients->values[1] = 0.0;  // b (y-coefficient)
        // coefficients->values[2] = 1.0;  // c (z-coefficient, normal pointing along Z-axis)
        // coefficients->values[3] = 0.0;  // d (distance from origin)

        // // Add the plane to the viewer
        // viewer->addPlane(*coefficients, "plane");

        // // Set the color of the plane to red (R, G, B format)
        // viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "plane");

        // // Set the transparency (alpha value) of the plane, where 0.0 is fully transparent and 1.0 is fully opaque
        // viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, "plane");

        // // Set camera position for a better view
        // viewer->setCameraPosition(0, 0, -3, 0, -1, 0); // Adjust the camera view
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////

        /////////////////////////////////////////// CUBE EXAMPLE /////////////////////////////////////////////////
        Box window;
        window.x_min = -10;
        window.x_max = 10;
        window.y_min = -10;
        window.y_max = 10;
        window.z_min = 0;
        window.z_max = 0;
        int zoom = 25;

        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Viewer"));
        viewer->setBackgroundColor(0, 0, 0);
        viewer->initCameraParameters();
        viewer->setCameraPosition(0, 0, zoom, 0, 1, 0);
        viewer->addCoordinateSystem(1.0);

        viewer->addCube(window.x_min, window.x_max, window.y_min, window.y_max, 0, 0, 1, 1, 1, "window");

        // Set the color of the cube to red using shape rendering properties (r = 1.0, g = 0.0, b = 0.0)
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "window"); // Red cube

        // Set transparency if needed (optional)
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,
                                            0.2,
                                            "window"); // Make it 20% opaque

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////

        // Main loop to keep the viewer open
        while (!viewer->wasStopped())
        {
            viewer->spinOnce(100);                                       // Update the viewer
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Sleep for a bit to reduce CPU usage
        }

        return 0;
    }
