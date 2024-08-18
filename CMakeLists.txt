cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Set C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)

# Add compiler flags for debugging
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")

# Set optimization level to none for debug builds
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")

# Enable all warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

project(playback)

# Find the PCL package
find_package(PCL 1.2 REQUIRED)

# Find the Boost package with filesystem component required
find_package(Boost REQUIRED COMPONENTS filesystem)

# Include directories for PCL and Boost
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(src/render)
include_directories(src/sensors)

# Link directories for PCL
link_directories(${PCL_LIBRARY_DIRS})

# Add definitions from PCL
add_definitions(${PCL_DEFINITIONS})

# Remove potentially problematic PCL library
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

# Add source files to the executable
add_executable(environment 
    src/environment.cpp 
    src/render/render.cpp 
    src/render/car.cpp
    src/processPointClouds.cpp
    src/sensors/lidar.cpp 
    src/sensors/ray.cpp
)

target_link_libraries (environment ${PCL_LIBRARIES} ${Boost_LIBRARIES})
