# minimal CMakeLists.txt for the AWS SDK for C++
cmake_minimum_required(VERSION 2.8)

project(MLX90640)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")


# Link to the SDK shared libraries.
add_definitions(-DUSE_IMPORT_EXPORT)

# The executable name and its sourcefiles
add_executable(MLX90640 sensor.cpp main.cpp)

# The libraries used by your executable.
target_link_libraries(MLX90640 m bcm2835)
