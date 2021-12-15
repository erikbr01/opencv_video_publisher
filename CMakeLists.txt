
cmake_minimum_required(VERSION 3.16)
project(mocap_publisher VERSION 0.1)
set(CMAKE_CXX_STANDARD 17)

# Find requirements
find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)

find_package(yaml-cpp CONFIG REQUIRED)

find_package(OpenCV REQUIRED)

add_subdirectory(lib)
add_subdirectory(app)
