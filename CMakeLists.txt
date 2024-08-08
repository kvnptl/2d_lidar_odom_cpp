cmake_minimum_required(VERSION 3.1)
project(2d_lidar_odom_cpp)

find_package(Eigen3 REQUIRED)
find_package(Open3D REQUIRED HINTS ${CMAKE_CURRENT_SOURCE_DIR}/open3d/lib/cmake)

add_subdirectory(dataloader)
add_subdirectory(viewer)
add_subdirectory(apps)
add_subdirectory(icp)