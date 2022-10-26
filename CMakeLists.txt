# fromhttps://github.com/isl-org/open3d-cmake-find-package
cmake_minimum_required(VERSION 3.18)

project(turret)

add_executable(main main.cpp)
find_package(Open3D REQUIRED)
target_link_libraries(main PRIVATE Open3D::Open3D)
find_package(Ceres REQUIRED)
target_link_libraries(main PRIVATE Ceres::ceres)