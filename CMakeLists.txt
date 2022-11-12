cmake_minimum_required(VERSION 3.22)
project(turret)

add_executable(newton newton.cpp)
add_executable(move moving_estimation.cpp)
target_include_directories(main PUBLIC autodiff eigen)