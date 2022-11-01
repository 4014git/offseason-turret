cmake_minimum_required(VERSION 3.22)
project(turret)

# add_executable(main main.cpp)
add_executable(main auto.cpp)
target_include_directories(main PUBLIC autodiff)