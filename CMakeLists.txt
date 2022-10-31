cmake_minimum_required(VERSION 3.22)
project(turret)

add_executable(main main.cpp)
target_include_directories(main PUBLIC Ascent/include ensmallen/include armadillo-code/include)