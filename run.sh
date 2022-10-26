#!/bin/sh
set -e
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DOpen3D_ROOT=${HOME}/open3d_install
cmake --build build
./build/main