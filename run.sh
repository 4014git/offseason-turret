#!/bin/sh
set -e
cmake -S . -B build
cmake --build build -j 24
./build/main