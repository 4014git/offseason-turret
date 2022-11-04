# Project Layout

All of the code currently is in [newton.cpp](auto.cpp). [run.sh](run.sh) is a convenience program for CMake to compile and run the code ( it works on my machine ), there are some config files like [CMakeLists.txt](CMakeLists.txt), [.gitmodules](.gitmodules), and gitignore.

# Running code

Currently, I have only tested it on linux, so I am not sure how it will work for windows.

## Install dependencies

Clone the repo with `--recursive` so that you get the autodiff dependency.

Install [CMake](https://cmake.org/) and [Ninja](https://ninja-build.org/)

## Compile and run the code

```bash
./run.sh
```

### If the run script is not working

This script assumes that you have a POSIX compliant shell, like bash, and the linux version of CMake ( I am not sure if the syntax is any different on other platforms ). If you need to dig into the weeds a bit more to compile the project, all CMake is currently doing is compiling [auto.cpp](auto.cpp) and adding the autodiff directory to the list of include directories. This should be possible to do with just a few flags with any c++ compiler.
