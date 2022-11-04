# Project Layout

All of the code is in [newton.cpp](newton.cpp). [run.sh](run.sh) is a convenience program for CMake to compile and run the code ( it works on my machine ), [CMakeLists.txt](CMakeLists.txt), [.gitmodules](.gitmodules), and [.gitignore](.gitignore) are config files, and [autodiff](autodiff) and [eigen](eigen) are header only dependencies packaged as a git submodule.

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

This script assumes that you have a POSIX compliant shell, like bash, and the linux version of CMake ( I am not sure if the syntax is any different on other platforms ). If you need to dig into the weeds a bit more to compile the project, all CMake is currently doing is compiling [newton.cpp](newton.cpp) and adding autodiff and eigen as include directories. This should be possible to do with just a few flags with any c++ compiler.
