cmake_minimum_required(VERSION 2.8)
project(plot)

set(CMAKE_VERBOSE_MAKEFILE ON)

list(APPEND CMAKE_CXX_FLAGS "-std=c++17 -Wall -Wextra")

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")

set(SOURCES "plot.h"
            "example.cpp")

add_executable(plot ${SOURCES})

