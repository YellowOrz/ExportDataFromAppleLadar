cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)

project(pcd2xyz)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#include_directories("/usr/include/eigen3")
add_executable(pcd2xyz pcd2xyz.cpp)
target_link_libraries(pcd2xyz ${PCL_LIBRARIES})