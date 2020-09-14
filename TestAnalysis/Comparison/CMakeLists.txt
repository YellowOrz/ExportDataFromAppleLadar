cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)

project(Comparison)

find_package(PCL REQUIRED)
find_package(g2o REQUIRED)
find_package(Sophus REQUIRED)
include_directories(${PCL_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        "/usr/include/eigen3/")
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(Comparison comparison.cpp)
target_link_libraries(Comparison
        g2o_core g2o_stuff
        ${PCL_LIBRARIES})


