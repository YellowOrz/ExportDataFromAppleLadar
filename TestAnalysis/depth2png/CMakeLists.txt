cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)

project(depth2png)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories("/usr/include/eigen3")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(depth2png depth2png.cpp)
target_link_libraries(depth2png ${PCL_LIBRARIES} ${OpenCV_LIBS})