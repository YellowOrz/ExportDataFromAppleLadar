cmake_minimum_required(VERSION 3.6)
project(TrajectoryVisualization)
set(CMAKE_CXX_STANDARD 17)
include_directories("/usr/include/eigen3")

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

add_executable(trajectory trajectory.cpp)
target_link_libraries(trajectory ${Pangolin_LIBRARIES})


