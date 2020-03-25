cmake_minimum_required(VERSION 2.8.3)
project(laser_slam)
add_definitions(-std=c++11)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
    include
)

add_subdirectory(./ros)


add_executable(laser_slam Main.cpp)










