cmake_minimum_required(VERSION 2.8.3)
project(laser_slam)
add_definitions(-std=c++11)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        std_msgs
        visualization_msgs
)

include_directories(
        include
        /usr/include/eigen3
        ${PCL_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

add_subdirectory(./ros)
add_subdirectory(./thirdPartLib)

add_executable(laser_slam Main.cpp)

target_link_libraries(
        laser_slam
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        libGeographiccc
)









