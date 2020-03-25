add_library(
        ros
        subscriber/cloud_subscriber.cpp
        subscriber/gnss_subscriber.cpp
        subscriber/imu_subscriber.cpp
)
target_link_libraries(
        ros
        ${catkin_LIBRARIES}
)

