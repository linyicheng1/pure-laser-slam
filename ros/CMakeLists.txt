add_library(
        ros
        sensor_data/gnss_data.cpp
        subscriber/cloud_subscriber.cpp
        subscriber/gnss_subscriber.cpp
        subscriber/imu_subscriber.cpp
        tf_listerner/tf_lisener.cpp
        publisher/cloud_publisher.cpp
        publisher/odometry_publisher.cpp
)
target_link_libraries(
        ros
        ${catkin_LIBRARIES}
)

