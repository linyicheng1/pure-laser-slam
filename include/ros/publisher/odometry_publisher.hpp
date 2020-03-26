#ifndef __ODOMETRY_PUBLISHER_H
#define __ODOMETRY_PUBLISHER_H

#include <string>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

namespace laser_slam
{
    class OdometryPublisher
    {
    public:
        OdometryPublisher(ros::NodeHandle& nh,
                std::string topic_name,
                std::string base_frame_id,
                std::string child_frame_id,
                size_t buff_size);
        OdometryPublisher() = default;
        void Publish(const Eigen::Matrix4f& transfrom_matrix);

    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        nav_msgs::Odometry odometry_;
    };
}

#endif //__ODOMETRY_PUBLISHER_H