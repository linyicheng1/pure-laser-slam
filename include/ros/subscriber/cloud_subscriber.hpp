/**
 * @brief 订阅点云数据
 */
#ifndef __CLOUD_SUB_H
#define __CLOUD_SUB_H
#include <#include <deque>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "ros/sensor_data/cloud_data.hpp"

namespace laser_slam
{
    class CloudSubscriber
    {
        CloudSubscriber(ros::NodeHandle &nh,std::string topic_name,size_t buff_size);
        CloudSubscriber() = default;
        void ParseData(std::deque<CloudData>& deque_cloud_data);

    private:
        void msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;
        std::deque<CloudData> new_cloud_data_;
    };
}

#endif //__CLOUD_SUB_H