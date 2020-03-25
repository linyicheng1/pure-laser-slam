/**
 * @brief 订阅imu数据
 */
#ifndef __SUBSCRIBER_IMU_
#define __SUBSCRIBER_IMU_

#include <deque>
#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "ros/sensor_data/imu_data.hpp"

namespace laser_slam
{
    class IMUSubscriber
    {
    public:
        IMUSubscriber(ros::NodeHandle &nh,std::string topic_name,size_t buff_size);
        IMUSubscriber() = default;
        void ParseData(std::deque<IMUData>& deque_imu_data);

    private:
        void msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;
        std::deque<IMUData> new_imu_data_;
    };
}
#endif //__SUBSCRIBRE_IMU_
