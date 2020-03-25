#ifndef __GNSS_SUB_H
#define __GNSS_SUB_H

#include <deque>
#include <ros/ros.h>
#include "sensor_msgs/NavSatFix.h"
#include "ros/sensor_data/gnss_data.hpp"

namespace laser_slam
{
    class GNSSSubscriber
    {
    public:
        GNSSSubscriber(ros::NodeHandle &nh,std::string topic_name,size_t buff_size);
        GNSSSubscriber() = default;
        void ParseData(std::deque<GNSSData>& deque_gnss_data);

    private:
        void msg_callback(const sensor_msgs::NavSatFixConstPtr &nav_sat_fix_ptr);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;
        std::deque<GNSSData> new_gnss_data_;
    };
}

#endif //__GNSS_SUB_H