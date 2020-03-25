#include <iostream>
#include <ros/ros.h>
#include "ros/subscriber/cloud_subscriber.hpp"
#include "ros/subscriber/imu_subscriber.hpp"
#include "ros/subscriber/gnss_subscriber.hpp"

int main(int argc,char *argv[])
{
    ros::init(argc,argv,"laser_slam");
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr = std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 100000);
    std::shared_ptr<IMUSubscriber> imu_sub_ptr = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr = std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);

    std::deque<CloudData> cloud_data_buff;
    std::deque<IMUData> imu_data_buff;
    std::deque<GNSSData> gnss_data_buff;

}
