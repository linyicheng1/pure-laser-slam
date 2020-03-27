#include "ros/sensor_data/gnss_data.hpp"
#include <iostream>

bool laser_slam::GNSSData::origin_position_inited = false;
GeographicLib::LocalCartesian laser_slam::GNSSData::geo_converter;

namespace laser_slam
{
    void GNSSData::InitOriginPosition()
    {
        geo_converter.Reset(latitude,longitude,altitude);
        origin_position_inited = true;
    }
    void GNSSData::UpdateXYZ()
    {
        if(!origin_position_inited)
        {
            std::cout<<"gnss data has not inited"<<std::endl;
            return;
        }
        geo_converter.Forward(latitude,longitude,altitude,local_E,local_N,local_U);
    }
}