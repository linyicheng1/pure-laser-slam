#ifndef __GNSS_DATA_H
#define __GNSS_DATA_H

#include <vector>
#include <string>
//#include <Geo>
namespace laser_slam
{
    class GNSSData
    {
    public:
        double time = 0.0;
        double longitude = 0.0;//经度
        double latitude = 0.0;//纬度
        double altitude = 0.0;//海拔
        double local_E;
        double local_N;
        double local_U;
        int status;
        int service;
    private:
        static bool origin_position_inited;
    public:
        void InitOriginPosition();
        void updateXYZ();
    };
}
#endif