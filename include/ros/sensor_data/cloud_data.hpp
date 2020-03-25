#ifndef __CLOUD_DATA_H
#define __CLOUD_DATA_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
namespace laser_slam
{
    class CloudData
    {
    public:
        using POINT = pcl::PointXYZ;
        using CLOUD = pcl::PointCloud<POINT>;
        using CLOUD_PTR = CLOUD::Ptr;
    public:
        CloudData()
                :cloud_ptr(new CLOUD())
        {

        }
    public:
        CLOUD_PTR cloud_ptr;
        double time = 0.0;
    };
}

#endif //__CLOUD_DATA_H