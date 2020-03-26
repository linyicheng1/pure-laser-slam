#ifndef __IMU_DATA_
#define __IMU_DATA_

#include <Eigen/Dense>
namespace laser_slam
{
    class IMUData
    {
        struct LinearAcceleration
        {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
        };
        struct AngularVelocity
        {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
        };
        struct Orientation
        {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            double w = 0.0;
        };
    public:
        double time = 0.0;
        LinearAcceleration linear_acceletation;
        AngularVelocity angular_velocity;
        Orientation orientation;
    public:
        Eigen::Matrix3f GetOrientationMatrix()
        {
            Eigen::Quaterniond q(orientation.w,orientation.x,orientation.y,orientation.z);
            Eigen::Matrix3f matrix = q.matrix().cast<float>();
            return matrix;
        }
    };
}

#endif