#ifndef COMMON_LIB_H
#define COMMON_LIB_H
#pragma once

// Your header file contents here

#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
using namespace std;
using namespace Eigen;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector6dVector = std::vector<Vector6d>;

typedef pcl::PointXYZRGB ColorPointType;
typedef pcl::PointCloud<ColorPointType>  PointCloudXYZRGB;
typedef pcl::PointXYZI PointTypeI;
typedef pcl::PointCloud<PointTypeI>    PointCloudXYZI;
typedef pcl::PointXYZ PointTypeXYZ;
typedef pcl::PointCloud<PointTypeXYZ> PointCloudXYZ;

struct EIGEN_ALIGN16 PointXYZIT
{
    PCL_ADD_POINT4D; 
    float intensity;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIT,
                                   (float, x, x) 
                                   (float, y, y)
                                   (float, z, z) 
                                   (float, intensity, intensity)
                                   (double, time, time)
)
typedef pcl::PointCloud<PointXYZIT> PointCloudXYZIT;
struct MeasureGroup
{
    MeasureGroup()
    {
        this->lidar.reset(new PointCloudXYZIT());
    };
    double lidar_beg_time;
    double lidar_end_time;
    PointCloudXYZIT::Ptr lidar;
    sensor_msgs::PointCloud2::ConstPtr lidar_msg;
    nav_msgs::Odometry GPS;
};
struct TimestampField {
    int offset;
    int size;
};
#endif
