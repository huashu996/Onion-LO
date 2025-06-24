
#pragma once

#include <Eigen/Core>
#include <tuple>
#include <vector>
#include <sophus/se3.hpp>
#include <pcl/point_types.h>
#include "VoxelColorMap.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

namespace Odom {
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector6dVector = std::vector<Vector6d>;
typedef pcl::PointXYZRGB ColorPointType;
typedef pcl::PointCloud<ColorPointType>  PointCloudXYZRGB;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType>  PointCloudXYZ;
struct Config {
    // map params
    double voxel_size = 1;
    std::string type = "LIVOX";
    bool deskew = false;
    int exp_key_num = 100;
};


class Onion_Odom {
public:

	typedef pcl::PointXYZI PointTypeI;
	typedef pcl::PointCloud<PointTypeI> PointCloudXYZI;
	typedef pcl::PointXYZRGB ColorPointType;
	typedef pcl::PointCloud<ColorPointType>  PointCloudXYZRGB;
	double scan_num = 1;
	bool odomtry_error = false;
public:
    explicit Onion_Odom(const Config &config)
        : config_(config),
          color_local_map_(config.voxel_size){}

    Onion_Odom() : Onion_Odom(Config{}) {}

public:
	Vector6dVector RegisterFrame(const Vector6dVector color_frame, const Vector6dVector map_frame, double factor, const Sophus::SE3d &initial_guess, Sophus::SE3d &new_pose);
	Sophus::SE3d Color_RegisterFrame(
						   const Vector6dVector &key_frame,
                           const Sophus::SE3d &initial_guess,
                           double factor);
public:
    // Extra C++ API to facilitate ROS debugging
    std::vector<Vector6d> LocalMap() const { return color_local_map_.Pointcloud(); }; 

private:
    
    std::vector<double> distance_;  
    Config config_;    
    Color_VoxelHashMap color_local_map_; 
};

}  // namespace
