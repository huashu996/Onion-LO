#pragma once
#ifndef Onion_LO_H
#define Onion_LO_H
//common
#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Core>
#include <vector>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <regex>
#include <filesystem>
#include <fstream>
#include <thread>
// ROS
#include "ros/ros.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "sensor_msgs/point_cloud2_iterator.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <signal.h>
#include <condition_variable>
#include <memory>
//file
#include <livox_ros_driver/CustomMsg.h>
#include <tbb/parallel_for.h>
#include <common_lib.h>
#include <onion/onion.hpp>
//pcl
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/random_sample.h>
#include <sophus/se3.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/math/Pose6D.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include "odom.hpp"
#include "kitti_helper.hpp"
using namespace std;
using namespace std::chrono;
namespace fs = std::filesystem;

using PointField = sensor_msgs::PointField;
MeasureGroup meas;
//----------------Class---------------------
class Onion_LO {
public:
    /// Ros node stuff
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber sub_lidar_;
    ros::Subscriber sub_pointcloud_;
    ros::Publisher repub_point;
    ros::Publisher odom_publisher_;
    ros::Publisher traj_publisher_;
    ros::Publisher frame_publisher_;
    ros::Publisher kpoints_publisher_;
    ros::Publisher local_map_publisher_;
    ros::Publisher bin_pointcloud_pub_;
    ros::Publisher octomap_pub_;
    nav_msgs::Path path_msg_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    ros::Time save_timestamp;
    ros::Time base_timestamp;
    bool first_scan{1};
    int queue_size_{1000};
	//yaml
	std::string odom_frame_{"odom"};
    std::string child_frame_{"base_link"};
    std::string lidar_type;
	std::string lidar_topic;

	int pcd_index = 0; 
	int save_frame_num_beg;
	int save_frame_num_end;
	bool Deskew = false;
	bool Save_path = false;
	bool pcd_save_en = false;
	double Resolution_v;
	double Resolution_h;
	//sync
	int scan_num = 0;
	double adj_voxel_size;
    double density;
    Odom::Onion_Odom Onion_odom_;
    Odom::Config config_;
    Onion onion;
	//PCL
	PointCloudXYZRGB::Ptr good_points;
	PointCloudXYZRGB::Ptr scan_keypoint_enhance;
	PointCloudXYZRGB::Ptr color_cloud;
	PointCloudXYZRGB::Ptr map_cloud;
	PointCloudXYZ::Ptr complete_map;
	PointCloudXYZRGB::Ptr save_map_points;
	std::vector<Sophus::SE3d> poses_;  
	Sophus::SE3d initial_guess;

	// ------------------------Function declaration------------------------------
	void processBinFolder(const std::string &folder_path);
	void KITTI_Callback(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg);
   	void livox_handler(const livox_ros_driver::CustomMsg::ConstPtr& livox_msg_in);
   	void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void Groundtruth_Callback(const nav_msgs::Odometry& odom_msg);
    void IMU_Callback(const sensor_msgs::Imu::ConstPtr &msg_in);
    bool Sync_packages(MeasureGroup &measgroup);
    void ruby128_handler(const sensor_msgs::PointCloud::ConstPtr& cloud_msg);
    void Groundtruth_Save( nav_msgs::Odometry& odom_msg);
    nav_msgs::Odometry Transform_Odom(nav_msgs::Odometry odom_msg);
    void DeskewLidarData(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg, Vector6dVector &deskew_scan);
	Vector6dVector XYZRGB2Vector6dVector(const PointCloudXYZRGB::Ptr& pcl_cloud);
	void resetParameters();
	Sophus::SE3d GetPredictionModel() const;
    Onion_LO(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    	: nh_(nh), pnh_(pnh){
		pnh_.getParam("common/pcd_save_en", pcd_save_en);
		pnh_.getParam("common/Save_path", Save_path);
		pnh_.getParam("common/save_frame_num_beg", save_frame_num_beg);
		pnh_.getParam("common/save_frame_num_end", save_frame_num_end);
		pnh_.getParam("common/lidar_topic", lidar_topic);
		pnh_.getParam("common/child_frame", child_frame_);
		pnh_.getParam("common/odom_frame", odom_frame_);
		
		pnh_.getParam("kiss_l/voxel_size", config_.voxel_size);
		pnh_.getParam("kiss_l/lidar_type", config_.type);
		pnh_.getParam("kiss_l/Deskew", config_.deskew);
		pnh_.getParam("kiss_l/exp_key_num", config_.exp_key_num);
		pnh_.getParam("kiss_l/Resolution_v", Resolution_v);
		pnh_.getParam("kiss_l/Resolution_h", Resolution_h);
		Onion_odom_ = Odom::Onion_Odom(config_);
		if (child_frame_ != "base_link") {
			static tf2_ros::StaticTransformBroadcaster br;
		    geometry_msgs::TransformStamped alias_transform_msg;
		    alias_transform_msg.header.stamp = ros::Time::now();
		    alias_transform_msg.transform.translation.x = 0.0;
		    alias_transform_msg.transform.translation.y = 0.0;
		    alias_transform_msg.transform.translation.z = 0.0;
		    alias_transform_msg.transform.rotation.x = 0.0;
		    alias_transform_msg.transform.rotation.y = 0.0;
		    alias_transform_msg.transform.rotation.z = 0.0;
		    alias_transform_msg.transform.rotation.w = 1.0;
		    alias_transform_msg.header.frame_id = child_frame_;
		    alias_transform_msg.child_frame_id = "base_link";
		    br.sendTransform(alias_transform_msg);
    	}
    	good_points.reset(new PointCloudXYZRGB());
    	scan_keypoint_enhance.reset(new PointCloudXYZRGB());
    	map_cloud.reset(new PointCloudXYZRGB());
    	save_map_points.reset(new PointCloudXYZRGB());
    	color_cloud.reset(new PointCloudXYZRGB());
    	complete_map.reset(new PointCloudXYZ());
		odom_publisher_ = pnh_.advertise<nav_msgs::Odometry>("odometry", queue_size_);
		frame_publisher_ = pnh_.advertise<sensor_msgs::PointCloud2>("frame", queue_size_);
		kpoints_publisher_ = pnh_.advertise<sensor_msgs::PointCloud2>("keypoints", queue_size_); 
		bin_pointcloud_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>("kitti_raw", queue_size_); 
		local_map_publisher_ = pnh_.advertise<sensor_msgs::PointCloud2>("local_map", queue_size_);
		octomap_pub_ = pnh_.advertise<octomap_msgs::Octomap>("octomap_binary", queue_size_);
		path_msg_.header.frame_id = odom_frame_;
		traj_publisher_ = pnh_.advertise<nav_msgs::Path>("trajectory", queue_size_);
		if (config_.type == "LIVOX") {
			sub_lidar_ = nh_.subscribe( lidar_topic, queue_size_, &Onion_LO::livox_handler, this);
		}
		else if (config_.type == "PANDAR"|| config_.type == "VELO") {
			sub_lidar_ = nh_.subscribe( lidar_topic, queue_size_, &Onion_LO::PointCloudCallback, this, ros::TransportHints().tcpNoDelay());
		}
		else if (config_.type == "KITTI"){
			sub_pointcloud_ = nh_.subscribe<sensor_msgs::PointCloud2>(lidar_topic, queue_size_, &Onion_LO::KITTI_Callback, this, ros::TransportHints().tcpNoDelay());
		}
		else if (config_.type == "RUBY") {
			sub_lidar_ = nh_.subscribe( lidar_topic, queue_size_, &Onion_LO::ruby128_handler, this, ros::TransportHints().tcpNoDelay());
		}
		else {
			printf( "Lidar type is wrong.\n" );
		}
		if(1)
        {
			cout << "[Ros_parameter]: common/lidar_topic: " <<lidar_topic<< endl;
			cout << "[Ros_parameter]: config_.voxel_size: " << config_.voxel_size << endl;
			cout << "[Ros_parameter]: config_.type: " << config_.type << endl;
			cout << "[Ros_parameter]: config_.deskew: " << config_.deskew << endl;
			cout << "[Ros_parameter]: LIDAR resolution_v: " << Resolution_v << endl;
			ROS_INFO("KISS-LV ROS 1 Odometry Node Initialized");
        }
    }
     ~Onion_LO(){};
};
#endif 
