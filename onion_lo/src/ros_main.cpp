#include "ros_main.hpp"
struct VoxelKey {
    int x, y, z;

    bool operator==(const VoxelKey &other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

namespace std {
using Header = std_msgs::Header;
template<>
struct hash<VoxelKey> {
    std::size_t operator()(const VoxelKey& k) const {
        return std::hash<int>()(k.x) ^ std::hash<int>()(k.y) ^ std::hash<int>()(k.z);
    }
};
}          
Vector6dVector Trans_Vector6(const std::vector<Eigen::Vector3d>& laser_data);  
PointCloudXYZRGB::Ptr Eigen6dToPointCloud2(const std::vector<Vector6d> &points);     
PointCloudXYZRGB::Ptr ConvertToXYZRGBPointCloud( Vector6dVector& map_cloud);
Vector6dVector PointCloud2ToEigen(const sensor_msgs::PointCloud2 &msg);
std::vector<double> Livox_time(const sensor_msgs::PointCloud2 &msg);
Vector6dVector DeSkewScan(const Vector6dVector &frame,
                                         const std::vector<double> &timestamps,
                                         const std::vector<Sophus::SE3<double>>& pose_deskew);
std::string FixFrameId(const std::string &frame_id);
PointCloudXYZRGB::Ptr V3ToXYZRGB( std::vector<Eigen::Vector3d>& map_cloud);
Vector6dVector XYZRGBToV6(const PointCloudXYZRGB::Ptr& cloud);
std::vector<double> GetTimestamps(const sensor_msgs::PointCloud2 &msg);
//-------------------------------------------------------------------------------------------------------------------------------------------
void Onion_LO::ruby128_handler(const sensor_msgs::PointCloud::ConstPtr& cloud_msg) {
    sensor_msgs::PointCloud2 cloud2_msg;
    sensor_msgs::convertPointCloudToPointCloud2(*cloud_msg, cloud2_msg);
	PointCloudCallback(boost::make_shared<sensor_msgs::PointCloud2>(cloud2_msg));
}
void Onion_LO::KITTI_Callback(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg){
		sensor_msgs::PointCloud2 new_msg = KITTICallback(lidar_msg);
		new_msg.header = lidar_msg->header;
		PointCloudCallback(boost::make_shared<sensor_msgs::PointCloud2>(new_msg));
}
void Onion_LO::livox_handler(const livox_ros_driver::CustomMsg::ConstPtr& livox_msg_in) {
	PointCloudXYZIT::Ptr pcl_data(new PointCloudXYZIT);
	auto time_end = livox_msg_in->points.back().offset_time;
    for (unsigned int i = 0; i < livox_msg_in->point_num; ++i) {
        PointXYZIT pt;
        pt.x = livox_msg_in->points[i].x;
        pt.y = livox_msg_in->points[i].y;
        pt.z = livox_msg_in->points[i].z;
        double dis_ = sqrt(pt.x*pt.x+pt.y*pt.y+pt.z*pt.z);
        if (dis_>0.1){
			double time = livox_msg_in->points[i].offset_time / (double)time_end;
			pt.time = time;
			pt.intensity = livox_msg_in->points[i].reflectivity;
			pcl_data->push_back(pt);
	    }
    }
    sensor_msgs::PointCloud2 T_pointcloud;
    pcl::toROSMsg(*pcl_data, T_pointcloud);
    T_pointcloud.header = livox_msg_in->header;
    PointCloudCallback(boost::make_shared<sensor_msgs::PointCloud2>(T_pointcloud));
}
void Onion_LO::PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg) {
	//1. deskew------------------------------------------------------
	cout << "\033[1;32m----------------Onion-LO----------------\033[0m" << endl;
	auto odom_start_time = std::chrono::high_resolution_clock::now();
	save_timestamp=lidar_msg->header.stamp;    
	int point_count = lidar_msg->width * lidar_msg->height;
	Vector6dVector deskew_scan;
	Vector6dVector color_scan;
	Vector6dVector key_points;
	Vector6dVector map_points;
	map_points.reserve(point_count);
	deskew_scan.reserve(point_count);
	color_scan.reserve(point_count);
	key_points.reserve(config_.exp_key_num);
	DeskewLidarData(lidar_msg, deskew_scan);
	//2. Onion--------------------------------------------------------
	auto start_time = std::chrono::high_resolution_clock::now();
	onion.Create_Onion(deskew_scan, Resolution_v,Resolution_h, config_.exp_key_num);
    color_scan = onion.Classifier();
    map_points = onion.NEW_Downsample_PointCloud(config_.exp_key_num, key_points);
    onion.clear();
    double factor = onion.Onion_Factor;
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end_time - start_time;

    //3.odom-------------------------------------------------------
    Sophus::SE3d new_pose = Sophus::SE3d();
    const auto prediction = GetPredictionModel();
    const auto last_pose = !poses_.empty() ? poses_.back() : Sophus::SE3d();
    const auto initial_guess = last_pose*prediction ;
	const auto keypoint = Onion_odom_.RegisterFrame(key_points, map_points, factor, initial_guess, new_pose);
	poses_.push_back(new_pose);
	auto save_local_map_ = Onion_odom_.LocalMap();
	auto odom_end_time = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> odom_duration = odom_end_time - odom_start_time;
	//4.results------------------------------------------------------
	color_cloud = Eigen6dToPointCloud2(color_scan);
    scan_keypoint_enhance = Eigen6dToPointCloud2(key_points);
	save_map_points = Eigen6dToPointCloud2(save_local_map_);
    const Eigen::Vector3d t_current = new_pose.translation();
    const Eigen::Quaterniond q_current = new_pose.unit_quaternion();

    // Broadcast the tf
    geometry_msgs::TransformStamped transform_msg;
    transform_msg.header.stamp = ros::Time::now();
    transform_msg.header.frame_id = odom_frame_;
    transform_msg.child_frame_id = child_frame_;
    transform_msg.transform.rotation.x = q_current.x();
    transform_msg.transform.rotation.y = q_current.y();
    transform_msg.transform.rotation.z = q_current.z();
    transform_msg.transform.rotation.w = q_current.w();
    transform_msg.transform.translation.x = t_current.x();
    transform_msg.transform.translation.y = t_current.y();
    transform_msg.transform.translation.z = t_current.z();
    tf_broadcaster_.sendTransform(transform_msg);
    // publish odometry msg
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = save_timestamp;                                                                    
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = child_frame_;
    odom_msg.pose.pose.orientation.x = q_current.x();
    odom_msg.pose.pose.orientation.y = q_current.y();
    odom_msg.pose.pose.orientation.z = q_current.z();
    odom_msg.pose.pose.orientation.w = q_current.w();
    odom_msg.pose.pose.position.x = t_current.x();
    odom_msg.pose.pose.position.y = t_current.y();
    odom_msg.pose.pose.position.z = t_current.z();
	
    // tum
    if (Save_path){
    	std::ofstream foutC(ROOT_DIR+string("results/Onion.txt"), std::ios::app);
		foutC.setf(std::ios::fixed, std::ios::floatfield);
		if (first_scan){
			base_timestamp = save_timestamp;
			first_scan = 0;
		}
		ros::Duration save_duration = save_timestamp - base_timestamp;
		foutC << save_timestamp << " "
		      << odom_msg.pose.pose.position.x << " "
		      << odom_msg.pose.pose.position.y << " "
		      << odom_msg.pose.pose.position.z << " "
		      << odom_msg.pose.pose.orientation.x << " "
		      << odom_msg.pose.pose.orientation.y << " "
		      << odom_msg.pose.pose.orientation.z << " "
		      << odom_msg.pose.pose.orientation.w << std::endl;
		foutC.close();
    }
    scan_num++;
    if (scan_num%1==0){
		geometry_msgs::PoseStamped pose_msg;
		pose_msg.pose = odom_msg.pose.pose;
		pose_msg.header = odom_msg.header;
		path_msg_.poses.push_back(pose_msg);
		traj_publisher_.publish(path_msg_);
	}
	sensor_msgs::PointCloud2 rgb_pointscan;
	pcl::toROSMsg(*color_cloud, rgb_pointscan);
	rgb_pointscan.header.stamp = ros::Time::now();
	rgb_pointscan.header.frame_id = child_frame_;
	frame_publisher_.publish(rgb_pointscan);
	
	sensor_msgs::PointCloud2 key_pointcloud;
	pcl::toROSMsg(*scan_keypoint_enhance, key_pointcloud);
	key_pointcloud.header.stamp = ros::Time::now();
	key_pointcloud.header.frame_id = child_frame_;
	kpoints_publisher_.publish(key_pointcloud);
	
    sensor_msgs::PointCloud2 map_msg;
	pcl::toROSMsg(*save_map_points, map_msg);	
	map_msg.header.stamp = ros::Time::now();
	map_msg.header.frame_id = odom_frame_;
	local_map_publisher_.publish(map_msg);
	Onion_LO::resetParameters();
}

Sophus::SE3d Onion_LO::GetPredictionModel() const {
    Sophus::SE3d pred = Sophus::SE3d();
    const size_t N = poses_.size();
    if (N < 2) return pred;
    return poses_[N - 2].inverse() * poses_[N - 1];
}
void Onion_LO::DeskewLidarData(const sensor_msgs::PointCloud2::ConstPtr& msg, Vector6dVector &deskew_scan) {
	sensor_msgs::PointCloud2 lidar_msg;
	lidar_msg = *msg;
	Vector6dVector points = PointCloud2ToEigen(lidar_msg);
	deskew_scan.resize(points.size());
	if (config_.deskew){
		const size_t N = poses_.size();
        if (N <= 2){
        	deskew_scan = points;
        }
        else{
			const auto timestamps = GetTimestamps(lidar_msg);
			deskew_scan = DeSkewScan(points, timestamps, poses_);

		}
	}
	if (!config_.deskew){
		deskew_scan = points;
	}
}
void Onion_LO::resetParameters(){ 
	  good_points->clear();
	  scan_keypoint_enhance->clear();
	  color_cloud->clear();
	  map_cloud->clear();
	  save_map_points->clear();
	  onion.onion_ball.clear();
}
Vector6dVector PointCloud2ToEigen(const sensor_msgs::PointCloud2 &msg)
{
    PointCloudXYZIT cloud;
    pcl::fromROSMsg(msg, cloud);
    Vector6dVector eigen_points;
    for (const auto& point : cloud.points)
    {
        Vector6d eigen_point;
        eigen_point << point.x, point.y, point.z, point.intensity, point.time, 0.0;
        eigen_points.push_back(eigen_point);
    }
    return eigen_points;
}
std::vector<double> Livox_time(const sensor_msgs::PointCloud2 &msg) {
    std::vector<double> timestamps;
    timestamps.reserve(msg.height * msg.width);
    sensor_msgs::PointCloud2ConstIterator<float> time(msg, "time");
    for (size_t i = 0; i < msg.height * msg.width; ++i, ++time) {
        timestamps.push_back(static_cast<double>(*time));
    }
    return timestamps;
}

bool isValidRotationMatrix(const Eigen::Matrix3d& R) {
    if (R.hasNaN()) {
        std::cerr << "Rotation matrix contains NaN or Inf!" << std::endl;
        return true;
    }
	bool isValid = R.isUnitary() && std::abs(R.determinant() - 1.0) < 1e-6;
	if (!isValid) {
		std::cerr << "Rotation matrix is not valid." << std::endl;
		 return true;
	}

    return false;
}

Vector6dVector DeSkewScan(const Vector6dVector &frame,
                                    const std::vector<double> &timestamps,
                                    const std::vector<Sophus::SE3d>& pose_deskew) {
    const size_t N = pose_deskew.size();
    double mid_pose_timestamp=0.5;
     if (N <= 3) return frame;
    const auto& start_pose = pose_deskew[N - 2];
    const auto& finish_pose = pose_deskew[N - 1];
    Eigen::Matrix3d start_rotation = start_pose.rotationMatrix();
    Eigen::Matrix3d finish_rotation = finish_pose.rotationMatrix();
    if (isValidRotationMatrix(start_rotation)) {
        std::cerr << "Start pose rotation matrix is invalid!" << std::endl;
        return frame;
    }

    if (isValidRotationMatrix(finish_rotation)) {
        std::cerr << "Finish pose rotation matrix is invalid!" << std::endl;
        return frame;
    }
    Vector6dVector corrected_frame(frame.size());
    const auto delta_pose = (start_pose.inverse() * finish_pose).log();
	tbb::parallel_for(size_t(0), frame.size(), [&](size_t i) {
	    const auto motion = Sophus::SE3d::exp((timestamps[i] - mid_pose_timestamp) * delta_pose);
	    corrected_frame[i].head<3>() = motion * frame[i].head<3>();
	    corrected_frame[i].tail<3>() = frame[i].tail<3>();
	});

    return corrected_frame;
}

auto GetTimestampField(const sensor_msgs::PointCloud2 &msg) {
    PointField timestamp_field;
    for (const auto &field : msg.fields) {
        if ((field.name == "t" || field.name == "timestamp" || field.name == "time")) {
            timestamp_field = field;
        }
    }
    if (!timestamp_field.count) {
        throw std::runtime_error("Field 't', 'timestamp', or 'time'  does not exist");
    }
    return timestamp_field;
}

std::vector<double> NormalizeTimestamps(const std::vector<double> &timestamps) {
    if (timestamps.empty()) return {};

    const double min_timestamp = *std::min_element(timestamps.cbegin(), timestamps.cend());
    const double max_timestamp = *std::max_element(timestamps.cbegin(), timestamps.cend());

    if (max_timestamp == min_timestamp) {
        return std::vector<double>(timestamps.size(), 0.5);
    }
    std::vector<double> timestamps_normalized(timestamps.size());
    std::transform(timestamps.cbegin(), timestamps.cend(), timestamps_normalized.begin(),
                   [&](const double &timestamp) { return (timestamp - min_timestamp) / (max_timestamp - min_timestamp); });

    return timestamps_normalized;
}
auto ExtractTimestampsFromMsg(const sensor_msgs::PointCloud2 &msg, const PointField &field) {
    // Extract timestamps from cloud_msg
    const size_t n_points = msg.height * msg.width;
    std::vector<double> timestamps;
    timestamps.reserve(n_points);

    // Option 1: Timestamps are unsigned integers -> epoch time.
    if (field.name == "t" || field.name == "timestamp") {
        sensor_msgs::PointCloud2ConstIterator<uint32_t> msg_t(msg, field.name);
        for (size_t i = 0; i < n_points; ++i, ++msg_t) { 
            timestamps.emplace_back(static_cast<double>(*msg_t));
        }
        // Covert to normalized time, between 0.0 and 1.0
        return NormalizeTimestamps(timestamps);
    }
    sensor_msgs::PointCloud2ConstIterator<double> msg_t(msg, field.name);
    for (size_t i = 0; i < n_points; ++i, ++msg_t) {
        timestamps.emplace_back(*msg_t);
    }
    return timestamps;
}
std::vector<double> GetTimestamps(const sensor_msgs::PointCloud2 &msg) {
    auto timestamp_field = GetTimestampField(msg);
    std::vector<double> timestamps = ExtractTimestampsFromMsg(msg, timestamp_field);

    return timestamps;
}

Vector6dVector Trans_Vector6(const std::vector<Eigen::Vector3d>& laser_data){
	std::vector<Vector6d> laser_data_extended;
	laser_data_extended.reserve(laser_data.size());
	for (const auto& point : laser_data) {
		Vector6d extended_point;
		extended_point << point, 55, 150, 55;
		laser_data_extended.push_back(extended_point);
	}
	return laser_data_extended;
}

struct VoxelInfo {
    std::set<int> pointIndices;
    bool isSparse = false;
};
struct Vector3iComparator {
    bool operator()(const Eigen::Vector3i& lhs, const Eigen::Vector3i& rhs) const {
        for (int i = 0; i < 3; ++i) {
            if (lhs[i] < rhs[i]) return true;
            if (lhs[i] > rhs[i]) return false;
        }
        return false;
    }
};



PointCloudXYZRGB::Ptr Eigen6dToPointCloud2(const std::vector<Vector6d> &points) {
    PointCloudXYZRGB::Ptr output_cloud(new PointCloudXYZRGB()); 
    for (const auto& point : points)
    {
        ColorPointType pcl_point;
        pcl_point.x = point[0];
        pcl_point.y = point[1];
        pcl_point.z = point[2];
        pcl_point.r = point[3];
        pcl_point.g = point[4];
        pcl_point.b = point[5];
        output_cloud->push_back(pcl_point);
    }
    return output_cloud;
}
PointCloudXYZRGB::Ptr ConvertToXYZRGBPointCloud( Vector6dVector& map_cloud) {
    PointCloudXYZRGB::Ptr cloud(new PointCloudXYZRGB);
    for (auto& vec : map_cloud) {
        pcl::PointXYZRGB point;
        point.x = static_cast<float>(vec[0]);
        point.y = static_cast<float>(vec[1]);
        point.z = static_cast<float>(vec[2]);

        uint8_t r = static_cast<uint8_t>(vec[3]);
        uint8_t g = static_cast<uint8_t>(vec[4]);
        uint8_t b = static_cast<uint8_t>(vec[5]);
        uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                        static_cast<uint32_t>(g) << 8 |
                        static_cast<uint32_t>(b));
        point.rgb = *reinterpret_cast<float*>(&rgb);

        cloud->push_back(point);
    }
    
    PointCloudXYZRGB::Ptr cloud_filtered(new PointCloudXYZRGB);
	pcl::RandomSample<pcl::PointXYZRGB> random_sample;
	random_sample.setInputCloud(cloud); 
	random_sample.setSample(10000); 
	random_sample.filter(*cloud_filtered);
	
    return cloud_filtered;
}
PointCloudXYZRGB::Ptr V3ToXYZRGB( std::vector<Eigen::Vector3d>& map_cloud) {
    PointCloudXYZRGB::Ptr cloud(new PointCloudXYZRGB);
    for (auto& vec : map_cloud) {
        pcl::PointXYZRGB point;
        point.x = static_cast<float>(vec[0]);
        point.y = static_cast<float>(vec[1]);
        point.z = static_cast<float>(vec[2]);
        uint8_t r = 0;
        uint8_t g =0;
        uint8_t b =0;
        uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                        static_cast<uint32_t>(g) << 8 |
                        static_cast<uint32_t>(b));
        point.rgb = *reinterpret_cast<float*>(&rgb);
        cloud->push_back(point);
    }
    return cloud;
}
Vector6dVector XYZRGBToV6(const PointCloudXYZRGB::Ptr& cloud) {
    Vector6dVector v6_vector;
    v6_vector.reserve(cloud->points.size());
    for (const auto& point : cloud->points) {
        Vector6d v6;
        v6 << point.x, point.y, point.z, 
              static_cast<double>(point.r), 
              static_cast<double>(point.g), 
              static_cast<double>(point.b);
        v6_vector.push_back(v6);
    }
    return v6_vector;
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "Onion_LO_main");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    Onion_LO* Onion_LO_instance = new Onion_LO(nh, nh_private);	
    ros::Rate rate(1000);
    ros::spin();
}
