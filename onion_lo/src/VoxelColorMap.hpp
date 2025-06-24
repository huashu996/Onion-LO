
#pragma once

#include <tsl/robin_map.h>
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <vector>
#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>
#include <thread>
#include <mutex>
#include <iostream>
using Vector6d = Eigen::Matrix<double, 6, 1>;
namespace Odom {

struct Color_VoxelHashMap {
	using Vector6dVector = std::vector<Vector6d>;
	using Vector6dVectorTuple = std::tuple<Vector6dVector, Vector6dVector>;
	using Vector6dVectorPlane = std::tuple<Vector6dVector, std::vector<Eigen::Matrix<double, 4, 1>>>;
	using Vector4i = Eigen::Matrix<int, 4, 1>;
	using Voxel = Vector4i;
	struct Color_VoxelBlock {
		Vector6dVector points;
		Color_VoxelBlock(const Vector6d& point) {
		    points.push_back(point);
		}
		inline void AddPoint(const Vector6d &point, double dis) const {
			auto &mutable_points = const_cast<Color_VoxelBlock*>(this)->points;
			if (mutable_points.size() < 100) {
				bool is_within_range = false;
				for (const auto &existing_point : mutable_points) {
						if (existing_point[3]==point[3] && existing_point[4]==point[4]){
							double distance = (existing_point.head<3>() - point.head<3>()).norm();
							if (distance < dis) {
								is_within_range = true;
								break;
							}
						}
				}
				if (!is_within_range) {
					mutable_points.push_back(point);
				}
		}
		}
		inline void NUM_AddPoint(const Vector6d &point, int NUM) const {
			auto &mutable_points = const_cast<Color_VoxelBlock*>(this)->points;
			if (mutable_points.size() < NUM) {
					mutable_points.push_back(point);
				}
		}
	};
	struct Color_VoxelHash {
        size_t operator()(const Voxel &voxel) const {
            const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
            return (((1 << 20) - 1) & vec[3]*(vec[0] * 73856093 ^ vec[1] * 19349663 ^ vec[2] * 83492791));
        }
	};

    
    explicit Color_VoxelHashMap(double voxel_size)
        : voxel_size_(voxel_size){}
    inline bool Color_Empty() const { return colormap_.empty(); }
    void Color_Update(const std::vector<Vector6d> &points, const Eigen::Vector3d &origin);
    void Color_Update(const std::vector<Vector6d> &points, const Sophus::SE3d &pose);
    void Color_AddPoints(const std::vector<Vector6d> &points);
    bool esti_plane(Eigen::Matrix<double, 4, 1> &pca_result, const Vector6dVector point_cloud, const double &threshold) const;
    void Color_RemovePointsFarFromLocation(const Eigen::Vector3d &origin);
    Vector6dVectorPlane Color_GetPlane(
    const Vector6dVector &points, double max_correspondance_distance) const ;
    Vector6dVectorTuple GetCorrespondences(const Vector6dVector &points, double factor) const;
    void Clean_color();
    std::vector<Vector6d> Pointcloud() const;
    double voxel_size_;
    double scan_num_;
    mutable int max_points_per_voxel_ = 100;
    mutable double dis_ = 0.5;
    mutable double max_distance_;
    mutable int num_range_ = 1;
    mutable int range_ = 27;
    tsl::robin_map<Voxel, Color_VoxelBlock, Color_VoxelHash> colormap_;
};
}  // namespace kiss_lv
