
// SOFTWARE.
#include "VoxelColorMap.hpp"

#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>
#include <cmath>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <Eigen/Core>
#include <algorithm>
#include <limits>
#include <tuple>
#include <utility>
#include <vector>
#include <future> 
using namespace std;

namespace {
struct ResultTuple {
    ResultTuple(std::size_t n) {
        source.reserve(n);
        target.reserve(n);
    }
    std::vector<Eigen::Vector3d> source;
    std::vector<Eigen::Vector3d> target;
};
struct ColorResultTuple {
    ColorResultTuple(std::size_t n) {
        source.reserve(n);
        target.reserve(n);
    }
    std::vector<Vector6d> source;
    std::vector<Vector6d> target;
};


}  // namespace
namespace Odom {
Color_VoxelHashMap::Vector6dVectorTuple Color_VoxelHashMap::GetCorrespondences(
    const Vector6dVector &points, double max_distance) const {
    auto GetClosestNeighboor = [&](const Vector6d &point) {
        auto kx = static_cast<int>(std::round(point[0] / voxel_size_));
        auto ky = static_cast<int>(std::round(point[1] / voxel_size_));
        auto kz = static_cast<int>(std::round(point[2] / voxel_size_));
        std::vector<Voxel> voxels; 
        voxels.reserve(range_);
		for (int i = kx-num_range_ ; i <= kx +num_range_; ++i) {
			for (int j = ky-num_range_ ; j <= ky+num_range_; ++j) {
				for (int k = kz-num_range_ ; k <= kz +num_range_; ++k) {
			    	Voxel voxel;
			    	voxel << i, j, k,1;
			    	if (point[4]==255)
        				voxel << i, j, k,-1;
			    	voxels.emplace_back(voxel);
				}
			}
		}
        using Vector6dVector = std::vector<Vector6d>;
        Vector6dVector neighboors;
        neighboors.reserve(range_*max_points_per_voxel_);
        std::for_each(voxels.cbegin(), voxels.cend(), [&](auto &voxel) {
            auto search = colormap_.find(voxel);
            if (search != colormap_.end()) {
		        const auto &points = search->second.points;
		            for (const auto &vpoint : points) {
		            	if ((vpoint.template head<3>() - point.template head<3>()).norm() < max_distance){
	                		if (vpoint[3]==point[3] && vpoint[4]==point[4])
	                			neighboors.emplace_back(vpoint);
	                		}
		            }
            }
        });
    std::vector<std::pair<Vector6d, double>> closest_neighbors;
    double closest_distance = std::numeric_limits<double>::max();
   	int NUM=3;
    std::for_each(neighboors.cbegin(), neighboors.cend(), [&](const auto &neighbor) {
		double distance;
		distance = ((neighbor.template head<3>() - point.template head<3>()).template cast<double>().norm());
	    if (closest_neighbors.size() < NUM) {
	        closest_neighbors.push_back(std::make_pair(neighbor, distance));
	        std::sort(closest_neighbors.begin(), closest_neighbors.end(), 
	                  [](const auto &a, const auto &b) { return a.second < b.second; });
	    } else if (distance < closest_neighbors.back().second) {
	        closest_neighbors.back() = std::make_pair(neighbor, distance);
	        std::sort(closest_neighbors.begin(), closest_neighbors.end(),
	                  [](const auto &a, const auto &b) { return a.second < b.second; });
	    }
    });
    Vector6d center = Vector6d::Zero(); 
    if (point[4]==255){
	if (closest_neighbors.size() == NUM){
		Vector6dVector vector6d_neighbors;
		for (const auto& neighbor : closest_neighbors) {
			vector6d_neighbors.push_back(neighbor.first);
		}
		Eigen::Matrix<double, 4, 1> plane;
		if (esti_plane(plane, vector6d_neighbors, 0.1f)){
			double a = plane[0];
			double b = plane[1];
			double c = plane[2];
			double d = plane[3];
			double norm = std::sqrt(a * a + b * b + c * c);
			if (norm < 1e-6) {
				std::cerr << "Plane normal norm is too small." << std::endl;
				return center;
			}
			Eigen::Vector3d normal(a, b, c);
			normal.normalize();
			double distance = (a * point[0] + b * point[1] + c * point[2] + d) / norm;
			Eigen::Vector3d projected_point;
			projected_point[0] = point[0] - normal[0] * distance;
			projected_point[1] = point[1] - normal[1] * distance;
			projected_point[2] = point[2] - normal[2] * distance;
			center.template head<3>() = projected_point;
			center.template tail<3>() = normal;
		}
	}
	}
	
	else{
		if (closest_neighbors.size() > 1){
			Eigen::Vector3d p1 = closest_neighbors[0].first.head<3>();
			Eigen::Vector3d p2 = closest_neighbors[1].first.head<3>();
			Eigen::Vector3d dir = p2 - p1;
			const double dir_norm = dir.norm();
			if (dir_norm < 1e-6) {
				std::cerr << "Coinciding points, cannot form line." << std::endl;
				return center;
			}
			dir.normalize();
			const Eigen::Vector3d vec_p1_to_point = point.head<3>() - p1;
			const double t = vec_p1_to_point.dot(dir);
			const Eigen::Vector3d projected_point = p1 + t * dir;
			center.template head<3>() = projected_point;
			center.template tail<3>() = dir;
		}
	}
   	return center;
    };
	using points_iterator = std::vector<Vector6d>::const_iterator;
	const auto [source, target] = tbb::parallel_reduce(
		tbb::blocked_range<points_iterator>{points.cbegin(), points.cend()},
		ColorResultTuple(points.size()),
		[max_distance, &GetClosestNeighboor](
		    const tbb::blocked_range<points_iterator>& r, ColorResultTuple res) -> ColorResultTuple {
		    auto& [src, tgt] = res;
		    src.reserve(r.size());
		    tgt.reserve(r.size());
		    for (const auto& point : r) {
		        Vector6d closest_neighboors = GetClosestNeighboor(point);
		        if (closest_neighboors[0]!=0 && closest_neighboors[1]!=0 && closest_neighboors[2]!=0){
		        	if ((closest_neighboors.template head<3>() - point.template head<3>()).norm() <max_distance){
						src.emplace_back(point);
						tgt.emplace_back(closest_neighboors);
				    }
				}
		    }
		    return res;
		},
		[](ColorResultTuple a, const ColorResultTuple& b) -> ColorResultTuple {
		    auto& [src, tgt] = a;
		    const auto& [srcp, tgtp] = b;
		    src.insert(src.end(),
		               std::make_move_iterator(srcp.begin()), std::make_move_iterator(srcp.end()));
		    tgt.insert(tgt.end(),
		               std::make_move_iterator(tgtp.begin()), std::make_move_iterator(tgtp.end()));
		    return a;
		});
    return std::make_tuple(source, target);
}
std::vector<Vector6d> Color_VoxelHashMap::Pointcloud() const { 

    std::vector<Vector6d> points;
    points.reserve(max_points_per_voxel_ * colormap_.size());
    for (const auto &[voxel, voxel_block] : colormap_) {
        (void)voxel;
        for (const auto &point : voxel_block.points) {
            points.push_back(point);
        }
    }
    return points;
}

bool Color_VoxelHashMap::esti_plane(Eigen::Matrix<double, 4, 1> &pca_result, const Vector6dVector point_cloud, const double &threshold) const{
    const int NUM_MATCH_POINTS = point_cloud.size(); 
    Eigen::Matrix<double, Eigen::Dynamic, 3> A(NUM_MATCH_POINTS, 3);
    Eigen::Matrix<double, Eigen::Dynamic, 1> b(NUM_MATCH_POINTS, 1);
    A.setZero();
    b.setOnes();
    b *= -1.0f;
    for (int j = 0; j < NUM_MATCH_POINTS; j++) {
        A(j, 0) = point_cloud[j](0);
        A(j, 1) = point_cloud[j](1);
        A(j, 2) = point_cloud[j](2);
    }
    Eigen::Matrix<double, 3, 1> normvec = A.colPivHouseholderQr().solve(b);
    double n = normvec.norm();
    pca_result(0) = normvec(0) / n;
    pca_result(1) = normvec(1) / n;
    pca_result(2) = normvec(2) / n;
    pca_result(3) = 1.0f / n;
    for (int j = 0; j < NUM_MATCH_POINTS; j++) {
         if (fabs(pca_result(0) * point_cloud[j](0)  +
           pca_result(1) * point_cloud[j](1)  +
           pca_result(2) * point_cloud[j](2)  +
           pca_result(3)) > threshold)  {
           return false;
        }
    }
    return true;
}

void Color_VoxelHashMap::Color_Update(const Vector6dVector &points, const Eigen::Vector3d &origin) {
    Color_AddPoints(points);
    Color_RemovePointsFarFromLocation(origin); 
}

void Color_VoxelHashMap::Color_Update(const Vector6dVector &points, const Sophus::SE3d &pose) {
	Vector6dVector vectors;
	vectors.reserve(points.size());
	std::transform(points.cbegin(), points.cend(), std::back_inserter(vectors),
		           [&](const auto &point) {
		               Eigen::Vector3d transformed_point = (pose * point.template head<3>()).template cast<double>();
		               Vector6d vector;
		               vector << transformed_point[0], transformed_point[1], transformed_point[2], point[3], point[4], point[5], point[6];
		               return vector;
		           });
	const Eigen::Vector3d &origin = pose.translation();
	Color_Update(vectors, origin);
}

double EuclideanDistance(const Vector6d& p1, const Vector6d& p2) {
    double dx = p1[0] - p2[0];
    double dy = p1[1] - p2[1];
    double dz = p1[2] - p2[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void Color_VoxelHashMap::Color_AddPoints(const std::vector<Vector6d> &points) {
    double save_dis  = std::clamp(std::round(dis_ * 10.0) / 10.0, 0.01, voxel_size_);
    for (const auto &point : points) {
        int voxel_x = static_cast<int>(std::round(point[0] / voxel_size_));
        int voxel_y = static_cast<int>(std::round(point[1] / voxel_size_));
        int voxel_z = static_cast<int>(std::round(point[2] / voxel_size_));
        Eigen::Vector4i voxel;
        voxel << voxel_x, voxel_y, voxel_z, 1;
        if (point[4]==255){
        	voxel << voxel_x, voxel_y, voxel_z, -1;
        }
        
    	auto search = colormap_.find(voxel);
	    if (search != colormap_.end()) {
	        auto &voxel_block = search->second;
	        voxel_block.AddPoint(point, save_dis);
	    }
	    else {
        	colormap_.emplace(voxel, Color_VoxelBlock(point));
    	}
    }
}

void Color_VoxelHashMap::Color_RemovePointsFarFromLocation(const Eigen::Vector3d &origin) {
    for (const auto &[voxel, voxel_block] : colormap_) {
            const auto &pt = voxel_block.points.front();
		    if ((pt.head<3>() - origin).norm() > (1000) ) {
		    	colormap_.erase(voxel);
		    }
       }
     }
}  // namespace



