
#include "odom.hpp"
#include <Eigen/Core>
#include <tuple>
#include <vector>
#include <pcl/common/concatenate.h>
#include <fstream>
#include <chrono>
using namespace std;
namespace {
using Vector3i = Eigen::Matrix<int, 3, 1>;
using Voxel = Vector3i;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix3d = Eigen::Matrix<double, 3, 3>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix3_6d = Eigen::Matrix<double, 3, 6>;
inline double square(double x) { return x * x; }
struct VoxelHash {
    size_t operator()(const Voxel &voxel) const {
        const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
        return ((1 << 20) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349663 ^ vec[2] * 83492791);
    }
};
struct ResultTuple {
    ResultTuple() {
        JTJ.setZero();
        JTr.setZero();
    }

    ResultTuple operator+(const ResultTuple &other) {
        this->JTJ += other.JTJ;
        this->JTr += other.JTr;
        return *this;
    }

    Matrix6d JTJ;
    Vector6d JTr;
};
} 
namespace Odom {
constexpr int MAX_NUM_ITERATIONS_ = 200;
double ESTIMATION_THRESHOLD_=0.0005;
Vector6dVector Onion_Odom::RegisterFrame(const Vector6dVector color_frame, const Vector6dVector map_frame, double factor, const Sophus::SE3d &initial_guess, Sophus::SE3d &new_pose) {
	new_pose = Color_RegisterFrame(color_frame,
                                          initial_guess,
                                          factor);
    
    color_local_map_.dis_ = 0.5*factor;
    if (3*color_local_map_.dis_<0.5*1.732*color_local_map_.voxel_size_)
    	color_local_map_.num_range_ = 0;
    else if (3*color_local_map_.dis_<1.5*1.732*color_local_map_.voxel_size_)
    	color_local_map_.num_range_ = 1;
    else
    	color_local_map_.num_range_ = 2;
    cout<<"color_local_map_.dis_-----------"<<color_local_map_.dis_<<endl;
    cout<<"color_local_map_.num_range_-----------"<<color_local_map_.num_range_<<endl;
    color_local_map_.Color_Update(map_frame, new_pose);
	scan_num++;
	return color_frame;
}
//-------------------------------------------------------------------------------------------------------

void Color_TransformPoints(const Sophus::SE3d &T, Vector6dVector &points) {
    std::transform(points.cbegin(), points.cend(), points.begin(),
                   [&](const auto &point) { 
                   Eigen::Vector4d homogeneous_point(point[0], point[1], point[2], 1.0);
				   Eigen::Vector4d transformed_homogeneous_point = T.matrix() * homogeneous_point;
				   Vector6d transformed_point;
				   transformed_point << transformed_homogeneous_point[0], transformed_homogeneous_point[1], transformed_homogeneous_point[2], point[3], point[4], point[5],point[6];
  				   return transformed_point; });
}

Sophus::SE3d AlignClouds(const Vector6dVector &source,
                         const Vector6dVector &target,
                         double th) {
    auto compute_jacobian_and_residual = [&](auto i) {
        const Eigen::Vector3d pos_residual = (source[i].template head<3>() - target[i].template head<3>()).template cast<double>();
        Matrix3_6d J_r;
        J_r.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        J_r.block<3, 3>(0, 3) = -1.0 * Sophus::SO3d::hat(source[i].template head<3>().template cast<double>());
        return std::make_tuple(J_r, pos_residual);
    };

    const auto &[JTJ, JTr] = tbb::parallel_reduce(
        tbb::blocked_range<size_t>{0, source.size()},
        ResultTuple(),
        [&](const tbb::blocked_range<size_t> &r, ResultTuple J) -> ResultTuple {
            auto Weight = [&](double residual, double th_) { return th_ / (th_ + residual); };
            auto &[JTJ_private, JTr_private] = J;
            for (auto i = r.begin(); i < r.end(); ++i) {
                const auto &[J_r, pos_residual] = compute_jacobian_and_residual(i);
                double depth = source[i].template head<3>().norm();
                double depth_weight = 1.0 / (depth + 1e-1);
                double w = Weight(pos_residual.norm(), th) * depth_weight;
                JTJ_private.noalias() += J_r.transpose() * w * J_r;
                JTr_private.noalias() += J_r.transpose() * w * pos_residual;
            }
            return J;
        },
        [&](ResultTuple a, const ResultTuple &b) -> ResultTuple { return a + b; });
    const Vector6d x = JTJ.ldlt().solve(-JTr);
    return Sophus::SE3d::exp(x);
}



Sophus::SE3d Onion_Odom::Color_RegisterFrame(
						   const Vector6dVector &key_frame,
                           const Sophus::SE3d &initial_guess,
                           double factor) {
    Vector6dVector key_source = key_frame; 
    Color_TransformPoints(initial_guess, key_source); 
    Sophus::SE3d T_icp = Sophus::SE3d();
	for (int j = 0; j < MAX_NUM_ITERATIONS_; ++j) {
		const auto &[src, tgt] = color_local_map_.GetCorrespondences(key_source, color_local_map_.dis_ * 3.0);
		auto estimation = AlignClouds(src, tgt, color_local_map_.dis_/3.0);
		if (estimation.matrix().isApprox(Eigen::Matrix4d::Identity(), 1e-5)) {
    		continue;
		}
		Color_TransformPoints(estimation, key_source);
		T_icp = estimation * T_icp;
		if (estimation.log().norm() < ESTIMATION_THRESHOLD_) {
		    break;
		}
	}
    return T_icp*initial_guess;
}
}  // namespace
