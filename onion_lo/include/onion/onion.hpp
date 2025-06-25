#include<iostream>
#include <cmath>
#include <omp.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues> 
#include <tbb/parallel_for.h>
#include <tbb/concurrent_hash_map.h>
#include <tbb/tbb.h>
#include <mutex>
#include <vector>
#include <functional> 
#include <unordered_map>
#include <algorithm>
#include <numeric>
#include <tsl/robin_map.h>

std::mutex print_mutex;
using namespace std;
class Onion {
public:
    double layer_thickness = 5;
    double onion_angle = 3.0;
private:
    //---------------------------------------
    int seg_min_num = 5;
    int exp_key_num = 1200;
    double min_range = 0.0;
    double max_range = 200;
    double angle_v = 0.4;
    double angle_h = 0.01;
    double seg_angle = 0.4;
    //------------------------------Layer------------------------------------
	struct Layer {
		int face;
		int R;
		bool operator==(const Layer& other) const {
		    return face == other.face && R == other.R;
		}
	};
	struct LayerHash {
		std::size_t hash(const Layer& layer) const {
		    std::size_t h1 = std::hash<int>()(layer.face);
		    std::size_t h2 = std::hash<int>()(layer.R);
		    return h1 ^ (h2 << 1);
		}
		std::size_t operator()(const Layer& layer) const {
		    return hash(layer);
		}
		bool equal(const Layer& lhs, const Layer& rhs) const {
		    return lhs.face == rhs.face && lhs.R == rhs.R;
		}
	};
	struct LayerBlock {
		double volume;
		double plane_num = 0;
		int seg_num;
		Vector6dVector seg_points;
		Vector6dVector p_points;
		Vector6dVector np_points;
		Vector6dVector points;
		LayerBlock() = default;
		LayerBlock(const Vector6d &point) {
		    AddPoint(point);
		}
		inline void AddPoint(const Vector6d &point) {
		    points.push_back(point);
		}
		void Merge(const LayerBlock& other) {
		    for (const auto& point : other.points) {
		        this->points.push_back(point);
		    }
		}
		void Merge(LayerBlock&& other) {
		    this->points.insert(this->points.end(), std::make_move_iterator(other.points.begin()), std::make_move_iterator(other.points.end()));
		}
	};
	//----------------------------Cell---------------------------------
	struct Cell {
		int x, y, z;
		bool operator==(const Cell& other) const {
		    return x == other.x && y == other.y && z == other.z;
		}
	};
	struct CellHash {
		std::size_t hash(const Cell& cell) const {
		    std::size_t hx = std::hash<int>()(cell.x);
		    std::size_t hy = std::hash<int>()(cell.y);
		    std::size_t hz = std::hash<int>()(cell.z);
		    return hx ^ (hy << 1) ^ (hz << 2);
		}
		bool equal(const Cell& a, const Cell& b) const {
		    return a == b;
		}
	};
	struct CellBlock {
		int num_points_ = 0;
		bool intensity_fea = 0;
		bool line_fea = 0;
		bool planar_fea = 0;
		Vector6dVector points;
		void AddPoint(const Vector6d &point) {
		    points.push_back(point);
		    num_points_++;
		}
		void Merge(const CellBlock& other) {
		    points.insert(points.end(), other.points.begin(), other.points.end());
		    num_points_ += other.num_points_;
		}
	};
	void thread_safe_print(const std::string& message1, const std::string& message2, const std::string& message3, const std::string& message4, const std::string& message5, const std::string& message6) {
		std::lock_guard<std::mutex> lock(print_mutex);
		std::cout <<message1<< message2<<message3<<message4<<message5<<message6<<std::endl;
	}
public:
	double Total_V;
	double Plane_rotio;
	int Total_NUM;
	int Raw_points_num;
	double Onion_Factor;
	Vector6dVector Seg_points;
	tbb::concurrent_hash_map<Layer, LayerBlock, LayerHash> onion_ball;
	void Create_Onion(Vector6dVector PointCloud, double Resolution_v,double Resolution_h, double key_num) {
		exp_key_num = key_num;
		angle_v = Resolution_v;
		angle_h = Resolution_h;
		seg_angle = std::max(angle_v, angle_h);
		Raw_points_num = PointCloud.size();
		tbb::combinable<std::unordered_map<Layer, LayerBlock, LayerHash>> local_onion_balls;
		tbb::parallel_for(tbb::blocked_range<size_t>(0, PointCloud.size()), [&](const tbb::blocked_range<size_t>& r) {
		    auto& local_map = local_onion_balls.local();
		    for (size_t i = r.begin(); i != r.end(); ++i) {
		        auto& point = PointCloud[i];
		        Layer layer;
		        double x = point[0];
		        double y = point[1];
		        double z = point[2];
		        double dist = std::sqrt(x * x + y * y + z * z);
		        if (dist>min_range || dist < max_range){
				    double theta = std::acos(z / dist);
				    double phi = atan2(y, x);
					layer.R = std::ceil(dist / layer_thickness);
					if (theta <= M_PI / 4) {
						layer.face = 1; 
					} else if (theta >= 3 * M_PI / 4) {
						layer.face = -1;
					} else if (phi >= -M_PI / 4 && phi <= M_PI / 4) {
						layer.face = 2; 
					} else if (phi >= 3 * M_PI / 4 && phi <= 5 * M_PI / 4) {
						layer.face = -2; 
					} else if (phi >= M_PI / 4 && phi <= 3 * M_PI / 4) {
						layer.face = 3; 
					} else {
						layer.face = -3; 
					}
					auto it = local_map.find(layer);
					if (it == local_map.end()) {
						local_map[layer] = LayerBlock(point);
					} else {
						it->second.AddPoint(point);
					}
				}
		    }
		});
		local_onion_balls.combine_each([&](std::unordered_map<Layer, LayerBlock, LayerHash>& local_map) {
		    for (auto& pair : local_map) {
		        tbb::concurrent_hash_map<Layer, LayerBlock, LayerHash>::accessor accessor;
		        if (onion_ball.insert(accessor, pair.first)) {
		            accessor->second = std::move(pair.second);
		        } else {
		            accessor->second.Merge(std::move(pair.second));
		        }
		    }
		});
	}
void IG_Classifier(const Vector6dVector& input_point, double voxelSize, double v_size,Vector6dVector& p_point, Vector6dVector& np_point, double &plane_rate, double &V, Vector6dVector& all_points) {
	double plane_num = 0;
	double no_plane_num = 0;
    p_point.reserve(input_point.size());
    all_points.reserve(input_point.size());
    np_point.reserve(input_point.size());
    tbb::concurrent_hash_map<Cell, CellBlock, CellHash> segVoxels;
    tbb::concurrent_hash_map<Cell, CellBlock, CellHash> fixedVoxels;
    for (const auto& p : input_point) {
        Cell cell_key{
            static_cast<int>(std::floor(p[0] / voxelSize)),
            static_cast<int>(std::floor(p[1] / voxelSize)),
            static_cast<int>(std::floor(p[2] / voxelSize))
        };
        tbb::concurrent_hash_map<Cell, CellBlock, CellHash>::accessor accessor;
        if (segVoxels.insert(accessor, cell_key)) {
            accessor->second = CellBlock();
        }
        accessor->second.AddPoint(p);
        //--------------------------------------------------------
        Cell fixed_cell_key{
		    static_cast<int>(std::floor(p[0] / v_size)),
		    static_cast<int>(std::floor(p[1] / v_size)),
		    static_cast<int>(std::floor(p[2] / v_size))
		};
		tbb::concurrent_hash_map<Cell, CellBlock, CellHash>::accessor fixed_accessor;
		if (fixedVoxels.insert(fixed_accessor, fixed_cell_key)) {
		    fixed_accessor->second = CellBlock();
		}
		fixed_accessor->second.AddPoint(p);
    }
    int occupiedVoxelCount = 0;
	for (auto it = fixedVoxels.begin(); it != fixedVoxels.end(); ++it) {
		if (it->second.num_points_ > 3) {
		    occupiedVoxelCount++;
		}
	}
	double computedVolume = occupiedVoxelCount * std::pow(v_size, 3);
	double NUM = 0;
	double p_NUM = 0;
	double np_NUM = 0;
	double use_points = 0;
    for (auto& [cell_key, cell_block] : segVoxels) {
        double point_count = cell_block.num_points_;
        double con_w = point_count/std::pow(v_size, 3);
        if (point_count >= seg_min_num) {
        	NUM++;
        	use_points += point_count;
        	std::vector<double> intensities;
            Eigen::Matrix3f covariance_matrix = Eigen::Matrix3f::Zero();
            Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
            Eigen::Matrix<float, 1, 1> covariance_matrix_i = Eigen::Matrix<float, 1, 1>::Zero();
			float intensity_mean = 0.0f;
            for (const auto& point : cell_block.points) {
                centroid[0] += point[0];
                centroid[1] += point[1];
                centroid[2] += point[2];
                intensity_mean += point[3];
                intensities.push_back(point[3]);
            }
            centroid /= static_cast<float>(point_count);
			intensity_mean /= static_cast<float>(point_count);
            //---------------------------------------------------------------------
			std::sort(intensities.begin(), intensities.end());
			double intensity_median;
			size_t size = intensities.size();
			if (size % 2 == 0) {
				intensity_median = (intensities[size / 2 - 1] + intensities[size / 2]) / 2.0;
			} else {
				intensity_median = intensities[size / 2];
			}
			double variance_sum = 0.0;
			for (const auto& intensity : intensities) {
				variance_sum += std::pow(intensity - intensity_mean, 2);
			}
			double variance = variance_sum / intensities.size();
			double standard_deviation = std::sqrt(variance);
			//---------------------------------------------------------------------
            for (const auto& point : cell_block.points) {
                Eigen::Vector3f p(point[0], point[1], point[2]);
                covariance_matrix += (p - centroid) * (p - centroid).transpose();
                covariance_matrix_i += Eigen::Matrix<float, 1, 1>::Constant((point[3] - intensity_mean) * (point[3] - intensity_mean));
            }
            covariance_matrix /= static_cast<float>(point_count);
            covariance_matrix_i /= static_cast<float>(point_count);
			Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 1, 1>> eigen_solver_i(covariance_matrix_i);
			float eigenvalue_i = eigen_solver_i.eigenvalues()[0];
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance_matrix);
            Eigen::Vector3f eigenvalues = eigen_solver.eigenvalues();
            float sum = eigenvalues.sum();
            float e0 = eigenvalues[0] / sum;
			float e1 = eigenvalues[1] / sum;
			float e2 = eigenvalues[2] / sum;
			double score = e1/(e0+e2);
            bool isintensity = (eigenvalue_i >50);
            bool isPlane = e2/e0>10 && e1/e0>10;
            if (isPlane) {
            	p_NUM++;
		        for (auto& p : cell_block.points) {
	        		p[3] = 0;
					p[4] = 255;
					p[5] = 0;
					
					if (isintensity && (p[3]<0.3*eigenvalue_i || p[3]>0.7*eigenvalue_i)){
						p[3] = 255;
						p[4] = 255;
					}
					plane_num++;
					p_point.push_back(p);
					all_points.push_back(p);
		        }
            }
            else{
            	np_NUM++;
            	for (auto& p : cell_block.points) {
					p[3] = 255;
					p[4] = 0;
					p[5] = 0;
					no_plane_num++;
					np_point.push_back(p);
					all_points.push_back(p);
		        }
            }
    }
    }
    if (use_points>0){
		plane_rate = p_NUM/(use_points);
		V = computedVolume;
    }
}

//-----------------------------------Classifier--------------------------------
Vector6dVector Classifier() {
    Seg_points.reserve(Raw_points_num);
    double total_volume = 0.0;
    double total_num = 0.0;
    double plane_num = 0.0;
	double Layer_size = 0.0;
	Layer_size=onion_ball.size();
    std::mutex mutex;
    tbb::parallel_for(tbb::blocked_range<size_t>(0, onion_ball.size()), [&](const tbb::blocked_range<size_t>& range) {
        Vector6dVector local_points_thread;
        double local_total_num = 0.0;
        double local_plane_num = 0.0;
        double local_total_volume = 0.0;
        for (size_t i = range.begin(); i < range.end(); ++i) {
            auto it = onion_ball.begin();
            std::advance(it, i);
            const auto& layer = it->first;
            auto& block = it->second;
            if (layer.R >= 1.0 && layer.R < 50.0 && block.points.size() > 30) {
                double R = layer.R * layer_thickness;
                double class_size = std::round(R * std::sin(3.0 *seg_angle * M_PI / 180.0) * 10.0) / 10.0;
                double v_size = std::round(R * std::sin(onion_angle * M_PI / 180.0) * 10.0) / 10.0;
                Vector6dVector plane_points_local;
                Vector6dVector no_plane_points_local;
                Vector6dVector total_points;
                double volume = 0.0;
                double plane_rate_local = 0.0;
                IG_Classifier(block.points, class_size, v_size, plane_points_local, 
                				no_plane_points_local, plane_rate_local, volume, total_points);
                block.seg_points = total_points;
                block.p_points = plane_points_local;
                block.np_points = no_plane_points_local;
                block.seg_num = total_points.size();
                block.plane_num = plane_points_local.size();
                block.volume = volume;
                local_points_thread.insert(local_points_thread.end(), total_points.begin(), total_points.end());
                local_total_num += block.seg_num;
                local_plane_num += block.plane_num;
                local_total_volume += block.volume;
                thread_safe_print(
                    "layer.Face " + std::to_string(layer.face),
                    "  layer.R " + std::to_string(layer.R),
                    "  v_size " + std::to_string(v_size),
                    "  class_size " + std::to_string(class_size),
                    "  block.volume " + std::to_string(block.volume),
                    "  block.seg_num " + std::to_string(block.seg_num)
                );
            }
        }  
        std::lock_guard<std::mutex> lock(mutex);
        Seg_points.insert(Seg_points.end(), local_points_thread.begin(), local_points_thread.end());
        total_num += local_total_num;
        plane_num += local_plane_num;
        total_volume += local_total_volume;
    });
    Total_NUM = Seg_points.size();
    Plane_rotio = Total_NUM > 0 ? static_cast<double>(plane_num) / Total_NUM : 0.0;
    Total_V = total_volume;
    Onion_Factor = std::pow(total_volume/exp_key_num, 1.0/3.0);
    return Seg_points;
}
using Vector3i = Eigen::Matrix<int, 3, 1>;
using Voxel = Vector3i;
struct VoxelHash {
    size_t operator()(const Voxel &voxel) const {
        const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
        return ((1 << 20) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349663 ^ vec[2] * 83492791);
    }
};
Vector6dVector D_PointCloud(double target_total_points, Vector6dVector inputs) {
        Vector6dVector downsampled_points;
    	downsampled_points.reserve(target_total_points);
        int step = std::round(inputs.size() / target_total_points);
        if (step < 1) step=1;
        for (int i = 0; i < inputs.size(); i += step) {
            downsampled_points.push_back(inputs[i]);
        }
        return downsampled_points;
}
Vector6dVector Voxel_Downsample(double target_total_points, const Vector6dVector& inputs, double V) {
    Vector6dVector frame_downsampled;
    if (inputs.size() > target_total_points) {
        double voxel_size = std::pow(V / target_total_points, 1.0 / 3.0);
        double inv_voxel_size = 1.0 / voxel_size;
        tsl::robin_map<Voxel, Vector6d, VoxelHash> grid;
        grid.reserve(inputs.size());
        for (const auto &point : inputs) {
            int voxel_x = static_cast<int>(point[0] * inv_voxel_size);
            int voxel_y = static_cast<int>(point[1] * inv_voxel_size);
            int voxel_z = static_cast<int>(point[2] * inv_voxel_size);
            Voxel voxel;
            voxel << voxel_x, voxel_y, voxel_z;
            grid.try_emplace(voxel, point);
        }
        frame_downsampled.reserve(grid.size());
        for (const auto &pair : grid) {
            frame_downsampled.push_back(pair.second);
        }
    } else {
        frame_downsampled = inputs;
    }
    return frame_downsampled;
}
Vector6dVector NEW_Downsample_PointCloud(int target_total_points, Vector6dVector &Key_PointCloud) {
    Vector6dVector Downsample_PointCloud;
    Downsample_PointCloud.reserve(target_total_points);
    Key_PointCloud.reserve(target_total_points);
    std::mutex mutex;
    tbb::parallel_for(tbb::blocked_range<size_t>(0, onion_ball.size()), [&](const tbb::blocked_range<size_t>& range) {
        Vector6dVector local_points_thread;
        Vector6dVector local_key_points_thread;
        local_points_thread.reserve(std::min(5 * target_total_points, 10000));
        local_key_points_thread.reserve(std::min(target_total_points, 10000));
        for (size_t i = range.begin(); i < range.end(); ++i) {
            auto it = onion_ball.begin();
            std::advance(it, i);
            const auto& layer = it->first;
            auto& block = it->second;
			Vector6dVector filter_points;
            if (layer.R >= 1.0 && layer.R < 100.0 && block.seg_points.size() > 30) {
                double num_weight = static_cast<double>( block.seg_num) / static_cast<double>(Total_NUM);
                double v_weight = block.volume / Total_V;
                int down_num = std::max(10, static_cast<int>(std::ceil((v_weight) * target_total_points)));
                Vector6dVector down_points;
                Vector6dVector key_points;
                down_points.reserve(std::min(down_num, static_cast<int>(block.seg_points.size())));
                key_points.reserve(std::min(down_num, static_cast<int>(block.seg_points.size())));
            	down_points = Voxel_Downsample(5.0*down_num, block.seg_points, block.volume);
            	key_points = Voxel_Downsample(down_num, down_points, block.volume);
                local_points_thread.insert(local_points_thread.end(), down_points.begin(), down_points.end());
                local_key_points_thread.insert(local_key_points_thread.end(), key_points.begin(), key_points.end());
            }
        }
        std::lock_guard<std::mutex> lock(mutex);
        Downsample_PointCloud.insert(Downsample_PointCloud.end(), local_points_thread.begin(), local_points_thread.end());
		Key_PointCloud.insert(Key_PointCloud.end(), local_key_points_thread.begin(), local_key_points_thread.end());
    });
    return Downsample_PointCloud;
}

void clear(){
	Seg_points.clear();
}
};
