#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointField.h"
#include "sensor_msgs/point_cloud2_iterator.h"
using PointCloud2 = sensor_msgs::PointCloud2;
using PointField = sensor_msgs::PointField;
using Header = std_msgs::Header;
std::string FixFrameId(const std::string &frame_id) {
    return std::regex_replace(frame_id, std::regex("^/"), "");
}
std::vector<Eigen::Vector3d> PointCloud2To3DEigen(const PointCloud2 &msg);
std::vector<Eigen::Vector3d> CorrectKITTIScan(const std::vector<Eigen::Vector3d> &frame);
std::vector<double> GetVelodyneTimestamps(const std::vector<Eigen::Vector3d> &points);
PointCloud2 EigenToPointCloud2(const std::vector<Eigen::Vector3d> &points,
                               const std::vector<double> &timestamps,
                               const Header &header);

PointCloud2 KITTICallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        const auto points = PointCloud2To3DEigen(*msg);
        const auto frame = CorrectKITTIScan(points);
        const auto timestamps = GetVelodyneTimestamps(frame);
        PointCloud2 msg_corr = EigenToPointCloud2(frame, timestamps, msg->header);
        return msg_corr;
}

std::vector<Eigen::Vector3d> PointCloud2To3DEigen(const PointCloud2 &msg) {
    std::vector<Eigen::Vector3d> points;
    points.reserve(msg.height * msg.width);
    sensor_msgs::PointCloud2ConstIterator<float> msg_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> msg_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> msg_z(msg, "z");
    for (size_t i = 0; i < msg.height * msg.width; ++i, ++msg_x, ++msg_y, ++msg_z) {
        points.emplace_back(*msg_x, *msg_y, *msg_z);
    }
    return points;
}
std::vector<Eigen::Vector3d> CorrectKITTIScan(const std::vector<Eigen::Vector3d> &frame) {
    constexpr double VERTICAL_ANGLE_OFFSET = (0.205 * M_PI) / 180.0;
    std::vector<Eigen::Vector3d> corrected_frame(frame.size());
    tbb::parallel_for(size_t(0), frame.size(), [&](size_t i) {
        const auto &pt = frame[i];
        const Eigen::Vector3d rotationVector = pt.cross(Eigen::Vector3d(0., 0., 1.));
        corrected_frame[i] =
            Eigen::AngleAxisd(VERTICAL_ANGLE_OFFSET, rotationVector.normalized()) * pt;
    });
    return corrected_frame;
}
std::vector<double> GetVelodyneTimestamps(const std::vector<Eigen::Vector3d> &points) {
    std::vector<double> timestamps;
    timestamps.reserve(points.size());
    std::for_each(points.cbegin(), points.cend(), [&](const auto &point) {
        const double yaw = -std::atan2(point.y(), point.x());
        timestamps.emplace_back(0.5 * (yaw / M_PI + 1.0));
    });
    return timestamps;
}
auto CreatePointCloud2Msg(const size_t n_points, const Header &header, bool timestamp = false) {
    PointCloud2 cloud_msg;
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    cloud_msg.header = header;
    cloud_msg.header.frame_id = FixFrameId(cloud_msg.header.frame_id);
    cloud_msg.fields.clear();
    int offset = 0;
    offset = addPointField(cloud_msg, "x", 1, PointField::FLOAT32, offset);
    offset = addPointField(cloud_msg, "y", 1, PointField::FLOAT32, offset);
    offset = addPointField(cloud_msg, "z", 1, PointField::FLOAT32, offset);
    offset += sizeOfPointField(PointField::FLOAT32);
    if (timestamp) {
        offset = addPointField(cloud_msg, "time", 1, PointField::FLOAT64, offset);
        offset += sizeOfPointField(PointField::FLOAT64);
    }

    cloud_msg.point_step = offset;
    cloud_msg.row_step = cloud_msg.width * cloud_msg.point_step;
    cloud_msg.data.resize(cloud_msg.height * cloud_msg.row_step);
    modifier.resize(n_points);
    return cloud_msg;
}
void FillPointCloud2XYZ(const std::vector<Eigen::Vector3d> &points, PointCloud2 &msg) {
    sensor_msgs::PointCloud2Iterator<float> msg_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> msg_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> msg_z(msg, "z");
    for (size_t i = 0; i < points.size(); i++, ++msg_x, ++msg_y, ++msg_z) {
        const Eigen::Vector3d &point = points[i];
        *msg_x = point.x();
        *msg_y = point.y();
        *msg_z = point.z();
    }
}

void FillPointCloud2Timestamp(const std::vector<double> &timestamps, PointCloud2 &msg) {
    sensor_msgs::PointCloud2Iterator<double> msg_t(msg, "time");
    for (size_t i = 0; i < timestamps.size(); i++, ++msg_t) *msg_t = timestamps[i];
}
PointCloud2 EigenToPointCloud2(const std::vector<Eigen::Vector3d> &points,
                               const std::vector<double> &timestamps,
                               const Header &header) {
    PointCloud2 msg = CreatePointCloud2Msg(points.size(), header, true);
    FillPointCloud2XYZ(points, msg);
    FillPointCloud2Timestamp(timestamps, msg);
    return msg;
}
