#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstddef>
#include <string>
#include <vector>

namespace visual_odometry
{
class CsvPosePrior
{
public:
    explicit CsvPosePrior(const std::string& csv_path);

    bool try_get_interpolated(double timestamp_sec, Eigen::Matrix4f& out_world_T_dvl) const;
    bool try_get_sample(size_t idx, double& out_timestamp_sec, Eigen::Matrix4f& out_world_T_dvl) const;

    size_t size() const { return timestamps_.size(); }
    double start_time_sec() const { return timestamps_.empty() ? 0.0 : timestamps_.front(); }
    double end_time_sec() const { return timestamps_.empty() ? 0.0 : timestamps_.back(); }

private:
    std::vector<double> timestamps_;
    std::vector<Eigen::Vector3d> positions_world_dvl_;
    std::vector<Eigen::Quaterniond> rotations_world_dvl_;
};
}  // namespace visual_odometry

