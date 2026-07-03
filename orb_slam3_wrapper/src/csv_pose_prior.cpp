#include <vo/csv_pose_prior.h>

#include <algorithm>
#include <cctype>
#include <fstream>
#include <stdexcept>

namespace visual_odometry
{
namespace
{
static std::string trim(std::string s)
{
    auto not_space = [](unsigned char c) { return !std::isspace(c); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
    s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
    return s;
}

static bool split_csv_line(const std::string& line, std::vector<std::string>& out_fields)
{
    // std::getline(stream, field, ',') in a loop silently drops a trailing
    // empty field: once the stream is positioned right after a final comma
    // with nothing left to extract, getline() fails instead of yielding one
    // more empty string, so e.g. "a,b," would split to ["a","b"] instead of
    // ["a","b",""]. Split on explicit delimiter positions instead so every
    // comma-separated field -- including a trailing empty one -- is counted.
    out_fields.clear();
    std::size_t start = 0;
    while (true)
    {
        const std::size_t comma = line.find(',', start);
        if (comma == std::string::npos)
        {
            out_fields.push_back(trim(line.substr(start)));
            break;
        }
        out_fields.push_back(trim(line.substr(start, comma - start)));
        start = comma + 1;
    }
    return !out_fields.empty();
}

static double parse_double(const std::string& s, const std::string& field_name, const std::string& path, int line_idx)
{
    try
    {
        size_t pos = 0;
        double v = std::stod(s, &pos);
        if (pos != s.size())
        {
            throw std::runtime_error("");
        }
        return v;
    }
    catch (...)
    {
        throw std::runtime_error("Invalid double for field '" + field_name + "' at " + path + ":" +
                                 std::to_string(line_idx));
    }
}
}  // namespace

CsvPosePrior::CsvPosePrior(const std::string& csv_path)
{
    std::ifstream in(csv_path);
    if (!in.is_open())
    {
        throw std::runtime_error("Failed to open CSV pose prior file: " + csv_path);
    }

    std::string line;
    std::vector<std::string> fields;

    // Header
    if (!std::getline(in, line))
    {
        throw std::runtime_error("CSV pose prior file is empty: " + csv_path);
    }
    split_csv_line(line, fields);
    const std::vector<std::string> expected = {"timestamp", "tx", "ty", "tz", "qw", "qx", "qy", "qz"};
    if (fields != expected)
    {
        throw std::runtime_error("CSV pose prior header mismatch in " + csv_path +
                                 " (expected: timestamp,tx,ty,tz,qw,qx,qy,qz)");
    }

    int line_idx = 1;
    while (std::getline(in, line))
    {
        ++line_idx;
        if (trim(line).empty())
        {
            continue;
        }
        split_csv_line(line, fields);
        if (fields.size() != expected.size())
        {
            throw std::runtime_error("CSV pose prior row has wrong number of fields at " + csv_path + ":" +
                                     std::to_string(line_idx));
        }

        const double t = parse_double(fields[0], "timestamp", csv_path, line_idx);
        const double tx = parse_double(fields[1], "tx", csv_path, line_idx);
        const double ty = parse_double(fields[2], "ty", csv_path, line_idx);
        const double tz = parse_double(fields[3], "tz", csv_path, line_idx);

        const double qw = parse_double(fields[4], "qw", csv_path, line_idx);
        const double qx = parse_double(fields[5], "qx", csv_path, line_idx);
        const double qy = parse_double(fields[6], "qy", csv_path, line_idx);
        const double qz = parse_double(fields[7], "qz", csv_path, line_idx);

        Eigen::Quaterniond q(qw, qx, qy, qz);
        q.normalize();

        if (!timestamps_.empty() && t <= timestamps_.back())
        {
            throw std::runtime_error("CSV pose prior timestamps must be strictly increasing; got " + csv_path + ":" +
                                     std::to_string(line_idx));
        }

        timestamps_.push_back(t);
        positions_world_dvl_.push_back(Eigen::Vector3d(tx, ty, tz));
        rotations_world_dvl_.push_back(q);
    }

    if (timestamps_.size() < 2)
    {
        throw std::runtime_error("CSV pose prior must contain at least 2 samples: " + csv_path);
    }
}

bool CsvPosePrior::try_get_interpolated(double timestamp_sec, Eigen::Matrix4f& out_world_T_dvl) const
{
    if (timestamps_.size() < 2)
    {
        return false;
    }

    auto upper_it = std::lower_bound(timestamps_.begin(), timestamps_.end(), timestamp_sec);
    // A query exactly at the first sample's timestamp lower_bounds to begin(),
    // which the "before the range" check below would otherwise reject even
    // though the sample exists. Treat it as the lower bracket of the first
    // interval instead.
    if (upper_it == timestamps_.begin() && upper_it != timestamps_.end() && *upper_it == timestamp_sec)
    {
        ++upper_it;
    }
    if (upper_it == timestamps_.begin() || upper_it == timestamps_.end())
    {
        return false;
    }

    const size_t upper_idx = static_cast<size_t>(std::distance(timestamps_.begin(), upper_it));
    const size_t lower_idx = upper_idx - 1;

    const double t0 = timestamps_[lower_idx];
    const double t1 = timestamps_[upper_idx];
    const double dt = t1 - t0;
    if (dt <= 0.0)
    {
        return false;
    }

    const double alpha_d = (timestamp_sec - t0) / dt;
    const double alpha_clamped = std::min(1.0, std::max(0.0, alpha_d));
    const float alpha = static_cast<float>(alpha_clamped);

    const Eigen::Vector3d p0 = positions_world_dvl_[lower_idx];
    const Eigen::Vector3d p1 = positions_world_dvl_[upper_idx];
    const Eigen::Vector3d p = (1.0 - alpha_clamped) * p0 + alpha_clamped * p1;

    Eigen::Quaterniond q0 = rotations_world_dvl_[lower_idx];
    Eigen::Quaterniond q1 = rotations_world_dvl_[upper_idx];
    q0.normalize();
    q1.normalize();
    const Eigen::Quaterniond q = q0.slerp(alpha, q1).normalized();

    out_world_T_dvl = Eigen::Matrix4f::Identity();
    out_world_T_dvl.block<3, 3>(0, 0) = q.toRotationMatrix().cast<float>();
    out_world_T_dvl.block<3, 1>(0, 3) = p.cast<float>();
    return true;
}

bool CsvPosePrior::try_get_sample(size_t idx, double& out_timestamp_sec, Eigen::Matrix4f& out_world_T_dvl) const
{
    if (idx >= timestamps_.size())
    {
        return false;
    }

    out_timestamp_sec = timestamps_[idx];
    const Eigen::Vector3d p = positions_world_dvl_[idx];
    Eigen::Quaterniond q = rotations_world_dvl_[idx];
    q.normalize();

    out_world_T_dvl = Eigen::Matrix4f::Identity();
    out_world_T_dvl.block<3, 3>(0, 0) = q.toRotationMatrix().cast<float>();
    out_world_T_dvl.block<3, 1>(0, 3) = p.cast<float>();
    return true;
}

}  // namespace visual_odometry

