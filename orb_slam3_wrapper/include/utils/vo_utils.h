#pragma once

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

namespace visual_odometry
{
inline std::string get_camera_channel(const std::string& platform)
{
    return platform + ".ACFR_AUV_VIS_RAWLOG";
}

inline std::string get_nav_channel(const std::string& platform)
{
    return platform + ".ACFR_NAV";
}

inline std::string get_imu_channel(const std::string& platform)
{
    return platform + ".NUCLEUS.IMU";
}

inline std::string get_magnetometer_channel(const std::string& platform)
{
    return platform + ".NUCLEUS.MAGNETOMETER";
}

inline std::string get_depth_channel(const std::string& platform)
{
    return platform + ".NUCLEUS.ALTIMETER";
}

inline std::string get_dvl_channel(const std::string& platform)
{
    return platform + ".NUCLEUS.BOTTOMTRACK";
}

inline std::string get_ahrs_channel(const std::string& platform)
{
    return platform + ".NUCLEUS.AHRS";
}

namespace utils
{
struct FrameLogEntry
{
    double timestamp = 0.0;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    int tracking_state = 0;
    bool is_keyframe = false;
    // Feature counts
    int keypoints_detected = 0;
    // Motion-model tracking
    bool motion_model_primary = false;
    bool motion_model_success = false;
    int motion_model_matches = 0;
    int motion_model_inliers = 0;
    bool motion_model_retry = false;
    // Reference keyframe tracking
    bool ref_kf_primary = false;
    bool ref_kf_fallback = false;
    bool ref_kf_success = false;
    int ref_kf_matches = 0;
    int ref_kf_inliers = 0;
    // Local map tracking
    bool local_map_success = false;
    int local_map_inliers = 0;
    // Map point stats
    int tracked_map_points = 0;
    int new_map_point_candidates = 0;
    // Timing
    double tracking_time_ms = 0.0;
    std::string image_name;
};

struct LogFiles
{
    std::string frame_log_path;
    std::string nav_log_path;
    std::string frame_stats_log_path;
    std::string keyframe_log_path;
    std::ofstream frame_log;
    std::ofstream nav_log;
    std::ofstream frame_stats_log;
    std::ofstream keyframe_log;
};

inline std::string make_log_path(const std::string& output_dir, const std::string& stem)
{
    std::filesystem::path dir(output_dir);
    std::string filename = stem + ".csv";
    return (dir / filename).string();
}

inline void configure_log_stream(std::ofstream& stream)
{
    stream << std::fixed << std::setprecision(6);
}

inline LogFiles setup_output_logs(const std::string& output_dir)
{
    std::filesystem::path output_path(output_dir);
    if (std::filesystem::exists(output_path) && !std::filesystem::is_directory(output_path))
    {
        throw std::runtime_error("Output path exists but is not a directory: " + output_dir);
    }
    if (!std::filesystem::exists(output_path))
    {
        std::error_code ec;
        std::filesystem::create_directories(output_path, ec);
        if (ec)
        {
            throw std::runtime_error("Failed to create output directory: " + output_dir + " (" + ec.message() + ")");
        }
    }

    LogFiles logs;
    logs.frame_log_path = make_log_path(output_path.string(), "trajectory_frames");
    logs.nav_log_path = make_log_path(output_path.string(), "trajectory_nav");
    logs.frame_stats_log_path = make_log_path(output_path.string(), "trajectory_frame_stats");
    logs.keyframe_log_path = make_log_path(output_path.string(), "trajectory_keyframes");

    logs.frame_log.open(logs.frame_log_path, std::ios::out | std::ios::trunc);
    logs.nav_log.open(logs.nav_log_path, std::ios::out | std::ios::trunc);
    logs.frame_stats_log.open(logs.frame_stats_log_path, std::ios::out | std::ios::trunc);
    logs.keyframe_log.open(logs.keyframe_log_path, std::ios::out | std::ios::trunc);
    if (!logs.frame_log || !logs.nav_log || !logs.frame_stats_log || !logs.keyframe_log)
    {
        throw std::runtime_error("Failed to open trajectory log files in: " + output_dir);
    }

    configure_log_stream(logs.frame_log);
    configure_log_stream(logs.nav_log);
    configure_log_stream(logs.frame_stats_log);
    configure_log_stream(logs.keyframe_log);

    logs.frame_log << "timestamp,tx,ty,tz,qw,qx,qy,qz,parent_frame_id,child_frame_id\n";
    logs.nav_log << "timestamp,tx,ty,tz,qw,qx,qy,qz,parent_frame_id,child_frame_id\n";
    logs.frame_stats_log
        << "timestamp,tracking_state,is_keyframe,"
           "keypoints_detected,"
           "motion_model_primary,motion_model_success,motion_model_matches,motion_model_inliers,motion_model_retry,"
           "ref_kf_primary,ref_kf_fallback,ref_kf_success,ref_kf_matches,ref_kf_inliers,"
           "local_map_success,local_map_inliers,"
           "tracked_map_points,new_map_point_candidates,"
           "tracking_time_ms,"
           "image_name\n";
    logs.keyframe_log << "timestamp,tx,ty,tz,qw,qx,qy,qz,image_name\n";
    return logs;
}

inline std::string escape_csv_field(const std::string& value)
{
    if (value.find_first_of(",\"\n\r") == std::string::npos)
    {
        return value;
    }
    std::string escaped_value = "\"";
    for (const char c : value)
    {
        if (c == '"')
        {
            escaped_value += "\"\"";
        }
        else
        {
            escaped_value += c;
        }
    }
    escaped_value += "\"";
    return escaped_value;
}

inline void write_frame_log(std::ofstream& stream, double timestamp, const Eigen::Matrix4f& pose,
                            const std::string& parent_frame_id, const std::string& child_frame_id)
{
    const Eigen::Vector3f translation = pose.block<3, 1>(0, 3);
    const Eigen::Matrix3f rotation_matrix = pose.block<3, 3>(0, 0);
    const Eigen::Quaternionf rotation = Eigen::Quaternionf(rotation_matrix).normalized();
    stream << timestamp << "," << translation.x() << "," << translation.y() << "," << translation.z() << ","
           << rotation.w() << "," << rotation.x() << "," << rotation.y() << "," << rotation.z() << ","
           << escape_csv_field(parent_frame_id) << "," << escape_csv_field(child_frame_id) << "\n";
}

inline void write_frame_stats_log(std::ofstream& stream, const FrameLogEntry& e)
{
    stream << e.timestamp << "," << e.tracking_state << "," << static_cast<int>(e.is_keyframe) << ","
           << e.keypoints_detected << "," << static_cast<int>(e.motion_model_primary) << ","
           << static_cast<int>(e.motion_model_success) << "," << e.motion_model_matches << "," << e.motion_model_inliers
           << "," << static_cast<int>(e.motion_model_retry) << "," << static_cast<int>(e.ref_kf_primary) << ","
           << static_cast<int>(e.ref_kf_fallback) << "," << static_cast<int>(e.ref_kf_success) << ","
           << e.ref_kf_matches << "," << e.ref_kf_inliers << "," << static_cast<int>(e.local_map_success) << ","
           << e.local_map_inliers << "," << e.tracked_map_points << "," << e.new_map_point_candidates << ","
           << e.tracking_time_ms << "," << escape_csv_field(e.image_name) << "\n";
}

inline void write_keyframe_log(std::ofstream& stream, double timestamp, const Eigen::Matrix4f& pose,
                               const std::string& image_name)
{
    const Eigen::Vector3f translation = pose.block<3, 1>(0, 3);
    const Eigen::Matrix3f rotation_matrix = pose.block<3, 3>(0, 0);
    const Eigen::Quaternionf rotation = Eigen::Quaternionf(rotation_matrix).normalized();
    stream << timestamp << "," << translation.x() << "," << translation.y() << "," << translation.z() << ","
           << rotation.w() << "," << rotation.x() << "," << rotation.y() << "," << rotation.z() << ","
           << escape_csv_field(image_name) << "\n";
}

class TrajectoryLogWriter
{
public:
    explicit TrajectoryLogWriter(const std::string& output_dir) : enabled_(!output_dir.empty())
    {
        if (!enabled_)
        {
            return;
        }
        logs_ = setup_output_logs(output_dir);
    }

    bool enabled() const { return enabled_; }

    std::string run_output_dir() const
    {
        if (!enabled_)
        {
            return "";
        }
        return std::filesystem::path(logs_.frame_log_path).parent_path().string();
    }

    std::string debug_video_path() const
    {
        if (!enabled_)
        {
            return "";
        }
        const std::filesystem::path frame_path(logs_.frame_log_path);
        return (frame_path.parent_path() / "debug_display.avi").string();
    }

    // Write stats row only (for all frames, including tracking failures).
    void write_frame_stats_only(const FrameLogEntry& e)
    {
        if (!enabled_)
        {
            return;
        }
        write_frame_stats_log(logs_.frame_stats_log, e);
    }

    // Write pose + keyframe rows (only when tracking succeeded).
    void write_frame(const FrameLogEntry& e)
    {
        if (!enabled_)
        {
            return;
        }
        write_frame_log(logs_.frame_log, e.timestamp, e.pose, "vo_map", "vo_camera");
        if (e.is_keyframe)
        {
            write_keyframe_log(logs_.keyframe_log, e.timestamp, e.pose, e.image_name);
        }
    }

    void write_nav_pose(double timestamp, const Eigen::Matrix4f& pose, const std::string& parent_frame_id,
                        const std::string& child_frame_id)
    {
        if (!enabled_)
        {
            return;
        }
        write_frame_log(logs_.nav_log, timestamp, pose, parent_frame_id, child_frame_id);
    }

private:
    bool enabled_;
    LogFiles logs_;
};

class DebugVideoWriter
{
public:
    DebugVideoWriter() = default;

    void configure(const std::string& video_path, bool enabled, double fps)
    {
        if (video_writer_.isOpened())
        {
            video_writer_.release();
        }
        video_path_ = video_path;
        enabled_ = enabled && !video_path_.empty();
        fps_ = fps;
    }

    void write(const cv::Mat& frame)
    {
        if (!enabled_ || frame.empty())
        {
            return;
        }
        if (!video_writer_.isOpened())
        {
            const int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
            if (!video_writer_.open(video_path_, fourcc, fps_, frame.size()))
            {
                std::cerr << "Failed to open debug video for writing: " << video_path_ << std::endl;
                enabled_ = false;
                return;
            }
        }
        video_writer_.write(frame);
    }

private:
    std::string video_path_;
    bool enabled_ = false;
    double fps_ = 0.0;
    cv::VideoWriter video_writer_;
};

inline void convert_image(const cv::Mat& image, cv::Mat& display_image)
{
    // Convert from 16-bit Bayer BGGR to BGR
    if (image.type() == CV_16UC1 && image.channels() == 1)
    {
        // Image is in bayer_bggr16 format, convert to BGR
        cv::Mat bgr_16bit;
        cv::cvtColor(image, bgr_16bit, cv::COLOR_BayerBG2BGR);

        // Convert 16-bit to 8-bit for display (scale from 0-65535 to 0-255)
        bgr_16bit.convertTo(display_image, CV_8UC3, 1.0 / 256.0);
    }
    else
    {
        // Already converted or different format, use as is
        if (image.type() == CV_16UC3)
        {
            // 16-bit BGR, convert to 8-bit
            image.convertTo(display_image, CV_8UC3, 1.0 / 256.0);
        }
        else
        {
            display_image = image.clone();
        }
    }
}

}  // namespace utils
}  // namespace visual_odometry
