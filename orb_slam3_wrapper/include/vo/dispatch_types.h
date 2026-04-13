#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <acfrlcm/auv_vis_rawlog_t.hpp>
#include <opencv2/opencv.hpp>

#include <utility>

namespace visual_odometry
{

using PoseStamped = std::pair<double, Eigen::Matrix4f>;

struct PendingMonoFrame
{
    double timestamp = 0.0;
    cv::Mat image;
    acfrlcm::auv_vis_rawlog_t raw_log;
};

struct PendingStereoFrame
{
    double timestamp = 0.0;
    cv::Mat left_image;
    cv::Mat right_image;
    acfrlcm::auv_vis_rawlog_t raw_log_left;
    acfrlcm::auv_vis_rawlog_t raw_log_right;
};

struct NavInterpolationResult
{
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    double bracketing_dt = 0.0;
};

struct DispatchContext
{
    NavInterpolationResult nav;
};

}  // namespace visual_odometry
