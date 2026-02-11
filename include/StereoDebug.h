#ifndef STEREO_DEBUG_H
#define STEREO_DEBUG_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <vector>

namespace ORB_SLAM3
{

struct MonocularDebugFrame
{
    cv::Mat image;
    std::vector<cv::KeyPoint> keypoints_detected;
    std::vector<cv::KeyPoint> keypoints_inlier;
    std::vector<cv::KeyPoint> keypoints_outlier;
};

enum class StereoDebugMode
{
    UNKNOWN = 0,
    METASHAPE_PINHOLE = 1,
    FISHEYE = 2,
};

struct StereoMatchDebug
{
    int left_idx = -1;
    int right_idx = -1;
    cv::Point2f left_point;
    cv::Point2f right_point;
    float disparity = -1.0f;
    float depth = -1.0f;
    bool has_depth = false;
};

struct StereoDebugFrame
{
    StereoDebugMode mode = StereoDebugMode::UNKNOWN;
    cv::Mat left_rectified;
    cv::Mat right_rectified;
    std::vector<cv::KeyPoint> left_keypoints;
    std::vector<cv::KeyPoint> right_keypoints;
    std::vector<StereoMatchDebug> matches;
    std::vector<cv::Vec4f> match_lines;
};

}  // namespace ORB_SLAM3

#endif  // STEREO_DEBUG_H
