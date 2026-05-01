#pragma once

#include <utils/vo_utils.h>
#include <vo/common_options.h>
#include <vo/csv_pose_prior.h>
#include <vo/dispatch_types.h>
#include <vo/image_dispatch_sync.h>
#include <vo/nav_prediction_buffer.h>

#include <static_tf/static_tf_tree.hpp>

#include <acfrlcm/auv_acfr_nav_t.hpp>
#include <acfrlcm/auv_vis_rawlog_t.hpp>
#include <opencv2/opencv.hpp>

#include <opencv2/imgproc.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>
#include <mutex>
#include <optional>
#include <vector>

struct KeypointStats
{
    std::vector<cv::KeyPoint> keypoints_detected;
    std::vector<cv::KeyPoint> keypoints_inlier;
    std::vector<cv::KeyPoint> keypoints_outlier;
};

struct VOTrackingStats
{
    int keypoints_detected = 0;
    bool motion_model_primary = false;
    bool motion_model_success = false;
    int motion_model_matches = 0;
    int motion_model_inliers = 0;
    bool motion_model_retry = false;
    bool ref_kf_primary = false;
    bool ref_kf_fallback = false;
    bool ref_kf_success = false;
    int ref_kf_matches = 0;
    int ref_kf_inliers = 0;
    bool local_map_success = false;
    int local_map_inliers = 0;
    int tracked_map_points = 0;
    int new_map_point_candidates = 0;
    double tracking_time_ms = 0.0;
};

struct VOResult
{
    bool tracking_ok = false;
    bool is_keyframe = false;
    Eigen::Matrix4f pose_matrix = Eigen::Matrix4f::Identity();
    std::vector<KeypointStats> keypoint_stats;
    std::optional<VOTrackingStats> tracking_stats;
};

namespace visual_odometry
{
class VisualOdometryTestHelper;

class VisualOdometry
{
public:
    VisualOdometry(const static_tf::StaticTfTree& platform_tree = static_tf::StaticTfTree(),
                   const cli::CommonOptions& options = cli::CommonOptions());
    virtual ~VisualOdometry();

    void handle_monocular_image(const cv::Mat& image, const acfrlcm::auv_vis_rawlog_t& raw_log);
    void handle_stereo_image(const cv::Mat& left_image, const cv::Mat& right_image,
                             const acfrlcm::auv_vis_rawlog_t& raw_log_left,
                             const acfrlcm::auv_vis_rawlog_t& raw_log_right);

    void handle_nav_message(const acfrlcm::auv_acfr_nav_t& msg);

    std::string get_run_output_dir() const { return log_writer_.run_output_dir(); }

protected:
    const static_tf::StaticTfTree& platform_tree_;

    std::vector<PoseStamped> pose_estimates_;

    utils::TrajectoryLogWriter log_writer_;
    utils::DebugVideoWriter debug_video_writer_;
    cv::Ptr<cv::CLAHE> clahe_;

    VOResult prev_vo_result_;

    double initialization_timestamp_sec = 0.0;
    double tracking_lost_timestamp_sec = 0.0;
    double first_frame_timestamp_sec = 0.0;

    cli::CommonOptions options_;

    void write_debug_video_frame(const cv::Mat& frame);

private:
    static constexpr double kDebugVideoFps = 10.0;

    std::unique_ptr<NavPredictionBuffer> nav_prediction_data_;
    std::unique_ptr<ImageDispatchSync> dispatch_sync_;
    mutable std::mutex processing_mutex_;

    void dispatch_mono_with_context(const PendingMonoFrame& frame, const DispatchContext& ctx);
    void dispatch_stereo_with_context(const PendingStereoFrame& frame, const DispatchContext& ctx);

    virtual VOResult process_mono_image_impl(const cv::Mat& image, const DispatchContext& context,
                                             double timestamp) = 0;
    virtual VOResult process_stereo_image_impl(const cv::Mat& left_image, const cv::Mat& right_image,
                                               const DispatchContext& context, double timestamp) = 0;

    friend class VisualOdometryTestHelper;
};
}  // namespace visual_odometry
