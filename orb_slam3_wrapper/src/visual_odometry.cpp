#include <vo/visual_odometry.h>

#include <cmath>
#include <thread>

namespace visual_odometry
{
namespace
{
utils::FrameLogEntry make_frame_log_entry(double timestamp, const std::string& image_name, const VOResult& vo_result)
{
    utils::FrameLogEntry e;
    e.timestamp = timestamp;
    e.pose = vo_result.pose_matrix;
    e.tracking_state = static_cast<int>(vo_result.tracking_ok);
    e.is_keyframe = vo_result.is_keyframe;
    e.image_name = image_name;

    if (vo_result.tracking_stats.has_value())
    {
        const VOTrackingStats& s = vo_result.tracking_stats.value();
        e.keypoints_detected = s.keypoints_detected;
        e.motion_model_primary = s.motion_model_primary;
        e.motion_model_success = s.motion_model_success;
        e.motion_model_matches = s.motion_model_matches;
        e.motion_model_inliers = s.motion_model_inliers;
        e.motion_model_retry = s.motion_model_retry;
        e.ref_kf_primary = s.ref_kf_primary;
        e.ref_kf_fallback = s.ref_kf_fallback;
        e.ref_kf_success = s.ref_kf_success;
        e.ref_kf_matches = s.ref_kf_matches;
        e.ref_kf_inliers = s.ref_kf_inliers;
        e.local_map_success = s.local_map_success;
        e.local_map_inliers = s.local_map_inliers;
        e.tracked_map_points = s.tracked_map_points;
        e.new_map_point_candidates = s.new_map_point_candidates;
        e.tracking_time_ms = s.tracking_time_ms;
    }

    return e;
}

void finalize_tracking_step(double timestamp, const VOResult& vo_result, const utils::FrameLogEntry& entry,
                            double& first_frame_timestamp_sec, double& tracking_lost_timestamp_sec,
                            double& initialization_timestamp_sec, VOResult& prev_vo_result,
                            std::vector<PoseStamped>& pose_estimates, utils::TrajectoryLogWriter& log_writer)
{
    log_writer.write_frame_stats_only(entry);

    if (first_frame_timestamp_sec == 0.0)
    {
        first_frame_timestamp_sec = timestamp;
        tracking_lost_timestamp_sec = timestamp;
    }

    if (vo_result.tracking_ok)
    {
        pose_estimates.push_back(std::make_pair(timestamp, vo_result.pose_matrix));
        log_writer.write_frame(entry);
    }

    if (!prev_vo_result.tracking_ok && vo_result.tracking_ok)
    {
        const double time_since_tracking_lost = timestamp - tracking_lost_timestamp_sec;
        std::cout << "[" << std::fixed << std::setprecision(6) << timestamp
                  << "] Tracking OK. Time since tracking lost: " << time_since_tracking_lost << " seconds" << std::endl;
        initialization_timestamp_sec = timestamp;
    }

    if (prev_vo_result.tracking_ok && !vo_result.tracking_ok)
    {
        const double time_since_initialization = timestamp - initialization_timestamp_sec;
        std::cout << "[" << std::fixed << std::setprecision(6) << timestamp
                  << "] Tracking LOST. Time since initialization: " << time_since_initialization << " seconds"
                  << std::endl;
        initialization_timestamp_sec = 0.0;
        tracking_lost_timestamp_sec = timestamp;
    }

    prev_vo_result = vo_result;
}

static NavInterpolationResult nav_interpolate(const PoseStamped& lower, const PoseStamped& upper, double alpha,
                                              double dt)
{
    NavInterpolationResult result;
    result.bracketing_dt = dt;
    const Eigen::Vector3f p0 = lower.second.block<3, 1>(0, 3);
    const Eigen::Vector3f p1 = upper.second.block<3, 1>(0, 3);
    const Eigen::Vector3f p = (1.0f - static_cast<float>(alpha)) * p0 + static_cast<float>(alpha) * p1;
    Eigen::Quaternionf q0(lower.second.block<3, 3>(0, 0));
    Eigen::Quaternionf q1(upper.second.block<3, 3>(0, 0));
    q0.normalize();
    q1.normalize();
    const Eigen::Quaternionf q = q0.slerp(static_cast<float>(alpha), q1).normalized();
    result.pose = Eigen::Matrix4f::Identity();
    result.pose.block<3, 3>(0, 0) = q.toRotationMatrix();
    result.pose.block<3, 1>(0, 3) = p;
    return result;
}
}  // namespace

Eigen::Matrix4f acfr_nav_to_eigen_matrix(const acfrlcm::auv_acfr_nav_t& nav)
{
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    matrix(0, 3) = nav.x;
    matrix(1, 3) = nav.y;
    matrix(2, 3) = nav.depth;
    matrix(3, 3) = 1.0f;

    const auto roll_angle = Eigen::AngleAxisf(nav.roll, Eigen::Vector3f::UnitX());
    const auto pitch_angle = Eigen::AngleAxisf(nav.pitch, Eigen::Vector3f::UnitY());
    const auto heading_angle = Eigen::AngleAxisf(nav.heading, Eigen::Vector3f::UnitZ());

    matrix.block<3, 3>(0, 0) = (heading_angle * pitch_angle * roll_angle).toRotationMatrix().normalized();

    return matrix;
}

VisualOdometry::VisualOdometry(const static_tf::StaticTfTree& platform_tree, const cli::CommonOptions& options)
    : platform_tree_(platform_tree), options_(options), log_writer_(options.output_dir)
{
    debug_video_writer_.configure(log_writer_.debug_video_path(), !options_.output_dir.empty() && options_.debug_video,
                                  kDebugVideoFps);

    if (options_.apply_clahe)
    {
        clahe_ = cv::createCLAHE(2.0, cv::Size(16, 16));
    }

    auto get_ts = [](const PoseStamped& p)
    {
        return p.first;
    };
    nav_prediction_data_ = std::make_unique<NavPredictionData>(get_ts, nav_interpolate);
    dispatch_sync_ = std::make_unique<ImageDispatchSync<NavPredictionData>>();
    dispatch_sync_->set_nav(nav_prediction_data_.get());
    dispatch_sync_->set_mono_callback(
        [this](const PendingMonoFrame& f, const DispatchContext& c)
        {
            std::lock_guard<std::mutex> lock(processing_mutex_);
            dispatch_mono_with_context(f, c);
        });
    dispatch_sync_->set_stereo_callback(
        [this](const PendingStereoFrame& f, const DispatchContext& c)
        {
            std::lock_guard<std::mutex> lock(processing_mutex_);
            dispatch_stereo_with_context(f, c);
        });
}

VisualOdometry::~VisualOdometry() {}

void VisualOdometry::handle_nav_message(const acfrlcm::auv_acfr_nav_t& msg)
{
    const double timestamp = static_cast<double>(msg.utime) / 1e6;
    const Eigen::Matrix4f nav_pose = acfr_nav_to_eigen_matrix(msg);

    log_writer_.write_nav_pose(timestamp, nav_pose, "map", "dvl");

    nav_prediction_data_->push(std::make_pair(timestamp, nav_pose));

    dispatch_sync_->on_nav_updated();
}

void VisualOdometry::handle_monocular_image(const cv::Mat& image, const acfrlcm::auv_vis_rawlog_t& raw_log)
{
    const double timestamp = static_cast<double>(raw_log.utime) / 1e6;

    double first_nav_t = 0.0;
    if (nav_prediction_data_->size() >= 2 && nav_prediction_data_->get_oldest_timestamp(first_nav_t) &&
        timestamp < first_nav_t)
    {
        return;
    }

    PendingMonoFrame frame;
    frame.timestamp = timestamp;
    frame.image = image.clone();
    frame.raw_log = raw_log;
    dispatch_sync_->push_frame(std::move(frame));
}

void VisualOdometry::dispatch_mono_with_context(const PendingMonoFrame& frame, const DispatchContext& ctx)
{
    cv::Mat rgb_image;

    utils::convert_image(frame.image, rgb_image);
    const double timestamp = frame.timestamp;
    const std::string image_name = frame.raw_log.image_name;

    cv::Mat grayscale_image;
    cv::cvtColor(rgb_image, grayscale_image, cv::COLOR_BGR2GRAY);

    // apply CLAHE to the grayscale image
    if (options_.apply_clahe)
    {
        clahe_->apply(grayscale_image, grayscale_image);
    }

    VOResult vo_result = process_mono_image_impl(grayscale_image, ctx, timestamp);

    const utils::FrameLogEntry entry = make_frame_log_entry(timestamp, image_name, vo_result);
    finalize_tracking_step(timestamp, vo_result, entry, first_frame_timestamp_sec, tracking_lost_timestamp_sec,
                           initialization_timestamp_sec, prev_vo_result_, pose_estimates_, log_writer_);
    // Throttle to kDebugVideoFps; VideoWriter needs paced writes to produce a valid output file.
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

void VisualOdometry::handle_stereo_image(const cv::Mat& left_image, const cv::Mat& right_image,
                                         const acfrlcm::auv_vis_rawlog_t& raw_log_left,
                                         const acfrlcm::auv_vis_rawlog_t& raw_log_right)
{
    const double timestamp = static_cast<double>(raw_log_left.utime) / 1e6;

    double first_nav_t = 0.0;
    if (nav_prediction_data_->size() >= 2 && nav_prediction_data_->get_oldest_timestamp(first_nav_t) &&
        timestamp < first_nav_t)
    {
        return;
    }

    PendingStereoFrame frame;
    frame.timestamp = timestamp;
    frame.left_image = left_image.clone();
    frame.right_image = right_image.clone();
    frame.raw_log_left = raw_log_left;
    frame.raw_log_right = raw_log_right;
    dispatch_sync_->push_frame(std::move(frame));
}

void VisualOdometry::dispatch_stereo_with_context(const PendingStereoFrame& frame, const DispatchContext& ctx)
{
    cv::Mat left_rgb_image, right_rgb_image;

    utils::convert_image(frame.left_image, left_rgb_image);
    utils::convert_image(frame.right_image, right_rgb_image);

    const double timestamp = frame.timestamp;
    const std::string image_name = frame.raw_log_left.image_name;

    cv::Mat left_grayscale_image, right_grayscale_image;
    cv::cvtColor(left_rgb_image, left_grayscale_image, cv::COLOR_BGR2GRAY);
    cv::cvtColor(right_rgb_image, right_grayscale_image, cv::COLOR_BGR2GRAY);

    // apply CLAHE to the grayscale images
    if (options_.apply_clahe)
    {
        clahe_->apply(left_grayscale_image, left_grayscale_image);
        clahe_->apply(right_grayscale_image, right_grayscale_image);
    }

    VOResult vo_result = process_stereo_image_impl(left_grayscale_image, right_grayscale_image, ctx, timestamp);

    const utils::FrameLogEntry entry = make_frame_log_entry(timestamp, image_name, vo_result);
    finalize_tracking_step(timestamp, vo_result, entry, first_frame_timestamp_sec, tracking_lost_timestamp_sec,
                           initialization_timestamp_sec, prev_vo_result_, pose_estimates_, log_writer_);
    // Throttle to kDebugVideoFps; VideoWriter needs paced writes to produce a valid output file.
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

void VisualOdometry::write_debug_video_frame(const cv::Mat& frame)
{
    debug_video_writer_.write(frame);
}

}  // namespace visual_odometry