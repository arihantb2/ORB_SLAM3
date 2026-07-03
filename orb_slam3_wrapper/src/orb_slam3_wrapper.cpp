#include "orb_slam3_wrapper.h"

#include <utils/vo_utils.h>

#include <Eigen/Core>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <opencv2/core/eigen.hpp>

#include "CameraModels/CameraCalibrationInput.h"
#include "ORB_SLAM3/System.h"

namespace visual_odometry
{

namespace
{

template <typename T>
T readParam(cv::FileStorage& fs, const std::string& key, bool& found, T default_val)
{
    cv::FileNode node = fs[key];
    found = !node.empty();
    if (found)
    {
        return static_cast<T>(node);
    }
    return default_val;
}

ORB_SLAM3::CameraCalibrationInput load_camera_calibration(const std::string& camera_calib_file, bool stereo,
                                                          float& out_scaling_factor)
{
    cv::FileStorage fs(camera_calib_file, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        throw std::runtime_error("Failed to open camera calibration file: " + camera_calib_file);
    }

    bool found;
    std::string camera_type = readParam<std::string>(fs, "Camera.type", found, std::string("Metashape"));
    const int width = readParam<int>(fs, "Camera.width", found, 0);
    const int height = readParam<int>(fs, "Camera.height", found, 0);
    const float scaling_factor = readParam<float>(fs, "Camera.ScalingFactor", found, 1.0f);
    out_scaling_factor = scaling_factor;
    if (width <= 0 || height <= 0)
    {
        throw std::runtime_error("Camera.width and Camera.height must be positive in " + camera_calib_file);
    }

    ORB_SLAM3::CameraCalibrationInput calib;
    calib.originalImSize = cv::Size(width, height);
    calib.newImSize = cv::Size(width, height);
    calib.bNeedToResize1 = false;

    if (camera_type == "PinHole" || camera_type == "Rectified")
    {
        calib.cameraType = (camera_type == "Rectified") ? 1 : 0;  // Settings::Rectified=1, PinHole=0
        const float fx = readParam<float>(fs, "Camera1.fx", found, 0.f);
        const float fy = readParam<float>(fs, "Camera1.fy", found, 0.f);
        const float cx = readParam<float>(fs, "Camera1.cx", found, 0.f);
        const float cy = readParam<float>(fs, "Camera1.cy", found, 0.f);
        calib.camera1.reset(ORB_SLAM3::CreatePinholeCamera(fx, fy, cx, cy));
        float k1 = readParam<float>(fs, "Camera1.k1", found, 0.f);
        if (found && (calib.cameraType == 0))
        {
            calib.bNeedToUndistort = true;
            calib.vPinHoleDistorsion1.push_back(k1);
            calib.vPinHoleDistorsion1.push_back(readParam<float>(fs, "Camera1.k2", found, 0.f));
            calib.vPinHoleDistorsion1.push_back(readParam<float>(fs, "Camera1.p1", found, 0.f));
            calib.vPinHoleDistorsion1.push_back(readParam<float>(fs, "Camera1.p2", found, 0.f));
            float k3 = readParam<float>(fs, "Camera1.k3", found, 0.f);
            if (found)
            {
                calib.vPinHoleDistorsion1.push_back(k3);
            }
        }
    }
    else if (camera_type == "Metashape")
    {
        calib.cameraType = 3;  // Settings::Metashape
        const float f = readParam<float>(fs, "Camera1.f", found, 0.f);
        const float cx = readParam<float>(fs, "Camera1.cx", found, 0.f);
        const float cy = readParam<float>(fs, "Camera1.cy", found, 0.f);
        const float b1 = readParam<float>(fs, "Camera1.b1", found, 0.f);
        const float b2 = readParam<float>(fs, "Camera1.b2", found, 0.f);
        const float k1 = readParam<float>(fs, "Camera1.k1", found, 0.f);
        const float k2 = readParam<float>(fs, "Camera1.k2", found, 0.f);
        const float k3 = readParam<float>(fs, "Camera1.k3", found, 0.f);
        const float k4 = readParam<float>(fs, "Camera1.k4", found, 0.f);
        const float p1 = readParam<float>(fs, "Camera1.p1", found, 0.f);
        const float p2 = readParam<float>(fs, "Camera1.p2", found, 0.f);
        calib.camera1.reset(ORB_SLAM3::CreateMetashapeCamera(f, cx, cy, width, height, b1, b2, k1, k2, k3, k4, p1, p2));
    }
    else
    {
        throw std::runtime_error("Unsupported Camera.type in calibration: " + camera_type);
    }

    if (stereo)
    {
        calib.bNeedToRectify = true;
        if (camera_type == "PinHole" || camera_type == "Rectified")
        {
            const float fx = readParam<float>(fs, "Camera2.fx", found, 0.f);
            const float fy = readParam<float>(fs, "Camera2.fy", found, 0.f);
            const float cx = readParam<float>(fs, "Camera2.cx", found, 0.f);
            const float cy = readParam<float>(fs, "Camera2.cy", found, 0.f);
            calib.camera2.reset(ORB_SLAM3::CreatePinholeCamera(fx, fy, cx, cy));
            float k1_r = readParam<float>(fs, "Camera2.k1", found, 0.f);
            if (found)
            {
                calib.vPinHoleDistorsion2.push_back(k1_r);
                calib.vPinHoleDistorsion2.push_back(readParam<float>(fs, "Camera2.k2", found, 0.f));
                calib.vPinHoleDistorsion2.push_back(readParam<float>(fs, "Camera2.p1", found, 0.f));
                calib.vPinHoleDistorsion2.push_back(readParam<float>(fs, "Camera2.p2", found, 0.f));
                float k3_r = readParam<float>(fs, "Camera2.k3", found, 0.f);
                if (found)
                {
                    calib.vPinHoleDistorsion2.push_back(k3_r);
                }
            }
        }
        else if (camera_type == "Metashape")
        {
            const float f = readParam<float>(fs, "Camera2.f", found, 0.f);
            const float cx = readParam<float>(fs, "Camera2.cx", found, 0.f);
            const float cy = readParam<float>(fs, "Camera2.cy", found, 0.f);
            const float b1 = readParam<float>(fs, "Camera2.b1", found, 0.f);
            const float b2 = readParam<float>(fs, "Camera2.b2", found, 0.f);
            const float k1 = readParam<float>(fs, "Camera2.k1", found, 0.f);
            const float k2 = readParam<float>(fs, "Camera2.k2", found, 0.f);
            const float k3 = readParam<float>(fs, "Camera2.k3", found, 0.f);
            const float k4 = readParam<float>(fs, "Camera2.k4", found, 0.f);
            const float p1 = readParam<float>(fs, "Camera2.p1", found, 0.f);
            const float p2 = readParam<float>(fs, "Camera2.p2", found, 0.f);
            calib.camera2.reset(
                ORB_SLAM3::CreateMetashapeCamera(f, cx, cy, width, height, b1, b2, k1, k2, k3, k4, p1, p2));
        }
        calib.thDepth = readParam<float>(fs, "Stereo.ThDepth", found, 30.f);
    }

    if (scaling_factor != 1.0f)
    {
        auto scale_camera = [scaling_factor](ORB_SLAM3::GeometricCamera* cam)
        {
            if (!cam)
            {
                return;
            }
            cam->setParameter(cam->getParameter(0) * scaling_factor, 0);                  // fx
            cam->setParameter(cam->getParameter(1) * scaling_factor, 1);                  // fy
            cam->setParameter((cam->getParameter(2) - 0.5f) * scaling_factor + 0.5f, 2);  // cx
            cam->setParameter((cam->getParameter(3) - 0.5f) * scaling_factor + 0.5f, 3);  // cy
            if (cam->size() > 10)
            {
                cam->setParameter(cam->getParameter(10) * scaling_factor, 10);  // skew (Metashape only)
            }
        };
        scale_camera(calib.camera1.get());
        scale_camera(calib.camera2.get());

        auto scale_dim = [scaling_factor](int d)
        {
            return static_cast<int>(std::round(d * scaling_factor));
        };
        calib.originalImSize = cv::Size(scale_dim(calib.originalImSize.width), scale_dim(calib.originalImSize.height));
        calib.newImSize = cv::Size(scale_dim(calib.newImSize.width), scale_dim(calib.newImSize.height));
    }

    return calib;
}

VOTrackingStats tracking_stats_from_result(const ORB_SLAM3::TrackingResult& result, double tracking_time_ms)
{
    VOTrackingStats stats;
    stats.keypoints_detected = static_cast<int>(result.keypoint_data.left_keypoints.size());
    stats.motion_model_primary = result.motion_model_tracking_primary;
    stats.motion_model_success = result.motion_model_result.success;
    stats.motion_model_matches = result.motion_model_result.num_matches;
    stats.motion_model_inliers = result.motion_model_result.num_matches_optimized;
    stats.motion_model_retry = result.motion_model_result.retry;
    stats.ref_kf_primary = result.ref_keyframe_tracking_primary;
    stats.ref_kf_fallback = result.ref_keyframe_tracking_fallback;
    stats.ref_kf_success = result.ref_key_frame_result.success;
    stats.ref_kf_matches = result.ref_key_frame_result.num_matches;
    stats.ref_kf_inliers = result.ref_key_frame_result.num_matches_optimized;
    stats.local_map_success = result.local_map_result.success;
    stats.local_map_inliers = result.local_map_result.num_matches;
    stats.tracked_map_points = static_cast<int>(result.all_tracked_map_points.size());
    stats.new_map_point_candidates = static_cast<int>(result.new_map_point_candidates.size());
    stats.tracking_time_ms = tracking_time_ms;
    return stats;
}

VOResult build_vo_result(const Eigen::Matrix4f& pose_matrix, bool tracking_ok, bool is_keyframe,
                         const VOTrackingStats& tracking_stats)
{
    VOResult vo_result;
    vo_result.pose_matrix = pose_matrix;
    vo_result.tracking_ok = tracking_ok;
    vo_result.is_keyframe = is_keyframe;
    vo_result.tracking_stats = tracking_stats;
    return vo_result;
}

}  // namespace

inline Sophus::SE3f to_se3f(const Eigen::Matrix4f& matrix)
{
    return Sophus::SE3f(Eigen::Quaternionf(matrix.block<3, 3>(0, 0)).normalized(), matrix.block<3, 1>(0, 3));
}

ORBSLAM3Wrapper::ORBSLAM3Wrapper(const static_tf::StaticTfTree& platform_tree, const std::string& vocab_file,
                                 const std::string& camera_calib_file, const std::string& config_file,
                                 const bool verbose, const bool synchronous_local_mapping,
                                 const cli::CommonOptions& common_options)
    : VisualOdometry(platform_tree, common_options),
      use_priors_(common_options.use_priors),
      stereo_(!common_options.monocular)
{
    calib_ = load_camera_calibration(camera_calib_file, stereo_, scaling_factor_);

    if (stereo_)
    {
        const std::string& left_frame = common_options.left_camera_frame_id;
        const std::string& right_frame = common_options.right_camera_frame_id;
        const Eigen::Isometry3d T_left_right = platform_tree.lookup(left_frame, right_frame);
        const Eigen::Matrix4f M = T_left_right.matrix().cast<float>();
        calib_.T_c1_c2 = Sophus::SE3f(Eigen::Quaternionf(M.block<3, 3>(0, 0)).normalized(), M.block<3, 1>(0, 3));
        const Eigen::Vector3f& t_left_right = calib_.T_c1_c2.translation();
        std::cout << std::fixed << std::setprecision(6) << "[ORBSLAM3Wrapper] static_tf translation: T_" << left_frame
                  << "_" << right_frame << " (" << left_frame << " <- " << right_frame << ") = ["
                  << t_left_right.transpose() << "]" << std::endl;
    }

    if (use_priors_)
    {
        dvl_T_cam_ = platform_tree.lookup(common_options.dvl_frame_id, common_options.left_camera_frame_id)
                         .matrix()
                         .cast<float>();
    }

    const std::string run_dir = common_options.output_dir;
    std::string log_path;
    if (run_dir.empty())
    {
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        auto micros = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
        std::string unique_name = "orbslam3_" + std::to_string(micros) + ".log";
        log_path = (std::filesystem::temp_directory_path() / unique_name).string();
    }
    else
    {
        log_path = (std::filesystem::path(run_dir) / "orbslam3.log").string();
    }
    const auto sensor = common_options.monocular ? ORB_SLAM3::System::MONOCULAR : ORB_SLAM3::System::STEREO;
    system_ = std::make_unique<ORB_SLAM3::System>(vocab_file, config_file, sensor, calib_, log_path, verbose,
                                                  synchronous_local_mapping);
    system_->SetLocalMappingCallback(
        [this](const ORB_SLAM3::LocalMappingResult& result)
        {
            {
                std::lock_guard<std::mutex> lock(local_mapping_mutex_);
                local_mapping_queue_.push(result);
            }
            local_mapping_cv_.notify_one();
        });
    local_mapping_worker_running_.store(true, std::memory_order_relaxed);
    local_mapping_worker_thread_ = std::thread(&ORBSLAM3Wrapper::local_mapping_worker_loop, this);
}

ORBSLAM3Wrapper::~ORBSLAM3Wrapper()
{
    // Shut down local mapping first: System::Shutdown() blocks until local
    // mapping finishes draining its own queue, which fires
    // SetLocalMappingCallback for every remaining keyframe. Stopping our
    // worker thread before this (the previous order) meant those results
    // were pushed into local_mapping_queue_ but never drained/published --
    // silently dropped on every shutdown.
    system_->Shutdown();

    local_mapping_worker_running_.store(false, std::memory_order_relaxed);
    local_mapping_cv_.notify_all();
    if (local_mapping_worker_thread_.joinable())
    {
        local_mapping_worker_thread_.join();
    }
}

void ORBSLAM3Wrapper::set_publishers(const std::shared_ptr<TrackingRosPublisher>& tracking,
                                      const std::shared_ptr<LocalMappingPublisher>& local_mapping)
{
    {
        std::lock_guard<std::mutex> lock(ros_publisher_mutex_);
        tracking_publisher_ = tracking;
        local_mapping_publisher_ = local_mapping;
    }

    if (!tracking)
    {
        return;
    }

    sensor_msgs::msg::CameraInfo left_info;
    const cv::Size& im_size = calib_.bNeedToResize1 ? calib_.newImSize : calib_.originalImSize;
    left_info.width = static_cast<uint32_t>(im_size.width);
    left_info.height = static_cast<uint32_t>(im_size.height);
    left_info.distortion_model = "plumb_bob";
    // Debug/tracking images are published raw (ORB-SLAM3 undistorts keypoints,
    // not the image), so CameraInfo.d must reflect the real lens distortion
    // for consumers that want to undistort/rectify downstream. vPinHoleDistorsion1
    // is stored in plumb_bob order [k1, k2, p1, p2, (k3)] -- see
    // load_camera_calibration(). Rectified/Metashape inputs have no
    // vPinHoleDistorsion1 (Metashape's b1/b2/skew model isn't expressible as
    // plumb_bob), so d is left zeroed for those, same as before.
    left_info.d.assign(5, 0.0);
    for (size_t i = 0; i < calib_.vPinHoleDistorsion1.size() && i < left_info.d.size(); ++i)
    {
        left_info.d[i] = static_cast<double>(calib_.vPinHoleDistorsion1[i]);
    }

    if (calib_.camera1)
    {
        const Eigen::Matrix3f K = calib_.camera1->toK_();
        left_info.k[0] = K(0, 0);
        left_info.k[2] = K(0, 2);
        left_info.k[4] = K(1, 1);
        left_info.k[5] = K(1, 2);
        left_info.k[8] = 1.0;
        left_info.p[0] = left_info.k[0];
        left_info.p[2] = left_info.k[2];
        left_info.p[5] = left_info.k[4];
        left_info.p[6] = left_info.k[5];
        left_info.p[10] = 1.0;
    }

    if (stereo_ && calib_.camera2)
    {
        sensor_msgs::msg::CameraInfo right_info = left_info;
        // right_info was copied from left_info, including left_info.d --
        // reset to the right camera's own distortion (or zero, if none).
        right_info.d.assign(5, 0.0);
        for (size_t i = 0; i < calib_.vPinHoleDistorsion2.size() && i < right_info.d.size(); ++i)
        {
            right_info.d[i] = static_cast<double>(calib_.vPinHoleDistorsion2[i]);
        }
        const Eigen::Matrix3f K2 = calib_.camera2->toK_();
        right_info.k[0] = K2(0, 0);
        right_info.k[2] = K2(0, 2);
        right_info.k[4] = K2(1, 1);
        right_info.k[5] = K2(1, 2);
        right_info.k[8] = 1.0;
        right_info.p[0] = right_info.k[0];
        right_info.p[2] = right_info.k[2];
        right_info.p[5] = right_info.k[4];
        right_info.p[6] = right_info.k[5];
        right_info.p[10] = 1.0;

        tracking->set_camera_info(left_info, &right_info);
    }
    else
    {
        tracking->set_camera_info(left_info, nullptr);
    }
}

VOResult ORBSLAM3Wrapper::post_process_tracking_result(const ORB_SLAM3::TrackingResult& result, double timestamp,
                                                       double tracking_duration_ms, cv::Mat debug_left,
                                                       const cv::Mat& debug_right)
{
    const bool tracking_ok = system_->GetTrackingState() == ORB_SLAM3::Tracking::eTrackingState::OK;
    const bool is_keyframe = system_->GetTracker()->isLastFrameKeyframe();
    if (!tracking_ok && was_tracking_ok_)
    {
        local_mapping_reset_.store(true, std::memory_order_relaxed);
    }
    was_tracking_ok_ = tracking_ok;

    if (!debug_left.empty())
    {
        const int marker_thickness = std::max(1, static_cast<int>(std::round(2.0f * scaling_factor_)));
        const int text_margin = static_cast<int>(std::round(80.0f * scaling_factor_));

        for (const auto& kp : result.keypoint_data.left_keypoints)
        {
            cv::circle(debug_left, kp.pt, static_cast<int>(std::round(8.0f * scaling_factor_)), cv::Scalar(0, 0, 255),
                       marker_thickness);
        }

        const auto& mm_matches = result.motion_model_result.frame_matches_optimized;
        for (const auto& m : mm_matches)
        {
            const auto size = static_cast<int>(std::round(20.0f * scaling_factor_));
            const auto half_size = size / 2;
            cv::rectangle(debug_left,
                          cv::Rect(m.current_kp.pt.x - half_size, m.current_kp.pt.y - half_size, size, size),
                          cv::Scalar(0, 255, 0), marker_thickness);
        }

        const auto tracking_state_color = tracking_ok ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
        const std::string tracking_text = tracking_ok ? "TRACKING: OK" : "TRACKING: NOT OK";
        cv::putText(debug_left, tracking_text, cv::Point(text_margin, debug_left.rows - text_margin),
                    cv::FONT_HERSHEY_SIMPLEX, 2.0 * scaling_factor_, tracking_state_color, marker_thickness);

        if (!debug_right.empty())
        {
            cv::Mat combined;
            cv::hconcat(debug_left, debug_right, combined);
            write_debug_video_frame(combined);
        }
        else
        {
            write_debug_video_frame(debug_left);
        }
    }

    std::shared_ptr<TrackingRosPublisher> tracking_pub;
    {
        std::lock_guard<std::mutex> lock(ros_publisher_mutex_);
        tracking_pub = tracking_publisher_;
    }
    if (tracking_pub)
    {
        tracking_pub->publish(timestamp, result, tracking_ok, is_keyframe);
        if (!debug_left.empty())
        {
            tracking_pub->publish_debug_images(timestamp, debug_left, debug_right);
        }
    }

    return build_vo_result(result.pose.matrix(), tracking_ok, is_keyframe,
                           tracking_stats_from_result(result, tracking_duration_ms));
}

VOResult ORBSLAM3Wrapper::process_mono_image_impl(const cv::Mat& image, const DispatchContext& context,
                                                  double timestamp)
{
    cv::Mat track_image = image;
    if (scaling_factor_ != 1.0f)
    {
        cv::resize(image, track_image, cv::Size(), scaling_factor_, scaling_factor_, cv::INTER_LINEAR);
    }

    ORB_SLAM3::TrackingResult result;
    const auto start_time = std::chrono::high_resolution_clock::now();
    if (use_priors_)
    {
        result = system_->TrackMonocular(track_image, timestamp, to_se3f(context.nav.pose * dvl_T_cam_));
    }
    else
    {
        result = system_->TrackMonocular(track_image, timestamp);
    }
    const double duration_ms = static_cast<double>(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time)
            .count());

    cv::Mat debug_image;
    cv::cvtColor(track_image, debug_image, cv::COLOR_GRAY2BGR);
    return post_process_tracking_result(result, timestamp, duration_ms, std::move(debug_image), cv::Mat{});
}

VOResult ORBSLAM3Wrapper::process_stereo_image_impl(const cv::Mat& left_image, const cv::Mat& right_image,
                                                    const DispatchContext& context, double timestamp)
{
    cv::Mat track_left = left_image, track_right = right_image;
    if (scaling_factor_ != 1.0f)
    {
        cv::resize(left_image, track_left, cv::Size(), scaling_factor_, scaling_factor_, cv::INTER_LINEAR);
        cv::resize(right_image, track_right, cv::Size(), scaling_factor_, scaling_factor_, cv::INTER_LINEAR);
    }

    ORB_SLAM3::TrackingResult result;
    const auto start_time = std::chrono::high_resolution_clock::now();
    if (use_priors_)
    {
        result = system_->TrackStereo(track_left, track_right, timestamp, to_se3f(context.nav.pose * dvl_T_cam_));
    }
    else
    {
        result = system_->TrackStereo(track_left, track_right, timestamp);
    }
    const double duration_ms = static_cast<double>(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time)
            .count());

    cv::Mat debug_left, debug_right;
    cv::cvtColor(track_left, debug_left, cv::COLOR_GRAY2BGR);
    cv::cvtColor(track_right, debug_right, cv::COLOR_GRAY2BGR);
    return post_process_tracking_result(result, timestamp, duration_ms, std::move(debug_left), debug_right);
}

void ORBSLAM3Wrapper::local_mapping_worker_loop()
{
    std::vector<ORB_SLAM3::LocalMappingResult> batch;

    while (local_mapping_worker_running_.load(std::memory_order_relaxed))
    {
        batch.clear();
        {
            std::unique_lock<std::mutex> lock(local_mapping_mutex_);
            if (local_mapping_queue_.empty())
            {
                // Wake immediately when SetLocalMappingCallback pushes a new
                // result (or the destructor stops the worker). The bounded
                // timeout is a safety net for local_mapping_reset_, which is
                // set by the Tracking thread independently of this queue/CV.
                local_mapping_cv_.wait_for(lock, std::chrono::milliseconds(5),
                                           [this]
                                           {
                                               return !local_mapping_queue_.empty() ||
                                                      !local_mapping_worker_running_.load(std::memory_order_relaxed);
                                           });
            }
            while (!local_mapping_queue_.empty())
            {
                batch.push_back(std::move(local_mapping_queue_.front()));
                local_mapping_queue_.pop();
            }
        }

        std::shared_ptr<LocalMappingPublisher> local_mapping_pub;
        {
            std::lock_guard<std::mutex> lock(ros_publisher_mutex_);
            local_mapping_pub = local_mapping_publisher_;
        }

        if (local_mapping_pub && local_mapping_reset_.exchange(false, std::memory_order_relaxed))
        {
            local_mapping_pub->reset();
        }

        if (local_mapping_pub && !batch.empty())
        {
            local_mapping_pub->publish_batch(batch);
        }
    }
}

}  // namespace visual_odometry
