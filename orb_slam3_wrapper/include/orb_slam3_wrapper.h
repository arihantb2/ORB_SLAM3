#pragma once

#include <vo/visual_odometry.h>

#include "CameraModels/CameraCalibrationInput.h"
#include "ORB_SLAM3/LocalMappingResult.h"
#include "ORB_SLAM3/System.h"
#include "ORB_SLAM3/Tracking.h"

#include <acfrlcm/auv_acfr_nav_t.hpp>
#include <acfrlcm/auv_vis_rawlog_t.hpp>

#include <utils/vo_utils.h>
#include <opencv2/opencv.hpp>

#include <atomic>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include "local_mapping_visualization_publisher.h"
#include "tracking_ros_publisher.h"

namespace visual_odometry
{
class ORBSLAM3Wrapper : public VisualOdometry
{
public:
    ORBSLAM3Wrapper(const static_tf::StaticTfTree& platform_tree, const std::string& vocab_file,
                    const std::string& camera_calib_file, const std::string& config_file, const bool verbose,
                    const bool synchronous_local_mapping, const cli::CommonOptions& common_options);
    ~ORBSLAM3Wrapper() override;

    void set_publishers(const std::shared_ptr<TrackingRosPublisher>& tracking,
                        const std::shared_ptr<LocalMappingPublisher>& local_mapping);

private:
    void local_mapping_worker_loop();

    ORB_SLAM3::CameraCalibrationInput calib_;
    std::unique_ptr<ORB_SLAM3::System> system_;

    std::shared_ptr<TrackingRosPublisher> tracking_publisher_;
    std::shared_ptr<LocalMappingPublisher> local_mapping_publisher_;
    mutable std::mutex ros_publisher_mutex_;

    std::queue<ORB_SLAM3::LocalMappingResult> local_mapping_queue_;
    std::mutex local_mapping_mutex_;
    std::atomic<bool> local_mapping_reset_{false};
    std::atomic<bool> local_mapping_worker_running_{false};
    std::thread local_mapping_worker_thread_;
    bool was_tracking_ok_{false};

    VOResult process_mono_image_impl(const cv::Mat& image, const DispatchContext& context, double timestamp) override;
    VOResult process_stereo_image_impl(const cv::Mat& left_image, const cv::Mat& right_image,
                                       const DispatchContext& context, double timestamp) override;

    VOResult post_process_tracking_result(const ORB_SLAM3::TrackingResult& result, double timestamp,
                                          double tracking_duration_ms, cv::Mat debug_left, const cv::Mat& debug_right);

    bool use_priors_ = false;
    bool stereo_ = false;
    float scaling_factor_ = 1.0f;
    Eigen::Matrix4f dvl_T_cam_ = Eigen::Matrix4f::Identity();
};

}  // namespace visual_odometry
