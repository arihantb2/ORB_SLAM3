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
#include <string>
#include <thread>
#include <vector>

#include "local_mapping_ros_publisher.h"
#include "tracking_ros_publisher.h"

namespace visual_odometry
{
class ORBSLAM3Wrapper : public VisualOdometry
{
public:
    ORBSLAM3Wrapper(const static_tf::StaticTfTree& platform_tree, const std::string& vocab_file,
                    const std::string& camera_calib_file, const std::string& config_file, const bool verbose,
                    const bool synchronous_local_mapping, const cli::CommonOptions& common_options);
    ~ORBSLAM3Wrapper();

    void set_publishers(std::shared_ptr<TrackingRosPublisher> tracking,
                        std::shared_ptr<LocalMappingRosPublisher> local_mapping);

private:
    class LocalMappingResultQueue
    {
    public:
        struct Entry
        {
            bool is_reset = false;
            ORB_SLAM3::LocalMappingResult result;
        };

        explicit LocalMappingResultQueue(size_t capacity);
        bool try_push(const ORB_SLAM3::LocalMappingResult& result);
        bool try_push_reset();
        bool try_pop(Entry& entry);

    private:
        std::vector<Entry> buffer_;
        const size_t capacity_;
        std::atomic<size_t> head_;
        std::atomic<size_t> tail_;
    };

    void local_mapping_worker_loop();

    ORB_SLAM3::CameraCalibrationInput calib_;
    std::unique_ptr<ORB_SLAM3::System> system_;

    std::shared_ptr<TrackingRosPublisher> tracking_publisher_;
    std::shared_ptr<LocalMappingRosPublisher> local_mapping_publisher_;
    mutable std::mutex ros_publisher_mutex_;

    LocalMappingResultQueue local_mapping_queue_{512};
    std::atomic<bool> local_mapping_worker_running_{false};
    std::thread local_mapping_worker_thread_;
    std::atomic<uint64_t> dropped_local_mapping_results_{0};
    bool was_tracking_ok_{false};

    VOResult process_mono_image_impl(const cv::Mat& image, const DispatchContext& context, double timestamp) override;
    VOResult process_stereo_image_impl(const cv::Mat& left_image, const cv::Mat& right_image,
                                       const DispatchContext& context, double timestamp) override;

    // Shared post-tracking logic: state update, debug drawing, publishing, result construction.
    // Pass an empty cv::Mat{} for debug_right in monocular mode.
    VOResult post_process_tracking_result(const ORB_SLAM3::TrackingResult& result, double timestamp,
                                          double tracking_duration_ms, cv::Mat debug_left, const cv::Mat& debug_right);

    bool use_priors_ = false;
    bool stereo_ = false;
    float scaling_factor_ = 1.0f;
    Eigen::Matrix4f dvl_T_cam_ = Eigen::Matrix4f::Identity();
};

}  // namespace visual_odometry