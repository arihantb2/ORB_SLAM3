#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <ORB_SLAM3/Tracking.h>

namespace visual_odometry
{

class TrackingRosPublisher
{
public:
    struct Options
    {
        std::string map_frame_id = "vo_map";
        std::string camera_frame_id = "vo_camera";

        std::string pose_topic = "/orb_slam3/tracking/pose";
        std::string odom_topic = "/orb_slam3/tracking/odometry";
        std::string keyframe_path_topic = "/orb_slam3/tracking/keyframe_path";

        std::string map_points_inlier_topic = "/orb_slam3/tracking/map_points/inlier_all";
        std::string map_points_local_inlier_topic = "/orb_slam3/tracking/map_points/local_inliers";
        std::string map_points_local_outlier_topic = "/orb_slam3/tracking/map_points/local_outliers";
        std::string map_points_new_candidates_topic = "/orb_slam3/tracking/map_points/new_candidates";

        std::string matches_marker_topic = "/orb_slam3/tracking/matches";
        std::string diagnostics_topic = "/orb_slam3/tracking/diagnostics";

        // Debug image topics are chosen to work well with RViz Camera display:
        // base: /orb_slam3/tracking/debug/left  → image: /image, camera info: /camera_info
        // base: /orb_slam3/tracking/debug/right → image: /image, camera info: /camera_info
        std::string debug_left_base = "/orb_slam3/tracking/debug/left";
        std::string debug_right_base = "/orb_slam3/tracking/debug/right";

        bool publish_pose = true;
        bool publish_odometry = true;
        bool publish_keyframe_path = true;
        bool publish_point_clouds = true;
        bool publish_matches = true;
        bool publish_diagnostics = true;
        bool publish_debug_images = true;
    };

    TrackingRosPublisher(const rclcpp::Node::SharedPtr& node, const Options& options);

    void publish(double timestamp_sec, const ORB_SLAM3::TrackingResult& result, bool tracking_ok, bool is_keyframe);
    void publish_debug_images(double timestamp_sec, const cv::Mat& left_image, const cv::Mat& right_image);

    void set_camera_info(const sensor_msgs::msg::CameraInfo& left_info,
                         const sensor_msgs::msg::CameraInfo* right_info = nullptr);

private:
    rclcpp::Node::SharedPtr node_;
    Options options_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr keyframe_path_pub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_inlier_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_local_inlier_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_local_outlier_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_new_candidates_pub_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr matches_marker_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_left_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_right_pub_;

    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_left_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_right_pub_;

    bool have_origin_ = false;
    Eigen::Matrix4f origin_pose_inv_ = Eigen::Matrix4f::Identity();

    nav_msgs::msg::Path keyframe_path_msg_;
    std::optional<sensor_msgs::msg::CameraInfo> left_camera_info_;
    std::optional<sensor_msgs::msg::CameraInfo> right_camera_info_;

    geometry_msgs::msg::PoseStamped make_pose_stamped(double timestamp_sec, const Eigen::Matrix4f& T_map_cam) const;
    nav_msgs::msg::Odometry make_odometry(double timestamp_sec, const Eigen::Matrix4f& T_map_cam) const;
    geometry_msgs::msg::TransformStamped make_transform(double timestamp_sec, const Eigen::Matrix4f& T_map_cam) const;

    sensor_msgs::msg::PointCloud2 make_point_cloud_from_observations(
        double timestamp_sec, const std::vector<ORB_SLAM3::MapPointObservation>& observations) const;
    sensor_msgs::msg::PointCloud2 make_point_cloud_from_candidates(
        double timestamp_sec, const std::vector<ORB_SLAM3::NewMapPointCandidate>& candidates) const;

    visualization_msgs::msg::MarkerArray make_matches_markers(double timestamp_sec,
                                                              const ORB_SLAM3::TrackingResult& result) const;
    diagnostic_msgs::msg::DiagnosticArray make_diagnostics(double timestamp_sec,
                                                           const ORB_SLAM3::TrackingResult& result,
                                                           bool tracking_ok) const;
};

}  // namespace visual_odometry
