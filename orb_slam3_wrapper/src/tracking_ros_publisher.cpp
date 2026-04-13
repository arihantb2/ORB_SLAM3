#include "tracking_ros_publisher.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sensor_msgs/msg/point_field.hpp>

namespace visual_odometry
{

namespace
{

inline rclcpp::Time to_ros_time(double timestamp_sec)
{
    const int64_t sec = static_cast<int64_t>(timestamp_sec);
    const uint32_t nsec = static_cast<uint32_t>((timestamp_sec - static_cast<double>(sec)) * 1e9);
    return rclcpp::Time(sec, nsec);
}

inline geometry_msgs::msg::Quaternion eigen_to_msg_quat(const Eigen::Quaternionf& q)
{
    geometry_msgs::msg::Quaternion msg;
    msg.x = static_cast<double>(q.x());
    msg.y = static_cast<double>(q.y());
    msg.z = static_cast<double>(q.z());
    msg.w = static_cast<double>(q.w());
    return msg;
}

inline geometry_msgs::msg::Point eigen_to_msg_point(const Eigen::Vector3f& p)
{
    geometry_msgs::msg::Point msg;
    msg.x = static_cast<double>(p.x());
    msg.y = static_cast<double>(p.y());
    msg.z = static_cast<double>(p.z());
    return msg;
}

inline std::pair<Eigen::Quaternionf, Eigen::Vector3f> decompose_pose(const Eigen::Matrix4f& T)
{
    Eigen::Quaternionf q(T.block<3, 3>(0, 0));
    q.normalize();
    return {q, T.block<3, 1>(0, 3)};
}

template <typename Container>
sensor_msgs::msg::PointCloud2 make_point_cloud_xyz(double timestamp_sec, const std::string& frame_id,
                                                   const Container& points)
{
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = to_ros_time(timestamp_sec);
    cloud.header.frame_id = frame_id;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(points.size());
    cloud.is_bigendian = false;
    cloud.is_dense = true;

    cloud.fields.resize(3);
    cloud.fields[0].name = "x";
    cloud.fields[0].offset = 0;
    cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[0].count = 1;
    cloud.fields[1].name = "y";
    cloud.fields[1].offset = 4;
    cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[1].count = 1;
    cloud.fields[2].name = "z";
    cloud.fields[2].offset = 8;
    cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[2].count = 1;

    cloud.point_step = 12;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.data.resize(cloud.row_step * cloud.height);

    uint8_t* ptr = cloud.data.data();
    for (const auto& item : points)
    {
        const Eigen::Vector3f& p = item.pos_world;
        float* fptr = reinterpret_cast<float*>(ptr);
        fptr[0] = p.x();
        fptr[1] = p.y();
        fptr[2] = p.z();
        ptr += cloud.point_step;
    }
    return cloud;
}

inline sensor_msgs::msg::Image make_bgr8_image_msg(double timestamp_sec, const cv::Mat& image,
                                                   const std::string& frame_id)
{
    sensor_msgs::msg::Image msg;
    msg.header.stamp = to_ros_time(timestamp_sec);
    msg.header.frame_id = frame_id;
    msg.height = static_cast<uint32_t>(image.rows);
    msg.width = static_cast<uint32_t>(image.cols);
    msg.encoding = "bgr8";
    msg.is_bigendian = false;
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(image.cols * 3);
    const std::size_t total_bytes = static_cast<std::size_t>(image.rows) * image.cols * 3;
    msg.data.assign(image.data, image.data + total_bytes);
    return msg;
}

}  // namespace

TrackingRosPublisher::TrackingRosPublisher(const rclcpp::Node::SharedPtr& node, const Options& options)
    : node_(node), options_(options)
{
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

    if (options_.publish_pose)
    {
        pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(options_.pose_topic, 10);
    }
    if (options_.publish_odometry)
    {
        odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(options_.odom_topic, 10);
    }
    if (options_.publish_keyframe_path)
    {
        keyframe_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(options_.keyframe_path_topic, 10);
        keyframe_path_msg_.header.frame_id = options_.map_frame_id;
    }
    if (options_.publish_point_clouds)
    {
        map_points_inlier_pub_ =
            node_->create_publisher<sensor_msgs::msg::PointCloud2>(options_.map_points_inlier_topic, 10);
        map_points_local_inlier_pub_ =
            node_->create_publisher<sensor_msgs::msg::PointCloud2>(options_.map_points_local_inlier_topic, 10);
        map_points_local_outlier_pub_ =
            node_->create_publisher<sensor_msgs::msg::PointCloud2>(options_.map_points_local_outlier_topic, 10);
        map_points_new_candidates_pub_ =
            node_->create_publisher<sensor_msgs::msg::PointCloud2>(options_.map_points_new_candidates_topic, 10);
    }
    if (options_.publish_matches)
    {
        matches_marker_pub_ =
            node_->create_publisher<visualization_msgs::msg::MarkerArray>(options_.matches_marker_topic, 10);
    }
    if (options_.publish_diagnostics)
    {
        diagnostics_pub_ =
            node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(options_.diagnostics_topic, 10);
    }
    if (options_.publish_debug_images)
    {
        const std::string left_image_topic = options_.debug_left_base + "/image";
        const std::string left_info_topic = options_.debug_left_base + "/camera_info";
        const std::string right_image_topic = options_.debug_right_base + "/image";
        const std::string right_info_topic = options_.debug_right_base + "/camera_info";

        debug_image_left_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(left_image_topic, 10);
        debug_image_right_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(right_image_topic, 10);

        camera_info_left_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(left_info_topic, 10);
        camera_info_right_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(right_info_topic, 10);
    }
}

geometry_msgs::msg::PoseStamped TrackingRosPublisher::make_pose_stamped(double timestamp_sec,
                                                                        const Eigen::Matrix4f& T_map_cam) const
{
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = to_ros_time(timestamp_sec);
    msg.header.frame_id = options_.map_frame_id;

    const auto [q, t] = decompose_pose(T_map_cam);
    msg.pose.position = eigen_to_msg_point(t);
    msg.pose.orientation = eigen_to_msg_quat(q);
    return msg;
}

nav_msgs::msg::Odometry TrackingRosPublisher::make_odometry(double timestamp_sec,
                                                            const Eigen::Matrix4f& T_map_cam) const
{
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = to_ros_time(timestamp_sec);
    msg.header.frame_id = options_.map_frame_id;
    msg.child_frame_id = options_.camera_frame_id;

    const auto [q, t] = decompose_pose(T_map_cam);
    msg.pose.pose.position = eigen_to_msg_point(t);
    msg.pose.pose.orientation = eigen_to_msg_quat(q);
    // Leave covariance at default; this node is a visualizer, not an estimator.
    return msg;
}

geometry_msgs::msg::TransformStamped TrackingRosPublisher::make_transform(double timestamp_sec,
                                                                          const Eigen::Matrix4f& T_map_cam) const
{
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = to_ros_time(timestamp_sec);
    msg.header.frame_id = options_.map_frame_id;
    msg.child_frame_id = options_.camera_frame_id;

    const auto [q, t] = decompose_pose(T_map_cam);
    msg.transform.translation.x = static_cast<double>(t.x());
    msg.transform.translation.y = static_cast<double>(t.y());
    msg.transform.translation.z = static_cast<double>(t.z());
    msg.transform.rotation = eigen_to_msg_quat(q);
    return msg;
}

sensor_msgs::msg::PointCloud2 TrackingRosPublisher::make_point_cloud_from_observations(
    double timestamp_sec, const std::vector<ORB_SLAM3::MapPointObservation>& observations) const
{
    return make_point_cloud_xyz(timestamp_sec, options_.map_frame_id, observations);
}

sensor_msgs::msg::PointCloud2 TrackingRosPublisher::make_point_cloud_from_candidates(
    double timestamp_sec, const std::vector<ORB_SLAM3::NewMapPointCandidate>& candidates) const
{
    return make_point_cloud_xyz(timestamp_sec, options_.map_frame_id, candidates);
}

visualization_msgs::msg::MarkerArray TrackingRosPublisher::make_matches_markers(
    double timestamp_sec, const ORB_SLAM3::TrackingResult& result) const
{
    const rclcpp::Time stamp = to_ros_time(timestamp_sec);

    auto make_match_lines = [&](const std::string& ns, int id, float r, float g, float b,
                                const auto& matches) -> visualization_msgs::msg::Marker
    {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = stamp;
        marker.header.frame_id = options_.camera_frame_id;
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.002;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 0.7f;
        marker.points.reserve(matches.size() * 2);
        for (const auto& m : matches)
        {
            geometry_msgs::msg::Point p_curr;
            p_curr.x = m.current_kp.pt.x;
            p_curr.y = m.current_kp.pt.y;
            p_curr.z = 0.0;
            geometry_msgs::msg::Point p_src;
            p_src.x = m.source_kp.pt.x;
            p_src.y = m.source_kp.pt.y;
            p_src.z = 0.0;
            marker.points.push_back(p_curr);
            marker.points.push_back(p_src);
        }
        return marker;
    };

    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(make_match_lines("motion_model_matches", 0, 0.0f, 1.0f, 0.0f,
                                               result.motion_model_result.frame_matches_optimized));
    markers.markers.push_back(make_match_lines("ref_kf_matches", 1, 0.0f, 0.0f, 1.0f,
                                               result.ref_key_frame_result.kf_matches_optimized));
    return markers;
}

diagnostic_msgs::msg::DiagnosticArray TrackingRosPublisher::make_diagnostics(double timestamp_sec,
                                                                             const ORB_SLAM3::TrackingResult& result,
                                                                             bool tracking_ok) const
{
    diagnostic_msgs::msg::DiagnosticArray array_msg;
    array_msg.header.stamp = to_ros_time(timestamp_sec);

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "orb_slam3_tracking";
    status.hardware_id = "orb_slam3";

    status.level =
        tracking_ok ? diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = tracking_ok ? "Tracking OK" : "Tracking lost or not initialized";

    auto add_kv = [&status](const std::string& key, const std::string& value)
    {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = key;
        kv.value = value;
        status.values.push_back(kv);
    };

    add_kv("tracking_result_success", result.success ? "true" : "false");
    add_kv("motion_model_success", result.motion_model_result.success ? "true" : "false");
    add_kv("ref_keyframe_success", result.ref_key_frame_result.success ? "true" : "false");
    add_kv("local_map_success", result.local_map_result.success ? "true" : "false");

    add_kv("motion_model_num_matches", std::to_string(result.motion_model_result.num_matches));
    add_kv("motion_model_num_inliers", std::to_string(result.motion_model_result.num_matches_optimized));
    add_kv("ref_kf_num_matches", std::to_string(result.ref_key_frame_result.num_matches));
    add_kv("ref_kf_num_inliers", std::to_string(result.ref_key_frame_result.num_matches_optimized));
    add_kv("local_map_num_matches", std::to_string(result.local_map_result.num_matches));
    add_kv("tracked_map_points", std::to_string(result.all_tracked_map_points.size()));
    add_kv("new_map_point_candidates", std::to_string(result.new_map_point_candidates.size()));

    array_msg.status.push_back(status);
    return array_msg;
}

void TrackingRosPublisher::publish(double timestamp_sec, const ORB_SLAM3::TrackingResult& result, bool tracking_ok,
                                   bool is_keyframe)
{
    const Eigen::Matrix4f cam_pose_matrix = result.pose.matrix();

    if (tracking_ok && !have_origin_)
    {
        have_origin_ = true;
    }

    if (!have_origin_)
    {
        if (options_.publish_diagnostics && diagnostics_pub_)
        {
            const auto diag_msg = make_diagnostics(timestamp_sec, result, false);
            diagnostics_pub_->publish(diag_msg);
        }
        return;
    }

    const Eigen::Matrix4f T_map_cam = origin_pose_inv_ * cam_pose_matrix;

    if (options_.publish_pose && pose_pub_)
    {
        const auto pose_msg = make_pose_stamped(timestamp_sec, T_map_cam);
        pose_pub_->publish(pose_msg);
    }

    if (options_.publish_odometry && odom_pub_)
    {
        const auto odom_msg = make_odometry(timestamp_sec, T_map_cam);
        odom_pub_->publish(odom_msg);
    }

    if (tf_broadcaster_)
    {
        const auto tf_msg = make_transform(timestamp_sec, T_map_cam);
        tf_broadcaster_->sendTransform(tf_msg);
    }

    if (options_.publish_keyframe_path && keyframe_path_pub_)
    {
        if (!tracking_ok)
        {
            keyframe_path_msg_.poses.clear();
            keyframe_path_msg_.header.stamp = to_ros_time(timestamp_sec);
            keyframe_path_pub_->publish(keyframe_path_msg_);
        }
        else if (is_keyframe)
        {
            const auto pose_msg = make_pose_stamped(timestamp_sec, T_map_cam);
            keyframe_path_msg_.poses.push_back(pose_msg);
            keyframe_path_msg_.header.stamp = pose_msg.header.stamp;
            keyframe_path_pub_->publish(keyframe_path_msg_);
        }
    }

    if (options_.publish_point_clouds)
    {
        if (map_points_inlier_pub_ && !result.all_tracked_map_points.empty())
        {
            const auto cloud = make_point_cloud_from_observations(timestamp_sec, result.all_tracked_map_points);
            map_points_inlier_pub_->publish(cloud);
        }

        if (map_points_local_inlier_pub_ && !result.local_map_result.inlier_observations.empty())
        {
            const auto cloud =
                make_point_cloud_from_observations(timestamp_sec, result.local_map_result.inlier_observations);
            map_points_local_inlier_pub_->publish(cloud);
        }

        if (map_points_local_outlier_pub_ && !result.local_map_result.outlier_observations.empty())
        {
            const auto cloud =
                make_point_cloud_from_observations(timestamp_sec, result.local_map_result.outlier_observations);
            map_points_local_outlier_pub_->publish(cloud);
        }

        if (map_points_new_candidates_pub_ && !result.new_map_point_candidates.empty())
        {
            const auto cloud = make_point_cloud_from_candidates(timestamp_sec, result.new_map_point_candidates);
            map_points_new_candidates_pub_->publish(cloud);
        }
    }

    if (options_.publish_matches && matches_marker_pub_)
    {
        const auto markers = make_matches_markers(timestamp_sec, result);
        matches_marker_pub_->publish(markers);
    }

    if (options_.publish_diagnostics && diagnostics_pub_)
    {
        const auto diag_msg = make_diagnostics(timestamp_sec, result, tracking_ok);
        diagnostics_pub_->publish(diag_msg);
    }
}

void TrackingRosPublisher::publish_debug_images(double timestamp_sec, const cv::Mat& left_image,
                                                const cv::Mat& right_image)
{
    if (!options_.publish_debug_images)
    {
        return;
    }

    if (debug_image_left_pub_ && !left_image.empty())
    {
        auto left_msg = make_bgr8_image_msg(timestamp_sec, left_image, options_.camera_frame_id);
        debug_image_left_pub_->publish(left_msg);

        if (camera_info_left_pub_ && left_camera_info_)
        {
            auto cam = *left_camera_info_;
            cam.header.stamp = left_msg.header.stamp;
            cam.header.frame_id = options_.camera_frame_id;
            camera_info_left_pub_->publish(cam);
        }
    }

    if (debug_image_right_pub_ && !right_image.empty())
    {
        auto right_msg = make_bgr8_image_msg(timestamp_sec, right_image, options_.camera_frame_id);
        debug_image_right_pub_->publish(right_msg);

        if (camera_info_right_pub_ && right_camera_info_)
        {
            auto cam = *right_camera_info_;
            cam.header.stamp = right_msg.header.stamp;
            cam.header.frame_id = options_.camera_frame_id;
            camera_info_right_pub_->publish(cam);
        }
    }
}

void TrackingRosPublisher::set_camera_info(const sensor_msgs::msg::CameraInfo& left_info,
                                           const sensor_msgs::msg::CameraInfo* right_info)
{
    left_camera_info_ = left_info;
    right_camera_info_ = right_info ? std::make_optional(*right_info) : std::nullopt;
}

}  // namespace visual_odometry
