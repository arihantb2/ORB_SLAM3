#pragma once

#include "ORB_SLAM3/LocalMappingResult.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace visual_odometry
{

class LocalMappingVisualizationPublisher
{
public:
    LocalMappingVisualizationPublisher(const rclcpp::Node::SharedPtr& node, const std::string& map_frame_id);

    void publish_batch(const std::vector<ORB_SLAM3::LocalMappingResult>& batch, uint64_t dropped_count);
    void reset();

private:
    void update_state_from_result(const ORB_SLAM3::LocalMappingResult& result);
    sensor_msgs::msg::PointCloud2 make_point_cloud(const std::vector<Eigen::Vector3f>& points,
                                                   const rclcpp::Time& stamp) const;
    void publish_visualization(const rclcpp::Time& stamp);

    rclcpp::Node::SharedPtr node_;
    std::string map_frame_id_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lba_map_points_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lba_map_point_outliers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr keyframes_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr covisibility_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr spanning_tree_pub_;

    std::unordered_map<unsigned long, Eigen::Vector3f> map_points_;
    std::unordered_map<unsigned long, Sophus::SE3f> keyframe_poses_;
    std::unordered_set<unsigned long> latest_lba_keyframes_;
    std::unordered_set<unsigned long> lba_opt_kfs_;
    std::unordered_set<unsigned long> lba_fixed_kfs_;
    std::vector<ORB_SLAM3::CovisibilityEdge> latest_covisibility_edges_;
    std::vector<Eigen::Vector3f> latest_lba_map_points_;
    std::vector<Eigen::Vector3f> latest_lba_outlier_points_;
    std::unordered_map<unsigned long, unsigned long> spanning_tree_parents_;
    std::unordered_set<unsigned long> root_keyframes_;
};

}  // namespace visual_odometry
