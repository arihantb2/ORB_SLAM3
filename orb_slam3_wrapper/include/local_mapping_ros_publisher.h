#pragma once

#include <memory>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ORB_SLAM3/LocalMappingResult.h>

#include "local_mapping_visualization_publisher.h"

namespace visual_odometry
{

class LocalMappingRosPublisher
{
public:
    struct Options
    {
        std::string map_frame_id = "vo_map";
        std::string diagnostics_topic = "/orb_slam3/local_mapping/diagnostics";
        bool publish_diagnostics = true;
    };

    LocalMappingRosPublisher(const rclcpp::Node::SharedPtr& node, const Options& options);

    void publish_batch(const std::vector<ORB_SLAM3::LocalMappingResult>& batch, uint64_t dropped_count);
    void reset_visualization();

private:
    rclcpp::Node::SharedPtr node_;
    Options options_;

    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
    std::shared_ptr<LocalMappingVisualizationPublisher> viz_pub_;

    diagnostic_msgs::msg::DiagnosticStatus make_diagnostic_status(const ORB_SLAM3::LocalMappingResult& result,
                                                                  uint64_t dropped_queue_total) const;
};

}  // namespace visual_odometry
