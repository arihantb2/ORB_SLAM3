#include "local_mapping_ros_publisher.h"

#include <sstream>

namespace visual_odometry
{

LocalMappingRosPublisher::LocalMappingRosPublisher(const rclcpp::Node::SharedPtr& node, const Options& options)
    : node_(node), options_(options)
{
    viz_pub_ = std::make_shared<LocalMappingVisualizationPublisher>(node_, options_.map_frame_id);
    if (options_.publish_diagnostics)
    {
        diagnostics_pub_ =
            node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(options_.diagnostics_topic, 10);
    }
}

diagnostic_msgs::msg::DiagnosticStatus LocalMappingRosPublisher::make_diagnostic_status(
    const ORB_SLAM3::LocalMappingResult& r, uint64_t dropped_queue_total) const
{
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "orb_slam3_local_mapping";
    status.hardware_id = "orb_slam3";
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;

    std::ostringstream summary;
    summary << "iteration=" << r.iteration << " keyframe_id=" << r.keyframe_id;
    status.message = summary.str();

    auto add_kv = [&status](const std::string& key, const std::string& value)
    {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = key;
        kv.value = value;
        status.values.push_back(kv);
    };

    add_kv("iteration", std::to_string(r.iteration));
    add_kv("keyframe_id", std::to_string(r.keyframe_id));
    add_kv("frame_id", std::to_string(r.frame_id));
    add_kv("keyframe_timestamp_sec", std::to_string(r.timestamp));

    const auto& pnkf = r.process_new_keyframe;
    add_kv("pnkf_num_map_point_slots", std::to_string(pnkf.num_kf_map_point_slots));
    add_kv("pnkf_num_associated_map_points", std::to_string(pnkf.num_associated_map_points));
    add_kv("pnkf_num_stereo_map_points_registered", std::to_string(pnkf.num_stereo_map_points_registered));
    add_kv("pnkf_new_keyframe_queue_size_after", std::to_string(pnkf.queue_size_after));
    add_kv("pnkf_stage_duration_ms", std::to_string(pnkf.duration_ms));

    const auto& mpc = r.map_point_culling;
    add_kv("mpc_num_recent_map_points_before", std::to_string(mpc.num_recent_map_points_before));
    add_kv("mpc_num_culled_already_bad", std::to_string(mpc.num_culled_already_bad));
    add_kv("mpc_num_culled_low_found_ratio", std::to_string(mpc.num_culled_low_found_ratio));
    add_kv("mpc_num_culled_too_few_observations", std::to_string(mpc.num_culled_too_few_observations));
    add_kv("mpc_num_graduated_from_recent_list", std::to_string(mpc.num_graduated));
    add_kv("mpc_num_recent_map_points_after", std::to_string(mpc.num_recent_map_points_after));
    add_kv("mpc_num_map_points_set_bad", std::to_string(mpc.culled_map_point_ids.size()));
    add_kv("mpc_stage_duration_ms", std::to_string(mpc.duration_ms));

    const auto& cnmp = r.create_new_map_points;
    add_kv("cnmp_num_neighbour_keyframes", std::to_string(cnmp.num_neighbour_kfs));
    add_kv("cnmp_num_epipolar_matches", std::to_string(cnmp.num_epipolar_matches));
    add_kv("cnmp_num_stereo_unproject_attempts", std::to_string(cnmp.num_stereo_unproject_attempts));
    add_kv("cnmp_num_map_points_created", std::to_string(cnmp.num_created));
    add_kv("cnmp_num_map_points_created_from_stereo", std::to_string(cnmp.num_created_from_stereo));
    add_kv("cnmp_aborted_early", cnmp.aborted_early ? "true" : "false");
    add_kv("cnmp_stage_duration_ms", std::to_string(cnmp.duration_ms));

    add_kv("search_in_neighbors_skipped", r.search_in_neighbors_skipped ? "true" : "false");
    const auto& sinr = r.search_in_neighbors;
    add_kv("sin_num_first_level_neighbours", std::to_string(sinr.num_first_level_neighbours));
    add_kv("sin_num_second_level_neighbours", std::to_string(sinr.num_second_level_neighbours));
    add_kv("sin_num_target_keyframes", std::to_string(sinr.num_target_kfs));
    add_kv("sin_aborted_early", sinr.aborted_early ? "true" : "false");
    add_kv("sin_stage_duration_ms", std::to_string(sinr.duration_ms));

    const auto& lba = r.lba;
    add_kv("lba_skipped", lba.skipped ? "true" : "false");
    add_kv("lba_skip_reason", lba.skip_reason);
    add_kv("lba_num_fixed_keyframes", std::to_string(lba.num_fixed_kfs));
    add_kv("lba_num_optimised_keyframes", std::to_string(lba.num_optimised_kfs));
    add_kv("lba_num_map_points_in_optimisation", std::to_string(lba.num_map_points));
    add_kv("lba_num_reprojection_edges", std::to_string(lba.num_edges));
    add_kv("lba_num_outlier_map_points", std::to_string(lba.num_outlier_map_points));
    add_kv("lba_num_covisibility_edges", std::to_string(lba.covisibility_edges.size()));
    add_kv("lba_num_spanning_tree_edges", std::to_string(lba.spanning_tree_edges.size()));
    add_kv("lba_stage_duration_ms", std::to_string(lba.duration_ms));

    const auto& kfc = r.keyframe_culling;
    add_kv("kfc_num_keyframes_checked", std::to_string(kfc.num_kfs_checked));
    add_kv("kfc_num_keyframes_culled", std::to_string(kfc.num_kfs_culled));
    add_kv("kfc_aborted_early", kfc.aborted_early ? "true" : "false");
    add_kv("kfc_stage_duration_ms", std::to_string(kfc.duration_ms));

    add_kv("num_added_map_points", std::to_string(r.added_map_points.size()));
    add_kv("num_culled_map_point_ids", std::to_string(r.culled_map_point_ids.size()));
    add_kv("num_lba_outlier_map_point_ids", std::to_string(r.lba_outlier_map_point_ids.size()));
    add_kv("total_duration_ms", std::to_string(r.total_duration_ms));
    add_kv("queue_dropped_total", std::to_string(dropped_queue_total));

    return status;
}

void LocalMappingRosPublisher::publish_batch(const std::vector<ORB_SLAM3::LocalMappingResult>& batch,
                                             uint64_t dropped_count)
{
    if (diagnostics_pub_ && !batch.empty())
    {
        diagnostic_msgs::msg::DiagnosticArray array_msg;
        array_msg.header.stamp = node_->now();
        array_msg.status.reserve(batch.size());
        for (const ORB_SLAM3::LocalMappingResult& result : batch)
        {
            array_msg.status.push_back(make_diagnostic_status(result, dropped_count));
        }
        diagnostics_pub_->publish(array_msg);
    }

    if (!viz_pub_)
    {
        return;
    }
    viz_pub_->publish_batch(batch, dropped_count);
}

void LocalMappingRosPublisher::reset_visualization()
{
    if (!viz_pub_)
    {
        return;
    }
    viz_pub_->reset();
}

}  // namespace visual_odometry
