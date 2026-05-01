#include "local_mapping_visualization_publisher.h"

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <sstream>

namespace visual_odometry
{
namespace
{
std_msgs::msg::ColorRGBA color(float r, float g, float b, float a)
{
    std_msgs::msg::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
}

geometry_msgs::msg::Point point_from(const Eigen::Vector3f& p)
{
    geometry_msgs::msg::Point out;
    out.x = static_cast<double>(p.x());
    out.y = static_cast<double>(p.y());
    out.z = static_cast<double>(p.z());
    return out;
}
}  // namespace

LocalMappingPublisher::LocalMappingPublisher(const rclcpp::Node::SharedPtr& node, const Options& options)
    : node_(node), map_frame_id_(options.map_frame_id)
{
    if (options.publish_diagnostics)
    {
        diagnostics_pub_ =
            node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(options.diagnostics_topic, 10);
    }
    global_map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/orb_slam3/map", 10);
    lba_map_points_pub_ =
        node_->create_publisher<sensor_msgs::msg::PointCloud2>("/orb_slam3/local_mapping/map_points", 10);
    lba_map_point_outliers_pub_ =
        node_->create_publisher<sensor_msgs::msg::PointCloud2>("/orb_slam3/local_mapping/map_point_outliers", 10);
    keyframes_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/orb_slam3/local_mapping/keyframes", 10);
    covisibility_pub_ =
        node_->create_publisher<visualization_msgs::msg::Marker>("/orb_slam3/local_mapping/covisibility", 10);
    spanning_tree_pub_ =
        node_->create_publisher<visualization_msgs::msg::Marker>("/orb_slam3/local_mapping/spanning_tree", 10);
}

void LocalMappingPublisher::publish_batch(const std::vector<ORB_SLAM3::LocalMappingResult>& batch)
{
    if (batch.empty())
    {
        return;
    }

    if (diagnostics_pub_)
    {
        diagnostic_msgs::msg::DiagnosticArray array_msg;
        array_msg.header.stamp = node_->now();
        array_msg.status.reserve(batch.size());
        for (const ORB_SLAM3::LocalMappingResult& result : batch)
        {
            array_msg.status.push_back(make_diagnostic_status(result));
        }
        diagnostics_pub_->publish(array_msg);
    }

    for (const ORB_SLAM3::LocalMappingResult& result : batch)
    {
        update_state_from_result(result);
    }
    publish_visualization(node_->now());
}

void LocalMappingPublisher::reset()
{
    map_points_.clear();
    keyframe_poses_.clear();
    latest_lba_keyframes_.clear();
    lba_opt_kfs_.clear();
    lba_fixed_kfs_.clear();
    latest_covisibility_edges_.clear();
    latest_lba_map_points_.clear();
    latest_lba_outlier_points_.clear();
    spanning_tree_parents_.clear();
    root_keyframes_.clear();

    publish_visualization(node_->now());
}

diagnostic_msgs::msg::DiagnosticStatus LocalMappingPublisher::make_diagnostic_status(
    const ORB_SLAM3::LocalMappingResult& r) const
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

    return status;
}

void LocalMappingPublisher::update_state_from_result(const ORB_SLAM3::LocalMappingResult& result)
{
    keyframe_poses_[result.process_new_keyframe.keyframe_id] = result.process_new_keyframe.pose;

    for (const unsigned long kf_id : result.keyframe_culling.culled_keyframe_ids)
    {
        keyframe_poses_.erase(kf_id);
        spanning_tree_parents_.erase(kf_id);
        root_keyframes_.erase(kf_id);
    }

    for (const ORB_SLAM3::NewMappingMapPoint& mp : result.added_map_points)
    {
        map_points_[mp.id] = mp.pos_world;
    }

    for (const unsigned long mp_id : result.culled_map_point_ids)
    {
        map_points_.erase(mp_id);
    }

    if (!result.lba.skipped)
    {
        latest_lba_keyframes_.clear();
        lba_opt_kfs_.clear();
        lba_fixed_kfs_.clear();
        for (const unsigned long id : result.lba.optimised_keyframe_ids)
        {
            latest_lba_keyframes_.insert(id);
            lba_opt_kfs_.insert(id);
        }
        for (const unsigned long id : result.lba.fixed_keyframe_ids)
        {
            latest_lba_keyframes_.insert(id);
            lba_fixed_kfs_.insert(id);
        }

        latest_covisibility_edges_ = result.lba.covisibility_edges;
        for (const ORB_SLAM3::CovisibilityEdge& edge : latest_covisibility_edges_)
        {
            latest_lba_keyframes_.insert(edge.kf_id_a);
            latest_lba_keyframes_.insert(edge.kf_id_b);
        }

        latest_lba_outlier_points_.clear();
        latest_lba_outlier_points_.reserve(result.lba_outlier_map_point_ids.size());
        latest_lba_map_points_.clear();
        latest_lba_map_points_.reserve(result.lba.lba_map_points.size());
        std::unordered_map<unsigned long, Eigen::Vector3f> lba_map_point_by_id;
        lba_map_point_by_id.reserve(result.lba.lba_map_points.size());
        for (const ORB_SLAM3::LBAMapPoint& mp : result.lba.lba_map_points)
        {
            lba_map_point_by_id[mp.id] = mp.pos_world;
            latest_lba_map_points_.push_back(mp.pos_world);
        }

        for (const unsigned long mp_id : result.lba_outlier_map_point_ids)
        {
            const auto lba_it = lba_map_point_by_id.find(mp_id);
            if (lba_it != lba_map_point_by_id.end())
            {
                latest_lba_outlier_points_.push_back(lba_it->second);
            }
        }

        for (const ORB_SLAM3::SpanningTreeEdge& e : result.lba.spanning_tree_edges)
        {
            if (e.parent_kf_id == 0)
            {
                root_keyframes_.insert(e.child_kf_id);
                spanning_tree_parents_.erase(e.child_kf_id);
            }
            else
            {
                spanning_tree_parents_[e.child_kf_id] = e.parent_kf_id;
            }
        }
    }

    for (const unsigned long mp_id : result.lba_outlier_map_point_ids)
    {
        map_points_.erase(mp_id);
    }
}

sensor_msgs::msg::PointCloud2 LocalMappingPublisher::make_point_cloud(const std::vector<Eigen::Vector3f>& points,
                                                                       const rclcpp::Time& stamp) const
{
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = map_frame_id_;
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
    for (const Eigen::Vector3f& p : points)
    {
        float* fptr = reinterpret_cast<float*>(ptr);
        fptr[0] = p.x();
        fptr[1] = p.y();
        fptr[2] = p.z();
        ptr += cloud.point_step;
    }
    return cloud;
}

void LocalMappingPublisher::publish_visualization(const rclcpp::Time& stamp)
{
    std::vector<Eigen::Vector3f> global_points;
    global_points.reserve(map_points_.size());
    for (const auto& kv : map_points_)
    {
        global_points.push_back(kv.second);
    }
    global_map_pub_->publish(make_point_cloud(global_points, stamp));
    lba_map_points_pub_->publish(make_point_cloud(latest_lba_map_points_, stamp));
    lba_map_point_outliers_pub_->publish(make_point_cloud(latest_lba_outlier_points_, stamp));

    visualization_msgs::msg::Marker keyframes;
    keyframes.header.stamp = stamp;
    keyframes.header.frame_id = map_frame_id_;
    keyframes.ns = "keyframes";
    keyframes.id = 1;
    keyframes.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    keyframes.action = visualization_msgs::msg::Marker::ADD;
    keyframes.scale.x = 0.08;
    keyframes.scale.y = 0.08;
    keyframes.scale.z = 0.08;
    for (const unsigned long id : latest_lba_keyframes_)
    {
        const auto pose_it = keyframe_poses_.find(id);
        if (pose_it == keyframe_poses_.end())
        {
            continue;
        }
        const Eigen::Vector3f t = pose_it->second.translation();
        keyframes.points.push_back(point_from(t));
        if (lba_opt_kfs_.count(id) > 0U)
        {
            keyframes.colors.push_back(color(0.0F, 1.0F, 0.0F, 1.0F));
        }
        else if (root_keyframes_.count(id) > 0U || lba_fixed_kfs_.count(id) > 0U)
        {
            keyframes.colors.push_back(color(1.0F, 0.0F, 0.0F, 1.0F));
        }
        else
        {
            keyframes.colors.push_back(color(0.0F, 0.0F, 1.0F, 1.0F));
        }
    }
    keyframes_pub_->publish(keyframes);

    auto make_covisibility_marker = [&](int id,
                                        const std_msgs::msg::ColorRGBA& marker_color) -> visualization_msgs::msg::Marker
    {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = stamp;
        marker.header.frame_id = map_frame_id_;
        marker.ns = "covisibility";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.01;
        marker.color = marker_color;
        return marker;
    };

    visualization_msgs::msg::Marker covisibility_low =
        make_covisibility_marker(20, color(1.0F, 0.0F, 0.0F, 0.3F));  // < 30 shared map points
    visualization_msgs::msg::Marker covisibility_mid =
        make_covisibility_marker(21, color(1.0F, 1.0F, 0.0F, 0.3F));  // [30, 100] shared map points
    visualization_msgs::msg::Marker covisibility_high =
        make_covisibility_marker(22, color(0.0F, 1.0F, 0.0F, 1.0F));  // > 100 shared map points
    covisibility_high.scale.x = 0.02;
    for (const ORB_SLAM3::CovisibilityEdge& edge : latest_covisibility_edges_)
    {
        const unsigned long a = edge.kf_id_a;
        const unsigned long b = edge.kf_id_b;
        const auto it_a = keyframe_poses_.find(a);
        const auto it_b = keyframe_poses_.find(b);
        if (it_a == keyframe_poses_.end() || it_b == keyframe_poses_.end())
        {
            continue;
        }
        visualization_msgs::msg::Marker* target_marker = &covisibility_high;
        if (edge.weight < 30)
        {
            target_marker = &covisibility_low;
        }
        else if (edge.weight <= 100)
        {
            target_marker = &covisibility_mid;
        }

        target_marker->points.push_back(point_from(it_a->second.translation()));
        target_marker->points.push_back(point_from(it_b->second.translation()));
    }
    covisibility_pub_->publish(covisibility_low);
    covisibility_pub_->publish(covisibility_mid);
    covisibility_pub_->publish(covisibility_high);

    visualization_msgs::msg::Marker spanning_tree;
    spanning_tree.header.stamp = stamp;
    spanning_tree.header.frame_id = map_frame_id_;
    spanning_tree.ns = "spanning_tree";
    spanning_tree.id = 3;
    spanning_tree.type = visualization_msgs::msg::Marker::LINE_LIST;
    spanning_tree.action = visualization_msgs::msg::Marker::ADD;
    spanning_tree.scale.x = 0.015;
    spanning_tree.color = color(0.0F, 0.7F, 0.2F, 0.8F);
    for (const auto& kv : spanning_tree_parents_)
    {
        const unsigned long child = kv.first;
        const unsigned long parent = kv.second;
        const auto it_c = keyframe_poses_.find(child);
        const auto it_p = keyframe_poses_.find(parent);
        if (it_c == keyframe_poses_.end() || it_p == keyframe_poses_.end())
        {
            continue;
        }
        spanning_tree.points.push_back(point_from(it_c->second.translation()));
        spanning_tree.points.push_back(point_from(it_p->second.translation()));
    }
    spanning_tree_pub_->publish(spanning_tree);
}

}  // namespace visual_odometry
