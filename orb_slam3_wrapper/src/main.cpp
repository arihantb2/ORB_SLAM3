#include "local_mapping_visualization_publisher.h"
#include "orb_slam3_wrapper.h"
#include "tracking_ros_publisher.h"

#include <lcm_log_player/lcm_session.h>
#include <lcm_log_player/lcm_replay_cli.hpp>

#include <boost/program_options.hpp>
#include <iostream>
#include <thread>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vo/common_options.h>
#include <acfrlcm/auv_vis_rawlog_t.hpp>

#include <static_tf/sensor_frame_loader.hpp>
#include <static_tf/static_tf_tree.hpp>

#include <rclcpp/rclcpp.hpp>

namespace po = boost::program_options;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    po::options_description desc("Allowed options");
    visual_odometry::cli::add_common_options(desc);
    lcm_log_player::cli::add_replay_program_options(desc);
    // clang-format off
    desc.add_options()
        ("vocab-file", po::value<std::string>()->required(), "Path to vocabulary file")
        ("camera-calib-file", po::value<std::string>()->required(), "Path to camera calibration YAML")
        ("config-file", po::value<std::string>()->required(), "Path to algorithm config YAML (tracking, ORB, etc.)")
        ("verbose", po::bool_switch()->default_value(false), "Enable ORB-SLAM3 verbose output on console")
        ("synchronous-local-mapping", po::bool_switch()->default_value(false), "Run local mapping in the same thread as tracking for deterministic behavior (useful for debugging, will reduce tracking FPS)");
    // clang-format on

    po::variables_map vm;
    try
    {
        po::store(po::parse_command_line(argc, argv, desc), vm);

        if (vm.count("help"))
        {
            std::cout << desc << std::endl;
            rclcpp::shutdown();
            return 0;
        }

        po::notify(vm);
    }
    catch (const po::error& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cerr << desc << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    const std::string vocab_file = vm["vocab-file"].as<std::string>();
    const std::string camera_calib_file = vm["camera-calib-file"].as<std::string>();
    const std::string config_file = vm["config-file"].as<std::string>();
    const bool verbose = vm["verbose"].as<bool>();
    const bool synchronous_local_mapping = vm["synchronous-local-mapping"].as<bool>();
    const visual_odometry::cli::CommonOptions common_options = visual_odometry::cli::get_common_options(vm);
    const lcm_log_player::PlaybackOptions playback_options =
        lcm_log_player::cli::playback_options_from_variables_map(vm);

    std::string image_name_filter = common_options.image_name_filter;
    if (!common_options.monocular)
    {
        image_name_filter = "";
    }

    static_tf::StaticTfTree platform_tree = static_tf::load_from_yaml(common_options.platform_config);

    auto node = std::make_shared<rclcpp::Node>("orb_slam3_vo");

    visual_odometry::TrackingRosPublisher::Options tracking_options;
    visual_odometry::LocalMappingPublisher::Options local_mapping_options;
    local_mapping_options.map_frame_id = tracking_options.map_frame_id;
    auto tracking_pub = std::make_shared<visual_odometry::TrackingRosPublisher>(node, tracking_options);
    auto local_mapping_pub = std::make_shared<visual_odometry::LocalMappingPublisher>(node, local_mapping_options);

    visual_odometry::ORBSLAM3Wrapper visual_odometry(platform_tree, vocab_file, camera_calib_file, config_file, verbose,
                                                     synchronous_local_mapping, common_options);
    visual_odometry.set_publishers(tracking_pub, local_mapping_pub);

    const bool enable_gui = lcm_log_player::cli::enable_gui_from_variables_map(vm);
    auto lcm_session = lcm_log_player::LCMSession::from_log(common_options.lcm_log, playback_options,
                                                            /*enable_console_controls=*/true, enable_gui);

    if (common_options.monocular)
    {
        lcm_session.register_image_callback(
            visual_odometry::get_camera_channel(common_options.platform),
            [&visual_odometry](const cv::Mat& img, const acfrlcm::auv_vis_rawlog_t& raw_log)
            { visual_odometry.handle_monocular_image(img, raw_log); }, image_name_filter);
    }
    else
    {
        lcm_session.register_stereo_image_callback(
            visual_odometry::get_camera_channel(common_options.platform),
            [&visual_odometry](const cv::Mat& left_img, const cv::Mat& right_img,
                               const acfrlcm::auv_vis_rawlog_t& raw_log_left,
                               const acfrlcm::auv_vis_rawlog_t& raw_log_right)
            { visual_odometry.handle_stereo_image(left_img, right_img, raw_log_left, raw_log_right); });
    }

    lcm_session.register_callback<acfrlcm::auv_acfr_nav_t>(
        visual_odometry::get_nav_channel(common_options.platform),
        [&visual_odometry](const acfrlcm::auv_acfr_nav_t& msg) -> void { visual_odometry.handle_nav_message(msg); });

    rclcpp::on_shutdown(
        [&lcm_session]()
        {
            // Ensure Ctrl+C (ROS shutdown) interrupts blocking replay loop.
            lcm_session.stop();
        });

    std::thread ros_thread([&node]() { rclcpp::spin(node); });
    std::thread replay_thread([&lcm_session]() { lcm_session.run(); });

    if (replay_thread.joinable())
    {
        replay_thread.join();
    }

    if (!rclcpp::ok())
    {
        lcm_session.stop();
    }
    rclcpp::shutdown();
    if (ros_thread.joinable())
    {
        ros_thread.join();
    }

    return 0;
}
