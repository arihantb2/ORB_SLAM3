#pragma once

#include <boost/program_options.hpp>
#include <string>

namespace visual_odometry
{
namespace cli
{
struct CommonOptions
{
    std::string lcm_log = "";
    std::string platform = "";
    std::string output_dir = "";
    bool debug_video = false;
    bool monocular = false;
    std::string platform_config = "";
    bool use_priors = false;
    std::string image_name_filter = "";
    bool apply_clahe = false;
    /// static_tf child frame ids for stereo: T_c1_c2 = lookup(left, right). Defaults match Seeker (AC=left=cam_aft, FC=right=cam_fwd).
    std::string left_camera_frame_id = "cam_aft";
    std::string right_camera_frame_id = "cam_fwd";
    std::string dvl_frame_id = "dvl";
};

inline void add_common_options(boost::program_options::options_description& desc)
{
    // clang-format off
    desc.add_options()
        ("help,h", "Produce help message")
        ("lcm-log,l", boost::program_options::value<std::string>()->required(), "Path to LCM log file")
        ("platform,p", boost::program_options::value<std::string>()->required(), "Platform name, eg. CHERYL, DURHAM, ENGLAND, ...")
        ("platform-config,c", boost::program_options::value<std::string>()->required(), "Path to platform configuration YAML file")
        ("output-dir,o", boost::program_options::value<std::string>()->default_value(""), "Directory to write trajectory logs")
        ("debug-video,d", boost::program_options::bool_switch()->default_value(false), "Disable debug display video (written when output-dir is set)")
        ("monocular,m", boost::program_options::bool_switch()->default_value(false), "Use monocular or stereo VO")
        ("use-priors,u", boost::program_options::bool_switch()->default_value(false), "Use priors for tracking, only used for monocular mode")
        ("image-name-filter,f", boost::program_options::value<std::string>()->default_value(""), "Filter image names by substring for monocular mode")
        ("apply-clahe,a", boost::program_options::bool_switch()->default_value(false), "Apply CLAHE to the grayscale image")
        ("left-camera-frame-id",
         boost::program_options::value<std::string>()->default_value("cam_aft"),
         "static_tf frame id for stereo left camera (Camera1 / T_c1_c2 left)")
        ("right-camera-frame-id",
         boost::program_options::value<std::string>()->default_value("cam_fwd"),
         "static_tf frame id for stereo right camera (Camera2 / T_c1_c2 right)")
        ("dvl-frame-id",
         boost::program_options::value<std::string>()->default_value("dvl"),
         "static_tf frame id for DVL");
    // clang-format on
}

inline CommonOptions get_common_options(const boost::program_options::variables_map& vm)
{
    CommonOptions opts;
    opts.lcm_log = vm["lcm-log"].as<std::string>();
    opts.platform = vm["platform"].as<std::string>();
    opts.output_dir = vm["output-dir"].as<std::string>();
    opts.debug_video = vm["debug-video"].as<bool>();
    opts.monocular = vm["monocular"].as<bool>();
    opts.platform_config = vm["platform-config"].as<std::string>();
    opts.use_priors = vm["use-priors"].as<bool>();
    opts.image_name_filter = vm["image-name-filter"].as<std::string>();
    opts.apply_clahe = vm["apply-clahe"].as<bool>();
    opts.left_camera_frame_id = vm["left-camera-frame-id"].as<std::string>();
    opts.right_camera_frame_id = vm["right-camera-frame-id"].as<std::string>();
    opts.dvl_frame_id = vm["dvl-frame-id"].as<std::string>();
    return opts;
}
}  // namespace cli
}  // namespace visual_odometry
