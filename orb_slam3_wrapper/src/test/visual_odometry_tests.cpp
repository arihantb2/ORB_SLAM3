#include <vo/dispatch_types.h>
#include <vo/visual_odometry.h>

#include <static_tf/static_tf_tree.hpp>

#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>

namespace visual_odometry
{

class VisualOdometryTestHelper
{
public:
    static size_t pending_mono_size(const VisualOdometry& vo)
    {
        return vo.dispatch_sync_ ? vo.dispatch_sync_->pending_mono_size() : 0;
    }

    static size_t pending_stereo_size(const VisualOdometry& vo)
    {
        return vo.dispatch_sync_ ? vo.dispatch_sync_->pending_stereo_size() : 0;
    }
};

class TestVisualOdometry : public VisualOdometry
{
public:
    using VisualOdometry::VisualOdometry;

    int mono_call_count = 0;
    int stereo_call_count = 0;

    VOResult process_mono_image_impl(const cv::Mat& /*image*/, const DispatchContext& /*context*/,
                                     double /*timestamp*/) override
    {
        VOResult result;
        result.tracking_ok = true;
        result.pose_matrix = Eigen::Matrix4f::Identity();
        ++mono_call_count;
        return result;
    }

    VOResult process_stereo_image_impl(const cv::Mat& /*left_image*/, const cv::Mat& /*right_image*/,
                                       const DispatchContext& /*context*/, double /*timestamp*/) override
    {
        VOResult result;
        result.tracking_ok = true;
        result.pose_matrix = Eigen::Matrix4f::Identity();
        ++stereo_call_count;
        return result;
    }
};

}  // namespace visual_odometry

using visual_odometry::TestVisualOdometry;
using visual_odometry::VisualOdometryTestHelper;

static acfrlcm::auv_acfr_nav_t make_nav(double t_sec, float x)
{
    acfrlcm::auv_acfr_nav_t nav{};
    nav.utime = static_cast<int64_t>(t_sec * 1e6);
    nav.x = x;
    nav.y = 0.0f;
    nav.depth = 0.0f;
    nav.roll = 0.0f;
    nav.pitch = 0.0f;
    nav.heading = 0.0f;
    return nav;
}

static acfrlcm::auv_vis_rawlog_t make_rawlog(double t_sec, const std::string& name)
{
    acfrlcm::auv_vis_rawlog_t raw{};
    raw.utime = static_cast<int64_t>(t_sec * 1e6);
    raw.image_name = name;
    return raw;
}

static void test_image_dispatched_only_when_sandwiched()
{
    TestVisualOdometry vo;

    cv::Mat dummy_image(10, 10, CV_8UC3, cv::Scalar(0, 0, 0));

    const double t_image = 2.0;
    const double t_nav_before = 1.0;
    const double t_nav_after = 3.0;

    auto raw = make_rawlog(t_image, "image_sandwiched");

    vo.handle_monocular_image(dummy_image, raw);
    assert(vo.mono_call_count == 0);
    assert(VisualOdometryTestHelper::pending_mono_size(vo) == 1);

    vo.handle_nav_message(make_nav(t_nav_before, 0.0f));
    assert(vo.mono_call_count == 0);
    assert(VisualOdometryTestHelper::pending_mono_size(vo) == 1);

    vo.handle_nav_message(make_nav(t_nav_after, 0.0f));

    assert(vo.mono_call_count == 1);
    assert(VisualOdometryTestHelper::pending_mono_size(vo) == 0);
}

static void test_images_before_first_nav_are_not_dispatched()
{
    TestVisualOdometry vo;

    cv::Mat dummy_image(10, 10, CV_8UC3, cv::Scalar(0, 0, 0));

    vo.handle_nav_message(make_nav(1.0, 0.0f));
    vo.handle_nav_message(make_nav(3.0, 0.0f));

    auto raw = make_rawlog(0.5, "image_before_nav");
    vo.handle_monocular_image(dummy_image, raw);

    assert(vo.mono_call_count == 0);
    assert(VisualOdometryTestHelper::pending_mono_size(vo) == 0);
}

static void test_acfr_nav_to_eigen_matrix_zero_angles_is_identity_rotation()
{
    // Regression test: acfr_nav_to_eigen_matrix() used to call
    // .normalized() on the rotation block, which divides by the matrix's
    // Frobenius norm (~sqrt(3) for a rotation matrix) rather than
    // orthonormalizing it. At zero roll/pitch/heading the "rotation" should
    // be exactly identity; under the bug it was scaled to ~1/sqrt(3) instead.
    acfrlcm::auv_acfr_nav_t nav{};
    nav.x = 1.0f;
    nav.y = 2.0f;
    nav.depth = 3.0f;
    nav.roll = 0.0f;
    nav.pitch = 0.0f;
    nav.heading = 0.0f;

    const Eigen::Matrix4f m = visual_odometry::acfr_nav_to_eigen_matrix(nav);
    const Eigen::Matrix3f R = m.block<3, 3>(0, 0);

    const float err = (R - Eigen::Matrix3f::Identity()).norm();
    assert(err < 1e-6f);

    assert(std::abs(m(0, 3) - 1.0f) < 1e-6f);
    assert(std::abs(m(1, 3) - 2.0f) < 1e-6f);
    assert(std::abs(m(2, 3) - 3.0f) < 1e-6f);
}

static void test_acfr_nav_to_eigen_matrix_produces_orthonormal_rotation()
{
    // The bug scaled every rotation matrix uniformly by 1/sqrt(3), so
    // R^T*R == I/3 instead of I for any angle combination, not just zero
    // angles -- this is the general-case regression guard for non-trivial
    // rotations.
    acfrlcm::auv_acfr_nav_t nav{};
    nav.roll = 0.3f;
    nav.pitch = -0.2f;
    nav.heading = static_cast<float>(M_PI) / 2.0f;

    const Eigen::Matrix4f m = visual_odometry::acfr_nav_to_eigen_matrix(nav);
    const Eigen::Matrix3f R = m.block<3, 3>(0, 0);

    const float orthonormalityErr = (R.transpose() * R - Eigen::Matrix3f::Identity()).norm();
    assert(orthonormalityErr < 1e-5f);

    const float detErr = std::abs(R.determinant() - 1.0f);
    assert(detErr < 1e-5f);

    // Cross-check against the equivalent double-precision Eigen construction.
    const auto roll_angle = Eigen::AngleAxisf(nav.roll, Eigen::Vector3f::UnitX());
    const auto pitch_angle = Eigen::AngleAxisf(nav.pitch, Eigen::Vector3f::UnitY());
    const auto heading_angle = Eigen::AngleAxisf(nav.heading, Eigen::Vector3f::UnitZ());
    const Eigen::Matrix3f expected = (heading_angle * pitch_angle * roll_angle).toRotationMatrix();
    const float err = (R - expected).norm();
    assert(err < 1e-6f);
}

static std::string write_temp_nav_csv()
{
    const std::string path = "/tmp/vo_nav_csv_tests.csv";
    std::ofstream out(path);
    assert(out.is_open());
    out << "timestamp,tx,ty,tz,qw,qx,qy,qz\n";
    out << "0,0,0,0,1,0,0,0\n";
    out << "10,10,0,0,1,0,0,0\n";
    out.close();
    return path;
}

static void test_nav_csv_dispatches_without_nav_messages()
{
    visual_odometry::cli::CommonOptions opts;
    opts.pose_prior_csv_path = write_temp_nav_csv();

    TestVisualOdometry vo(static_tf::StaticTfTree(), opts);

    cv::Mat dummy_image(10, 10, CV_8UC3, cv::Scalar(0, 0, 0));
    auto raw = make_rawlog(5.0, "image_with_csv_nav");
    vo.handle_monocular_image(dummy_image, raw);

    assert(vo.mono_call_count == 1);
    assert(VisualOdometryTestHelper::pending_mono_size(vo) == 0);
}

int main()
{
    test_acfr_nav_to_eigen_matrix_zero_angles_is_identity_rotation();
    test_acfr_nav_to_eigen_matrix_produces_orthonormal_rotation();
    test_image_dispatched_only_when_sandwiched();
    test_images_before_first_nav_are_not_dispatched();
    test_nav_csv_dispatches_without_nav_messages();

    std::cout << "All VisualOdometry tests passed." << std::endl;
    return 0;
}
