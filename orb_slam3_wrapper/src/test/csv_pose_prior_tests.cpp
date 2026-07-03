#include <vo/csv_pose_prior.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

using visual_odometry::CsvPosePrior;

static std::string write_temp_csv()
{
    const std::string path = "/tmp/csv_pose_prior_tests.csv";
    std::ofstream out(path);
    assert(out.is_open());

    // timestamp,tx,ty,tz,qw,qx,qy,qz
    out << "timestamp,tx,ty,tz,qw,qx,qy,qz\n";

    // t=0, pose at origin with identity rotation
    out << "0,0,0,0,1,0,0,0\n";

    // t=10, translate 10m in x and rotate 180deg about Z
    const Eigen::Quaterniond q1(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
    out << "10,10,0,0," << q1.w() << "," << q1.x() << "," << q1.y() << "," << q1.z() << "\n";
    out.close();
    return path;
}

static void test_interpolation_midpoint()
{
    const std::string path = write_temp_csv();
    CsvPosePrior prior(path);

    Eigen::Matrix4f world_T_dvl;
    const bool ok = prior.try_get_interpolated(5.0, world_T_dvl);
    assert(ok);

    const Eigen::Vector3f t = world_T_dvl.block<3, 1>(0, 3);
    assert(std::abs(t.x() - 5.0f) < 1e-6f);
    assert(std::abs(t.y() - 0.0f) < 1e-6f);
    assert(std::abs(t.z() - 0.0f) < 1e-6f);

    const Eigen::Matrix3f R = world_T_dvl.block<3, 3>(0, 0);
    const Eigen::Matrix3d R_expected = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const double err = (R.cast<double>() - R_expected).norm();
    assert(err < 1e-6);
}

static void test_out_of_range_returns_false()
{
    const std::string path = write_temp_csv();
    CsvPosePrior prior(path);

    Eigen::Matrix4f world_T_dvl;
    assert(!prior.try_get_interpolated(-1.0, world_T_dvl));
    assert(!prior.try_get_interpolated(11.0, world_T_dvl));
}

static void test_interpolation_at_first_sample_timestamp()
{
    // Regression test: try_get_interpolated() used to fail when queried
    // exactly at timestamps_.front() because lower_bound() lands on begin(),
    // which the "before the range" guard rejected even though the sample
    // exists there.
    const std::string path = write_temp_csv();
    CsvPosePrior prior(path);

    Eigen::Matrix4f world_T_dvl;
    const bool ok = prior.try_get_interpolated(0.0, world_T_dvl);
    assert(ok && "query exactly at the first sample's timestamp should succeed");

    const Eigen::Vector3f t = world_T_dvl.block<3, 1>(0, 3);
    assert(std::abs(t.x() - 0.0f) < 1e-6f);
    assert(std::abs(t.y() - 0.0f) < 1e-6f);
    assert(std::abs(t.z() - 0.0f) < 1e-6f);
}

static void test_trailing_comma_reports_correct_field()
{
    // Regression test: the CSV field splitter used to silently drop a
    // trailing empty field (std::getline(stream, field, ',') in a loop does
    // not yield one final empty string), so a row with a blank final field
    // like "0,0,0,0,1,0,0," was miscounted as 7 fields instead of 8 and
    // rejected with a "wrong number of fields" error that didn't point at
    // the actual problem. It must now be counted correctly as 8 fields and
    // fail while parsing the empty qz field, naming that field explicitly.
    const std::string path = "/tmp/csv_pose_prior_trailing_comma_tests.csv";
    std::ofstream out(path);
    assert(out.is_open());
    out << "timestamp,tx,ty,tz,qw,qx,qy,qz\n";
    out << "0,0,0,0,1,0,0,\n";  // qz field left blank
    out.close();

    bool threw = false;
    std::string message;
    try
    {
        CsvPosePrior prior(path);
    }
    catch (const std::runtime_error& e)
    {
        threw = true;
        message = e.what();
    }
    assert(threw);
    assert(message.find("qz") != std::string::npos);
}

int main()
{
    test_interpolation_midpoint();
    test_out_of_range_returns_false();
    test_interpolation_at_first_sample_timestamp();
    test_trailing_comma_reports_correct_field();
    std::cout << "All CsvPosePrior tests passed." << std::endl;
    return 0;
}

