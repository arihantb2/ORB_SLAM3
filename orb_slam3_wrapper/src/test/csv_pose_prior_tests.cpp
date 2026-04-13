#include <vo/csv_pose_prior.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
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

int main()
{
    test_interpolation_midpoint();
    test_out_of_range_returns_false();
    std::cout << "All CsvPosePrior tests passed." << std::endl;
    return 0;
}

