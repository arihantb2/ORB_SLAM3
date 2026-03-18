/**
 * Unit tests for custom GTSAM factor types (ORB_SLAM3).
 * Verifies error at consistent state and analytical vs numerical Jacobians.
 */

#include "CameraModels/Pinhole.h"
#include "GTSAMTypes.h"

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/Values.h>

#include <gtest/gtest.h>
#include <cmath>
#include <functional>
#include <vector>

using namespace ORB_SLAM3;

namespace
{

const double kTolError = 1e-6;
const double kTolJacobian = 1e-4;

void ExpectMatrixNear(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, double tol, const char* label)
{
    ASSERT_EQ(A.rows(), B.rows()) << label << " row mismatch";
    ASSERT_EQ(A.cols(), B.cols()) << label << " col mismatch";
    const double diff = (A - B).norm();
    EXPECT_NEAR(0.0, diff, tol) << label << " matrix diff norm " << diff << " > tol " << tol;
}

// -----------------------------------------------------------------------------
// FourDOFBetweenFactor
// -----------------------------------------------------------------------------
TEST(GTSAMFactors, FourDOFBetweenFactor)
{
    const gtsam::Key keyI = fourDofKey(0);
    const gtsam::Key keyJ = fourDofKey(1);
    const Eigen::Matrix3d Rcwi = Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                                 Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitY()).toRotationMatrix();
    const Eigen::Vector3d tcwi(1.0, 0.2, 0.5);
    const gtsam::Rot3 rotI(Rcwi);
    const gtsam::Point3 transI(tcwi(0), tcwi(1), tcwi(2));
    const gtsam::Pose3 poseI(rotI, transI);

    const Eigen::Matrix3d Rcwj = Eigen::AngleAxisd(0.15, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                                 Eigen::AngleAxisd(-0.02, Eigen::Vector3d::UnitY()).toRotationMatrix();
    const Eigen::Vector3d tcwj(1.3, 0.1, 0.6);
    const gtsam::Rot3 rotJ(Rcwj);
    const gtsam::Point3 transJ(tcwj(0), tcwj(1), tcwj(2));
    const gtsam::Pose3 poseJ(rotJ, transJ);

    const Eigen::Matrix3d dRij = Rcwi * Rcwj.transpose();
    const Eigen::Vector3d twcj = -Rcwj.transpose() * tcwj;
    const Eigen::Vector3d dtij = Rcwi * twcj + tcwi;

    gtsam::SharedNoiseModel noise = gtsam::noiseModel::Unit::Create(6);
    FourDOFBetweenFactor factor(keyI, keyJ, dRij, dtij, noise);

    gtsam::Vector err = factor.evaluateError(poseI, poseJ, boost::none, boost::none);
    EXPECT_NEAR(0.0, err.norm(), kTolError) << "FourDOFBetweenFactor error at consistent state";

    gtsam::Pose3 poseJ_pert = poseJ.retract((gtsam::Vector(6) << 1e-4, 0.0, 0.0, 0.0, 0.0, 0.0).finished());
    gtsam::Vector err_pert = factor.evaluateError(poseI, poseJ_pert, boost::none, boost::none);
    EXPECT_GT(err_pert.norm(), kTolError) << "FourDOFBetweenFactor non-zero error check";

    Eigen::MatrixXd H1_analytical(6, 6), H2_analytical(6, 6);
    (void)factor.evaluateError(poseI, poseJ, H1_analytical, H2_analytical);

    std::function<gtsam::Vector(const gtsam::Pose3&, const gtsam::Pose3&)> err_fn =
        [&factor](const gtsam::Pose3& a, const gtsam::Pose3& b) -> gtsam::Vector
    {
        return factor.evaluateError(a, b, boost::none, boost::none);
    };
    Eigen::MatrixXd H1_num =
        gtsam::numericalDerivative21<gtsam::Vector, gtsam::Pose3, gtsam::Pose3>(err_fn, poseI, poseJ);
    Eigen::MatrixXd H2_num =
        gtsam::numericalDerivative22<gtsam::Vector, gtsam::Pose3, gtsam::Pose3>(err_fn, poseI, poseJ);

    ExpectMatrixNear(H1_analytical, H1_num, kTolJacobian, "FourDOFBetweenFactor H1");
    ExpectMatrixNear(H2_analytical, H2_num, kTolJacobian, "FourDOFBetweenFactor H2");
}

// -----------------------------------------------------------------------------
// MonoOnlyPoseFactor
// -----------------------------------------------------------------------------
TEST(GTSAMFactors, MonoOnlyPoseFactor)
{
    const std::vector<float> params = {500.f, 500.f, 320.f, 240.f};
    Pinhole camera(params);
    const gtsam::Pose3 Tbc = gtsam::Pose3::Identity();
    const Eigen::Vector3d Xw(2.0, 0.5, 5.0);
    const gtsam::Pose3 Twb(gtsam::Rot3::Identity(), gtsam::Point3(0, 0, 0));
    const gtsam::Pose3 Twc = Twb.compose(Tbc);
    const gtsam::Pose3 Tcw = Twc.inverse();
    const Eigen::Vector3d Xc = Tcw.transformFrom(gtsam::Point3(Xw));
    const Eigen::Vector2d obs = camera.project(Xc);

    gtsam::SharedNoiseModel noise = gtsam::noiseModel::Unit::Create(2);
    MonoOnlyPoseFactor factor(poseKey(0), Xw, obs, noise, &camera, Tbc);

    gtsam::Vector err = factor.evaluateError(Twb, boost::none);
    EXPECT_NEAR(0.0, err.norm(), kTolError) << "MonoOnlyPoseFactor error at consistent state";

    gtsam::Pose3 Twb_pert = Twb.retract((gtsam::Vector(6) << 0.0, 0.0, 0.0, 1e-4, 0.0, 0.0).finished());
    gtsam::Vector err_pert = factor.evaluateError(Twb_pert, boost::none);
    EXPECT_GT(err_pert.norm(), kTolError) << "MonoOnlyPoseFactor non-zero error check";

    Eigen::MatrixXd H_analytical(2, 6);
    factor.evaluateError(Twb, H_analytical);

    auto err_fn = [&factor](const gtsam::Pose3& Twb_) -> gtsam::Vector
    {
        return factor.evaluateError(Twb_, boost::none);
    };
    Eigen::MatrixXd H_num = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(err_fn, Twb);
    ExpectMatrixNear(H_analytical, H_num, kTolJacobian, "MonoOnlyPoseFactor Jacobian");
}

// -----------------------------------------------------------------------------
// StereoOnlyPoseFactor
// -----------------------------------------------------------------------------
TEST(GTSAMFactors, StereoOnlyPoseFactor)
{
    const std::vector<float> params = {500.f, 500.f, 320.f, 240.f};
    Pinhole camera(params);
    const double bf = 100.0;
    const gtsam::Pose3 Tbc = gtsam::Pose3::Identity();
    const Eigen::Vector3d Xw(2.0, 0.5, 5.0);
    const gtsam::Pose3 Twb(gtsam::Rot3::Identity(), gtsam::Point3(0, 0, 0));
    const gtsam::Pose3 Tcw = Twb.compose(Tbc).inverse();
    const Eigen::Vector3d Xc = Tcw.transformFrom(gtsam::Point3(Xw));
    const Eigen::Vector2d proj2 = camera.project(Xc);
    Eigen::Vector3d obs3;
    obs3 << proj2(0), proj2(1), proj2(0) - bf / Xc(2);

    gtsam::SharedNoiseModel noise = gtsam::noiseModel::Unit::Create(3);
    StereoOnlyPoseFactor factor(poseKey(0), Xw, obs3, bf, noise, &camera, Tbc);

    gtsam::Vector err = factor.evaluateError(Twb, boost::none);
    EXPECT_NEAR(0.0, err.norm(), kTolError) << "StereoOnlyPoseFactor error at consistent state";

    gtsam::Pose3 Twb_pert = Twb.retract((gtsam::Vector(6) << 0.0, 0.0, 0.0, 1e-4, 0.0, 0.0).finished());
    gtsam::Vector err_pert = factor.evaluateError(Twb_pert, boost::none);
    EXPECT_GT(err_pert.norm(), kTolError) << "StereoOnlyPoseFactor non-zero error check";

    Eigen::MatrixXd H_analytical(3, 6);
    factor.evaluateError(Twb, H_analytical);

    auto err_fn = [&factor](const gtsam::Pose3& Twb_) -> gtsam::Vector
    {
        return factor.evaluateError(Twb_, boost::none);
    };
    Eigen::MatrixXd H_num = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(err_fn, Twb);
    ExpectMatrixNear(H_analytical, H_num, kTolJacobian, "StereoOnlyPoseFactor Jacobian");
}

// -----------------------------------------------------------------------------
// PinholeMonoPoseTcwFactor
// -----------------------------------------------------------------------------
TEST(GTSAMFactors, PinholeMonoPoseTcwFactor)
{
    const std::vector<float> params = {500.f, 500.f, 320.f, 240.f};
    Pinhole camera(params);

    const Eigen::Vector3d Xw(2.0, -0.3, 4.5);
    const gtsam::Pose3 Tcw(gtsam::Rot3::RzRyRx(0.05, -0.02, 0.01), gtsam::Point3(0.1, -0.05, 0.2));

    const gtsam::Point3 Xw_p(Xw.x(), Xw.y(), Xw.z());
    const gtsam::Point3 Xc_p = Tcw.transformFrom(Xw_p);
    const Eigen::Vector3d Xc(Xc_p.x(), Xc_p.y(), Xc_p.z());
    const Eigen::Vector2d obs = camera.project(Xc);

    gtsam::SharedNoiseModel noise = gtsam::noiseModel::Unit::Create(2);
    PinholeMonoPoseTcwFactor factor(poseKey(0), Xw, obs, noise, &camera);

    gtsam::Vector err = factor.evaluateError(Tcw, boost::none);
    EXPECT_NEAR(0.0, err.norm(), kTolError) << "PinholeMonoPoseTcwFactor error at consistent state";

    gtsam::Pose3 Tcw_pert = Tcw.retract((gtsam::Vector(6) << 0.0, 0.0, 0.0, 1e-4, 0.0, 0.0).finished());
    gtsam::Vector err_pert = factor.evaluateError(Tcw_pert, boost::none);
    EXPECT_GT(err_pert.norm(), kTolError) << "PinholeMonoPoseTcwFactor non-zero error check";

    gtsam::Matrix H_analytical(2, 6);
    factor.evaluateError(Tcw, H_analytical);

    auto err_fn = [&factor](const gtsam::Pose3& Tcw_) -> gtsam::Vector
    {
        return factor.evaluateError(Tcw_, boost::none);
    };
    Eigen::MatrixXd H_num = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(err_fn, Tcw);
    ExpectMatrixNear(H_analytical, H_num, kTolJacobian, "PinholeMonoPoseTcwFactor Jacobian");
}

// -----------------------------------------------------------------------------
// PinholeStereoPoseTcwFactor
// -----------------------------------------------------------------------------
TEST(GTSAMFactors, PinholeStereoPoseTcwFactor)
{
    const std::vector<float> params = {500.f, 500.f, 320.f, 240.f};
    Pinhole camera(params);
    const double bf = 100.0;

    const Eigen::Vector3d Xw(1.5, 0.4, 3.5);
    const gtsam::Pose3 Tcw(gtsam::Rot3::RzRyRx(-0.03, 0.01, 0.02), gtsam::Point3(-0.05, 0.02, 0.15));

    const gtsam::Point3 Xw_p(Xw.x(), Xw.y(), Xw.z());
    const gtsam::Point3 Xc_p = Tcw.transformFrom(Xw_p);
    const Eigen::Vector3d Xc(Xc_p.x(), Xc_p.y(), Xc_p.z());

    const Eigen::Vector2d proj2 = camera.project(Xc);
    Eigen::Vector3d obs3;
    obs3 << proj2(0), proj2(1), proj2(0) - bf / Xc.z();

    gtsam::SharedNoiseModel noise = gtsam::noiseModel::Unit::Create(3);
    PinholeStereoPoseTcwFactor factor(poseKey(0), Xw, obs3, bf, noise, &camera);

    gtsam::Vector err = factor.evaluateError(Tcw, boost::none);
    EXPECT_NEAR(0.0, err.norm(), kTolError) << "PinholeStereoPoseTcwFactor error at consistent state";

    gtsam::Pose3 Tcw_pert = Tcw.retract((gtsam::Vector(6) << 0.0, 0.0, 0.0, 1e-4, 0.0, 0.0).finished());
    gtsam::Vector err_pert = factor.evaluateError(Tcw_pert, boost::none);
    EXPECT_GT(err_pert.norm(), kTolError) << "PinholeStereoPoseTcwFactor non-zero error check";

    gtsam::Matrix H_analytical(3, 6);
    factor.evaluateError(Tcw, H_analytical);

    auto err_fn = [&factor](const gtsam::Pose3& Tcw_) -> gtsam::Vector
    {
        return factor.evaluateError(Tcw_, boost::none);
    };
    Eigen::MatrixXd H_num = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(err_fn, Tcw);
    ExpectMatrixNear(H_analytical, H_num, kTolJacobian, "PinholeStereoPoseTcwFactor Jacobian");
}

// -----------------------------------------------------------------------------
// PinholeMonoTcwFactor
// -----------------------------------------------------------------------------
TEST(GTSAMFactors, PinholeMonoTcwFactor)
{
    const std::vector<float> params = {500.f, 500.f, 320.f, 240.f};
    Pinhole camera(params);

    const gtsam::Pose3 Tcw(gtsam::Rot3::RzRyRx(0.02, -0.01, 0.03), gtsam::Point3(0.05, -0.03, 0.1));
    const gtsam::Point3 Xw(1.8, -0.2, 4.0);

    const gtsam::Point3 Xc_p = Tcw.transformFrom(Xw);
    const Eigen::Vector3d Xc(Xc_p.x(), Xc_p.y(), Xc_p.z());
    const Eigen::Vector2d obs = camera.project(Xc);

    gtsam::SharedNoiseModel noise = gtsam::noiseModel::Unit::Create(2);
    PinholeMonoTcwFactor factor(poseKey(0), pointKey(0), obs, noise, &camera);

    gtsam::Vector err = factor.evaluateError(Tcw, Xw, boost::none, boost::none);
    EXPECT_NEAR(0.0, err.norm(), kTolError) << "PinholeMonoTcwFactor error at consistent state";

    gtsam::Matrix H1_analytical(2, 6), H2_analytical(2, 3);
    factor.evaluateError(Tcw, Xw, H1_analytical, H2_analytical);

    auto err_fn = [&factor](const gtsam::Pose3& Tcw_, const gtsam::Point3& Xw_) -> gtsam::Vector
    {
        return factor.evaluateError(Tcw_, Xw_, boost::none, boost::none);
    };
    Eigen::MatrixXd H1_num = gtsam::numericalDerivative21<gtsam::Vector, gtsam::Pose3, gtsam::Point3>(err_fn, Tcw, Xw);
    Eigen::MatrixXd H2_num = gtsam::numericalDerivative22<gtsam::Vector, gtsam::Pose3, gtsam::Point3>(err_fn, Tcw, Xw);

    ExpectMatrixNear(H1_analytical, H1_num, kTolJacobian, "PinholeMonoTcwFactor H_pose");
    ExpectMatrixNear(H2_analytical, H2_num, kTolJacobian, "PinholeMonoTcwFactor H_point");
}

// -----------------------------------------------------------------------------
// PinholeStereoTcwFactor
// -----------------------------------------------------------------------------
TEST(GTSAMFactors, PinholeStereoTcwFactor)
{
    const std::vector<float> params = {500.f, 500.f, 320.f, 240.f};
    Pinhole camera(params);
    const double bf = 100.0;

    const gtsam::Pose3 Tcw(gtsam::Rot3::RzRyRx(-0.01, 0.02, -0.02), gtsam::Point3(-0.08, 0.04, 0.12));
    const gtsam::Point3 Xw(1.2, 0.25, 3.2);

    const gtsam::Point3 Xc_p = Tcw.transformFrom(Xw);
    const Eigen::Vector3d Xc(Xc_p.x(), Xc_p.y(), Xc_p.z());

    const Eigen::Vector2d proj2 = camera.project(Xc);
    Eigen::Vector3d obs3;
    obs3 << proj2(0), proj2(1), proj2(0) - bf / Xc.z();

    gtsam::SharedNoiseModel noise = gtsam::noiseModel::Unit::Create(3);
    PinholeStereoTcwFactor factor(poseKey(0), pointKey(0), obs3, bf, noise, &camera);

    gtsam::Vector err = factor.evaluateError(Tcw, Xw, boost::none, boost::none);
    EXPECT_NEAR(0.0, err.norm(), kTolError) << "PinholeStereoTcwFactor error at consistent state";

    gtsam::Matrix H1_analytical(3, 6), H2_analytical(3, 3);
    factor.evaluateError(Tcw, Xw, H1_analytical, H2_analytical);

    auto err_fn = [&factor](const gtsam::Pose3& Tcw_, const gtsam::Point3& Xw_) -> gtsam::Vector
    {
        return factor.evaluateError(Tcw_, Xw_, boost::none, boost::none);
    };
    Eigen::MatrixXd H1_num = gtsam::numericalDerivative21<gtsam::Vector, gtsam::Pose3, gtsam::Point3>(err_fn, Tcw, Xw);
    Eigen::MatrixXd H2_num = gtsam::numericalDerivative22<gtsam::Vector, gtsam::Pose3, gtsam::Point3>(err_fn, Tcw, Xw);

    ExpectMatrixNear(H1_analytical, H1_num, kTolJacobian, "PinholeStereoTcwFactor H_pose");
    ExpectMatrixNear(H2_analytical, H2_num, kTolJacobian, "PinholeStereoTcwFactor H_point");
}
// -----------------------------------------------------------------------------
// Sim3ProjectionFactor
// -----------------------------------------------------------------------------
TEST(GTSAMFactors, Sim3ProjectionFactor)
{
    const std::vector<float> params = {500.f, 500.f, 320.f, 240.f};
    Pinhole camera(params);
    const Eigen::Vector3d P3Dc(0.1, -0.05, 2.0);
    const gtsam::Similarity3 S12(gtsam::Rot3::Identity(), gtsam::Point3(0, 0, 0), 1.0);
    const gtsam::Point3 p = S12.transformFrom(gtsam::Point3(P3Dc.x(), P3Dc.y(), P3Dc.z()));
    const Eigen::Vector2d obs = camera.project(Eigen::Vector3d(p.x(), p.y(), p.z()));

    gtsam::SharedNoiseModel noise = gtsam::noiseModel::Unit::Create(2);
    Sim3ProjectionFactor factor(sim3Key(0), P3Dc, obs, noise, &camera);

    gtsam::Vector err = factor.evaluateError(S12, boost::none);
    EXPECT_NEAR(0.0, err.norm(), kTolError) << "Sim3ProjectionFactor error at consistent state";

    Eigen::MatrixXd H_analytical(2, 7);
    factor.evaluateError(S12, H_analytical);

    auto err_fn = [&factor](const gtsam::Similarity3& S) -> gtsam::Vector
    {
        return factor.evaluateError(S, boost::none);
    };
    Eigen::MatrixXd H_num = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Similarity3>(err_fn, S12);
    ExpectMatrixNear(H_analytical, H_num, kTolJacobian, "Sim3ProjectionFactor Jacobian");
}

// -----------------------------------------------------------------------------
// InverseSim3ProjectionFactor
// -----------------------------------------------------------------------------
TEST(GTSAMFactors, InverseSim3ProjectionFactor)
{
    const std::vector<float> params = {500.f, 500.f, 320.f, 240.f};
    Pinhole camera(params);
    const Eigen::Vector3d P3Dc2(0.2, 0.1, 3.0);
    const gtsam::Similarity3 S12(gtsam::Rot3::Identity(), gtsam::Point3(0, 0, 0), 1.0);
    const gtsam::Similarity3 Sinv = S12.inverse();
    const gtsam::Point3 p = Sinv.transformFrom(gtsam::Point3(P3Dc2.x(), P3Dc2.y(), P3Dc2.z()));
    const Eigen::Vector2d obs = camera.project(Eigen::Vector3d(p.x(), p.y(), p.z()));

    gtsam::SharedNoiseModel noise = gtsam::noiseModel::Unit::Create(2);
    InverseSim3ProjectionFactor factor(sim3Key(0), P3Dc2, obs, noise, &camera);

    gtsam::Vector err = factor.evaluateError(S12, boost::none);
    EXPECT_NEAR(0.0, err.norm(), kTolError) << "InverseSim3ProjectionFactor error at consistent state";

    Eigen::MatrixXd H_analytical(2, 7);
    factor.evaluateError(S12, H_analytical);

    auto err_fn = [&factor](const gtsam::Similarity3& S) -> gtsam::Vector
    {
        return factor.evaluateError(S, boost::none);
    };
    Eigen::MatrixXd H_num = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Similarity3>(err_fn, S12);
    ExpectMatrixNear(H_analytical, H_num, kTolJacobian, "InverseSim3ProjectionFactor Jacobian");
}

}  // namespace
